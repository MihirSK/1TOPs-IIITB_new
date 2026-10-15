// ============================================================================
//  tb_uart_full.sv  -  Functional verification testbench for uart_16550
//  Plain SystemVerilog (no covergroups/SVA, no dut.* references), xsim/Verilator.
//
//  Everything is observed at the pins:
//   * TX monitor: decodes every frame from the tx pin, checks that each edge
//     lies on a bit boundary (bit time = 16 * divisor clocks, exact), start bit
//     0, 8 data bits LSB first, stop bit 1, minimum frame spacing, and compares
//     every byte with the queue of bytes the CPU wrote (tx_exp).
//   * RX driver: a separate process drives rx frames (exact / +-2% / back-to-back /
//     glitches / bad stop); every byte is compared with rx_exp when read.
//   * Bus monitor: ack one cycle after an accepted request, one cycle wide,
//     never without a request, no X.
//   * LSR rules: TEMT=1 only when nothing is pending or shifting, THRE=1 only
//     when the FIFO is empty, DR matches the RX queue.
//  Master styles: PULSE (req high for one edge) is the contract (main suite);
//  HOLD (req held until ack is seen, as a synchronous master does) is tested in
//  O1 and reported as an OPEN ITEM (bus handshake decision).
//
//  Result lines:  RESULT: PASS | PASS WITH OPEN ITEMS (n) | FAIL
//  -define STRICT_OPEN makes open items fail.
// ============================================================================
`timescale 1ns/1ps

module tb_uart_full;

  localparam int DEF_DIV = 100_000_000 / (115_200 * 16);   // 54

  logic        clk   = 1'b0;
  logic        rst_n = 1'b0;
  always #5 clk = ~clk;

  logic [2:0]  addr  = '0;
  logic [7:0]  wdata = '0;
  logic        req   = 1'b0;
  logic        we    = 1'b0;
  logic [7:0]  rdata;
  logic        ack;
  logic        tx;
  logic        rx    = 1'b1;

  uart_16550 dut (.*);

  // --------------------------------------------------------------------------
  // Error bookkeeping
  // --------------------------------------------------------------------------
  int errors    = 0;
  int open_hits = 0;
  bit open_mode = 1'b0;

  function automatic void err(input string m);
    if (open_mode) begin
`ifdef STRICT_OPEN
      errors++;
      if (errors <= 60) $display("[%0t] ERROR (open item, strict): %s", $time, m);
`else
      open_hits++;
      if (open_hits <= 12) $display("[%0t] OPEN-ITEM: %s", $time, m);
`endif
    end else begin
      errors++;
      if (errors <= 60) $display("[%0t] ERROR: %s", $time, m);
    end
  endfunction

  // clock edge counter: monitors see the index of the current edge, drivers
  // (which act at negedge) see the index of the NEXT edge.
  int cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;

  // --------------------------------------------------------------------------
  // Expectations and test state
  // --------------------------------------------------------------------------
  logic [7:0] tx_exp [$];     // bytes written to THR and not yet seen on tx
  logic [7:0] rx_exp [$];     // bytes fully received on rx and not yet read from RBR
  int  cur_n = DEF_DIV;       // divisor currently programmed
  int  bt    = 16 * DEF_DIV;  // expected bit time in clocks
  bit  armed = 1'b0;
  bit  hold_mode = 1'b0;

  // coverage counters (a hole fails the run)
  int cov_rw   [8][2];
  int cov_txb1 [8];
  int cov_txb0 [8];
  int cov_rxb1 [8];
  int cov_rxb0 [8];
  int cov_div  [9];           // divisors 1,2,3,4,5,8,16,54,100 seen on tx
  int cov_ev [16];            // named events, see report_cov

  // --------------------------------------------------------------------------
  // Passive monitor: bus protocol + TX frame decoder
  // --------------------------------------------------------------------------
  logic tx_q = 1'b1, req_q = 1'b0, ack_q = 1'b0;
  int   tx_state = 0;
  int   t0 = 0, last_t0 = -1000000;
  logic [7:0] tx_sh = '0;
  int   tx_dec = 0;
  int   ev_stop = -1, ev_stop_prev = -1, n_ev_stop = 0;
  int   ph, rem, kk;
  localparam int TOL = 1;
  logic [7:0] exp_b;

  function automatic int div_idx(input int n);
    case (n)
      1: return 0;  2: return 1;  3: return 2;  4: return 3;  5: return 4;
      8: return 5;  16: return 6; 54: return 7; 100: return 8;
      default: return -1;
    endcase
  endfunction

  always @(posedge clk) begin
    if (!rst_n) begin
      tx_state = 0; tx_q = 1'b1; req_q = 1'b0; ack_q = 1'b0; last_t0 = -1000000;
    end else if (armed) begin
      if ($isunknown(tx))  err("X/Z on tx");
      if ($isunknown(ack)) err("X/Z on ack");
      if (ack === 1'b1 && !req_q) err("ack without req in the previous cycle");
      if (ack === 1'b1 && ack_q)  err("ack asserted for more than one cycle");

      if (tx_state == 0) begin
        if (tx_q === 1'b1 && tx === 1'b0) begin
          tx_state = 1; t0 = cyc;
          if (cyc - last_t0 < 10 * bt - TOL)
            err($sformatf("new start bit %0d clocks after the previous one (< 10 bit times = %0d)", cyc - last_t0, 10 * bt));
          last_t0 = cyc;
        end
      end else begin
        ph = cyc - t0;
        if (tx !== tx_q) begin
          rem = ph % bt;
          if (rem > TOL && rem < bt - TOL)
            err($sformatf("tx edge %0d clocks into a bit (bit time %0d): baud rate / bit timing wrong", rem, bt));
          if (ph == 9 * bt && tx === 1'b1) begin
            ev_stop_prev = ev_stop; ev_stop = cyc; n_ev_stop++;
          end
        end
        if (ph % bt == bt / 2) begin
          kk = ph / bt;
          if (kk == 0) begin
            if (tx !== 1'b0) err("start bit is not low in the middle of the bit");
          end else if (kk <= 8) begin
            tx_sh[kk - 1] = tx;
          end else begin
            if (tx !== 1'b1) err("stop bit is not high");
            tx_state = 0;
            tx_dec++;
            if (tx_exp.size() == 0) err($sformatf("unexpected TX byte %h (nothing pending)", tx_sh));
            else begin
              exp_b = tx_exp.pop_front();
              if (tx_sh !== exp_b) err($sformatf("TX byte %h, expected %h", tx_sh, exp_b));
            end
            for (int b = 0; b < 8; b++) begin
              if (tx_sh[b]) cov_txb1[b]++; else cov_txb0[b]++;
            end
            if (div_idx(cur_n) >= 0) cov_div[div_idx(cur_n)]++;
          end
        end
      end
      tx_q = tx; req_q = req; ack_q = (ack === 1'b1);
    end
  end

  // --------------------------------------------------------------------------
  // Bus master (drives at negedge, returns at a negedge)
  // --------------------------------------------------------------------------
  task automatic bus_op(input logic [2:0] a, input bit w, input logic [7:0] d, output logic [7:0] rd);
    addr = a; we = w; wdata = d; req = 1'b1;
    if (!hold_mode) begin
      @(negedge clk);                                  // one edge after the request
      if (ack !== 1'b1) err("no ack in the cycle after the request");
      rd = rdata; req = 1'b0;
      @(negedge clk);
    end else begin
      @(posedge clk);                                  // request accepted here
      @(posedge clk);                                  // synchronous master sees ack here
      if (ack !== 1'b1) err("HOLD master: no ack one edge after the accepted request");
      rd = rdata;
      #1 req = 1'b0;
      @(negedge clk);
    end
    cov_rw[a][w ? 1 : 0]++;
  endtask

  task automatic bus_wr(input logic [2:0] a, input logic [7:0] d);
    logic [7:0] dummy;
    bus_op(a, 1'b1, d, dummy);
  endtask

  task automatic bus_rd(input logic [2:0] a, output logic [7:0] v);
    bus_op(a, 1'b0, 8'h00, v);
    if ($isunknown(v)) err($sformatf("X/Z in read data of register %0d", a));
  endtask

  task automatic idle(input int n);
    req = 1'b0;
    repeat (n) @(negedge clk);
  endtask

  task automatic banner(input string s);
    $display("[%0t] ---- %s", $time, s);
  endtask

  task automatic do_reset();
    req = 1'b0; we = 1'b0; rst_n = 1'b0;
    repeat (3) @(negedge clk);
    armed = 1'b1;
    tx_exp.delete(); rx_exp.delete();
    rst_n = 1'b1;
    @(negedge clk);
    bt = 16 * DEF_DIV; cur_n = DEF_DIV; last_t0 = -1000000;
  endtask

  // asynchronous reset asserted in the middle of a clock phase
  task automatic reset_async(input int cycles);
    #2 rst_n = 1'b0;
    repeat (cycles) @(negedge clk);
    tx_exp.delete(); rx_exp.delete();
    rst_n = 1'b1;
    @(negedge clk);
    bt = 16 * DEF_DIV; cur_n = DEF_DIV;
  endtask

  task automatic prog_div(input int n);
    logic [7:0] v;
    int prev;
    prev = cur_n;
    bus_wr(3, 8'h83);
    bus_wr(0, n[7:0]);
    bus_wr(1, n[15:8]);
    bus_rd(0, v); if (v !== n[7:0])  err($sformatf("DLL read back %h, expected %h", v, n[7:0]));
    bus_rd(1, v); if (v !== n[15:8]) err($sformatf("DLM read back %h, expected %h", v, n[15:8]));
    bus_wr(3, 8'h03);
    cur_n = n; bt = 16 * n; last_t0 = -1000000;
    idle(2 * prev + 2 * n + 10);            // let the baud generator settle
  endtask

  task automatic thr(input logic [7:0] b);
    bus_wr(0, b);
    tx_exp.push_back(b);
  endtask

  task automatic wait_tx_all(input int tmo);
    int c = 0;
    while (tx_exp.size() > 0 && c < tmo) begin @(negedge clk); c = c + 1; end
    if (tx_exp.size() > 0) begin
      err($sformatf("TX: %0d byte(s) never transmitted", tx_exp.size()));
      tx_exp.delete();
    end
  endtask

  task automatic wait_tx_done();            // all bytes out, stop bit finished
    wait_tx_all((tx_exp.size() + 1) * 14 * bt + 400);
    idle(bt + 20);
  endtask

  // --------------------------------------------------------------------------
  // RX driver process: frames are queued, the process drives them at negedge
  //   kind 0 normal frame (byte pushed to rx_exp at the end of the stop bit)
  //   kind 1 frame with stop bit = 0 (framing error, nothing expected)
  //   kind 2 glitch: rx low for val clocks, then high
  //   kind 3 idle high for val clocks
  // --------------------------------------------------------------------------
  int   rxq_kind [$];
  int   rxq_val  [$];
  int   rxq_bit  [$];
  int   rxq_gap  [$];
  bit   rx_drv_busy = 1'b0;
  int   rx_start_cyc = 0;

  task automatic rx_enq(input int kind, input logic [7:0] v, input int bitclk, input int gap);
    rxq_kind.push_back(kind); rxq_val.push_back(int'(v)); rxq_bit.push_back(bitclk); rxq_gap.push_back(gap);
  endtask

  task automatic rx_driver();
    int k, v, b, g;
    logic [7:0] byt;
    forever begin
      @(negedge clk);
      if (rxq_kind.size() > 0) begin
        k = rxq_kind.pop_front(); v = rxq_val.pop_front();
        b = rxq_bit.pop_front();  g = rxq_gap.pop_front();
        byt = v[7:0];
        rx_drv_busy = 1'b1;
        case (k)
          0, 1: begin
            rx_start_cyc = cyc;
            rx = 1'b0; repeat (b) @(negedge clk);
            for (int i = 0; i < 8; i++) begin rx = byt[i]; repeat (b) @(negedge clk); end
            rx = (k == 1) ? 1'b0 : 1'b1; repeat (b) @(negedge clk);
            if (k == 0) begin
              rx_exp.push_back(byt);
              for (int i = 0; i < 8; i++) begin
                if (byt[i]) cov_rxb1[i]++; else cov_rxb0[i]++;
              end
            end
            rx = 1'b1;
            if (g > 0) repeat (g) @(negedge clk);
          end
          2: begin rx = 1'b0; repeat (v) @(negedge clk); rx = 1'b1; if (g > 0) repeat (g) @(negedge clk); end
          default: begin rx = 1'b1; repeat (v) @(negedge clk); end
        endcase
        rx_drv_busy = 1'b0;
      end
    end
  endtask

  initial rx_driver();

  task automatic rx_wait();
    while (rxq_kind.size() > 0 || rx_drv_busy) @(negedge clk);
    idle(2 * bt + 40);
  endtask

  // exact frames at the programmed divisor, 1 bit of idle between frames
  task automatic rx_send(input logic [7:0] v);
    rx_enq(0, v, bt, bt);
  endtask

  // --------------------------------------------------------------------------
  // Register read helpers with the LSR rules
  // --------------------------------------------------------------------------
  task automatic lsr_rd(output logic [7:0] v);
    int pend;
    bus_rd(5, v);
    pend = tx_exp.size();
    if (v[6] && pend > 0)  err("LSR.TEMT=1 while a byte is still queued or being shifted out");
    if (v[5] && pend >= 2) err("LSR.THRE=1 while bytes are still queued in the TX FIFO");
    if (!v[5] && pend == 0) err("LSR.THRE=0 with nothing queued");
    if (!rx_drv_busy && rxq_kind.size() == 0)
      if (v[0] !== (rx_exp.size() > 0 ? 1'b1 : 1'b0))
        err($sformatf("LSR.DR=%0d but %0d byte(s) are expected in the RX FIFO", v[0], rx_exp.size()));
  endtask

  task automatic rbr_rd_chk();
    logic [7:0] v, e;
    bus_rd(0, v);
    if (rx_exp.size() == 0) err($sformatf("RBR read %h but nothing was expected", v));
    else begin
      e = rx_exp.pop_front();
      if (v !== e) err($sformatf("RBR read %h, expected %h", v, e));
    end
  endtask

  task automatic rx_drain();
    logic [7:0] v;
    int g = 0;
    while (rx_exp.size() > 0 && g < 40) begin
      lsr_rd(v);
      rbr_rd_chk();
      g = g + 1;
    end
    lsr_rd(v);                                       // DR must be 0 now
    if (v[0]) err("RX FIFO not empty after reading every expected byte");
  endtask

  // --------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------
  task automatic test_reset_defaults();
    logic [7:0] v;
    banner("U1 reset defaults and idle line");
    do_reset();
    idle(20);
    lsr_rd(v); if (v !== 8'h60) err($sformatf("LSR after reset = %h, expected 60", v));
    bus_rd(2, v); if (v !== 8'hC1) err($sformatf("IIR after reset = %h, expected C1", v));
    bus_rd(3, v); if (v !== 8'h03) err($sformatf("LCR after reset = %h, expected 03", v));
    bus_rd(1, v); if (v !== 8'h00) err($sformatf("IER after reset = %h, expected 00", v));
    bus_rd(4, v); if (v !== 8'h00) err($sformatf("MCR after reset = %h, expected 00", v));
    bus_rd(6, v);
    bus_wr(3, 8'h83);
    bus_rd(0, v); if (v !== DEF_DIV[7:0]) err($sformatf("DLL after reset = %h, expected %h", v, DEF_DIV[7:0]));
    bus_rd(1, v); if (v !== DEF_DIV[15:8]) err($sformatf("DLM after reset = %h, expected %h", v, DEF_DIV[15:8]));
    bus_wr(3, 8'h03);
    for (int i = 0; i < 100; i++) begin
      if (tx !== 1'b1) err("tx not idle-high after reset");
      @(negedge clk);
    end
  endtask

  task automatic test_registers();
    logic [7:0] v;
    banner("U2 register read/write, DLAB aliasing, read-only registers");
    bus_wr(1, 8'h0F); bus_rd(1, v); if ((v & 8'h0F) !== 8'h0F) err($sformatf("IER low nibble %h", v));
    bus_wr(1, 8'h05); bus_rd(1, v); if ((v & 8'h0F) !== 8'h05) err($sformatf("IER low nibble %h", v));
    bus_wr(3, 8'h1B); bus_rd(3, v); if (v !== 8'h1B) err($sformatf("LCR %h, expected 1B", v));
    bus_wr(3, 8'h03); bus_rd(3, v); if (v !== 8'h03) err($sformatf("LCR %h, expected 03", v));
    bus_wr(4, 8'h13); bus_rd(4, v); if ((v & 8'h1F) !== 8'h13) err($sformatf("MCR %h, expected 13", v));
    bus_wr(4, 8'h0B); bus_rd(4, v); if ((v & 8'h1F) !== 8'h0B) err($sformatf("MCR %h, expected 0B", v));
    bus_wr(4, 8'h00);
    bus_wr(7, 8'h55); bus_rd(7, v); if (v !== 8'h55) err($sformatf("SCR %h, expected 55", v));
    bus_wr(7, 8'hAA); bus_rd(7, v); if (v !== 8'hAA) err($sformatf("SCR %h, expected AA", v));
    // divisor latch access, THR / IER must not be touched
    bus_wr(3, 8'h83);
    bus_wr(0, 8'h12); bus_rd(0, v); if (v !== 8'h12) err($sformatf("DLL %h, expected 12", v));
    bus_wr(1, 8'h34); bus_rd(1, v); if (v !== 8'h34) err($sformatf("DLM %h, expected 34", v));
    bus_wr(3, 8'h03);
    bus_rd(1, v); if ((v & 8'h0F) !== 8'h05) err($sformatf("IER changed by a DLM write: %h", v));
    // writes to read-only registers change nothing
    bus_wr(5, 8'hFF); bus_wr(6, 8'hFF);
    lsr_rd(v); if (v !== 8'h60) err($sformatf("LSR after write to LSR = %h", v));
    bus_rd(2, v); if (v !== 8'hC1) err($sformatf("IIR = %h", v));
    bus_wr(1, 8'h00);
    // THR write with DLAB=1 goes to DLL and sends nothing; RBR read with DLAB=1 reads DLL and pops nothing
    prog_div(2);
    rx_send(8'hC3); rx_wait();
    bus_wr(3, 8'h83);
    bus_wr(0, 8'h02);                               // DLL = 2 again, must not transmit
    bus_rd(0, v); if (v !== 8'h02) err($sformatf("DLL %h expected 02", v));
    bus_wr(3, 8'h03);
    idle(2 * bt);
    lsr_rd(v);
    rbr_rd_chk();                                   // byte C3 still there
    rx_drain();
  endtask

  task automatic test_baud_frames();
    int dv [9];
    logic [7:0] pat [6];
    banner("U3 TX baud rate and frame format for divisors 1..100 and the default divisor");
    do_reset();
    thr(8'h5A);                                     // default divisor after reset
    wait_tx_done();
    dv[0] = 1; dv[1] = 2; dv[2] = 3; dv[3] = 4; dv[4] = 5; dv[5] = 8; dv[6] = 16; dv[7] = 54; dv[8] = 100;
    pat[0] = 8'h55; pat[1] = 8'hA3; pat[2] = 8'h00; pat[3] = 8'hFF; pat[4] = 8'h80; pat[5] = 8'h01;
    for (int i = 0; i < 9; i++) begin
      prog_div(dv[i]);
      for (int j = 0; j < ((dv[i] >= 54) ? 2 : 6); j++) thr(pat[j]);
      wait_tx_done();
    end
  endtask

  task automatic test_tx_fifo();
    int d0, k;
    banner("U4 TX FIFO: 16 queued bytes in order, overflow keeps 16 + 1 bytes and drops the tail");
    do_reset(); prog_div(2);
    for (int i = 0; i < 16; i++) thr(8'(8'h20 + i));
    wait_tx_done();
    // overflow: 40 quick writes, the accepted ones must be the first 17 bytes (16 FIFO + shifter)
    d0 = tx_dec;
    for (int i = 0; i < 40; i++) thr(8'(8'h40 + i));
    begin
      int q = 0, last = tx_dec, tmo = 0;
      while (q < 24 * bt && tmo < 800 * bt) begin
        @(negedge clk); tmo = tmo + 1;
        if (tx_dec != last) begin last = tx_dec; q = 0; end else q = q + 1;
      end
    end
    k = tx_dec - d0;
    if (k != 17) err($sformatf("overflow: %0d bytes transmitted, expected 17 (16-byte FIFO + the byte in the shifter)", k));
    cov_ev[0]++;
    tx_exp.delete();
    idle(bt + 20);
  endtask

  task automatic test_lsr_tx();
    logic [7:0] v;
    int g;
    banner("U5 LSR THRE/TEMT while transmitting");
    do_reset(); prog_div(2);
    lsr_rd(v); if (v !== 8'h60) err($sformatf("LSR idle = %h", v));
    thr(8'h11); thr(8'h22); thr(8'h33); thr(8'h44);
    lsr_rd(v);
    if (v[5] || v[6]) err($sformatf("LSR right after 4 writes = %h, THRE/TEMT must be 0", v));
    g = 0;
    while (tx_exp.size() > 0 && g < 3000) begin
      lsr_rd(v);
      g = g + 1;
    end
    idle(bt + 30);
    lsr_rd(v); if (v !== 8'h60) err($sformatf("LSR after everything is sent = %h, expected 60", v));
  endtask

  task automatic test_tx_collision();
    int S2, Pfr, X, t;
    banner("U6 THR write landing exactly on the TX FIFO pop (sweep over the whole frame end window)");
    for (int d = -3; d <= 16 * 2 + 2 * 2 + 10; d++) begin
      do_reset(); prog_div(2);
      thr(8'h11); thr(8'h22); thr(8'h33); thr(8'h44);
      t = 0;
      while (n_ev_stop < 2 && t < 100 * bt) begin @(negedge clk); t = t + 1; end
      // ev_stop counts stop-bit edges since the start of simulation: use the last two
      Pfr = ev_stop - ev_stop_prev;
      X   = ev_stop + Pfr + d;
      while (cyc < X) @(negedge clk);
      thr(8'h55);
      wait_tx_all(20 * bt);
      idle(bt + 10);
      cov_ev[1]++;
    end
  endtask

  task automatic test_fcr_tx();
    int d0, k;
    banner("U7 FCR bit 2 clears the TX FIFO");
    do_reset(); prog_div(4);
    d0 = tx_dec;
    for (int i = 0; i < 8; i++) thr(8'(8'h11 * (i + 1)));
    bus_wr(2, 8'h05);
    idle(14 * bt);
    k = tx_dec - d0;
    if (k < 1 || k > 2) err($sformatf("after FCR TX-clear %0d bytes were transmitted, expected 1 (or 2)", k));
    tx_exp.delete();
    thr(8'h66);
    wait_tx_done();
    cov_ev[2]++;
  endtask

  task automatic test_rx_basic();
    int dv [7];
    logic [7:0] pat [8];
    banner("U8 RX exact timing for divisors 1..54, all bit patterns");
    dv[0] = 1; dv[1] = 2; dv[2] = 3; dv[3] = 4; dv[4] = 8; dv[5] = 16; dv[6] = 54;
    pat[0] = 8'h00; pat[1] = 8'hFF; pat[2] = 8'h55; pat[3] = 8'hAA; pat[4] = 8'h80; pat[5] = 8'h01;
    pat[6] = 8'h0F; pat[7] = 8'hF0;
    do_reset();
    for (int i = 0; i < 7; i++) begin
      prog_div(dv[i]);
      for (int j = 0; j < ((dv[i] == 54) ? 2 : 8); j++) rx_send(pat[j]);
      rx_wait();
      rx_drain();
    end
  endtask

  task automatic test_rx_backtoback();
    banner("U9 RX back-to-back frames without idle time (exact timing)");
    do_reset();
    for (int n = 2; n <= 4; n++) begin
      prog_div(n);
      for (int j = 0; j < 12; j++) rx_enq(0, 8'($urandom), bt, 0);
      rx_wait();
      rx_drain();
    end
  endtask

  task automatic test_rx_tolerance();
    logic [7:0] v;
    banner("U10 RX with a sender that is 2% slow / 2% fast");
    do_reset(); prog_div(16);
    for (int j = 0; j < 8; j++) rx_enq(0, 8'($urandom), 261, 261);
    rx_wait(); rx_drain(); cov_ev[3]++;
    for (int j = 0; j < 8; j++) rx_enq(0, 8'($urandom), 251, 251);
    rx_wait(); rx_drain(); cov_ev[4]++;
    for (int j = 0; j < 8; j++) rx_enq(0, 8'($urandom), 261, 0);       // slow, no idle
    rx_wait(); rx_drain();
    for (int j = 0; j < 8; j++) rx_enq(0, 8'($urandom), 251, 0);       // fast, no idle
    rx_wait(); rx_drain();
  endtask

  task automatic test_rx_fifo();
    logic [7:0] v;
    banner("U11 RX FIFO: 16 bytes in order, the 17th is dropped");
    do_reset(); prog_div(2);
    for (int j = 0; j < 17; j++) begin
      rx_send(8'(8'h30 + j));
      if (j == 16) begin end
    end
    rx_wait();
    // the 17th byte pushed to rx_exp by the driver is not kept by the DUT
    if (rx_exp.size() == 17) rx_exp.pop_back();
    rx_drain();
    cov_ev[5]++;
  endtask

  task automatic test_rx_glitch();
    logic [7:0] v;
    banner("U12 RX ignores a start-bit glitch shorter than half a bit");
    do_reset(); prog_div(8);
    for (int gl = 0; gl < 3; gl++) begin
      rx_enq(2, 8'(gl == 0 ? 2 : (gl == 1 ? 20 : 50)), 0, 4 * bt);
      rx_send(8'h3C);
      rx_wait();
      rx_drain();
    end
    cov_ev[6]++;
  endtask

  task automatic test_rx_collision();
    logic [7:0] v;
    int X;
    banner("U13 RBR read landing exactly on the RX push (sweep over the stop bit window)");
    for (int off = -4; off <= 16 * 2 + 8; off++) begin
      do_reset(); prog_div(2);
      rx_send(8'hA5); rx_wait();
      rx_send(8'h5A);
      while (!rx_drv_busy) @(negedge clk);
      X = rx_start_cyc + 9 * bt + off;
      while (cyc < X) @(negedge clk);
      rbr_rd_chk();                                   // A
      rx_wait();
      rx_drain();                                     // B must still be there
      cov_ev[7]++;
    end
  endtask

  task automatic test_fcr_rx();
    logic [7:0] v;
    banner("U14 FCR bit 1 clears the RX FIFO");
    do_reset(); prog_div(2);
    rx_send(8'h11); rx_send(8'h22); rx_send(8'h33);
    rx_wait();
    rx_exp.delete();                                // discarded by the clear
    bus_wr(2, 8'h03);
    lsr_rd(v); if (v[0]) err("LSR.DR still set after FCR RX-clear");
    rx_send(8'h99); rx_wait(); rx_drain();
    cov_ev[8]++;
  endtask

  task automatic test_reset_mid();
    logic [7:0] v;
    banner("U15 reset in the middle of a TX frame and of an RX frame");
    do_reset(); prog_div(2);
    thr(8'h21); thr(8'h22); thr(8'h23);
    while (tx !== 1'b0) @(negedge clk);
    idle(3 * bt);
    reset_async(2);
    if (tx !== 1'b1) err("tx not high right after reset");
    lsr_rd(v); if (v !== 8'h60) err($sformatf("LSR after reset during TX = %h", v));
    bus_rd(3, v); if (v !== 8'h03) err($sformatf("LCR after reset = %h", v));
    idle(30 * DEF_DIV);                             // no frame may appear (unexpected bytes are errors)
    cov_ev[9]++;
    prog_div(2);
    thr(8'h21); wait_tx_done();
    // reset while a frame is being received
    rx_send(8'h81); rx_send(8'h42);
    while (!rx_drv_busy) @(negedge clk);
    idle(15 * bt);
    reset_async(2);
    lsr_rd(v); if (v[0]) err("LSR.DR set right after reset");
    rx_exp.delete(); rxq_kind.delete(); rxq_val.delete(); rxq_bit.delete(); rxq_gap.delete();
    while (rx_drv_busy) @(negedge clk);
    idle(20);
    do_reset();                                     // second reset: garbage from the truncated frame is gone
    lsr_rd(v); if (v !== 8'h60) err($sformatf("LSR after second reset = %h", v));
    prog_div(2);
    rx_send(8'h44); rx_wait(); rx_drain();
    cov_ev[10]++;
  endtask

  task automatic test_random(input int nops);
    logic [7:0] v, b;
    int r, g;
    banner("U16 constrained random (THR/RBR/LSR/SCR traffic, RX frames, divisor changes)");
    do_reset(); prog_div(2);
    for (int i = 0; i < nops; i++) begin
      if (i % 150 == 149) begin
        wait_tx_done(); rx_wait(); rx_drain();
        prog_div($urandom_range(1, 6));
      end
      r = $urandom_range(0, 99);
      if (r < 28) begin
        if (tx_exp.size() < 14) thr(8'($urandom));
      end else if (r < 42) begin
        lsr_rd(v);
      end else if (r < 56) begin
        if (rxq_kind.size() < 2 && rx_exp.size() + rxq_kind.size() + (rx_drv_busy ? 1 : 0) < 12)
          rx_enq(0, 8'($urandom), bt, $urandom_range(0, 3) * bt);
      end else if (r < 76) begin
        if (rx_exp.size() > 0 && !rx_drv_busy) rbr_rd_chk();
      end else if (r < 82) begin
        b = 8'($urandom); bus_wr(7, b); bus_rd(7, v); if (v !== b) err($sformatf("SCR %h expected %h", v, b));
      end else if (r < 86) begin
        bus_wr(5, 8'($urandom)); bus_wr(6, 8'($urandom));
      end else if (r < 90) begin
        bus_rd(2, v); if (v !== 8'hC1) err($sformatf("IIR %h", v));
      end else begin
        idle($urandom_range(1, 12));
      end
    end
    wait_tx_done(); rx_wait(); rx_drain();
  endtask

  // ------------------------------------------------------------------------
  // OPEN ITEM: a synchronous master that holds req until it sees ack.
  // With  ack <= req  the design sees req for two edges and repeats the access.
  // ------------------------------------------------------------------------
  task automatic test_hold_open();
    logic [7:0] v, e;
    banner("O1 OPEN ITEM: master holds req until ack (THR write, RBR read, register writes)");
    do_reset(); prog_div(2);
    open_mode = 1'b1; hold_mode = 1'b1;
    thr(8'h35);                                     // must be transmitted exactly once
    wait_tx_done();
    bus_wr(7, 8'h6C); bus_rd(7, v); if (v !== 8'h6C) err($sformatf("HOLD: SCR %h", v));
    bus_wr(1, 8'h09); bus_rd(1, v); if ((v & 8'h0F) !== 8'h09) err($sformatf("HOLD: IER %h", v));
    bus_wr(1, 8'h00);
    hold_mode = 1'b0;
    rx_send(8'h71); rx_send(8'h72); rx_wait();
    hold_mode = 1'b1;
    e = rx_exp.pop_front();
    bus_rd(0, v); if (v !== e) err($sformatf("HOLD: RBR %h expected %h", v, e));
    hold_mode = 1'b0;
    rx_drain();                                     // the second byte must still be there (one pop per read)
    hold_mode = 1'b1;
    lsr_rd(v);
    hold_mode = 1'b0;
    idle(5);
    open_mode = 1'b0;
    do_reset();
  endtask

  // information only: features the design does not implement
  task automatic probes();
    logic [7:0] v;
    banner("INFO probes (not counted): overrun / framing flags");
    do_reset(); prog_div(2);
    for (int j = 0; j < 17; j++) rx_send(8'(j));
    rx_wait();
    bus_rd(5, v);
    $display("  INFO: LSR after 17 bytes into a 16-byte FIFO = %h (bit1 overrun %s)", v, v[1] ? "set" : "NOT implemented");
    do_reset(); prog_div(2);
    rx_enq(1, 8'h55, bt, bt); rx_wait();
    bus_rd(5, v);
    $display("  INFO: LSR after a frame with stop bit 0 = %h (bit3 framing error %s)", v, v[3] ? "set" : "NOT implemented");
    rx_exp.delete();
    do_reset();
  endtask

  // --------------------------------------------------------------------------
  // Coverage closure: a hole is a failure
  // --------------------------------------------------------------------------
  task automatic report_cov();
    int h;
    for (int a = 0; a < 8; a++)
      for (int w = 0; w < 2; w++)
        if (cov_rw[a][w] == 0) err($sformatf("COVERAGE: register %0d %s never accessed", a, w ? "write" : "read"));
    for (int b = 0; b < 8; b++) begin
      if (cov_txb1[b] == 0 || cov_txb0[b] == 0) err($sformatf("COVERAGE: TX data bit %0d never seen as both 0 and 1", b));
      if (cov_rxb1[b] == 0 || cov_rxb0[b] == 0) err($sformatf("COVERAGE: RX data bit %0d never seen as both 0 and 1", b));
    end
    for (int i = 0; i < 9; i++)
      if (cov_div[i] == 0) err($sformatf("COVERAGE: divisor index %0d (1,2,3,4,5,8,16,54,100) never transmitted", i));
    if (cov_ev[0]  == 0) err("COVERAGE: TX FIFO overflow never exercised");
    if (cov_ev[1]  == 0) err("COVERAGE: TX push/pop collision sweep never ran");
    if (cov_ev[2]  == 0) err("COVERAGE: FCR TX clear never exercised");
    if (cov_ev[3]  == 0 || cov_ev[4] == 0) err("COVERAGE: RX tolerance (+2% / -2%) never exercised");
    if (cov_ev[5]  == 0) err("COVERAGE: RX FIFO overflow never exercised");
    if (cov_ev[6]  == 0) err("COVERAGE: RX glitch rejection never exercised");
    if (cov_ev[7]  == 0) err("COVERAGE: RX push/pop collision sweep never ran");
    if (cov_ev[8]  == 0) err("COVERAGE: FCR RX clear never exercised");
    if (cov_ev[9]  == 0 || cov_ev[10] == 0) err("COVERAGE: reset during TX / RX never exercised");
    $display(" coverage: registers 16/16 bits tx/rx 8/8 divisors 9/9 collision trials tx/rx=%0d/%0d tx frames=%0d",
             cov_ev[1], cov_ev[7], tx_dec);
  endtask

  // --------------------------------------------------------------------------
  initial begin
    @(negedge clk);
    do_reset();

    test_reset_defaults();
    test_registers();
    test_baud_frames();
    test_tx_fifo();
    test_lsr_tx();
    test_fcr_tx();
    test_rx_basic();
    test_rx_backtoback();
    test_rx_tolerance();
    test_rx_fifo();
    test_rx_glitch();
    test_fcr_rx();
    test_reset_mid();
    test_tx_collision();
    test_rx_collision();
    test_random(2500);
    test_hold_open();
    probes();

    idle(10);
    $display("==================================================");
    $display(" tx frames decoded=%0d", tx_dec);
    report_cov();
    $display(" contract errors = %0d   open-item hits = %0d  (bus handshake: req held until ack)", errors, open_hits);
    if (errors != 0)         $display(" *** RESULT: FAIL ***");
    else if (open_hits != 0) $display(" *** RESULT: PASS WITH OPEN ITEMS (%0d) - not signed off ***", open_hits);
    else                     $display(" *** RESULT: PASS ***");
    $display("==================================================");
    $finish;
  end

  initial begin
    #3_000_000_000;
    $display("GLOBAL WATCHDOG TIMEOUT");
    $display(" *** RESULT: FAIL ***");
    $finish;
  end

endmodule

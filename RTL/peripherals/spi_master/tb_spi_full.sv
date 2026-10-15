// ============================================================================
//  tb_spi_full.sv  -  Functional verification testbench for spi_master
//  Plain SystemVerilog (no covergroups/SVA, no dut.* references), xsim/Verilator.
//
//  Everything is observed at the pins:
//   * SPI slave model, mode-aware (CPOL/CPHA), changes MISO only on its shift
//     edge (1 ns after the SCK edge) and samples MOSI with real setup/hold
//     rules: the value BEFORE the SCK edge is used and a MOSI change at the
//     sampling edge is an error.  Every received byte is compared with the byte
//     the CPU wrote (tx_exp); every DATA read is compared with the slave's reply.
//   * Pin monitor: SCK half period must be exactly DIV+1 clocks, 16 edges per
//     byte, CSn asserted at least one half period before the first edge and
//     released at least one half period after the last, SCK idles at CPOL, no
//     SCK toggling while CSn is inactive, no X on the pins.
//   * APB: pready must be 1, no X on prdata.
//  Assumptions (documented in the report): SCK = clk/(2*(DIV+1)); CSn_POL=0 means
//  CSn is active low; CS_HOLD (CTRL[4]) keeps CSn asserted between transfers;
//  configuration and DATA writes are ignored while BUSY; DATA writes are ignored
//  while EN=0; reading DATA does not clear RX_VALID.
//
//  Result lines:  RESULT: PASS | FAIL
// ============================================================================
`timescale 1ns/1ps

module tb_spi_full;

  logic        clk   = 1'b0;
  logic        rst_n = 1'b0;
  always #5 clk = ~clk;

  logic [3:0]  paddr   = '0;
  logic [31:0] pwdata  = '0;
  logic        psel    = 1'b0;
  logic        penable = 1'b0;
  logic        pwrite  = 1'b0;
  logic [31:0] prdata;
  logic        pready;
  logic        spi_sck;
  logic        spi_mosi;
  logic        spi_miso = 1'b0;
  logic        spi_csn;

  spi_master dut (.*);

  // --------------------------------------------------------------------------
  int errors = 0;
  function automatic void err(input string m);
    errors++;
    if (errors <= 60) $display("[%0t] ERROR: %s", $time, m);
  endfunction

  int cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;

  // --------------------------------------------------------------------------
  // Model of the programmed configuration (updated only by cfg(), i.e. when idle)
  // --------------------------------------------------------------------------
  bit m_en = 0, m_cpol = 0, m_cpha = 0, m_csnpol = 0, m_hold = 0;
  int m_div = 7;
  bit armed = 1'b0;

  logic [7:0] slv_rsp [$];      // bytes the slave will send, one per byte received
  logic [7:0] tx_exp  [$];      // bytes the CPU wrote and the slave must receive

  // coverage counters (a hole fails the run)
  int cov_mode [4];
  int cov_divc [6];             // div 0,1,2,3, 4..99, >=100
  int cov_txb1 [8];
  int cov_txb0 [8];
  int cov_rxb1 [8];
  int cov_rxb0 [8];
  int cov_rw   [4][2];
  int cov_ev [12];

  function automatic bit cs_on(input logic c);
    return (c === (m_csnpol ? 1'b1 : 1'b0));
  endfunction

  function automatic int divc(input int d);
    if (d < 4) return d;
    if (d < 100) return 4;
    return 5;
  endfunction

  // --------------------------------------------------------------------------
  // SPI slave model
  // --------------------------------------------------------------------------
  logic [7:0] slv_tx = 8'hC5, slv_sh = 8'h00, slv_byte = 8'h00, slv_e;
  int   slv_nb = 0;
  bit   slv_cs = 1'b0;
  bit   slv_lead, slv_samp;
  logic mosi_hist = 1'b0;
  always @(posedge clk) mosi_hist <= spi_mosi;      // value BEFORE the edge, at edge time

  always @(spi_csn) begin
    if (armed && rst_n) begin
      if (cs_on(spi_csn)) begin
        slv_cs = 1'b1; slv_nb = 0;
        if (slv_rsp.size() > 0) slv_tx = slv_rsp.pop_front(); else slv_tx = 8'hC5;
        if (!m_cpha) begin #1 spi_miso = slv_tx[7]; end
      end else slv_cs = 1'b0;
    end
  end

  always @(spi_sck) begin
    if (armed && rst_n && slv_cs) begin
      slv_lead = (spi_sck !== m_cpol);                 // leaving the idle level
      slv_samp = m_cpha ? !slv_lead : slv_lead;
      if (slv_samp) begin
        if (spi_mosi !== mosi_hist) err("MOSI changes at the SCK sampling edge (no hold time)");
        slv_sh = {slv_sh[6:0], mosi_hist};
        slv_nb = slv_nb + 1;
        if (slv_nb == 8) begin
          slv_byte = slv_sh;
          slv_nb = 0;
          if (tx_exp.size() == 0) err($sformatf("slave received %h but no byte was written", slv_byte));
          else begin
            slv_e = tx_exp.pop_front();
            if (slv_byte !== slv_e) err($sformatf("slave received %h, CPU wrote %h (mode %0d%0d, div %0d)", slv_byte, slv_e, m_cpol, m_cpha, m_div));
            for (int b = 0; b < 8; b++) begin
              if (slv_e[b]) cov_txb1[b]++; else cov_txb0[b]++;
            end
          end
          if (slv_rsp.size() > 0) slv_tx = slv_rsp.pop_front(); else slv_tx = 8'hC5;
        end
      end else begin
        #1 spi_miso = slv_tx[7 - slv_nb];
      end
    end
  end

  // --------------------------------------------------------------------------
  // Pin monitor
  // --------------------------------------------------------------------------
  logic sck_q = 1'b0, csn_q = 1'b1;
  bit   have_q = 1'b0;
  int   cs_t = 0, last_edge_t = 0, edge_cnt = 0, idle_ok_until = -1, hpc, ref_t;

  always @(posedge clk) begin
    if (!rst_n) begin
      have_q = 1'b0; edge_cnt = 0;
    end else if (armed) begin
      hpc = m_div + 1;
      if ($isunknown(spi_sck) || $isunknown(spi_csn) || $isunknown(spi_mosi))
        err("X/Z on spi_sck / spi_csn / spi_mosi");
      if (psel && penable && pready !== 1'b1) err("pready is not 1 in the access phase");
      if (have_q) begin
        if (csn_q !== spi_csn) begin
          if (cs_on(spi_csn)) begin
            cs_t = cyc; edge_cnt = 0;
          end else begin
            if (edge_cnt % 16 != 0) err($sformatf("CSn released after %0d SCK edges (not whole bytes)", edge_cnt));
            if (edge_cnt > 0 && cyc - last_edge_t < hpc)
              err($sformatf("CSn released %0d clocks after the last SCK edge, need >= %0d", cyc - last_edge_t, hpc));
            if (spi_sck !== m_cpol) err("SCK is not at its idle level when CSn is released");
            edge_cnt = 0;
          end
        end
        if (sck_q !== spi_sck) begin
          if (!cs_on(spi_csn)) begin
            if (cyc > idle_ok_until) err("SCK toggles while CSn is inactive");
          end else begin
            if (edge_cnt % 16 == 0) begin
              ref_t = (edge_cnt == 0) ? cs_t : last_edge_t;
              if (cyc - ref_t < hpc)
                err($sformatf("first SCK edge of a byte %0d clocks after CSn/previous edge, need >= %0d (half period)", cyc - ref_t, hpc));
            end else if (cyc - last_edge_t != hpc)
              err($sformatf("SCK half period %0d clocks, expected %0d (DIV=%0d)", cyc - last_edge_t, hpc, m_div));
            edge_cnt++; last_edge_t = cyc;
          end
        end
      end
      sck_q = spi_sck; csn_q = spi_csn; have_q = 1'b1;
    end
  end

  // --------------------------------------------------------------------------
  // APB master (drives at negedge, returns at a negedge)
  // --------------------------------------------------------------------------
  task automatic apb_wr(input logic [3:0] a, input logic [31:0] d);
    paddr = a; pwdata = d; pwrite = 1'b1; psel = 1'b1; penable = 1'b0;
    if (a[3:2] == 2'h0) idle_ok_until = cyc + 8;       // an idle-level change of SCK is legal after a CTRL write
    @(negedge clk); penable = 1'b1;
    @(posedge clk);                                    // access edge: write accepted here
    @(negedge clk); psel = 1'b0; penable = 1'b0; pwrite = 1'b0;
    cov_rw[a[3:2]][1]++;
  endtask

  task automatic apb_rd(input logic [3:0] a, output logic [31:0] d);
    paddr = a; pwrite = 1'b0; psel = 1'b1; penable = 1'b0;
    @(negedge clk); penable = 1'b1;
    @(posedge clk); d = prdata;                        // sampled before the edge
    @(negedge clk); psel = 1'b0; penable = 1'b0;
    if ($isunknown(d)) err($sformatf("X/Z in prdata of register %0d", a[3:2]));
    cov_rw[a[3:2]][0]++;
  endtask

  task automatic idle(input int n);
    psel = 1'b0; penable = 1'b0;
    repeat (n) @(negedge clk);
  endtask

  task automatic banner(input string s);
    $display("[%0t] ---- %s", $time, s);
  endtask

  task automatic model_defaults();
    m_en = 0; m_cpol = 0; m_cpha = 0; m_csnpol = 0; m_hold = 0; m_div = 7;
    slv_rsp.delete(); tx_exp.delete();
  endtask

  task automatic do_reset();
    psel = 1'b0; penable = 1'b0; rst_n = 1'b0;
    repeat (3) @(negedge clk);
    armed = 1'b1;
    model_defaults();
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  task automatic reset_async(input int cycles);
    #2 rst_n = 1'b0;
    repeat (cycles) @(negedge clk);
    model_defaults();
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  // configure (device must be idle)
  task automatic cfg(input bit en, input bit cpol, input bit cpha, input bit csnpol,
                     input bit hold, input int div);
    apb_wr(4'h0, {27'h0, hold, csnpol, cpha, cpol, en});
    m_en = en; m_cpol = cpol; m_cpha = cpha; m_csnpol = csnpol; m_hold = hold;   // pins change at this write
    apb_wr(4'h4, 32'(div));
    m_div = div;
  endtask

  task automatic chk_idle(input bit chk_cs);
    idle(3);
    if (spi_sck !== m_cpol) err($sformatf("SCK idle level %b, expected CPOL=%b", spi_sck, m_cpol));
    if (chk_cs) begin
      if (!m_hold || !m_en) begin
        if (cs_on(spi_csn)) err("CSn still asserted while idle (no CS_HOLD)");
      end
    end
  endtask

  task automatic wait_done();
    logic [31:0] s;
    int c, lim;
    bit first, fin;
    c = 0; first = 1'b1; fin = 1'b0;
    lim = (20 * (m_div + 1) + 60) / 2 + 30;
    while (!fin) begin
      apb_rd(4'hC, s);
      if (first && s[0] !== 1'b1) err("BUSY not set right after the DATA write");
      first = 1'b0;
      if (s[0] === 1'b1 && s[1] === 1'b1) err("RX_VALID set while BUSY");
      if (s[0] === 1'b0) begin
        if (s[1] !== 1'b1) err("RX_VALID not set when the transfer is done");
        fin = 1'b1;
      end else begin
        c = c + 1;
        if (c > lim) begin err("TIMEOUT waiting for BUSY to clear"); fin = 1'b1; end
      end
    end
  endtask

  // one byte; the slave reply must already be queued when CS asserts, so it is
  // pushed here (single transfers) or by the caller (CS_HOLD sequences)
  task automatic xfer_np(input logic [7:0] tx, input logic [7:0] resp);
    logic [31:0] d;
    tx_exp.push_back(tx);
    apb_wr(4'h8, {24'h0, tx});
    wait_done();
    apb_rd(4'h8, d);
    if (d !== {24'h0, resp}) err($sformatf("DATA read %h, slave sent %h (mode %0d%0d, div %0d)", d, resp, m_cpol, m_cpha, m_div));
    for (int b = 0; b < 8; b++) begin
      if (resp[b]) cov_rxb1[b]++; else cov_rxb0[b]++;
    end
    cov_mode[{m_cpol, m_cpha}]++;
    cov_divc[divc(m_div)]++;
  endtask

  task automatic xfer(input logic [7:0] tx, input logic [7:0] resp);
    slv_rsp.push_back(resp);
    xfer_np(tx, resp);
  endtask

  function automatic logic [7:0] rnd8();
    return 8'($urandom);
  endfunction

  // --------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------
  task automatic test_reset_defaults();
    logic [31:0] d;
    banner("S1 reset defaults and idle pins");
    do_reset();
    idle(10);
    apb_rd(4'h0, d); if (d !== 32'h0) err($sformatf("CTRL after reset = %h", d));
    apb_rd(4'h4, d); if (d !== 32'h7) err($sformatf("DIV after reset = %h, expected 7", d));
    apb_rd(4'h8, d); if (d !== 32'h0) err($sformatf("DATA after reset = %h, expected 0", d));
    apb_rd(4'hC, d); if (d !== 32'h0) err($sformatf("STATUS after reset = %h, expected 0", d));
    if (spi_sck !== 1'b0) err("SCK not low after reset");
    if (spi_csn !== 1'b1) err("CSn not inactive (high) after reset");
    idle(100);
  endtask

  task automatic test_registers();
    logic [31:0] d;
    logic [15:0] dv [6];
    banner("S2 register read/write, reserved bits, read-only STATUS");
    for (int v = 0; v < 16; v++) begin
      apb_wr(4'h0, 32'hFFFF_FFE0 | v);               // reserved bits set, CS_HOLD clear
      m_en = v[0]; m_cpol = v[1]; m_cpha = v[2]; m_csnpol = v[3]; m_hold = 1'b0;
      apb_rd(4'h0, d);
      if (d[3:0] !== v[3:0])         err($sformatf("CTRL[3:0] = %h, wrote %h", d[3:0], v[3:0]));
      if ((d & 32'hFFFF_FFE0) !== 0) err($sformatf("CTRL reserved bits read %h", d));
    end
    apb_wr(4'h0, 32'h0);
    m_en = 0; m_cpol = 0; m_cpha = 0; m_csnpol = 0; m_hold = 0;
    dv[0] = 16'h0000; dv[1] = 16'h0001; dv[2] = 16'hFFFF; dv[3] = 16'h1234; dv[4] = 16'h8000; dv[5] = 16'h0007;
    for (int i = 0; i < 6; i++) begin
      apb_wr(4'h4, {16'hA5A5, dv[i]});
      apb_rd(4'h4, d);
      if (d !== {16'h0, dv[i]}) err($sformatf("DIV read %h, wrote %h", d, dv[i]));
    end
    apb_wr(4'h4, 32'h7); m_div = 7;
    apb_wr(4'hC, 32'hFFFF_FFFF);
    apb_rd(4'hC, d); if (d !== 0) err($sformatf("STATUS changed by a write: %h", d));
    apb_rd(4'h8, d); if (d !== 0) err($sformatf("DATA changed by a STATUS write: %h", d));
  endtask

  task automatic test_modes();
    int dv [5];
    logic [7:0] pat [6];
    logic [7:0] rsp;
    banner("S3 all four SPI modes x divisors 0,1,2,3,7, CS polarity, random start phase");
    dv[0] = 0; dv[1] = 1; dv[2] = 2; dv[3] = 3; dv[4] = 7;
    pat[0] = 8'h00; pat[1] = 8'hFF; pat[2] = 8'hA5; pat[3] = 8'h5A; pat[4] = 8'h81; pat[5] = 8'h7E;
    for (int mode = 0; mode < 4; mode++) begin
      for (int i = 0; i < 5; i++) begin
        cfg(1'b1, mode[1], mode[0], (i == 2) ? 1'b1 : 1'b0, 1'b0, dv[i]);
        chk_idle(1'b1);
        if (i == 2) cov_ev[0]++;
        for (int j = 0; j < 6; j++) begin
          idle($urandom_range(0, 3 * (dv[i] + 1)));
          rsp = (j == 0) ? 8'hFF : ((j == 1) ? 8'h00 : rnd8());
          xfer(pat[j], rsp);
          chk_idle(1'b1);
        end
      end
    end
  endtask

  task automatic test_large_div();
    int dv [4];
    banner("S4 large divisors (upper divider byte) in all four modes");
    dv[0] = 15; dv[1] = 100; dv[2] = 256; dv[3] = 1000;
    for (int i = 0; i < 4; i++)
      for (int mode = 0; mode < 4; mode++) begin
        cfg(1'b1, mode[1], mode[0], 1'b0, 1'b0, dv[i]);
        xfer(rnd8(), rnd8());
        if (dv[i] < 256) xfer(rnd8(), rnd8());
      end
  endtask

  task automatic test_status();
    logic [31:0] d, s0, s1;
    logic [7:0] r1, r2;
    banner("S5 STATUS / DATA semantics: BUSY, RX_VALID, DATA held until the next transfer completes");
    cfg(1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 3);
    r1 = 8'h3D; r2 = 8'hC2;
    xfer(8'h11, r1);
    apb_rd(4'h8, d); if (d[7:0] !== r1) err("DATA changed between two reads");
    apb_rd(4'hC, d); if (d[1:0] !== 2'b10) err($sformatf("STATUS after a transfer = %b, expected RX_VALID only", d[1:0]));
    apb_rd(4'hC, d); if (d[1:0] !== 2'b10) err("RX_VALID cleared by a read");
    slv_rsp.push_back(r2); tx_exp.push_back(8'h22);
    apb_wr(4'h8, 32'h22);
    apb_rd(4'hC, s0);
    if (s0[1:0] !== 2'b01) err($sformatf("STATUS during a transfer = %b, expected BUSY only", s0[1:0]));
    apb_rd(4'h8, d); if (d[7:0] !== r1) err("DATA changed while the next transfer is running");
    wait_done();
    apb_rd(4'h8, d); if (d[7:0] !== r2) err($sformatf("DATA %h, expected %h", d[7:0], r2));
    cov_ev[1]++;
  endtask

  task automatic test_cs_hold();
    logic [31:0] d;
    logic [7:0] rs [4];
    banner("S6 CS_HOLD: CSn stays asserted across bytes, released when CS_HOLD is cleared");
    for (int mode = 0; mode < 4; mode++) begin
      cfg(1'b1, mode[1], mode[0], mode[0] ^ mode[1], 1'b0, 3);
      for (int k = 0; k < 4; k++) begin rs[k] = rnd8(); slv_rsp.push_back(rs[k]); end
      apb_wr(4'h0, {27'h0, 1'b1, m_csnpol, m_cpha, m_cpol, m_en});   // CS_HOLD = 1
      m_hold = 1'b1;
      idle(3);
      if (!cs_on(spi_csn)) err("CSn not asserted after CS_HOLD was set");
      for (int k = 0; k < 4; k++) begin
        xfer_np(rnd8(), rs[k]);
        if (!cs_on(spi_csn)) err("CSn released between the bytes of a CS_HOLD transaction");
      end
      apb_wr(4'h0, {27'h0, 1'b0, m_csnpol, m_cpha, m_cpol, m_en});   // CS_HOLD = 0
      m_hold = 1'b0;
      idle(3);
      if (cs_on(spi_csn)) err("CSn still asserted after CS_HOLD was cleared");
      chk_idle(1'b1);
      cov_ev[2]++;
    end
    // CS polarity 1: idle level low, asserted high
    cfg(1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 2);
    idle(3);
    if (spi_csn !== 1'b0) err("CSn idle level is not low with CSn_POL=1");
    xfer(8'hB4, 8'h4B);
    if (spi_csn !== 1'b0) err("CSn not back to its idle level after a CSn_POL=1 transfer");
  endtask

  task automatic test_enable();
    logic [31:0] d, s0;
    banner("S7 EN=0: DATA writes ignored, pins stay idle, EN=1 works again");
    cfg(1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 3);
    xfer(8'h96, 8'h69);
    apb_rd(4'hC, s0);
    cfg(1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 3);
    chk_idle(1'b1);
    apb_wr(4'h8, 32'h5A);                           // must be ignored
    idle(80);
    apb_rd(4'hC, d);
    if (d[0] !== 1'b0) err("BUSY set although EN=0");
    if (d[1] !== s0[1]) err("RX_VALID changed by a write with EN=0");
    chk_idle(1'b1);
    cfg(1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 3);          // CS_HOLD with EN=0 must not assert CSn
    idle(3);
    if (cs_on(spi_csn)) err("CSn asserted with EN=0");
    cfg(1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 3);
    chk_idle(1'b1);
    xfer(8'h0F, 8'hF0);
    cov_ev[3]++;
  endtask

  task automatic test_write_busy();
    logic [31:0] d;
    banner("S8 DATA / CTRL / DIV writes during a transfer must not corrupt it");
    for (int mode = 0; mode < 4; mode++) begin
      cfg(1'b1, mode[1], mode[0], 1'b0, 1'b0, 7);
      slv_rsp.push_back(8'hD1); tx_exp.push_back(8'h6B);
      apb_wr(4'h8, 32'h6B);
      apb_wr(4'h8, 32'hFF);                          // ignored: no second byte, no restart
      apb_wr(4'h4, 32'h0);                           // ignored: timing unchanged
      apb_wr(4'h0, {27'h0, 1'b0, 1'b0, ~m_cpha, ~m_cpol, 1'b1});   // ignored: mode unchanged
      wait_done();
      apb_rd(4'h8, d); if (d[7:0] !== 8'hD1) err($sformatf("DATA %h after writes during the transfer, expected D1", d[7:0]));
      cov_ev[4]++;
      cfg(1'b1, mode[1], mode[0], 1'b0, 1'b0, 7);    // known state again
      chk_idle(1'b1);
    end
  endtask

  task automatic test_divider();
    banner("S9 divider: exact first half period at any start phase; DIV change right after a transfer");
    cfg(1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 7);
    for (int i = 0; i < 24; i++) begin
      idle($urandom_range(0, 24));
      xfer(rnd8(), rnd8());
    end
    cfg(1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 3);
    for (int i = 0; i < 16; i++) begin
      idle($urandom_range(0, 12));
      xfer(rnd8(), rnd8());
    end
    // the free-running counter of a slow divider must not stall a faster one
    cfg(1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 200);
    xfer(rnd8(), rnd8());
    cfg(1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 3);
    xfer(rnd8(), rnd8());
    cfg(1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1000);
    idle(5);
    cfg(1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 2);
    xfer(rnd8(), rnd8());
    cov_ev[5]++;
  endtask

  task automatic test_reset_mid();
    logic [31:0] d;
    banner("S10 reset in the middle of a transfer");
    for (int mode = 0; mode < 4; mode++) begin
      cfg(1'b1, mode[1], mode[0], 1'b0, 1'b0, 7);
      slv_rsp.push_back(8'h9C); tx_exp.push_back(8'h5A);
      apb_wr(4'h8, 32'h5A);
      idle(20 + 13 * mode);
      reset_async(2);
      idle(2);
      if (spi_sck !== 1'b0) err("SCK not low right after reset");
      if (spi_csn !== 1'b1) err("CSn not inactive right after reset");
      apb_rd(4'hC, d); if (d !== 0) err($sformatf("STATUS after reset in a transfer = %h", d));
      apb_rd(4'h0, d); if (d !== 0) err($sformatf("CTRL after reset = %h", d));
      apb_rd(4'h4, d); if (d !== 7) err($sformatf("DIV after reset = %h", d));
      idle(200);                                     // no activity may follow (monitor)
      cfg(1'b1, mode[1], mode[0], 1'b0, 1'b0, 3);
      xfer(rnd8(), rnd8());
      cov_ev[6]++;
    end
  endtask

  task automatic test_backtoback();
    banner("S11 back-to-back transfers with no gap");
    for (int mode = 0; mode < 4; mode++) begin
      cfg(1'b1, mode[1], mode[0], 1'b0, 1'b0, (mode < 2) ? 1 : 3);
      for (int k = 0; k < 10; k++) xfer(rnd8(), rnd8());
    end
  endtask

  task automatic test_random(input int n);
    int r, len;
    logic [7:0] rs [4];
    logic [31:0] d;
    banner("S12 constrained random (configs, bytes, CS_HOLD sequences, writes during busy, resets)");
    cfg(1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 3);
    for (int i = 0; i < n; i++) begin
      r = $urandom_range(0, 99);
      if (r < 15) begin
        cfg(1'b1, 1'($urandom), 1'($urandom), 1'($urandom_range(0, 3) == 0), 1'b0,
            ($urandom_range(0, 9) == 0) ? $urandom_range(20, 60) : $urandom_range(0, 7));
        chk_idle(1'b1);
      end else if (r < 55) begin
        idle($urandom_range(0, 12));
        xfer(rnd8(), rnd8());
      end else if (r < 70) begin
        len = $urandom_range(2, 4);
        for (int k = 0; k < len; k++) begin rs[k] = rnd8(); slv_rsp.push_back(rs[k]); end
        apb_wr(4'h0, {27'h0, 1'b1, m_csnpol, m_cpha, m_cpol, m_en}); m_hold = 1'b1;
        for (int k = 0; k < len; k++) xfer_np(rnd8(), rs[k]);
        apb_wr(4'h0, {27'h0, 1'b0, m_csnpol, m_cpha, m_cpol, m_en}); m_hold = 1'b0;
        chk_idle(1'b1);
      end else if (r < 82) begin
        slv_rsp.push_back(8'h77); tx_exp.push_back(8'h3E);
        apb_wr(4'h8, 32'h3E);
        apb_wr(4'h8, 32'(rnd8()));
        apb_wr(4'h4, 32'($urandom_range(0, 9)));
        wait_done();
        apb_rd(4'h8, d); if (d[7:0] !== 8'h77) err($sformatf("random: DATA %h after writes during a transfer", d[7:0]));
        cfg(1'b1, m_cpol, m_cpha, m_csnpol, 1'b0, m_div);
      end else if (r < 90) begin
        cfg(1'b0, m_cpol, m_cpha, m_csnpol, 1'b0, m_div);
        apb_wr(4'h8, 32'(rnd8()));
        idle(20);
        chk_idle(1'b1);
        cfg(1'b1, m_cpol, m_cpha, m_csnpol, 1'b0, m_div);
      end else if (r < 94) begin
        slv_rsp.push_back(rnd8()); tx_exp.push_back(rnd8());
        apb_wr(4'h8, 32'(rnd8()));
        idle($urandom_range(3, 30));
        reset_async(2);
        idle(4);
        cfg(1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 3);
      end else begin
        apb_rd(4'hC, d);
        apb_rd(4'h8, d);
      end
    end
  endtask

  // --------------------------------------------------------------------------
  task automatic report_cov();
    for (int m = 0; m < 4; m++) if (cov_mode[m] == 0) err($sformatf("COVERAGE: SPI mode %0d never exercised", m));
    for (int i = 0; i < 6; i++) if (cov_divc[i] == 0) err($sformatf("COVERAGE: divisor class %0d (0,1,2,3,4..99,>=100) never exercised", i));
    for (int b = 0; b < 8; b++) begin
      if (cov_txb1[b] == 0 || cov_txb0[b] == 0) err($sformatf("COVERAGE: TX bit %0d never seen as 0 and 1", b));
      if (cov_rxb1[b] == 0 || cov_rxb0[b] == 0) err($sformatf("COVERAGE: RX bit %0d never seen as 0 and 1", b));
    end
    for (int a = 0; a < 4; a++) for (int w = 0; w < 2; w++)
      if (cov_rw[a][w] == 0) err($sformatf("COVERAGE: APB register %0d %s never accessed", a, w ? "write" : "read"));
    if (cov_ev[0] == 0) err("COVERAGE: CSn_POL=1 never exercised");
    if (cov_ev[1] == 0) err("COVERAGE: STATUS/DATA semantics test never ran");
    if (cov_ev[2] == 0) err("COVERAGE: CS_HOLD sequence never exercised");
    if (cov_ev[3] == 0) err("COVERAGE: EN=0 never exercised");
    if (cov_ev[4] == 0) err("COVERAGE: writes during a transfer never exercised");
    if (cov_ev[5] == 0) err("COVERAGE: divider change test never ran");
    if (cov_ev[6] == 0) err("COVERAGE: reset during a transfer never exercised");
    $display(" coverage: modes 4/4 divisor classes 6/6 tx/rx bits 8/8 registers 8/8 bytes checked=%0d",
             cov_mode[0] + cov_mode[1] + cov_mode[2] + cov_mode[3]);
  endtask

  initial begin
    @(negedge clk);
    do_reset();

    test_reset_defaults();
    test_registers();
    test_modes();
    test_large_div();
    test_status();
    test_cs_hold();
    test_enable();
    test_write_busy();
    test_divider();
    test_backtoback();
    test_reset_mid();
    test_random(400);

    idle(10);
    $display("==================================================");
    report_cov();
    $display(" contract errors = %0d", errors);
    if (errors != 0) $display(" *** RESULT: FAIL ***");
    else             $display(" *** RESULT: PASS ***");
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

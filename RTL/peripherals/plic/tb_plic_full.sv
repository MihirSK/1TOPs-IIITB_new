// ============================================================================
//  tb_plic_full.sv  -  Functional verification testbench for plic.sv
//  Plain SystemVerilog (no covergroups/SVA, no dut.* references), xsim/Verilator.
//
//  Top module tb_plic_full runs two configurations together:
//      NSOURCES=64, NCONTEXTS=2 (the SoC)     and     NSOURCES=32, NCONTEXTS=3
//  Reference model (independent of the RTL): priority, threshold, enable, pending
//  and "in flight" bits per source, updated at every clock edge from the bus and
//  irq_sources. Every cycle (when the sources have been stable for 7 cycles, the
//  synchroniser latency is not specified):
//     * eip[c] == "some pending, enabled source has priority > threshold[c]"
//     * an accepted request is answered with ack one cycle later, never otherwise
//     * read data == model (priority, pending, enable, threshold, claim; unmapped
//       and out-of-range addresses read 0, never stale data)
//  Claim = highest priority, ties to the lowest id, clears pending; complete =
//  write of the id (ignored if the source is not enabled for the context).
//  Bus operations wait until the sources are stable, so the model does not need
//  the synchroniser latency.
//  Masters: PULSE (req for one edge, contract) and HOLD (req held until ack, O1).
//  Gateway semantics that are a design decision (level gateway: a source that is
//  still asserted when it is completed becomes pending again, pulses while in
//  flight are ignored) are tested in O2 and reported as OPEN ITEMS.
//
//  Result lines:  RESULT: PASS | PASS WITH OPEN ITEMS (n) | FAIL
//  -define STRICT_OPEN makes open items fail.
// ============================================================================
`timescale 1ns/1ps

module tb_plic_core #(
    parameter int    NSOURCES  = 64,
    parameter int    NCONTEXTS = 2,
    parameter string TAG       = "S64C2"
)(
    output bit done,
    output int nerr,
    output int nopen
);

  localparam int NW = (NSOURCES + 31) / 32;

  logic        clk   = 1'b0;
  logic        rst_n = 1'b0;
  always #5 clk = ~clk;

  logic [NSOURCES-1:0]  irq_sources = '0;
  logic [27:0]          addr  = '0;
  logic [31:0]          wdata = '0;
  logic                 req   = 1'b0;
  logic                 we    = 1'b0;
  logic [31:0]          rdata;
  logic                 ack;
  logic [NCONTEXTS-1:0] eip;

  plic #(.NSOURCES(NSOURCES), .NCONTEXTS(NCONTEXTS)) dut (.*);

  // --------------------------------------------------------------------------
  bit open_mode = 1'b0;
  function automatic void err(input string m);
    if (open_mode) begin
`ifdef STRICT_OPEN
      nerr++;
      if (nerr <= 40) $display("[%0t] %s ERROR (open item, strict): %s", $time, TAG, m);
`else
      nopen++;
      if (nopen <= 8) $display("[%0t] %s OPEN-ITEM: %s", $time, TAG, m);
`endif
    end else begin
      nerr++;
      if (nerr <= 40) $display("[%0t] %s ERROR: %s", $time, TAG, m);
    end
  endfunction

  bit armed = 1'b0;
  int cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;
  int last_src_chg = -100;

  // coverage counters (a hole fails the run)
  int cov_pend  [NSOURCES];
  int cov_claim [NSOURCES];
  int cov_pri   [8];
  int cov_thr   [8];
  int cov_enw   [NCONTEXTS][NW];
  int cov_eip   [NCONTEXTS][2];       // eip seen 0 and 1 (quiet)
  int cov_ev    [16];

  // --------------------------------------------------------------------------
  // Reference model
  // --------------------------------------------------------------------------
  logic [2:0]          m_pty  [NSOURCES];
  logic [2:0]          m_thr  [NCONTEXTS];
  bit                  m_en   [NCONTEXTS][NSOURCES];
  bit                  m_pend [NSOURCES];
  bit                  m_infl [NSOURCES];
  bit                  pset   [NSOURCES];

  function automatic int best(input int c);
    int id;
    logic [2:0] bp;
    id = 0; bp = 3'h0;
    for (int s = NSOURCES - 1; s >= 1; s--)
      if (m_pend[s] && m_en[c][s] && (m_pty[s] > m_thr[c]) && (m_pty[s] >= bp)) begin id = s; bp = m_pty[s]; end
    return id;
  endfunction

  // 0 unmapped, 1 priority, 2 pending, 3 enable, 4 threshold, 5 claim/complete
  function automatic int dk_kind(input logic [27:0] a);
    int ctx, w, s;
    if (a[27:12] == 16'h0000) begin
      s = int'(a[11:2]);
      return (s >= 1 && s < NSOURCES) ? 1 : 0;
    end else if (a[27:7] == 21'h20) begin
      w = int'(a[6:2]);
      return (w < NW) ? 2 : 0;
    end else if (a[27:12] >= 16'h0002 && a[27:12] < 16'h0200) begin
      ctx = int'((a - 28'h2000) >> 7); w = int'(a[6:2]);
      return (ctx < NCONTEXTS && w < NW) ? 3 : 0;
    end else if (a[27:12] >= 16'h0200) begin
      ctx = int'(a[27:12] - 16'h0200);
      if (ctx >= NCONTEXTS) return 0;
      if (a[11:0] == 12'h000) return 4;
      if (a[11:0] == 12'h004) return 5;
      return 0;
    end
    return 0;
  endfunction

  function automatic int dk_c(input logic [27:0] a);
    if (a[27:12] >= 16'h0200) return int'(a[27:12] - 16'h0200);
    return int'((a - 28'h2000) >> 7);
  endfunction

  function automatic int dk_i(input logic [27:0] a);
    if (a[27:12] == 16'h0000) return int'(a[11:2]);
    return int'(a[6:2]);
  endfunction

  bit          pend = 1'b0, pend_rd = 1'b0;
  logic [31:0] pend_exp = '0;
  int          kind, kc, ki, claimed_id;
  bit          acc_m, claim_fire, quiet, mvec;
  logic [31:0] ex32;
  int          idv;

  always @(posedge clk) begin
    if (!rst_n) begin
      for (int s = 0; s < NSOURCES; s++) begin m_pty[s] = 3'h0; m_pend[s] = 1'b0; m_infl[s] = 1'b0; end
      for (int c = 0; c < NCONTEXTS; c++) begin
        m_thr[c] = 3'h0;
        for (int s = 0; s < NSOURCES; s++) m_en[c][s] = 1'b0;
      end
      pend = 1'b0;
      if (armed && ack !== 1'b0) err("ack asserted while in reset");
    end else if (armed) begin
      quiet = (cyc - last_src_chg) > 6;
      for (int c = 0; c < NCONTEXTS; c++) begin
        if ($isunknown(eip[c])) err("X/Z on eip");
        else if (quiet) begin
          mvec = (best(c) != 0);
          if (eip[c] !== mvec) err($sformatf("eip[%0d]=%b, model %b", c, eip[c], mvec));
          cov_eip[c][eip[c] ? 1 : 0]++;
        end
      end
      if ($isunknown(ack)) err("X/Z on ack");
      if (pend) begin
        if (ack !== 1'b1) err("no ack one cycle after an accepted request");
        else if (pend_rd && rdata !== pend_exp)
          err($sformatf("read data %h, expected %h", rdata, pend_exp));
      end else if (ack === 1'b1) err("ack without an accepted request (or ack held for two cycles)");

      // apply this clock edge to the model
      acc_m = (req === 1'b1) && (ack !== 1'b1);
      claim_fire = 1'b0; claimed_id = 0;
      for (int s = 0; s < NSOURCES; s++) pset[s] = (s >= 1) && irq_sources[s] && !m_infl[s];
      pend = acc_m; pend_rd = 1'b0;
      if (acc_m) begin
        kind = dk_kind(addr); kc = dk_c(addr); ki = dk_i(addr);
        ex32 = 32'h0;
        case (kind)
          1: begin
            ex32 = {29'h0, m_pty[ki]};
            if (we) begin m_pty[ki] = wdata[2:0]; cov_pri[wdata[2:0]]++; end
          end
          2: begin
            for (int i = 0; i < 32; i++) if (ki * 32 + i < NSOURCES) ex32[i] = m_pend[ki * 32 + i];
          end
          3: begin
            for (int i = 0; i < 32; i++) if (ki * 32 + i < NSOURCES) ex32[i] = m_en[kc][ki * 32 + i];
            if (we) begin
              for (int i = 0; i < 32; i++)
                if (ki * 32 + i < NSOURCES) m_en[kc][ki * 32 + i] = (ki * 32 + i >= 1) ? wdata[i] : 1'b0;
              cov_enw[kc][ki]++;
            end
          end
          4: begin
            ex32 = {29'h0, m_thr[kc]};
            if (we) begin m_thr[kc] = wdata[2:0]; cov_thr[wdata[2:0]]++; end
          end
          5: begin
            if (!we) begin
              idv = best(kc);
              ex32 = 32'(idv);
              if (idv != 0) begin
                m_pend[idv] = 1'b0; m_infl[idv] = 1'b1;
                claim_fire = 1'b1; claimed_id = idv;
                cov_claim[idv]++;
              end
            end else begin
              if (wdata >= 32'd1 && wdata < 32'(NSOURCES)) begin
                if (m_en[kc][int'(wdata)]) begin m_infl[int'(wdata)] = 1'b0; cov_ev[0]++; end
                else cov_ev[1]++;                       // complete of a source that is not enabled
              end
            end
          end
          default: ex32 = 32'h0;
        endcase
        pend_exp = ex32;
        pend_rd  = !we;
        if (kind == 0 && !we) cov_ev[2]++;
      end
      for (int s = 1; s < NSOURCES; s++)
        if (pset[s] && !(claim_fire && claimed_id == s)) begin
          if (!m_pend[s]) cov_pend[s]++;
          m_pend[s] = 1'b1;
        end
    end
  end

  // --------------------------------------------------------------------------
  // Drivers
  // --------------------------------------------------------------------------
  bit hold_mode = 1'b0;

  task automatic wait_quiet();
    while ((cyc - last_src_chg) <= 6) begin req = 1'b0; @(negedge clk); end
  endtask

  task automatic src_set(input logic [NSOURCES-1:0] v);
    irq_sources = v;
    last_src_chg = cyc;
  endtask

  task automatic src_pulse(input int s, input int len);
    logic [NSOURCES-1:0] m;
    m = '0; m[s] = 1'b1;
    src_set(irq_sources | m);
    repeat (len) @(negedge clk);
    src_set(irq_sources & ~m);
  endtask

  task automatic bus_op(input logic [27:0] a, input bit w, input logic [31:0] d, output logic [31:0] rd);
    wait_quiet();
    addr = a; we = w; wdata = d; req = 1'b1;
    if (!hold_mode) begin
      @(negedge clk);
      rd = rdata; req = 1'b0;
      @(negedge clk);
    end else begin
      @(negedge clk);
      @(posedge clk);
      rd = rdata;
      #1 req = 1'b0;
      @(negedge clk);
    end
  endtask

  task automatic idle(input int n);
    req = 1'b0;
    repeat (n) @(negedge clk);
  endtask

  task automatic banner(input string s);
    $display("[%0t] %s ---- %s", $time, TAG, s);
  endtask

  task automatic do_reset();
    req = 1'b0; rst_n = 1'b0;
    repeat (3) @(negedge clk);
    armed = 1'b1;
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  // address helpers
  function automatic logic [27:0] a_pri (input int s);            return 28'(4 * s); endfunction
  function automatic logic [27:0] a_pend(input int w);            return 28'(28'h1000 + 4 * w); endfunction
  function automatic logic [27:0] a_en  (input int c, input int w); return 28'(28'h2000 + 28'h80 * c + 4 * w); endfunction
  function automatic logic [27:0] a_thr (input int c);            return 28'(28'h200000 + 28'h1000 * c); endfunction
  function automatic logic [27:0] a_clm (input int c);            return 28'(28'h200004 + 28'h1000 * c); endfunction

  task automatic wr(input logic [27:0] a, input logic [31:0] d);
    logic [31:0] x;
    bus_op(a, 1'b1, d, x);
  endtask
  task automatic rd(input logic [27:0] a, output logic [31:0] v);
    bus_op(a, 1'b0, 32'h0, v);
  endtask

  // enable one source for a context, set its priority, threshold 0
  task automatic setup_src(input int c, input int s, input int p);
    logic [31:0] v;
    rd(a_en(c, s / 32), v);
    wr(a_en(c, s / 32), v | (32'h1 << (s % 32)));
    wr(a_pri(s), 32'(p));
  endtask

  task automatic clear_all();
    for (int c = 0; c < NCONTEXTS; c++) begin
      for (int w = 0; w < NW; w++) wr(a_en(c, w), 32'h0);
      wr(a_thr(c), 32'h0);
    end
    for (int s = 1; s < NSOURCES; s++) wr(a_pri(s), 32'h0);
  endtask

  function automatic logic [31:0] en_mask(input int w);
    logic [31:0] m;
    m = 32'h0;
    for (int i = 0; i < 32; i++) if (w * 32 + i >= 1 && w * 32 + i < NSOURCES) m[i] = 1'b1;
    return m;
  endfunction

  function automatic logic [31:0] rnd32(); return $urandom; endfunction

  // --------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------
  task automatic test_reset_defaults();
    logic [31:0] v;
    banner("P1 reset defaults: every register reads 0, eip low");
    do_reset();
    idle(4);
    if (eip !== '0) err("eip not 0 after reset");
    for (int s = 0; s < NSOURCES; s++) begin rd(a_pri(s), v); if (v !== 0) err($sformatf("priority[%0d] after reset = %h", s, v)); end
    for (int w = 0; w < NW; w++) begin rd(a_pend(w), v); if (v !== 0) err($sformatf("pending[%0d] after reset = %h", w, v)); end
    for (int c = 0; c < NCONTEXTS; c++) begin
      for (int w = 0; w < NW; w++) begin rd(a_en(c, w), v); if (v !== 0) err($sformatf("enable[%0d][%0d] after reset = %h", c, w, v)); end
      rd(a_thr(c), v); if (v !== 0) err($sformatf("threshold[%0d] after reset = %h", c, v));
      rd(a_clm(c), v); if (v !== 0) err($sformatf("claim[%0d] after reset = %h", c, v));
    end
  endtask

  task automatic test_priority();
    logic [31:0] v;
    banner("P2 priority registers: 3 bits, per source, source 0 and out-of-range sources do not exist");
    for (int s = 1; s < NSOURCES; s++) begin
      for (int p = 0; p < 8; p++) begin
        wr(a_pri(s), 32'(p));
        rd(a_pri(s), v); if (v !== 32'(p)) err($sformatf("priority[%0d] read %h, wrote %0d", s, v, p));
      end
      wr(a_pri(s), 32'hFFFF_FFFF);
      rd(a_pri(s), v); if (v !== 32'h7) err($sformatf("priority[%0d] read %h after writing FFFFFFFF, expected 7", s, v));
    end
    for (int s = 1; s < NSOURCES; s++) wr(a_pri(s), 32'((s * 3) % 8));
    for (int s = 1; s < NSOURCES; s++) begin
      rd(a_pri(s), v); if (v !== 32'((s * 3) % 8)) err($sformatf("priority[%0d] = %h, overwritten by another source", s, v));
    end
    wr(a_pri(0), 32'h7);                                  // source 0 does not exist
    rd(a_pri(0), v); if (v !== 0) err($sformatf("priority[0] read %h, expected 0", v));
    rd(a_pri(1), v); if (v !== 32'((1 * 3) % 8)) err("priority[1] changed by a write to priority[0]");
    wr(a_pri(NSOURCES), 32'h7);                           // beyond the last source
    rd(a_pri(NSOURCES), v); if (v !== 0) err($sformatf("priority[%0d] (out of range) read %h", NSOURCES, v));
    wr(a_pri(1023), 32'h7);
    rd(a_pri(1023), v); if (v !== 0) err($sformatf("priority[1023] read %h", v));
    rd(a_pri(NSOURCES - 1), v); if (v !== 32'(((NSOURCES - 1) * 3) % 8)) err("last priority changed by an out-of-range write");
    clear_all();
    cov_ev[3]++;
  endtask

  task automatic test_enable();
    logic [31:0] v;
    banner("P3 enable registers: per context and word, bit 0 / bits beyond NSOURCES read 0, contexts independent");
    for (int c = 0; c < NCONTEXTS; c++)
      for (int w = 0; w < NW; w++) begin
        wr(a_en(c, w), 32'hFFFF_FFFF);
        rd(a_en(c, w), v); if (v !== en_mask(w)) err($sformatf("enable[%0d][%0d] = %h after writing FFFFFFFF, expected %h", c, w, v, en_mask(w)));
        wr(a_en(c, w), 32'h0);
        for (int b = 0; b < 32; b++) begin
          wr(a_en(c, w), 32'h1 << b);
          rd(a_en(c, w), v); if (v !== (en_mask(w) & (32'h1 << b))) err($sformatf("enable[%0d][%0d] walking bit %0d read %h", c, w, b, v));
        end
        wr(a_en(c, w), 32'h0);
      end
    // independence
    for (int c = 0; c < NCONTEXTS; c++)
      for (int w = 0; w < NW; w++) wr(a_en(c, w), rnd32());
    for (int c = 0; c < NCONTEXTS; c++)
      for (int w = 0; w < NW; w++) begin
        logic [31:0] r;
        r = rnd32();
        wr(a_en(c, w), r);
        rd(a_en(c, w), v); if (v !== (r & en_mask(w))) err($sformatf("enable[%0d][%0d] read %h, wrote %h", c, w, v, r));
      end
    // out of range context / word
    wr(a_en(NCONTEXTS, 0), 32'hFFFF_FFFF);
    rd(a_en(NCONTEXTS, 0), v); if (v !== 0) err($sformatf("enable of context %0d (out of range) read %h", NCONTEXTS, v));
    wr(a_en(0, NW), 32'hFFFF_FFFF);
    rd(a_en(0, NW), v); if (v !== 0) err($sformatf("enable word %0d (out of range) read %h", NW, v));
    for (int c = 0; c < NCONTEXTS; c++)
      for (int w = 0; w < NW; w++) wr(a_en(c, w), 32'h0);
    cov_ev[4]++;
  endtask

  task automatic test_threshold();
    logic [31:0] v;
    banner("P4 threshold registers: 3 bits per context, independent");
    for (int c = 0; c < NCONTEXTS; c++) begin
      for (int p = 0; p < 8; p++) begin
        wr(a_thr(c), 32'(p));
        rd(a_thr(c), v); if (v !== 32'(p)) err($sformatf("threshold[%0d] read %h, wrote %0d", c, v, p));
      end
      wr(a_thr(c), 32'hFFFF_FFFF);
      rd(a_thr(c), v); if (v !== 32'h7) err($sformatf("threshold[%0d] read %h after FFFFFFFF", c, v));
    end
    for (int c = 0; c < NCONTEXTS; c++) wr(a_thr(c), 32'(c + 1));
    for (int c = 0; c < NCONTEXTS; c++) begin rd(a_thr(c), v); if (v !== 32'(c + 1)) err($sformatf("threshold[%0d] = %h, overwritten", c, v)); end
    wr(a_thr(NCONTEXTS), 32'h7);
    rd(a_thr(NCONTEXTS), v); if (v !== 0) err($sformatf("threshold of context %0d (out of range) read %h", NCONTEXTS, v));
    for (int c = 0; c < NCONTEXTS; c++) wr(a_thr(c), 32'h0);
    cov_ev[5]++;
  endtask

  task automatic test_pending();
    logic [31:0] v, w0, w1;
    banner("P5 pending: set by a pulse on every source (also disabled / priority 0), read-only, source 0 ignored");
    for (int s = 1; s < NSOURCES; s++) begin
      do_reset();
      src_pulse(s, 2);
      for (int w = 0; w < NW; w++) begin
        rd(a_pend(w), v);
        if (v !== ((w == s / 32) ? (32'h1 << (s % 32)) : 32'h0)) err($sformatf("pending[%0d] = %h after a pulse on source %0d", w, v, s));
      end
      wr(a_pend(s / 32), 32'h0);                                         // read-only
      rd(a_pend(s / 32), v); if (v !== (32'h1 << (s % 32))) err("pending register was changed by a bus write");
      if (eip !== '0) err("eip set for a disabled source");
    end
    do_reset();
    src_pulse(0, 3);                                                     // source 0 does not exist
    rd(a_pend(0), v); if (v !== 0) err($sformatf("pending after a pulse on source 0 = %h", v));
    setup_src(0, 1, 7);
    if (eip !== '0) err("eip set by source 0");
    // several sources at once
    do_reset();
    begin
      logic [NSOURCES-1:0] m;
      m = '0;
      for (int s = 1; s < NSOURCES; s += 5) m[s] = 1'b1;
      src_set(m); idle(3); src_set('0);
      for (int w = 0; w < NW; w++) begin
        rd(a_pend(w), v);
        w1 = '0;
        for (int i = 0; i < 32; i++) if (w * 32 + i < NSOURCES) w1[i] = m[w * 32 + i];
        if (v !== w1) err($sformatf("pending[%0d] = %h, expected %h for a group of sources", w, v, w1));
      end
    end
    cov_ev[6]++;
  endtask

  task automatic test_eip();
    logic [31:0] v;
    int s1, s2;
    banner("P6 eip: pending & enabled & priority > threshold, per context; priority 0 never fires");
    s1 = 5; s2 = NSOURCES - 2;
    for (int c = 0; c < NCONTEXTS; c++) begin
      do_reset();
      setup_src(c, s1, 3);
      src_pulse(s1, 2); idle(8);
      if (eip[c] !== 1'b1) err($sformatf("eip[%0d] not set: pending, enabled, priority 3 > threshold 0", c));
      for (int k = 0; k < NCONTEXTS; k++) if (k != c && eip[k] !== 1'b0) err($sformatf("eip[%0d] set by a source enabled only for context %0d", k, c));
      wr(a_thr(c), 32'h3); idle(1);
      if (eip[c] !== 1'b0) err("eip still set with priority == threshold");
      wr(a_thr(c), 32'h2); idle(1);
      if (eip[c] !== 1'b1) err("eip not set with priority > threshold");
      wr(a_pri(s1), 32'h0); idle(1);
      if (eip[c] !== 1'b0) err("eip set for priority 0");
      wr(a_pri(s1), 32'h7); wr(a_thr(c), 32'h7); idle(1);
      if (eip[c] !== 1'b0) err("eip set with threshold 7");
      wr(a_thr(c), 32'h6); idle(1);
      if (eip[c] !== 1'b1) err("eip not set with priority 7 > threshold 6");
      wr(a_en(c, s1 / 32), 32'h0); idle(1);
      if (eip[c] !== 1'b0) err("eip still set after the source was disabled");
      wr(a_en(c, s1 / 32), 32'h1 << (s1 % 32)); idle(1);
      if (eip[c] !== 1'b1) err("eip not set after the source was enabled again");
      // a second, high-numbered source with a lower priority keeps eip up when the first is claimed
      setup_src(c, s2, 2);
      wr(a_thr(c), 32'h0);
      src_pulse(s2, 2); idle(8);
      rd(a_clm(c), v); if (v !== 32'(s1)) err($sformatf("claim returned %0d, expected %0d (priority 7)", v, s1));
      if (eip[c] !== 1'b1) err("eip dropped while a second source is pending");
      rd(a_clm(c), v); if (v !== 32'(s2)) err($sformatf("second claim returned %0d, expected %0d", v, s2));
      if (eip[c] !== 1'b0) err("eip still set with nothing pending");
      cov_ev[7]++;
    end
    // two contexts, different thresholds on the same source
    if (NCONTEXTS >= 2) begin
      do_reset();
      setup_src(0, 9, 4); setup_src(1, 9, 4);
      wr(a_thr(0), 32'h1); wr(a_thr(1), 32'h4);
      src_pulse(9, 2); idle(8);
      if (eip[0] !== 1'b1) err("context 0 (threshold 1) should see the interrupt");
      if (eip[1] !== 1'b0) err("context 1 (threshold 4) should not see priority 4");
      cov_ev[8]++;
    end
  endtask

  task automatic test_claim_order();
    logic [31:0] v;
    int ids [4];
    banner("P7 claim: highest priority first, ties to the lowest id, clears pending, empty claim returns 0");
    do_reset();
    ids[0] = 3; ids[1] = 9; ids[2] = 17; ids[3] = NSOURCES - 1;
    setup_src(0, ids[0], 2);
    setup_src(0, ids[1], 1);
    setup_src(0, ids[2], 6);
    setup_src(0, ids[3], 6);
    rd(a_clm(0), v); if (v !== 0) err($sformatf("claim with nothing pending returned %0d", v));
    for (int i = 0; i < 4; i++) src_pulse(ids[i], 2);
    idle(8);
    rd(a_clm(0), v); if (v !== 32'(ids[2])) err($sformatf("claim 1 returned %0d, expected %0d (priority 6, lowest id)", v, ids[2]));
    rd(a_pend(ids[2] / 32), v); if (v[ids[2] % 32] !== 1'b0) err("pending not cleared by the claim");
    wr(a_clm(0), 32'(ids[2]));                                           // complete
    rd(a_clm(0), v); if (v !== 32'(ids[3])) err($sformatf("claim 2 returned %0d, expected %0d", v, ids[3]));
    wr(a_clm(0), 32'(ids[3]));
    rd(a_clm(0), v); if (v !== 32'(ids[0])) err($sformatf("claim 3 returned %0d, expected %0d (priority 2)", v, ids[0]));
    wr(a_clm(0), 32'(ids[0]));
    rd(a_clm(0), v); if (v !== 32'(ids[1])) err($sformatf("claim 4 returned %0d, expected %0d (priority 1)", v, ids[1]));
    wr(a_clm(0), 32'(ids[1]));
    rd(a_clm(0), v); if (v !== 0) err($sformatf("claim 5 returned %0d, expected 0", v));
    if (eip[0] !== 1'b0) err("eip still set after all claims");
    // priority beats id: low id with higher priority wins over a higher id
    do_reset();
    setup_src(0, 2, 5); setup_src(0, 30, 3);
    src_pulse(30, 2); src_pulse(2, 2); idle(8);
    rd(a_clm(0), v); if (v !== 2) err($sformatf("claim returned %0d, expected 2", v));
    rd(a_clm(0), v); if (v !== 30) err($sformatf("claim returned %0d, expected 30", v));
    // a source enabled for two contexts is claimed once
    if (NCONTEXTS >= 2) begin
      do_reset();
      setup_src(0, 20, 4); setup_src(1, 20, 4);
      src_pulse(20, 2); idle(8);
      if (eip[0] !== 1'b1 || eip[1] !== 1'b1) err("both contexts should see the interrupt");
      rd(a_clm(1), v); if (v !== 20) err($sformatf("context 1 claim returned %0d, expected 20", v));
      if (eip[0] !== 1'b0) err("context 0 still sees an interrupt claimed by context 1");
      rd(a_clm(0), v); if (v !== 0) err($sformatf("context 0 claim returned %0d, expected 0", v));
    end
    // every source can be claimed
    do_reset();
    for (int s = 1; s < NSOURCES; s++) begin
      setup_src(0, s, 1 + (s % 7));
    end
    for (int s = 1; s < NSOURCES; s++) begin
      src_pulse(s, 2); idle(8);
      rd(a_clm(0), v); if (v !== 32'(s)) err($sformatf("claim returned %0d, expected %0d", v, s));
      wr(a_clm(0), 32'(s));
    end
    cov_ev[9]++;
  endtask

  task automatic test_unmapped();
    logic [31:0] v;
    logic [27:0] offs [14];
    banner("P8 unmapped / reserved / out-of-range addresses read 0 (never stale data), writes ignored");
    do_reset();
    wr(a_pri(1), 32'h7);
    offs[0] = 28'h1080;                    offs[1] = 28'h1FFC;                     offs[2] = a_pend(NW);
    offs[3] = a_en(NCONTEXTS, 0);          offs[4] = a_en(0, NW);                  offs[5] = a_thr(NCONTEXTS);
    offs[6] = a_thr(0) + 28'h8;            offs[7] = a_thr(0) + 28'hFFC;           offs[8] = a_pri(NSOURCES);
    offs[9] = a_pri(1023);                 offs[10] = 28'hFFF_FFFC;               offs[11] = 28'h0400000;
    offs[12] = 28'h03F_FFFC;               offs[13] = a_clm(NCONTEXTS);
    for (int i = 0; i < 14; i++) begin
      rd(a_pri(1), v);                                    // makes stale data visible
      rd(offs[i], v);
      if (v !== 32'h0) err($sformatf("read of unmapped address %h returned %h", offs[i], v));
      wr(offs[i], 32'hFFFF_FFFF);
    end
    rd(a_pri(1), v); if (v !== 32'h7) err("priority[1] changed by a write to an unmapped address");
    rd(a_thr(0), v); if (v !== 32'h0) err("threshold changed by a write to an unmapped address");
    cov_ev[10]++;
  endtask

  task automatic test_reset_state();
    logic [31:0] v;
    banner("P9 reset with configured and pending state");
    do_reset();
    for (int c = 0; c < NCONTEXTS; c++) begin setup_src(c, 4, 5); wr(a_thr(c), 32'h2); end
    src_pulse(4, 2); idle(8);
    if (eip[0] !== 1'b1) err("eip not set before the reset");
    #3 rst_n = 1'b0;
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    idle(4);
    if (eip !== '0) err("eip not cleared by reset");
    for (int c = 0; c < NCONTEXTS; c++) begin
      rd(a_thr(c), v); if (v !== 0) err("threshold not cleared by reset");
      rd(a_en(c, 0), v); if (v !== 0) err("enable not cleared by reset");
    end
    rd(a_pend(0), v); if (v !== 0) err("pending not cleared by reset");
    rd(a_pri(4), v); if (v !== 0) err("priority not cleared by reset");
    // in-flight state is cleared too: a source that was claimed before the reset pends again
    for (int c = 0; c < NCONTEXTS; c++) setup_src(c, 6, 3);
    src_pulse(6, 2); idle(8);
    rd(a_clm(0), v); if (v !== 6) err("claim before the second reset");
    #3 rst_n = 1'b0;
    repeat (2) @(negedge clk);
    rst_n = 1'b1; @(negedge clk);
    setup_src(0, 6, 3);
    src_pulse(6, 2); idle(8);
    rd(a_clm(0), v); if (v !== 6) err("source 6 did not pend again after a reset (in-flight state kept?)");
    cov_ev[11]++;
  endtask

  task automatic test_random(input int n);
    logic [31:0] v, r;
    int rr, s, c, w, id;
    int claimed [$];
    int cc [$];
    banner("P10 constrained random (registers, pulses, level sources, claim/complete, unmapped)");
    do_reset();
    for (int i = 0; i < n; i++) begin
      rr = $urandom_range(0, 99);
      s = $urandom_range(1, NSOURCES - 1);
      c = $urandom_range(0, NCONTEXTS - 1);
      w = $urandom_range(0, NW - 1);
      if (rr < 12) begin
        wr(a_pri(s), rnd32());
      end else if (rr < 20) begin
        wr(a_en(c, w), rnd32());
      end else if (rr < 27) begin
        wr(a_thr(c), rnd32());
      end else if (rr < 45) begin
        if (!m_infl[s]) begin
          setup_src(c, s, $urandom_range(0, 7));
          src_pulse(s, $urandom_range(1, 3));
        end
      end else if (rr < 52) begin
        // a level source: held through the claim, dropped before the complete
        if (!m_infl[s] && !m_pend[s]) begin
          setup_src(c, s, $urandom_range(1, 7));
          src_set(irq_sources | (NSOURCES'(1) << s)); idle(9);
          rd(a_clm(c), v);
          src_set(irq_sources & ~(NSOURCES'(1) << s)); idle(1);
          if (v != 0) begin claimed.push_back(int'(v)); cc.push_back(c); end
        end
      end else if (rr < 72) begin
        rd(a_clm(c), v);
        if (v != 0) begin claimed.push_back(int'(v)); cc.push_back(c); end
      end else if (rr < 84) begin
        if (claimed.size() > 0) begin
          id = claimed.pop_front(); s = cc.pop_front();
          if (irq_sources[id] === 1'b0) wr(a_clm(s), 32'(id));
        end
      end else if (rr < 90) begin
        rd(a_pend(w), v);
        rd(a_en(c, w), v);
        rd(a_thr(c), v);
      end else if (rr < 95) begin
        rd(a_pri(s), v);
        rd(28'($urandom) & 28'hFFF_FFFC, v);
      end else begin
        idle($urandom_range(1, 6));
      end
    end
    src_set('0); idle(10);
    cov_ev[12]++;
  endtask

  // ------------------------------------------------------------------------
  // OPEN ITEM O1: a master that holds req until it sees ack.  A claim read has a
  // side effect, so a second execution silently swallows another interrupt.
  // ------------------------------------------------------------------------
  task automatic test_hold_open();
    logic [31:0] v;
    banner("O1 OPEN ITEM: master holds req until ack (a claim must be executed once)");
    do_reset();
    open_mode = 1'b1; hold_mode = 1'b1;
    setup_src(0, 7, 4); setup_src(0, 8, 3); setup_src(0, 9, 2);
    src_pulse(7, 2); src_pulse(8, 2); src_pulse(9, 2); idle(8);
    rd(a_clm(0), v); if (v !== 7) err($sformatf("HOLD: first claim returned %0d, expected 7", v));
    wr(a_clm(0), 32'd7);
    rd(a_clm(0), v); if (v !== 8) err($sformatf("HOLD: second claim returned %0d, expected 8 (one source swallowed?)", v));
    wr(a_clm(0), 32'd8);
    rd(a_clm(0), v); if (v !== 9) err($sformatf("HOLD: third claim returned %0d, expected 9", v));
    hold_mode = 1'b0;
    idle(4);
    open_mode = 1'b0;
    do_reset();
    cov_ev[13]++;
  endtask

  // ------------------------------------------------------------------------
  // OPEN ITEM O2: gateway semantics (level gateway, in-flight masking)
  // ------------------------------------------------------------------------
  task automatic test_gateway_open();
    logic [31:0] v;
    banner("O2 OPEN ITEM: gateway: a source still asserted at complete pends again; pulses while in flight ignored");
    do_reset();
    open_mode = 1'b1;
    setup_src(0, 12, 4);
    // level held through the complete
    src_set(irq_sources | (NSOURCES'(1) << 12)); idle(9);
    rd(a_clm(0), v); if (v !== 12) err("gateway: first claim");
    idle(8); rd(a_clm(0), v); if (v !== 0) err($sformatf("gateway: source in flight must not pend again before complete (claim %0d)", v));
    wr(a_clm(0), 32'd12);                                                 // complete while the level is still high
    idle(4);
    if (eip[0] !== 1'b1) err("gateway: source still asserted at complete did not pend again");
    rd(a_clm(0), v); if (v !== 12) err($sformatf("gateway: claim after re-pend returned %0d", v));
    src_set(irq_sources & ~(NSOURCES'(1) << 12));
    wr(a_clm(0), 32'd12);
    idle(8);
    if (eip[0] !== 1'b0) err("gateway: eip set after the source dropped and was completed");
    // pulse while in flight is ignored; complete of a source that is not enabled is ignored
    src_pulse(12, 2); idle(8);
    rd(a_clm(0), v); if (v !== 12) err("gateway: claim");
    src_pulse(12, 2); idle(8);
    wr(a_clm(0), 32'd12);
    idle(4);
    if (eip[0] !== 1'b0) err("gateway: a pulse while in flight must be ignored");
    wr(a_en(0, 0), 32'h0);
    setup_src(0, 13, 4); wr(a_en(0, 0), 32'h1 << 13);
    src_pulse(13, 2); idle(8);
    rd(a_clm(0), v); if (v !== 13) err("gateway: claim of 13");
    wr(a_en(0, 0), 32'h0);
    wr(a_clm(0), 32'd13);                                                 // not enabled: ignored, stays in flight
    wr(a_en(0, 0), 32'h1 << 13);
    src_pulse(13, 2); idle(8);
    if (eip[0] !== 1'b0) err("gateway: complete of a disabled source must be ignored");
    idle(2);
    open_mode = 1'b0;
    do_reset();
    cov_ev[14]++;
  endtask

  task automatic report_cov();
    for (int s = 1; s < NSOURCES; s++) begin
      if (cov_pend[s] == 0)  err($sformatf("COVERAGE: source %0d never became pending", s));
      if (cov_claim[s] == 0) err($sformatf("COVERAGE: source %0d never claimed", s));
    end
    for (int p = 0; p < 8; p++) begin
      if (cov_pri[p] == 0) err($sformatf("COVERAGE: priority value %0d never written", p));
      if (cov_thr[p] == 0) err($sformatf("COVERAGE: threshold value %0d never written", p));
    end
    for (int c = 0; c < NCONTEXTS; c++) begin
      for (int w = 0; w < NW; w++) if (cov_enw[c][w] == 0) err($sformatf("COVERAGE: enable[%0d][%0d] never written", c, w));
      if (cov_eip[c][0] == 0 || cov_eip[c][1] == 0) err($sformatf("COVERAGE: eip[%0d] never seen both 0 and 1", c));
    end
    for (int i = 0; i < 15; i++) if (cov_ev[i] == 0) err($sformatf("COVERAGE: scenario %0d never ran", i));
    $display(" %s coverage: %0d sources claimed, 8 priorities, 8 thresholds, %0d contexts, unmapped reads=%0d, completes valid/ignored=%0d/%0d",
             TAG, NSOURCES - 1, NCONTEXTS, cov_ev[2], cov_ev[0], cov_ev[1]);
  endtask

  initial begin
    done = 1'b0; nerr = 0; nopen = 0;
    @(negedge clk);
    do_reset();
    test_reset_defaults();
    test_priority();
    test_enable();
    test_threshold();
    test_pending();
    test_eip();
    test_claim_order();
    test_unmapped();
    test_reset_state();
    test_random(2500);
    test_hold_open();
    test_gateway_open();
    idle(10);
    report_cov();
    done = 1'b1;
  end

endmodule

// ============================================================================
module tb_plic_full;
  bit da, db;
  int ea, eb, oa, ob;
  int tot_e, tot_o;

  tb_plic_core #(.NSOURCES(64), .NCONTEXTS(2), .TAG("S64C2")) ua (.done(da), .nerr(ea), .nopen(oa));
  tb_plic_core #(.NSOURCES(32), .NCONTEXTS(3), .TAG("S32C3")) ub (.done(db), .nerr(eb), .nopen(ob));

  initial begin
    wait (da && db);
    #100;
    tot_e = ea + eb; tot_o = oa + ob;
    $display("==================================================");
    $display(" contract errors = %0d (S64C2 %0d, S32C3 %0d)   open-item hits = %0d  (bus handshake / gateway semantics)", tot_e, ea, eb, tot_o);
    if (tot_e != 0)      $display(" *** RESULT: FAIL ***");
    else if (tot_o != 0) $display(" *** RESULT: PASS WITH OPEN ITEMS (%0d) - not signed off ***", tot_o);
    else                 $display(" *** RESULT: PASS ***");
    $display("==================================================");
    $finish;
  end

  initial begin
    #2_000_000_000;
    $display("GLOBAL WATCHDOG TIMEOUT");
    $display(" *** RESULT: FAIL ***");
    $finish;
  end
endmodule

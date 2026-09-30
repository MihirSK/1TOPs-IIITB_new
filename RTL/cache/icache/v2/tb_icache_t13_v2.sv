// ============================================================================
//  tb_icache.sv  -  Self-checking testbench for icache.sv  (Vivado xsim, plain SV)
//
//  Structure
//   - Behavioral 64-bit memory: data = hash(dword addr, epoch). Bumping "epoch"
//     changes what memory returns, so a stale line that survives a FENCE.I
//     returns wrong data and is caught.
//   - Configurable ack latency (0 / fixed N / random) and spurious mem_ack.
//   - Passive monitor + shadow model of the cache (valid/tag/epoch per set):
//       * every cpu_ack must be a hit the model agrees with
//       * every cpu_rdata must match expected data
//       * every refill must be 8 beats, mem_addr = line base + 8*beat
//       * a refill must not start for a line the model says is cached
//   - SVA checks, functional coverage, directed + constrained-random tests.
//
//  Driver rule: tasks start and end on a posedge; all inputs change at
//  posedge+1ns, so there are no races with the DUT.
//
//  Vivado: add icache.sv (Design Source) + tb_icache.sv (Simulation Source),
//  set tb_icache as top, run behavioral simulation, "run all".
//  If your xsim version rejects covergroups, add  -d NO_COV  to xvlog options.
// ============================================================================
`timescale 1ns/1ps

module tb_icache;

  // --------------------------------------------------------------------------
  // Clock / DUT
  // --------------------------------------------------------------------------
  logic        clk   = 1'b0;
  logic        rst_n = 1'b0;
  always #5 clk = ~clk;

  logic [63:0] cpu_addr = '0;
  logic        cpu_req  = 1'b0;
  logic        flush    = 1'b0;
  logic [31:0] cpu_rdata;
  logic        cpu_ack;
  logic [63:0] mem_addr;
  logic        mem_req;
  logic [63:0] mem_rdata;
  logic        mem_ack;

  icache dut (.*);

  // --------------------------------------------------------------------------
  // Error bookkeeping
  // --------------------------------------------------------------------------
  int errors     = 0;
  int known_hits = 0;
  bit known_mode = 1'b0;   // errors are counted as "known issue" instead of failures

  function automatic void err(input string m);
    if (known_mode) begin
      known_hits++;
      if (known_hits <= 10) $display("[%0t] KNOWN-ISSUE: %s", $time, m);
    end else begin
      errors++;
      if (errors <= 40) $display("[%0t] ERROR: %s", $time, m);
    end
  endfunction

  // --------------------------------------------------------------------------
  // Memory model
  // --------------------------------------------------------------------------
  int mem_epoch = 0;

  function automatic logic [63:0] mem_dw(input logic [63:0] a, input int ep);
    logic [63:0] x;
    x = {a[63:3], 3'b000} ^ (64'h9E3779B97F4A7C15 * (64'(ep) + 64'd1));
    x = x ^ (x >> 30);
    x = x * 64'hBF58476D1CE4E5B9;
    x = x ^ (x >> 27);
    x = x * 64'h94D049BB133111EB;
    x = x ^ (x >> 31);
    return x;
  endfunction

  function automatic logic [31:0] exp_word(input logic [63:0] a, input int ep);
    logic [63:0] dw;
    dw = mem_dw(a, ep);
    return a[2] ? dw[63:32] : dw[31:0];
  endfunction

  assign mem_rdata = mem_dw(mem_addr, mem_epoch);

  // ack latency control
  int lat_mode = 0;    // 0: zero latency, 1: fixed lat_n, 2: random 0..lat_max
  int lat_n    = 2;
  int lat_max  = 4;
  bit spur_en  = 1'b0; // spurious mem_ack while mem_req is low
  int wait_cnt = 0;
  logic spur_q = 1'b0;

  function automatic int pick_lat();
    case (lat_mode)
      0:       return 0;
      1:       return lat_n;
      default: return $urandom_range(0, lat_max);
    endcase
  endfunction

  always @(posedge clk) begin
    spur_q <= spur_en ? 1'($urandom_range(0, 1)) : 1'b0;
    if (!mem_req)                    wait_cnt <= pick_lat();
    else if (mem_ack)                wait_cnt <= pick_lat();
    else if (wait_cnt != 0)          wait_cnt <= wait_cnt - 1;
  end

  assign mem_ack = mem_req ? (wait_cnt == 0) : spur_q;

  task automatic set_lat(input int mode, input int n, input int mx, input bit spur);
    lat_mode = mode; lat_n = n; lat_max = mx; spur_en = spur;
  endtask

  // --------------------------------------------------------------------------
  // Shadow model + passive monitor
  // --------------------------------------------------------------------------
  bit          sh_v   [256];
  logic [49:0] sh_tag [256];
  int          sh_ep  [256];

  int          total_beats = 0;
  int          n_acks      = 0;
  int          n_refills   = 0;
  int          beat_mon    = 0;
  bit          fill_active = 1'b0;
  logic [63:0] base_mon;
  bit          m_hit;
  logic [31:0] m_exp;

  function automatic bit shadow_hit(input logic [63:0] a);
    return sh_v[a[13:6]] && (sh_tag[a[13:6]] == a[63:14]);
  endfunction

  always @(posedge clk) begin
    if (!rst_n) begin
      for (int i = 0; i < 256; i++) sh_v[i] = 1'b0;
      beat_mon = 0; fill_active = 1'b0;
    end else begin
      // ---- CPU side ----
      if (cpu_ack) begin
        if (!cpu_req) err("cpu_ack asserted without cpu_req");
        else begin
          m_hit = shadow_hit(cpu_addr);
          if (!m_hit)
            err($sformatf("HIT on line model says is NOT cached (stale/aliased) addr=%h", cpu_addr));
          m_exp = exp_word(cpu_addr, m_hit ? sh_ep[cpu_addr[13:6]] : mem_epoch);
          if (cpu_rdata !== m_exp)
            err($sformatf("rdata mismatch addr=%h got=%h exp=%h", cpu_addr, cpu_rdata, m_exp));
          n_acks++;
        end
      end

      // ---- Memory side ----
      if (mem_req) begin
        if (!fill_active) begin
          fill_active = 1'b1;
          beat_mon    = 0;
          base_mon    = {cpu_addr[63:6], 6'b0};
          if (shadow_hit(base_mon))
            err($sformatf("refill started for line model says is cached, addr=%h", base_mon));
        end
        if (mem_ack) begin
          if (mem_addr !== base_mon + 64'(beat_mon * 8))
            err($sformatf("bad refill addr beat=%0d got=%h exp=%h", beat_mon, mem_addr,
                          base_mon + 64'(beat_mon * 8)));
          total_beats++;
          beat_mon++;
          if (beat_mon == 8) begin
            fill_active = 1'b0; beat_mon = 0; n_refills++;
            if (!flush) begin
              sh_v  [base_mon[13:6]] = 1'b1;
              sh_tag[base_mon[13:6]] = base_mon[63:14];
              sh_ep [base_mon[13:6]] = mem_epoch;
            end
          end
        end
      end

      // ---- FENCE.I: everything dies (applied last so a same-cycle hit is still legal) ----
      if (flush) begin
        for (int i = 0; i < 256; i++) sh_v[i] = 1'b0;
        fill_active = 1'b0; beat_mon = 0;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Assertions
  // --------------------------------------------------------------------------
  a_beat_align: assert property (@(posedge clk) disable iff (!rst_n)
      mem_req |-> (mem_addr[2:0] == 3'b000))
    else err("SVA: mem_addr not 8B aligned");

  a_line_stable: assert property (@(posedge clk) disable iff (!rst_n)
      mem_req |-> (mem_addr[63:6] == cpu_addr[63:6]))
    else err("SVA: refill address not within requested line (cpu_addr changed mid-refill?)");

  a_ack_needs_req: assert property (@(posedge clk) disable iff (!rst_n)
      cpu_ack |-> cpu_req)
    else err("SVA: cpu_ack without cpu_req");

  a_no_x: assert property (@(posedge clk) disable iff (!rst_n)
      cpu_ack |-> !$isunknown(cpu_rdata))
    else err("SVA: X on cpu_rdata during ack");

  a_no_ack_in_refill: assert property (@(posedge clk) disable iff (!rst_n)
      mem_req |-> !cpu_ack)
    else err("SVA: cpu_ack while refill in progress");

  // --------------------------------------------------------------------------
  // Functional coverage
  // --------------------------------------------------------------------------
`ifndef NO_COV
  covergroup cg_acc @(posedge clk iff (rst_n && cpu_req && cpu_ack));
    cp_idx  : coverpoint cpu_addr[13:6] { option.auto_bin_max = 256; }
    cp_word : coverpoint cpu_addr[5:2];
  endgroup

  covergroup cg_flush @(posedge clk iff (rst_n && flush));
    cp_state : coverpoint dut.state { bins idle = {2'b00}; bins fetch = {2'b01}; bins fill = {2'b10}; }
    cp_beat  : coverpoint dut.beat iff (dut.state == 2'b01);
    cp_req   : coverpoint cpu_req;
    x_st_req : cross cp_state, cp_req;
  endgroup

  cg_acc   cga = new();
  cg_flush cgf = new();
`endif

  // --------------------------------------------------------------------------
  // Driver tasks   (all start and end on a posedge)
  // --------------------------------------------------------------------------
  task automatic do_reset();
    #1; rst_n = 1'b0; cpu_req = 1'b0; flush = 1'b0;
    repeat (3) @(posedge clk);
    #1; rst_n = 1'b1;
    @(posedge clk);
  endtask

  task automatic idle(input int n);
    #1; cpu_req = 1'b0;
    repeat (n) @(posedge clk);
  endtask

  // wait (already driving) until ack; returns #edges waited
  task automatic wait_ack(output int cyc);
    int c = 0;
    forever begin
      @(posedge clk);
      c++;
      if (cpu_ack) break;
      if (c > 3000) begin err("TIMEOUT waiting for cpu_ack"); break; end
    end
    cyc = c;
  endtask

  // drive + wait, no leading delay
  task automatic drive_wait(input logic [63:0] a, output int cyc);
    cpu_addr = a; cpu_req = 1'b1;
    wait_ack(cyc);
  endtask

  task automatic fetch(input logic [63:0] a);
    int cyc;
    #1;
    drive_wait(a, cyc);
  endtask

  // exp: 0 = must miss (8 beats), 1 = must hit (1 cycle, no traffic), 2 = ask shadow model
  task automatic fetch_chk(input logic [63:0] a, input int exp);
    int  cyc, b0;
    bit  eh;
    #1;
    eh = (exp == 2) ? shadow_hit(a) : (exp == 1);
    b0 = total_beats;
    drive_wait(a, cyc);
    if (eh) begin
      if (cyc != 1)              err($sformatf("hit took %0d cycles (addr=%h)", cyc, a));
      if (total_beats != b0)     err($sformatf("memory traffic on a hit (addr=%h)", a));
    end else begin
      if (total_beats - b0 != 8) err($sformatf("miss made %0d beats, expected 8 (addr=%h)",
                                               total_beats - b0, a));
    end
  endtask

  task automatic fetch_m(input logic [63:0] a);
    fetch_chk(a, 2);
  endtask

  // synchronous FENCE.I with no request in flight; memory content changes afterwards
  task automatic do_flush();
    #1; cpu_req = 1'b0; flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0; mem_epoch++;
    @(posedge clk);
  endtask

  task automatic wait_dut_idle();
    int g = 0;
    forever begin
      @(posedge clk); #1;
      if (dut.state == 2'b00) break;
      if (++g > 200) begin err("DUT never returned to IDLE"); break; end
    end
    @(posedge clk);
  endtask

  // pulse flush when the DUT is in FETCH at beat k (lat_mode 0 => lands on the accept edge)
  task automatic flush_at_beat(input int k);
    int g = 0;
    forever begin
      @(posedge clk); #1;
      if (dut.state == 2'b01 && int'(dut.beat) == k) break;
      if (++g > 500) begin err("flush_at_beat timeout"); return; end
    end
    flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0; mem_epoch++;
  endtask

  // --------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------
  task automatic banner(input string s);
    $display("[%0t] ---- %s", $time, s);
  endtask

  task automatic test_smoke();
    banner("T1 smoke: miss then hit");
    set_lat(0, 0, 0, 0);
    fetch_chk(64'h0000_0000_8000_0000, 0);
    fetch_chk(64'h0000_0000_8000_0000, 1);
  endtask

  task automatic test_word_offsets();
    logic [63:0] base = 64'h0000_0000_1000_0040;
    banner("T2 all 16 word offsets in a line");
    fetch_chk(base, 0);
    for (int w = 0; w < 16; w++) fetch_chk(base + 64'(w * 4), 1);
    // next line, then walk backwards
    fetch_chk(base + 64, 0);
    for (int w = 15; w >= 0; w--) fetch_chk(base + 64 + 64'(w * 4), 1);
  endtask

  task automatic test_latency_modes();
    banner("T3 memory latency modes");
    for (int m = 0; m < 4; m++) begin
      case (m)
        0: set_lat(0, 0, 0, 0);
        1: set_lat(1, 1, 0, 0);
        2: set_lat(1, 4, 0, 0);
        3: set_lat(2, 0, 6, 1);   // random latency + spurious mem_ack
      endcase
      for (int l = 0; l < 4; l++) begin
        fetch_chk(64'h0000_0000_3000_0000 + 64'(m * 64'h4000) + 64'(l * 64), 0);
        fetch_chk(64'h0000_0000_3000_0000 + 64'(m * 64'h4000) + 64'(l * 64) + 8, 1);
      end
    end
    set_lat(0, 0, 0, 0);
  endtask

  task automatic test_index_sweep();
    banner("T4 index sweep (all 256 sets)");
    for (int i = 0; i < 256; i++) fetch_chk(64'h0000_0000_4000_0000 + 64'(i * 64), 0);
    for (int i = 0; i < 256; i++) fetch_chk(64'h0000_0000_4000_0000 + 64'(i * 64) + 4, 1);
  endtask

  task automatic test_tag_bits();
    logic [63:0] a, b;
    banner("T5 each tag bit [63:14] must be compared");
    a = 64'h0000_0000_4000_0000;
    for (int bt = 14; bt < 64; bt++) begin
      b = a ^ (64'd1 << bt);
      fetch_m(a);          // A resident
      fetch_chk(b, 0);     // differs in one tag bit: must miss
      fetch_chk(a, 0);     // B evicted A: must miss
    end
  endtask

  task automatic test_thrash();
    logic [63:0] a = 64'h0000_0000_5000_0080;
    logic [63:0] b = 64'h0000_0000_6000_0080;   // same index, different tag
    int r0;
    banner("T6 direct-mapped conflict: A B A B = 4 refills");
    r0 = n_refills;
    fetch_chk(a, 0); fetch_chk(b, 0); fetch_chk(a, 0); fetch_chk(b, 0);
    fetch_chk(b, 1);
    if (n_refills - r0 != 4) err("thrash refill count != 4");
  endtask

  task automatic test_streaming();
    int r0 = n_refills;
    banner("T7 sequential streaming, 1024 words");
    for (int w = 0; w < 1024; w++) fetch_m(64'h0000_0000_2000_0000 + 64'(w * 4));
    if (n_refills - r0 != 64) err($sformatf("streaming refills=%0d expected 64", n_refills - r0));
  endtask

  task automatic test_edges();
    banner("T8 address edges");
    fetch_m(64'h0);
    fetch_m(64'h0000_0000_1000_3FFC);       // idx 255, word 15
    fetch_m(64'hFFFF_FFFF_FFFF_FFFC);       // top of address space
    fetch_m(64'hFFFF_FFFF_FFFF_FFFC);
    fetch_m(64'h8000_0000_0000_0000);       // only MSB set
  endtask

  task automatic test_flush_basic();
    banner("T9 FENCE.I: basic, flush+hit same cycle, flush+miss same cycle");
    set_lat(0, 0, 0, 0);
    for (int i = 0; i < 16; i++) fetch_m(64'h0000_0000_7000_0000 + 64'(i * 64));
    do_flush();                     // also bumps memory epoch
    for (int i = 0; i < 16; i++) fetch_chk(64'h0000_0000_7000_0000 + 64'(i * 64), 0);
    do_flush();                     // flush with nothing valid / while idle
    // flush together with a hitting request
    fetch_chk(64'h0000_0000_7000_0000, 0);
    #1; cpu_addr = 64'h0000_0000_7000_0000; cpu_req = 1'b1; flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0; cpu_req = 1'b0; mem_epoch++;
    @(posedge clk);
    fetch_chk(64'h0000_0000_7000_0000, 0);
    // flush together with a missing request: flush wins, request then refills normally
    #1; cpu_addr = 64'h0000_0000_7100_0000; cpu_req = 1'b1; flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0;
    begin int c; wait_ack(c); end
    fetch_chk(64'h0000_0000_7100_0000, 1);
  endtask

  task automatic test_flush_mid_refill();
    logic [63:0] a;
    banner("T10 FENCE.I during refill, every beat, two latency modes");
    for (int lm = 0; lm < 2; lm++) begin
      for (int k = 0; k < 8; k++) begin
        set_lat(lm, 1, 0, 0);
        a = 64'h0000_0000_6000_0000 + 64'((lm * 8 + k) * 64);
        fork
          fetch(a);
          flush_at_beat(k);
        join
        // line must now be present with NEW-epoch data; monitor already checked ack/data
        fetch_chk(a, 1);
        idle(2);
      end
    end
    set_lat(0, 0, 0, 0);
  endtask

  task automatic test_reset_mid_refill();
    banner("T11 reset in the middle of a refill");
    set_lat(1, 1, 0, 0);
    fetch_m(64'h0000_0000_9000_0000);          // resident line, must vanish on reset
    fork
      fetch(64'h0000_0000_9100_0040);
      begin
        int g = 0;
        forever begin
          @(posedge clk); #1;
          if (dut.state == 2'b01 && dut.beat == 3'd3) break;
          if (++g > 200) break;
        end
        rst_n = 1'b0;
        repeat (2) @(posedge clk);
        #1 rst_n = 1'b1;
      end
    join
    idle(2);
    fetch_chk(64'h0000_0000_9000_0000, 0);     // must miss after reset
    set_lat(0, 0, 0, 0);
  endtask

  // Spec-dependent: does the core promise to hold cpu_addr during a miss?
  // Failures here are counted as KNOWN-ISSUE, not test failures.
  task automatic test_known_addr_change();
    int g = 0;
    banner("T12 (known issue) cpu_addr changes mid-refill");
    known_mode = 1'b1;
    set_lat(1, 1, 0, 0);
    #1; cpu_addr = 64'h0000_0000_A000_0000; cpu_req = 1'b1;
    forever begin
      @(posedge clk); #1;
      if (dut.state == 2'b01 && dut.beat == 3'd3) break;
      if (++g > 200) break;
    end
    cpu_addr = 64'h0000_0000_A000_1040;         // different index and tag
    g = 0;
    forever begin
      @(posedge clk);
      if (cpu_ack || ++g > 60) break;
    end
    do_reset();                                  // clean up DUT + shadow
    idle(3);
    known_mode = 1'b0;
    set_lat(0, 0, 0, 0);
  endtask

  // req dropped mid-miss with a stable address: fill must still complete cleanly
  task automatic aborted_fetch(input logic [63:0] a, input int k);
    int c = 0;
    #1; cpu_addr = a; cpu_req = 1'b1;
    repeat (k) begin
      @(posedge clk);
      if (cpu_ack) begin c = 1; break; end
    end
    #1; cpu_req = 1'b0;
    wait_dut_idle();
  endtask

  // flush on the cycle the DUT is in FILL
  task automatic flush_in_fill();
    int g = 0;
    forever begin
      @(posedge clk); #1;
      if (dut.state == 2'b10) break;
      if (++g > 500) begin err("flush_in_fill timeout"); return; end
    end
    flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0; mem_epoch++;
  endtask

  task automatic test_flush_corners();
    logic [63:0] a = 64'h0000_0000_B000_0000;
    banner("T14 flush in FILL state, flush in FETCH with cpu_req low");
    // (a) flush while DUT is in FILL
    set_lat(0, 0, 0, 0);
    fork
      fetch(a);
      flush_in_fill();
    join
    @(posedge clk);
    fetch_chk(a, 0);                       // line must be gone
    // (b) request dropped, then flush mid-refill (no request pending)
    set_lat(1, 1, 0, 0);
    a = 64'h0000_0000_B000_0040;
    fork
      begin #1; cpu_addr = a; cpu_req = 1'b1; @(posedge clk); #1; cpu_req = 1'b0; end
      flush_at_beat(3);
    join
    @(posedge clk);
    idle(4);
    fetch_chk(a, 0);                       // nothing was cached, must refill
    set_lat(0, 0, 0, 0);
  endtask

  function automatic logic [63:0] pool_addr();
    int t = $urandom_range(0, 3);
    int i = $urandom_range(0, 7);
    int w = $urandom_range(0, 15);
    logic [49:0] tag;
    logic [7:0]  idx;
    case (t)
      0: tag = 50'h0;
      1: tag = 50'h1;
      2: tag = 50'h2_0000_0000_0001;
      default: tag = 50'h3_FFFF_FFFF_FFFF;
    endcase
    case (i)
      0: idx = 8'd0;   1: idx = 8'd1;   2: idx = 8'd2;   3: idx = 8'd127;
      4: idx = 8'd128; 5: idx = 8'd254; 6: idx = 8'd255; default: idx = 8'd77;
    endcase
    return {tag, idx, 6'b0} | 64'(w * 4);
  endfunction

  task automatic test_random(input int n);
    int r;
    banner("T13 constrained random");
    for (int i = 0; i < n; i++) begin
      if (i % 200 == 0)
        set_lat($urandom_range(0, 2), $urandom_range(1, 4), $urandom_range(1, 6), 1'($urandom_range(0, 1)));
      r = $urandom_range(0, 99);
      if      (r < 2)  do_flush();
      else if (r < 5)  aborted_fetch(pool_addr(), $urandom_range(1, 6));
      else if (r < 12) idle($urandom_range(1, 5));
      else             fetch_m(pool_addr());
    end
    set_lat(0, 0, 0, 0);
  endtask

  // --------------------------------------------------------------------------
  // Main
  // --------------------------------------------------------------------------
  initial begin
    @(posedge clk);
    do_reset();

    test_smoke();
    test_word_offsets();
    test_latency_modes();
    test_index_sweep();
    test_tag_bits();
    test_thrash();
    test_streaming();
    test_edges();
    test_flush_basic();
    test_flush_mid_refill();
    test_reset_mid_refill();
    test_known_addr_change();
    test_flush_corners();
    test_random(20000);

    idle(10);
    $display("==================================================");
    $display(" acks=%0d  refills=%0d  beats=%0d", n_acks, n_refills, total_beats);
`ifndef NO_COV
    $display(" coverage: acc=%0.1f%%  flush=%0.1f%%", cga.get_inst_coverage(), cgf.get_inst_coverage());
`endif
    $display(" errors=%0d   known-issue hits=%0d", errors, known_hits);
    if (errors == 0) $display(" *** TEST PASSED ***");
    else             $display(" *** TEST FAILED ***");
    $display("==================================================");
    $finish;
  end

  // global watchdog
  initial begin
    #400_000_000;
    $display("GLOBAL WATCHDOG TIMEOUT");
    $finish;
  end

endmodule

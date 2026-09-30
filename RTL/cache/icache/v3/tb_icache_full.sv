// ============================================================================
//  tb_icache_full.sv  -  Functional verification testbench for icache.sv (v2)
//  Plain SystemVerilog, no DUT-internal references, no covergroups/SVA needed
//  (checks are procedural so xsim / Verilator / other tools behave the same).
//
//  What is new compared with tb_icache_t13_v2:
//   * Nothing peeks at dut.state / dut.beat. DUT state is reconstructed from the
//     pins (mem_req, mem_ack, beat count), so the TB survives RTL refactoring.
//   * mem_rdata is POISONED (random junk) whenever mem_ack is low, so a DUT that
//     samples data outside the ack cycle can no longer pass.
//   * Second memory personality (mem_mode=1): registered ack/data, one ack pulse
//     per beat (level req / ack handshake). Old TB only had a combinational slave.
//   * Core-like fetch driver (cpu_req always high, address changes only after an
//     ack, optional stall/hold cycles, taken branches) - what riscv_core really does.
//   * Independent architectural scoreboard: every acked word must equal memory
//     content at some legal epoch (>= last FENCE.I), regardless of cache internals.
//   * Reset sweep over all 256 sets, reset in FILL / during hit streams, async
//     reset at a non-clock-aligned time, no activity while in reset.
//   * Flush matrix: flush at every refill beat x cpu_req 0/1, in FILL x req 0/1,
//     in IDLE x req 0/1, flush held for several cycles.
//   * X/Z checks on cpu_ack / mem_req / cpu_rdata (old TB treated X as "not ack").
//   * Byte offsets with addr[1:0] != 0 (design ignores them; core traps earlier).
//   * "cpu_addr changes while a refill is outstanding" is now a real, separately
//     reported OPEN ITEM (redirect mid-miss, hit-under-miss, req dropped +
//     address scrambled) instead of being hidden in a known-issue bucket.
//   * Coverage holes are FAILURES (counter based, works in every simulator).
//
//  Result lines at the end:
//     *** RESULT: PASS ***                     contract clean, no open items
//     *** RESULT: PASS WITH OPEN ITEMS (n) *** contract clean, spec question open
//     *** RESULT: FAIL ***                     at least one contract check failed
//  Compile with  -define STRICT_OPEN  (xvlog -d STRICT_OPEN) to make open items fail.
//
//  Vivado: icache.sv = Design Source, this file = Simulation Source, set
//  tb_icache_full as top, run behavioral simulation, "run all".
// ============================================================================
`timescale 1ns/1ps

module tb_icache_full;

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
  //   errors    : contract violations (always fail the run)
  //   open_hits : violations seen while open_mode=1 (spec decision pending)
  // --------------------------------------------------------------------------
  int errors    = 0;
  int open_hits = 0;
  int open_grp  = 0;          // 1: address change mid-refill, 2: memory handshake assumption
  int open_cnt [3];
  bit open_mode = 1'b0;

  function automatic void err(input string m);
    if (open_mode) begin
`ifdef STRICT_OPEN
      errors++;
      if (errors <= 60) $display("[%0t] ERROR (open item, strict): %s", $time, m);
`else
      open_hits++;
      open_cnt[open_grp]++;
      if (open_hits <= 14) $display("[%0t] OPEN-ITEM(%0d): %s", $time, open_grp, m);
`endif
    end else begin
      errors++;
      if (errors <= 60) $display("[%0t] ERROR: %s", $time, m);
    end
  endfunction

  // --------------------------------------------------------------------------
  // Reference memory: pure function of (dword address, epoch)
  // --------------------------------------------------------------------------
  int mem_epoch   = 0;   // what memory holds right now
  int flush_epoch = 0;   // memory epoch in force at the last FENCE.I

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

  // architectural rule: after FENCE.I only data from epochs >= flush_epoch is legal
  function automatic bit arch_ok(input logic [63:0] a, input logic [31:0] d);
    for (int e = flush_epoch; e <= mem_epoch; e++)
      if (d === exp_word(a, e)) return 1'b1;
    return 1'b0;
  endfunction

  task automatic flush_done();     // call right after a FENCE.I pulse ended
    mem_epoch++;
    flush_epoch = mem_epoch;
  endtask

  // --------------------------------------------------------------------------
  // Memory-side model.  mem_mode 0: combinational slave (ack/data follow req/addr)
  //                     mem_mode 1: registered slave (ack <= req & !ack, data
  //                                 latched when the beat is accepted by the slave)
  //  mem_rdata carries junk whenever mem_ack is low (poison).
  // --------------------------------------------------------------------------
  int   mem_mode = 0;    // 0 comb slave, 1 registered ack<=req&!ack, 2 naive ack<=req
  int   lat_mode = 0;    // 0: zero latency, 1: fixed lat_n, 2: random 0..lat_max
  int   lat_n    = 2;
  int   lat_max  = 4;
  bit   spur_en  = 1'b0; // spurious mem_ack while mem_req is low (mode 0 only)
  int   wait_cnt = 0;
  logic spur_q   = 1'b0;

  function automatic int pick_lat();
    case (lat_mode)
      0:       return 0;
      1:       return lat_n;
      default: return $urandom_range(0, lat_max);
    endcase
  endfunction

  always @(posedge clk) begin
    spur_q <= spur_en ? 1'($urandom_range(0, 1)) : 1'b0;
    if (mem_req !== 1'b1)   wait_cnt <= pick_lat();
    else if (mem_ack)       wait_cnt <= pick_lat();
    else if (wait_cnt != 0) wait_cnt <= wait_cnt - 1;
  end

  logic        ack_r  = 1'b0;
  logic [63:0] data_r = '0;
  int          rc     = 0;
  always @(posedge clk) begin
    if (mem_mode == 1) begin
      if (ack_r) begin
        ack_r <= 1'b0;
        rc    <= pick_lat();
      end else if (mem_req === 1'b1) begin
        if (rc == 0) begin
          ack_r  <= 1'b1;
          data_r <= mem_dw(mem_addr, mem_epoch);
        end else rc <= rc - 1;
      end else rc <= pick_lat();
    end else if (mem_mode == 2) begin
      ack_r  <= (mem_req === 1'b1);          // typical quick BRAM wrapper: ack <= req
      data_r <= mem_dw(mem_addr, mem_epoch); //                             rdata <= mem[addr]
    end else ack_r <= 1'b0;
  end

  logic [63:0] junk;
  always @(posedge clk) junk <= {$urandom, $urandom};

  assign mem_ack   = (mem_mode != 0) ? ack_r : ((mem_req === 1'b1) ? (wait_cnt == 0) : spur_q);
  assign mem_rdata = mem_ack ? ((mem_mode != 0) ? data_r : mem_dw(mem_addr, mem_epoch)) : junk;

  task automatic set_lat(input int mode, input int n, input int mx, input bit spur);
    lat_mode = mode; lat_n = n; lat_max = mx; spur_en = spur;
  endtask

  // --------------------------------------------------------------------------
  // Passive monitor: shadow model + protocol checks + architectural scoreboard.
  // DUT state is rebuilt from pins:  FETCH = mem_req, FILL = cycle after the
  // last accepted beat (fill_flag), else IDLE.
  // --------------------------------------------------------------------------
  bit          sh_v   [256];
  logic [49:0] sh_tag [256];
  int          sh_ep  [256];

  int          total_beats = 0;
  int          n_acks      = 0;
  int          n_refills   = 0;
  int          beat_mon    = 0;
  bit          fill_active = 1'b0;
  bit          fill_flag   = 1'b0;
  logic [63:0] base_mon;
  bit          miss_prev   = 1'b0;
  logic [63:0] miss_line;
  bit          flush_prev  = 1'b0;
  bit          armed       = 1'b0;   // set after the first reset
  bit          hold_req    = 1'b1;   // 1: core promises to hold cpu_addr during a miss
  bit          m_hit, idle_miss, fill_next;
  logic [31:0] m_exp;
  int          cur_st;               // 0 idle, 1 fetch, 2 fill

  // functional coverage counters (a hole is a failure, see report_cov)
  int cov_idx  [256];
  int cov_word [16];
  int cov_fl   [10][2];   // [0..7]=flush in FETCH beat k, [8]=FILL, [9]=IDLE  x  cpu_req
  int cov_lb   [4];       // cpu_addr[1:0] seen on acks
  int cov_evict = 0;      // refill that replaced a valid line with another tag
  int cov_mm   [2];       // acks per memory personality

  function automatic bit shadow_hit(input logic [63:0] a);
    return sh_v[a[13:6]] && (sh_tag[a[13:6]] == a[63:14]);
  endfunction

  always @(posedge clk) begin
    if (!rst_n) begin
      if (armed && (cpu_ack !== 1'b0 || mem_req !== 1'b0))
        err("activity (cpu_ack/mem_req) while in reset");
      for (int i = 0; i < 256; i++) sh_v[i] = 1'b0;
      beat_mon = 0; fill_active = 1'b0; fill_flag = 1'b0;
      miss_prev = 1'b0; flush_prev = 1'b0;
    end else if (armed) begin
      cur_st    = (mem_req === 1'b1) ? 1 : (fill_flag ? 2 : 0);
      fill_next = 1'b0;

      // ---- X / protocol checks -------------------------------------------
      if ($isunknown(cpu_ack) || $isunknown(mem_req))
        err("X/Z on cpu_ack or mem_req");
      if (cpu_ack === 1'b1 && $isunknown(cpu_rdata))
        err("X/Z on cpu_rdata during cpu_ack");
      if (cpu_ack === 1'b1 && !cpu_req)
        err("cpu_ack asserted without cpu_req");
      if (mem_req === 1'b1 && mem_addr[2:0] !== 3'b000)
        err("mem_addr not 8-byte aligned");
      if (flush_prev && mem_req === 1'b1)
        err("mem_req high in the cycle right after a flush");
      if (hold_req && mem_req === 1'b1 && cpu_ack === 1'b1)
        err("cpu_ack while a refill is in progress");
      if (hold_req && mem_req === 1'b1 && mem_addr[63:6] !== cpu_addr[63:6])
        err("refill address left the requested line (cpu_addr changed mid-refill?)");

      // idle-miss cycle: DUT idle, request present, line not cached, no flush
      idle_miss = cpu_req && !shadow_hit(cpu_addr) && (mem_req !== 1'b1) && !fill_flag && !flush;

      // ---- CPU side ---------------------------------------------------------
      if (cpu_ack === 1'b1 && cpu_req) begin
        m_hit = shadow_hit(cpu_addr);
        if (!m_hit)
          err($sformatf("ack on a line the model says is NOT cached (stale/aliased) addr=%h", cpu_addr));
        m_exp = exp_word(cpu_addr, m_hit ? sh_ep[cpu_addr[13:6]] : mem_epoch);
        if (cpu_rdata !== m_exp)
          err($sformatf("rdata mismatch addr=%h got=%h exp=%h", cpu_addr, cpu_rdata, m_exp));
        if (!arch_ok(cpu_addr, cpu_rdata))
          err($sformatf("ARCH: rdata is not legal memory content addr=%h got=%h", cpu_addr, cpu_rdata));
        n_acks++;
        cov_idx [cpu_addr[13:6]]++;
        cov_word[cpu_addr[5:2]]++;
        cov_lb  [cpu_addr[1:0]]++;
        cov_mm  [mem_mode]++;
      end

      // ---- Memory side ------------------------------------------------------
      if (mem_req === 1'b1) begin
        if (!fill_active) begin
          fill_active = 1'b1;
          beat_mon    = 0;
          if (!miss_prev) begin
            err("refill started without a preceding idle-miss request cycle");
            base_mon = {cpu_addr[63:6], 6'b0};
          end else base_mon = miss_line;
          if (shadow_hit(base_mon))
            err($sformatf("refill started for a line the model says is cached, addr=%h", base_mon));
          if (sh_v[base_mon[13:6]] && sh_tag[base_mon[13:6]] != base_mon[63:14]) cov_evict++;
        end
        if (mem_ack === 1'b1) begin
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
              fill_next = 1'b1;
            end
          end
        end
      end

      // ---- FENCE.I: everything dies (applied last: a same-cycle hit is legal) --
      if (flush) begin
        cov_fl[(cur_st == 1) ? beat_mon : ((cur_st == 2) ? 8 : 9)][cpu_req ? 1 : 0]++;
        for (int i = 0; i < 256; i++) sh_v[i] = 1'b0;
        fill_active = 1'b0; beat_mon = 0;
      end

      miss_line  = {cpu_addr[63:6], 6'b0};
      miss_prev  = idle_miss;
      fill_flag  = fill_next;
      flush_prev = flush;
    end
  end

  // --------------------------------------------------------------------------
  // Driver tasks. All start on a posedge; inputs change at posedge+1ns.
  // --------------------------------------------------------------------------
  task automatic do_reset();
    #1; rst_n = 1'b0; cpu_req = 1'b0; flush = 1'b0;
    repeat (3) @(posedge clk);
    armed = 1'b1;
    #1; rst_n = 1'b1;
    @(posedge clk);
  endtask

  // async reset asserted mid-cycle (not clock aligned), cpu_req/addr left as is
  task automatic reset_mid_cycle(input int cycles);
    #3; rst_n = 1'b0;
    repeat (cycles) @(posedge clk);
    #1; rst_n = 1'b1;
    @(posedge clk);
  endtask

  task automatic idle(input int n);
    #1; cpu_req = 1'b0;
    repeat (n) @(posedge clk);
  endtask

  task automatic wait_ack(output int cyc);
    int c = 0;
    forever begin
      @(posedge clk);
      c++;
      if (cpu_ack === 1'b1) break;
      if (c > 3000) begin err("TIMEOUT waiting for cpu_ack"); break; end
    end
    cyc = c;
  endtask

  task automatic drive_wait(input logic [63:0] a, output int cyc);
    cpu_addr = a; cpu_req = 1'b1;
    wait_ack(cyc);
  endtask

  task automatic fetch(input logic [63:0] a);
    int cyc;
    #1;
    drive_wait(a, cyc);
  endtask

  // exp: 0 = must miss (8 beats), 1 = must hit (1 cycle, no traffic), 2 = ask shadow
  task automatic fetch_chk(input logic [63:0] a, input int exp);
    int  cyc, b0;
    bit  eh;
    #1;
    eh = (exp == 2) ? shadow_hit(a) : (exp == 1);
    b0 = total_beats;
    drive_wait(a, cyc);
    if (eh) begin
      if (cyc != 1)          err($sformatf("hit took %0d cycles (addr=%h)", cyc, a));
      if (total_beats != b0) err($sformatf("memory traffic on a hit (addr=%h)", a));
    end else begin
      if (total_beats - b0 != 8)
        err($sformatf("miss made %0d beats, expected 8 (addr=%h)", total_beats - b0, a));
    end
  endtask

  task automatic fetch_m(input logic [63:0] a);
    fetch_chk(a, 2);
  endtask

  task automatic do_flush();       // FENCE.I with no request; memory changes afterwards
    #1; cpu_req = 1'b0; flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0; flush_done();
    @(posedge clk);
  endtask

  // returns at posedge+1 with the DUT in FETCH requesting beat k (pins only)
  task automatic wait_beat(input int k);
    int g = 0;
    forever begin
      @(posedge clk); #1;
      if (mem_req === 1'b1 && beat_mon == k) break;
      g = g + 1;
      if (g > 800) begin err("wait_beat timeout"); return; end
    end
  endtask

  // returns at posedge+1 with the DUT in FILL (pins only)
  task automatic wait_fill();
    int g = 0;
    forever begin
      @(posedge clk); #1;
      if (fill_flag) break;
      g = g + 1;
      if (g > 800) begin err("wait_fill timeout"); return; end
    end
  endtask

  task automatic wait_dut_idle();
    int g = 0;
    forever begin
      @(posedge clk); #1;
      if (mem_req !== 1'b1 && !fill_flag) break;
      g = g + 1;
      if (g > 400) begin err("DUT never returned to idle"); break; end
    end
    @(posedge clk);
  endtask

  task automatic flush_at_beat(input int k);
    wait_beat(k);
    flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0; flush_done();
  endtask

  task automatic flush_in_fill();
    wait_fill();
    flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0; flush_done();
  endtask

  // single-process version of "fetch(a) || flush_at_beat(k)": request a line that is
  // known to miss, pulse flush when the DUT requests beat k, wait for the (restarted) ack
  task automatic fetch_flush_at(input logic [63:0] a, input int k);
    bit flushed, acked;
    int g;
    #1; cpu_addr = a; cpu_req = 1'b1; flushed = 1'b0; g = 0;
    forever begin
      @(posedge clk);
      acked = (cpu_ack === 1'b1);
      #1;
      if (flush) begin flush = 1'b0; flush_done(); end
      if (acked) break;
      if (!flushed && mem_req === 1'b1 && beat_mon == k) begin flush = 1'b1; flushed = 1'b1; end
      g = g + 1;
      if (g > 3000) begin err("fetch_flush_at timeout"); break; end
    end
  endtask

  // request a miss, drop cpu_req exactly in the FILL cycle and flush there
  task automatic flush_fill_noreq(input logic [63:0] a);
    #1; cpu_addr = a; cpu_req = 1'b1;
    wait_fill();
    cpu_req = 1'b0; flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0; flush_done();
  endtask

  task automatic banner(input string s);
    $display("[%0t] ---- %s", $time, s);
  endtask

  function automatic logic [63:0] rnd_addr(input bit wide, input bit bytes);
    logic [63:0] r;
    logic [49:0] tg;
    logic [7:0]  ix;
    logic [5:0]  of;
    r  = {$urandom, $urandom};
    tg = 50'($urandom_range(0, 3));
    if ($urandom_range(0, 9) == 0) tg = r[49:0];
    ix = wide ? 8'($urandom_range(0, 255)) : 8'($urandom_range(0, 7));
    of = bytes ? 6'($urandom_range(0, 63)) : 6'($urandom_range(0, 15) * 4);
    return {tg, ix, of};
  endfunction

  // --------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------
  task automatic test_smoke();
    banner("T1 smoke: miss then hit");
    set_lat(0, 0, 0, 0);
    fetch_chk(64'h0000_0000_8000_0000, 0);
    fetch_chk(64'h0000_0000_8000_0000, 1);
  endtask

  task automatic test_word_offsets();
    logic [63:0] base = 64'h0000_0000_1000_0040;
    banner("T2 all 16 word offsets in a line, forward and backward");
    fetch_chk(base, 0);
    for (int w = 0; w < 16; w++) fetch_chk(base + 64'(w * 4), 1);
    fetch_chk(base + 64, 0);
    for (int w = 15; w >= 0; w--) fetch_chk(base + 64 + 64'(w * 4), 1);
  endtask

  task automatic test_latency_modes();
    banner("T3 memory latency modes (comb slave), poisoned rdata");
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
      fetch_m(a);
      fetch_chk(b, 0);
      fetch_chk(a, 0);
    end
  endtask

  task automatic test_thrash();
    logic [63:0] a = 64'h0000_0000_5000_0080;
    logic [63:0] b = 64'h0000_0000_6000_0080;
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
    fetch_m(64'h0000_0000_1000_3FFC);
    fetch_m(64'hFFFF_FFFF_FFFF_FFFC);
    fetch_m(64'hFFFF_FFFF_FFFF_FFFC);
    fetch_m(64'h8000_0000_0000_0000);
  endtask

  task automatic test_unaligned();
    logic [63:0] b = 64'h0000_0000_E000_0040;
    banner("N7 addr[1:0] != 0: aligned word is returned, tag/index untouched");
    fetch_chk(b + 3, 0);                          // miss taken at a byte offset
    for (int o = 0; o < 64; o++) fetch_chk(b + 64'(o), 1);
  endtask

  task automatic test_flush_basic();
    int c;
    banner("T9 FENCE.I: basic, flush+hit same cycle, flush+miss same cycle");
    set_lat(0, 0, 0, 0);
    for (int i = 0; i < 16; i++) fetch_m(64'h0000_0000_7000_0000 + 64'(i * 64));
    do_flush();
    for (int i = 0; i < 16; i++) fetch_chk(64'h0000_0000_7000_0000 + 64'(i * 64), 0);
    do_flush();
    // flush together with a hitting request (ack in that cycle is legal, line must die)
    fetch_chk(64'h0000_0000_7000_0000, 0);
    #1; cpu_addr = 64'h0000_0000_7000_0000; cpu_req = 1'b1; flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0; cpu_req = 1'b0; flush_done();
    @(posedge clk);
    fetch_chk(64'h0000_0000_7000_0000, 0);
    // flush together with a missing request: flush wins, request then refills normally
    #1; cpu_addr = 64'h0000_0000_7100_0000; cpu_req = 1'b1; flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0;
    wait_ack(c);
    fetch_chk(64'h0000_0000_7100_0000, 1);
  endtask

  // flush held for several cycles with a missing request: no refill may start
  task automatic test_flush_hold();
    int c;
    banner("N8 flush held 4 cycles with a missing request pending");
    set_lat(0, 0, 0, 0);
    fetch_chk(64'h0000_0000_7200_0000, 0);
    #1; cpu_addr = 64'h0000_0000_7300_0040; cpu_req = 1'b1; flush = 1'b1;
    repeat (4) @(posedge clk);
    #1; flush = 1'b0; flush_done();
    wait_ack(c);
    fetch_chk(64'h0000_0000_7300_0040, 1);
    fetch_chk(64'h0000_0000_7200_0000, 0);
  endtask

  task automatic test_flush_mid_refill();
    logic [63:0] a;
    banner("T10 FENCE.I during refill, every beat, 3 latency modes x 2 memory personalities");
    for (int mm = 0; mm < 2; mm++) begin
      mem_mode = mm;
      for (int lm = 0; lm < 3; lm++) begin
        for (int k = 0; k < 8; k++) begin
          set_lat(lm, 1, 3, 0);
          a = 64'h0000_0000_6000_0000 + 64'(((mm * 3 + lm) * 8 + k) * 64);
          fork
            fetch(a);
            flush_at_beat(k);
          join
          fetch_chk(a, 1);       // refill restarted after the flush -> new-epoch line resident
          idle(2);
        end
      end
    end
    mem_mode = 0;
    set_lat(0, 0, 0, 0);
  endtask

  // flush at every FETCH beat x cpu_req {0,1}, in FILL x req {0,1}
  task automatic test_flush_matrix();
    logic [63:0] a;
    int n = 0;
    banner("N6 flush matrix: beat 0..7 / FILL / IDLE  x  cpu_req 0/1");
    for (int lm = 0; lm < 2; lm++) begin
      for (int k = 0; k < 8; k++) begin
        for (int rq = 0; rq < 2; rq++) begin
          set_lat(lm, 1, 0, 0);
          a = 64'h0000_0000_D000_0000 + 64'(n * 64);
          n = n + 1;
          if (rq == 1) begin
            fork
              fetch(a);
              flush_at_beat(k);
            join
            fetch_chk(a, 1);
          end else begin
            fork
              begin #1; cpu_addr = a; cpu_req = 1'b1; @(posedge clk); #1; cpu_req = 1'b0; end
              flush_at_beat(k);
            join
            idle(3);
            wait_dut_idle();
            fetch_chk(a, 0);     // request was dropped, flush killed the refill: nothing cached
          end
          idle(2);
        end
      end
    end
    // FILL x cpu_req 1 (flush + hit in the FILL cycle) and FILL x cpu_req 0
    set_lat(0, 0, 0, 0);
    a = 64'h0000_0000_D100_0000;
    fork
      fetch(a);
      flush_in_fill();
    join
    @(posedge clk);
    fetch_chk(a, 0);
    a = 64'h0000_0000_D100_0040;
    flush_fill_noreq(a);
    idle(3);
    fetch_chk(a, 0);
    set_lat(0, 0, 0, 0);
  endtask

  task automatic test_reset_mid_refill();
    banner("T11 reset in the middle of a refill (async, pin-timed)");
    set_lat(1, 1, 0, 0);
    fetch_m(64'h0000_0000_9000_0000);
    fork
      fetch(64'h0000_0000_9100_0040);
      begin
        wait_beat(3);
        rst_n = 1'b0;
        repeat (2) @(posedge clk);
        #1 rst_n = 1'b1;
      end
    join
    idle(2);
    fetch_chk(64'h0000_0000_9000_0000, 0);
    set_lat(0, 0, 0, 0);
  endtask

  // fill every set, reset at a clock-unaligned time, every set must be empty
  task automatic test_reset_sweep();
    banner("N1 reset clears all 256 sets (async, mid-cycle, cpu_req held high)");
    set_lat(0, 0, 0, 0);
    for (int i = 0; i < 256; i++) fetch_chk(64'h0000_0000_C000_0000 + 64'(i * 64), 0);
    for (int i = 0; i < 256; i++) fetch_chk(64'h0000_0000_C000_0000 + 64'(i * 64) + 60, 1);
    #1; cpu_addr = 64'h0000_0000_C000_0000 + 64'(37 * 64); cpu_req = 1'b1;   // resident line requested
    #3; rst_n = 1'b0;                       // async, mid-cycle, request still high
    repeat (3) @(posedge clk);
    #1; cpu_req = 1'b0;                     // request removed while still in reset
    @(posedge clk);
    #1; rst_n = 1'b1;
    @(posedge clk);
    idle(2);
    for (int i = 0; i < 256; i++) fetch_chk(64'h0000_0000_C000_0000 + 64'(i * 64), 0);
  endtask

  bit kill_stream = 1'b0;

  task automatic test_reset_variants();
    logic [63:0] a;
    banner("N2 reset in FILL, reset during a hit stream, back-to-back resets");
    set_lat(0, 0, 0, 0);
    // (a) reset while the DUT is in FILL
    a = 64'h0000_0000_C800_0000;
    fetch_chk(64'h0000_0000_C900_0040, 0);
    fork
      fetch(a);
      begin
        wait_fill();
        rst_n = 1'b0;
        repeat (2) @(posedge clk);
        #1 rst_n = 1'b1;
      end
    join
    idle(2);
    fetch_chk(64'h0000_0000_C900_0040, 0);      // resident before reset -> must be gone
    // (b) reset in the middle of a 1-word-per-cycle hit stream
    for (int i = 0; i < 16; i++) fetch_chk(64'h0000_0000_CA00_0000 + 64'(i * 4), (i == 0) ? 0 : 1);
    kill_stream = 1'b0;
    fork
      begin
        for (int i = 0; i < 40; i++) begin
          if (kill_stream) break;
          fetch(64'h0000_0000_CA00_0000 + 64'((i % 16) * 4));
        end
      end
      begin
        repeat (6) @(posedge clk);
        #3 rst_n = 1'b0;
        kill_stream = 1'b1;
        repeat (2) @(posedge clk);
        #1 rst_n = 1'b1;
      end
    join
    idle(2);
    wait_dut_idle();
    for (int i = 0; i < 16; i++) fetch_m(64'h0000_0000_CA00_0000 + 64'(i * 4));   // shadow decides hit/miss
    // (c) two resets with almost no run time in between
    fetch_chk(64'h0000_0000_CB00_0000, 0);
    do_reset();
    do_reset();
    fetch_chk(64'h0000_0000_CB00_0000, 0);
  endtask

  // cpu_req low with a wandering address: no traffic, no ack, no state change
  task automatic test_idle_noise();
    int b0, a0;
    banner("N5 idle noise: cpu_req low while cpu_addr wanders (incl. cached lines)");
    #1;                                   // let the monitor finish the previous edge
    b0 = total_beats;
    a0 = n_acks;
    fetch_chk(64'h0000_0000_F000_0000, 0);
    for (int i = 0; i < 400; i++) begin
      #1; cpu_req = 1'b0; cpu_addr = rnd_addr(1'b1, 1'b1);
      @(posedge clk);
    end
    if (total_beats != b0 + 8) err("memory traffic while cpu_req low");
    if (n_acks != a0 + 1)      err("cpu_ack while cpu_req low");
    fetch_chk(64'h0000_0000_F000_0000, 1);
  endtask

  // slave with registered ack/data: must give the same architectural results
  task automatic test_registered_mem();
    banner("N3 registered memory personality (ack <= req & !ack), poisoned rdata");
    mem_mode = 1;
    for (int lm = 0; lm < 3; lm++) begin
      set_lat(lm, 2, 4, 0);
      do_flush();
      test_smoke();
      test_word_offsets();
      test_thrash();
    end
    set_lat(0, 0, 0, 0);
    test_streaming();
    set_lat(2, 0, 5, 0);
    for (int i = 0; i < 300; i++) fetch_m(rnd_addr(1'b1, 1'b1));
    mem_mode = 0;
    set_lat(0, 0, 0, 0);
  endtask

  // Core-like fetch: cpu_req always 1, address changes only after an ack,
  // optional hold (load-use stall) and taken branches to random lines.
  task automatic core_stream(input int n, input int brpct, input int holdpct, input bit wide);
    logic [63:0] pc;
    int c;
    pc = rnd_addr(wide, 1'b0);
    #1; cpu_addr = pc; cpu_req = 1'b1;
    for (int k = 0; k < n; k++) begin
      wait_ack(c);
      #1;
      if ($urandom_range(0, 99) < holdpct) begin
        repeat ($urandom_range(1, 3)) begin
          @(posedge clk); #1;
        end
      end
      if ($urandom_range(0, 99) < brpct) pc = rnd_addr(wide, 1'b0);
      else                                pc = pc + 64'd4;
      cpu_addr = pc;
    end
  endtask

  task automatic test_core_stream();
    banner("N4 core-like fetch stream (req always high, ack-driven PC, stalls, branches)");
    for (int mm = 0; mm < 2; mm++) begin
      mem_mode = mm;
      for (int lm = 0; lm < 3; lm++) begin
        set_lat(lm, 2, 4, 0);
        core_stream(1500, 4, 10, 1'b0);
        core_stream(1500, 10, 0, 1'b1);
      end
    end
    mem_mode = 0;
    set_lat(0, 0, 0, 0);
    idle(3);
  endtask

  // ------------------------------------------------------------------------
  // OPEN ITEM: the address moves while a refill is outstanding.  The current
  // core never does this, but it must, once lost-redirect handling is fixed.
  // Expected behaviour (needs an address latched at miss detection):
  //   * refill of the ORIGINAL line runs to completion and installs that line
  //   * a hit on the new address may be acked meanwhile with correct data
  //   * a miss on the new address is served after the old refill finished
  // ------------------------------------------------------------------------
  task automatic test_redirect_open();
    logic [63:0] a, b, c;
    int cy;
    banner("O1 OPEN ITEM: cpu_addr changes while a refill is outstanding");
    open_mode = 1'b1; open_grp = 1; hold_req = 1'b0;
    set_lat(1, 1, 0, 0);

    // (a) redirect to a different missing line at beat 3
    a = 64'h0000_0000_A000_0000; b = 64'h0000_0000_A100_1040;
    #1; cpu_addr = a; cpu_req = 1'b1;
    wait_beat(3);
    cpu_addr = b;
    wait_ack(cy);
    idle(2); wait_dut_idle();
    fetch_chk(a, 1);
    fetch_chk(b, 1);
    do_reset();

    // (b) redirect to a resident line (hit under miss)
    b = 64'h0000_0000_A200_2000;
    a = 64'h0000_0000_A300_0080;
    fetch_chk(b, 0);
    #1; cpu_addr = a; cpu_req = 1'b1;
    wait_beat(3);
    cpu_addr = b + 64'd8;
    wait_ack(cy);
    idle(2); wait_dut_idle();
    fetch_chk(a, 1);
    fetch_chk(b, 1);
    do_reset();

    // (c) cpu_req dropped and the address scrambled while the refill runs
    a = 64'h0000_0000_A400_00C0; c = 64'h0000_0000_A500_3F00;
    #1; cpu_addr = a; cpu_req = 1'b1;
    wait_beat(3);
    cpu_req = 1'b0; cpu_addr = c;
    wait_dut_idle();
    fetch_chk(a, 1);
    fetch_chk(c, 0);
    do_reset();

    hold_req = 1'b1; open_mode = 1'b0;
    set_lat(0, 0, 0, 0);
  endtask

  // ------------------------------------------------------------------------
  // OPEN ITEM: memory wrapper of the form  ack <= req; rdata <= mem[addr].
  // The DUT keeps mem_req high during the cycle in which it accepts a beat and
  // moves mem_addr on the same edge.  Such a wrapper then answers the NEXT beat
  // with the data of the PREVIOUS address.  Whether this matters depends on the
  // real BRAM wrapper of the SoC (not part of the files provided).
  // ------------------------------------------------------------------------
  task automatic test_naive_wrapper_open();
    banner("O2 OPEN ITEM: memory wrapper with ack <= req (pipelined, no ack gap)");
    open_mode = 1'b1; open_grp = 2;
    mem_mode = 2;
    set_lat(0, 0, 0, 0);
    do_flush();
    fetch_chk(64'h0000_0000_E800_0000, 0);
    for (int w = 0; w < 16; w++) fetch_chk(64'h0000_0000_E800_0000 + 64'(w * 4), 1);
    fetch_chk(64'h0000_0000_E801_0040, 0);
    do_reset();
    mem_mode = 0;
    open_mode = 1'b0;
  endtask

  // req dropped mid-miss with a stable address: fill must still complete cleanly
  task automatic aborted_fetch(input logic [63:0] a, input int k);
    int c = 0;
    #1; cpu_addr = a; cpu_req = 1'b1;
    repeat (k) begin
      @(posedge clk);
      if (cpu_ack === 1'b1) begin c = 1; break; end
    end
    #1; cpu_req = 1'b0;
    wait_dut_idle();
  endtask

  task automatic test_random(input int n);
    int r, kb;
    logic [63:0] a;
    banner("T13 constrained random (flush, mid-refill flush, aborts, resets, streams, byte offsets)");
    for (int i = 0; i < n; i++) begin
      if (i % 200 == 0) begin
        idle(3);
        mem_mode = $urandom_range(0, 1);
        set_lat($urandom_range(0, 2), $urandom_range(1, 4), $urandom_range(1, 6),
                (mem_mode == 0) ? 1'($urandom_range(0, 1)) : 1'b0);
      end
      r = $urandom_range(0, 999);
      a = rnd_addr(($urandom_range(0, 2) == 0), ($urandom_range(0, 2) == 0));
      if      (r < 15)  do_flush();
      else if (r < 35) begin
        if (shadow_hit(a)) fetch_m(a);
        else begin
          kb = $urandom_range(0, 7);
          fetch_flush_at(a, kb);
        end
      end
      else if (r < 60)  aborted_fetch(a, $urandom_range(1, 6));
      else if (r < 110) idle($urandom_range(1, 5));
      else if (r < 125) core_stream($urandom_range(10, 60), 8, 10, 1'b1);
      else if (r < 128) begin
        #1; cpu_req = 1'b0;
        do_reset();
      end
      else              fetch_m(a);
    end
    mem_mode = 0;
    set_lat(0, 0, 0, 0);
  endtask

  // --------------------------------------------------------------------------
  // Coverage closure: a hole is a failure (counters, no simulator features)
  // --------------------------------------------------------------------------
  task automatic report_cov();
    int h;
    h = 0;
    for (int i = 0; i < 256; i++) if (cov_idx[i] == 0) h++;
    if (h != 0) err($sformatf("COVERAGE: %0d of 256 sets never acked", h));
    h = 0;
    for (int i = 0; i < 16; i++) if (cov_word[i] == 0) h++;
    if (h != 0) err($sformatf("COVERAGE: %0d of 16 word offsets never acked", h));
    for (int i = 0; i < 4; i++)
      if (cov_lb[i] == 0) err($sformatf("COVERAGE: addr[1:0]=%0d never acked", i));
    for (int i = 0; i < 10; i++)
      for (int j = 0; j < 2; j++)
        if (cov_fl[i][j] == 0) begin
          if (i < 8)       err($sformatf("COVERAGE: flush bin never hit: FETCH beat %0d x cpu_req=%0d", i, j));
          else if (i == 8) err($sformatf("COVERAGE: flush bin never hit: FILL x cpu_req=%0d", j));
          else             err($sformatf("COVERAGE: flush bin never hit: IDLE x cpu_req=%0d", j));
        end
    if (cov_evict == 0)  err("COVERAGE: no eviction (same index, new tag) observed");
    if (cov_mm[0] == 0 || cov_mm[1] == 0) err("COVERAGE: a memory personality was never used");
    $display(" coverage: sets=%0d words=%0d evictions=%0d flush bins=%0d/20 acks(mem0/mem1)=%0d/%0d",
             256, 16, cov_evict, 20, cov_mm[0], cov_mm[1]);
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
    test_unaligned();
    test_flush_basic();
    test_flush_hold();
    test_flush_mid_refill();
    test_flush_matrix();
    test_reset_mid_refill();
    test_reset_sweep();
    test_reset_variants();
    test_idle_noise();
    test_registered_mem();
    test_core_stream();
    test_redirect_open();
    test_naive_wrapper_open();
    test_random(30000);

    idle(10);
    $display("==================================================");
    $display(" acks=%0d  refills=%0d  beats=%0d", n_acks, n_refills, total_beats);
    report_cov();
    $display(" contract errors = %0d   open-item hits = %0d  (addr-change mid-refill: %0d, naive ack<=req wrapper: %0d)",
             errors, open_hits, open_cnt[1], open_cnt[2]);
    if (errors != 0)         $display(" *** RESULT: FAIL ***");
    else if (open_hits != 0) $display(" *** RESULT: PASS WITH OPEN ITEMS (%0d) - not signed off ***", open_hits);
    else                     $display(" *** RESULT: PASS ***");
    $display("==================================================");
    $finish;
  end

  initial begin
    #900_000_000;
    $display("GLOBAL WATCHDOG TIMEOUT");
    $display(" *** RESULT: FAIL ***");
    $finish;
  end

endmodule

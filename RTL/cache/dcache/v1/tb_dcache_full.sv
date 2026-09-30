// ============================================================================
//  tb_dcache_full.sv  -  Functional verification testbench for dcache.sv
//  Plain SystemVerilog (no covergroups/SVA, no dut.* references), xsim/Verilator.
//
//  Reference model (independent of cache internals):
//    arch : what the CPU must observe (every acked store merged by byte strobes)
//    mem  : the backing store, updated ONLY by beats the DUT actually sends
//  Checks:
//    * every acked read returns arch data (also AMO reads)
//    * every write-back beat carries the CPU-visible data of its own address
//      (catches evicting the wrong line / wrong beat / stale data)
//    * refill/evict beat sequences: 8 beats, ascending, aligned, right line
//    * traffic per request (hit = 1 cycle & no beats, miss = 8 (+8 if dirty))
//    * after FENCE flush completes: mem == arch for every touched address
//    * flush pulse in ANY DUT state must not be lost, and a store acked in the
//      flush cycle must not be lost
//    * AMO to a cached line must stay coherent
//    * reset: all sets invalid, dirty data discarded, no activity in reset
//    * X/Z, ack without req, cpu_err, activity while in reset
//  mem_rdata is poisoned when mem_ack is low. Two memory personalities
//  (combinational ack, registered ack with a gap) and 3 latency modes.
//
//  Result lines:  RESULT: PASS | PASS WITH OPEN ITEMS (n) | FAIL
//  -define STRICT_OPEN (xvlog -d STRICT_OPEN) makes open items fail.
// ============================================================================
`timescale 1ns/1ps

module tb_dcache_full;

  logic        clk   = 1'b0;
  logic        rst_n = 1'b0;
  always #5 clk = ~clk;

  logic [63:0] cpu_addr  = '0;
  logic [63:0] cpu_wdata = '0;
  logic [7:0]  cpu_strb  = '0;
  logic        cpu_req   = 1'b0;
  logic        cpu_we    = 1'b0;
  logic        cpu_amo   = 1'b0;
  logic [63:0] cpu_rdata;
  logic        cpu_ack;
  logic        cpu_err;
  logic [63:0] mem_addr;
  logic [63:0] mem_wdata;
  logic [7:0]  mem_strb;
  logic        mem_req;
  logic        mem_we;
  logic [63:0] mem_rdata;
  logic        mem_ack;
  logic        flush    = 1'b0;

  dcache dut (.*);

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
      if (open_hits <= 14) $display("[%0t] OPEN-ITEM: %s", $time, m);
`endif
    end else begin
      errors++;
      if (errors <= 60) $display("[%0t] ERROR: %s", $time, m);
    end
  endfunction

  // --------------------------------------------------------------------------
  // Memory / architectural models (sparse, keyed by dword address)
  // --------------------------------------------------------------------------
  logic [63:0] mem  [logic [63:0]];
  logic [63:0] arch [logic [63:0]];
  bit          seen [logic [63:0]];
  logic [63:0] touched [$];
  int          mem_ver = 0;                 // bumps on every memory write (comb slave sensitivity)

  function automatic logic [63:0] init_dw(input logic [63:0] a);
    logic [63:0] x;
    x = {a[63:3], 3'b000} ^ 64'h9E3779B97F4A7C15;
    x = x ^ (x >> 30);
    x = x * 64'hBF58476D1CE4E5B9;
    x = x ^ (x >> 27);
    x = x * 64'h94D049BB133111EB;
    x = x ^ (x >> 31);
    return x;
  endfunction

  function automatic logic [63:0] dwk(input logic [63:0] a);
    return {a[63:3], 3'b000};
  endfunction

  function automatic logic [63:0] mem_get(input logic [63:0] a);
    logic [63:0] k;
    k = dwk(a);
    if (mem.exists(k)) return mem[k];
    return init_dw(k);
  endfunction

  function automatic logic [63:0] arch_get(input logic [63:0] a);
    logic [63:0] k;
    k = dwk(a);
    if (arch.exists(k)) return arch[k];
    return init_dw(k);
  endfunction

  function automatic logic [63:0] merge(input logic [63:0] old, input logic [63:0] nw, input logic [7:0] st);
    logic [63:0] r;
    r = old;
    for (int b = 0; b < 8; b++) if (st[b]) r[b*8 +: 8] = nw[b*8 +: 8];
    return r;
  endfunction

  function automatic void touch(input logic [63:0] k);
    if (!seen.exists(k)) begin seen[k] = 1'b1; touched.push_back(k); end
  endfunction

  function automatic void arch_put(input logic [63:0] a, input logic [63:0] d, input logic [7:0] st);
    logic [63:0] k;
    k = dwk(a);
    arch[k] = merge(arch_get(k), d, st);
    touch(k);
  endfunction

  function automatic void mem_put(input logic [63:0] a, input logic [63:0] d, input logic [7:0] st);
    logic [63:0] k;
    k = dwk(a);
    mem[k] = merge(mem_get(k), d, st);
    mem_ver++;
    touch(k);
  endfunction

  function automatic logic [63:0] mem_rd(input logic [63:0] a, input int ver);
    return mem_get(a);
  endfunction

  // reset discards dirty data: what the CPU may see afterwards is memory
  function automatic void arch_sync_to_mem();
    for (int i = 0; i < touched.size(); i++) arch[touched[i]] = mem_get(touched[i]);
  endfunction

  // --------------------------------------------------------------------------
  // Memory slave.  mem_mode 0: comb ack/data.  mem_mode 1: registered ack with a
  // one-cycle gap, data latched when the request is taken.  Poison when !ack.
  // --------------------------------------------------------------------------
  int   mem_mode = 0;
  int   lat_mode = 0;
  int   lat_n    = 2;
  int   lat_max  = 4;
  bit   spur_en  = 1'b0;
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
          data_r <= mem_get(mem_addr);
        end else rc <= rc - 1;
      end else rc <= pick_lat();
    end else ack_r <= 1'b0;
  end

  logic [63:0] junk;
  always @(posedge clk) junk <= {$urandom, $urandom};

  assign mem_ack   = (mem_mode == 1) ? ack_r : ((mem_req === 1'b1) ? (wait_cnt == 0) : spur_q);
  assign mem_rdata = mem_ack ? ((mem_mode == 1) ? data_r : mem_rd(mem_addr, mem_ver)) : junk;

  task automatic set_lat(input int mode, input int n, input int mx, input bit spur);
    lat_mode = mode; lat_n = n; lat_max = mx; spur_en = spur;
  endtask

  // --------------------------------------------------------------------------
  // Shadow of an ideal write-back/write-allocate cache (for traffic expectations)
  // --------------------------------------------------------------------------
  bit          sh_v   [256];
  bit          sh_d   [256];
  logic [49:0] sh_tag [256];

  task automatic sh_clear();
    for (int i = 0; i < 256; i++) begin sh_v[i] = 1'b0; sh_d[i] = 1'b0; end
  endtask

  function automatic bit sh_hit(input logic [63:0] a);
    return sh_v[a[13:6]] && (sh_tag[a[13:6]] == a[63:14]);
  endfunction

  // --------------------------------------------------------------------------
  // Passive monitor
  // --------------------------------------------------------------------------
  bit          armed     = 1'b0;
  bit          hold_req  = 1'b1;    // core promises to hold the request until ack
  bit          amo_flight= 1'b0;    // set by the driver while an AMO op is presented
  logic [63:0] amo_a, amo_d;        // AMO request as issued by the driver
  logic [7:0]  amo_s;
  bit          amo_w;
  int          rd_beats  = 0;       // read beats accepted (refill + AMO read)
  int          wr_beats  = 0;       // write beats accepted (write-back + AMO write)
  int          n_acks    = 0;
  int          wb_beat   = 0;
  int          rf_beat   = 0;
  int          wb_pre    = 0;
  int          rf_pre    = 0;
  logic [63:0] wb_base, rf_base;
  bit          sig_amo;

  int cov_set  [256];
  int cov_dw   [8];
  int cov_lane [8];               // single byte-lane stores
  int cov_ev_d = 0;               // dirty eviction (expected by shadow)
  int cov_ev_c = 0;               // clean eviction
  int cov_amo  [6];               // {rd,wr} x {uncached, cached clean, cached dirty}
  int cov_fl   [3][8][2];         // [0]=WB beat k, [1]=RF beat k  (x cpu_req)
  int cov_fl_i [2];               // flush pulse in idle x cpu_req
  int cov_fl_a = 0;               // flush pulse during AMO
  int cov_mm   [2];

  function automatic bit amo_sig(input bit dummy);
    logic [63:0] m;
    m = 64'h0;
    for (int b = 0; b < 8; b++) m[b*8 +: 8] = amo_s[b] ? 8'hFF : 8'h00;
    return amo_w && (mem_addr[63:3] == amo_a[63:3]) && (mem_strb == amo_s) &&
           ((mem_wdata & m) == (amo_d & m));
  endfunction

  always @(posedge clk) begin
    if (!rst_n) begin
      if (armed && (cpu_ack !== 1'b0 || mem_req !== 1'b0))
        err("activity (cpu_ack/mem_req) while in reset");
      sh_clear();
      wb_beat = 0; rf_beat = 0;
    end else if (armed) begin
      // ---- X / protocol -------------------------------------------------------
      if ($isunknown(cpu_ack) || $isunknown(mem_req)) err("X/Z on cpu_ack or mem_req");
      if (mem_req === 1'b1 && $isunknown(mem_we))     err("X/Z on mem_we");
      if (cpu_ack === 1'b1 && !cpu_we && $isunknown(cpu_rdata)) err("X/Z on cpu_rdata during read ack");
      if (cpu_ack === 1'b1 && !cpu_req) err("cpu_ack asserted without cpu_req");
      if (cpu_err !== 1'b0)             err("cpu_err not 0");
      if (hold_req && !amo_flight && mem_req === 1'b1 && cpu_ack === 1'b1)
        err("cpu_ack while a refill/write-back is in progress");

      sig_amo = amo_flight && (mem_we !== 1'b1 || amo_sig(1'b0));

      // ---- CPU side -------------------------------------------------------------
      if (cpu_ack === 1'b1 && cpu_req) begin
        if (cpu_we) arch_put(cpu_addr, cpu_wdata, cpu_strb);
        else if (cpu_rdata !== arch_get(cpu_addr))
          err($sformatf("read data mismatch addr=%h got=%h exp=%h amo=%0d", cpu_addr, cpu_rdata,
                        arch_get(cpu_addr), cpu_amo));
        if (cpu_amo) begin
          if (sh_hit(cpu_addr)) begin sh_v[cpu_addr[13:6]] = 1'b0; sh_d[cpu_addr[13:6]] = 1'b0; end
        end else begin
          if (!sh_hit(cpu_addr)) sh_d[cpu_addr[13:6]] = 1'b0;
          sh_v[cpu_addr[13:6]]   = 1'b1;
          sh_tag[cpu_addr[13:6]] = cpu_addr[63:14];
          if (cpu_we) sh_d[cpu_addr[13:6]] = 1'b1;
          cov_set[cpu_addr[13:6]]++;
          cov_dw[cpu_addr[5:3]]++;
        end
        if (cpu_we) for (int b = 0; b < 8; b++) if (cpu_strb == (8'h01 << b)) cov_lane[b]++;
        n_acks++;
        cov_mm[mem_mode]++;
      end

      // ---- Memory side ----------------------------------------------------------
      wb_pre = wb_beat; rf_pre = rf_beat;
      if (mem_req === 1'b1 && mem_ack === 1'b1) begin
        if (mem_we === 1'b1) begin
          if (sig_amo) begin
            mem_put(mem_addr, mem_wdata, mem_strb);
            wr_beats++;
          end else begin
            if (mem_addr[2:0] !== 3'b000) err("write-back address not 8-byte aligned");
            if (mem_strb !== 8'hFF)       err("write-back beat with partial strobe");
            if (wb_beat == 0) begin
              wb_base = mem_addr;
              if (mem_addr[5:0] !== 6'b0) err("write-back line base not 64B aligned");
            end else if (mem_addr !== wb_base + 64'(wb_beat * 8))
              err($sformatf("bad write-back addr beat=%0d got=%h exp=%h", wb_beat, mem_addr, wb_base + 64'(wb_beat * 8)));
            if (mem_wdata !== arch_get(mem_addr))
              err($sformatf("write-back data != CPU-visible data addr=%h got=%h exp=%h", mem_addr, mem_wdata, arch_get(mem_addr)));
            mem_put(mem_addr, mem_wdata, 8'hFF);
            wr_beats++;
            wb_beat = (wb_beat == 7) ? 0 : wb_beat + 1;
          end
        end else begin
          rd_beats++;
          if (!sig_amo) begin
            if (mem_addr[2:0] !== 3'b000) err("refill address not 8-byte aligned");
            if (rf_beat == 0) begin
              rf_base = mem_addr;
              if (mem_addr[5:0] !== 6'b0) err("refill line base not 64B aligned");
              if (hold_req && (!cpu_req || mem_addr[63:6] !== cpu_addr[63:6]))
                err($sformatf("refill of a line that was not requested base=%h cpu_addr=%h req=%0d", mem_addr, cpu_addr, cpu_req));
            end else if (mem_addr !== rf_base + 64'(rf_beat * 8))
              err($sformatf("bad refill addr beat=%0d got=%h exp=%h", rf_beat, mem_addr, rf_base + 64'(rf_beat * 8)));
            rf_beat = (rf_beat == 7) ? 0 : rf_beat + 1;
          end
        end
      end

      // ---- flush pulse: coverage of the DUT state it arrives in ------------------
      if (flush === 1'b1) begin
        if (mem_req === 1'b1) begin
          if (sig_amo)              cov_fl_a++;
          else if (mem_we === 1'b1) cov_fl[0][wb_pre][cpu_req ? 1 : 0]++;
          else                      cov_fl[1][rf_pre][cpu_req ? 1 : 0]++;
        end else cov_fl_i[cpu_req ? 1 : 0]++;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Driver
  // --------------------------------------------------------------------------
  localparam logic [63:0] DUMMY = 64'h0000_0000_F7F0_0FC0;   // quiescing read (set 63)

  task automatic banner(input string s);
    $display("[%0t] ---- %s", $time, s);
  endtask

  task automatic do_reset();
    #1; rst_n = 1'b0; cpu_req = 1'b0; flush = 1'b0; amo_flight = 1'b0;
    repeat (3) @(posedge clk);
    armed = 1'b1;
    #1; rst_n = 1'b1;
    @(posedge clk);
    arch_sync_to_mem();
    sh_clear();
  endtask

  task automatic idle(input int n);
    #1; cpu_req = 1'b0; amo_flight = 1'b0;
    repeat (n) @(posedge clk);
  endtask

  // present a request now and wait for the ack (inputs already valid at posedge+1)
  task automatic op_go(input logic [63:0] a, input bit we, input logic [63:0] wd,
                       input logic [7:0] st, input bit amo, output int cyc);
    int c = 0;
    cpu_addr = a; cpu_we = we; cpu_wdata = wd; cpu_strb = st; cpu_amo = amo;
    cpu_req = 1'b1; amo_flight = amo;
    amo_a = a; amo_d = wd; amo_s = st; amo_w = we;
    forever begin
      @(posedge clk);
      c = c + 1;
      if (cpu_ack === 1'b1) break;
      if (c > 400000) begin err("TIMEOUT waiting for cpu_ack"); break; end
    end
    cyc = c;
  endtask

  task automatic op(input logic [63:0] a, input bit we, input logic [63:0] wd,
                    input logic [7:0] st, input bit amo);
    int cyc;
    #1;
    op_go(a, we, wd, st, amo, cyc);
  endtask

  // op with traffic expectation from the shadow (valid only when shadow is in sync)
  task automatic op_chk(input logic [63:0] a, input bit we, input logic [63:0] wd,
                        input logic [7:0] st, input bit amo);
    int cyc, r0, w0, er, ew, s;
    bit ch, dr;
    #1;
    s  = a[13:6];
    ch = sh_hit(a);
    dr = sh_v[s] && sh_d[s];
    if (!amo) begin
      er = ch ? 0 : 8;
      ew = ch ? 0 : (dr ? 8 : 0);
      if (!ch && dr)        cov_ev_d++;
      if (!ch && sh_v[s] && !dr) cov_ev_c++;
    end else begin
      er = we ? 0 : 1;
      ew = ((ch && dr) ? 8 : 0) + (we ? 1 : 0);
      cov_amo[(we ? 3 : 0) + ((ch && dr) ? 2 : (ch ? 1 : 0))]++;
    end
    r0 = rd_beats; w0 = wr_beats;
    op_go(a, we, wd, st, amo, cyc);
    #1;
    if (!amo && ch && cyc != 1) err($sformatf("hit took %0d cycles (addr=%h)", cyc, a));
    if (rd_beats - r0 != er)
      err($sformatf("read beats=%0d expected %0d (addr=%h we=%0d amo=%0d)", rd_beats - r0, er, a, we, amo));
    if (wr_beats - w0 != ew)
      err($sformatf("write beats=%0d expected %0d (addr=%h we=%0d amo=%0d)", wr_beats - w0, ew, a, we, amo));
  endtask

  task automatic rd(input logic [63:0] a);           op_chk(a, 1'b0, 64'h0, 8'h00, 1'b0); endtask
  task automatic wr(input logic [63:0] a, input logic [63:0] d, input logic [7:0] s);
    op_chk(a, 1'b1, d, s, 1'b0);
  endtask
  task automatic amo_rd(input logic [63:0] a);       op_chk(a, 1'b0, 64'h0, 8'h00, 1'b1); endtask
  task automatic amo_wr(input logic [63:0] a, input logic [63:0] d, input logic [7:0] s);
    op_chk(a, 1'b1, d, s, 1'b1);
  endtask

  function automatic logic [63:0] rnd64();
    return {$urandom, $urandom};
  endfunction

  // mem vs arch over every touched dword (call only when all dirty data is flushed)
  task automatic check_mem(input string why);
    int bad = 0;
    for (int i = 0; i < touched.size(); i++) begin
      if (mem_get(touched[i]) !== arch_get(touched[i])) begin
        bad++;
        if (bad <= 3) err($sformatf("%s: memory != CPU-visible data at %h mem=%h exp=%h", why,
                                    touched[i], mem_get(touched[i]), arch_get(touched[i])));
      end
    end
    if (bad > 3) err($sformatf("%s: %0d more mismatching dwords", why, bad - 3));
  endtask

  task automatic wait_quiet();
    int q = 0, g = 0;
    while (q < 4) begin
      @(posedge clk); #1;
      if (mem_req === 1'b1) q = 0; else q = q + 1;
      g = g + 1;
      if (g > 100000) begin err("DUT never went quiet"); break; end
    end
  endtask

  // read that only returns when the DUT is idle again (flush/refill/evict finished)
  task automatic quiesce();
    op(DUMMY, 1'b0, 64'h0, 8'h00, 1'b0);
  endtask

  task automatic do_flush_sync();
    #1; cpu_req = 1'b0; amo_flight = 1'b0; flush = 1'b1;
    @(posedge clk);
    #1; flush = 1'b0;
    sh_clear();
    quiesce();
    check_mem("after FENCE flush");
  endtask

  // ------------------------------------------------------------------------
  // Single-process "event at DUT state" helpers (pins only, no fork)
  //   kind 1: write-back beat k    2: refill beat k    3: AMO beat in flight
  //   act 0: pulse flush   act 1: reset   act 2: change address   act 3: drop cpu_req
  // ------------------------------------------------------------------------
  function automatic bit cond_hit(input int kind, input int k);
    case (kind)
      1: return (mem_req === 1'b1) && (mem_we === 1'b1) && !(amo_flight && amo_sig(1'b0)) && (wb_beat == k);
      2: return (mem_req === 1'b1) && (mem_we !== 1'b1) && !amo_flight && (rf_beat == k);
      3: return (mem_req === 1'b1) && amo_flight;
      default: return 1'b0;
    endcase
  endfunction

  bit ev_fired = 1'b0;

  task automatic op_event(input logic [63:0] a, input bit we, input logic [63:0] wd,
                          input logic [7:0] st, input bit amo,
                          input int kind, input int k, input int act,
                          input logic [63:0] a2);
    bit fired = 1'b0, acked;
    int g = 0;
    ev_fired = 1'b0;
    #1;
    cpu_addr = a; cpu_we = we; cpu_wdata = wd; cpu_strb = st; cpu_amo = amo;
    cpu_req = 1'b1; amo_flight = amo;
    amo_a = a; amo_d = wd; amo_s = st; amo_w = we;
    forever begin
      @(posedge clk);
      acked = (cpu_ack === 1'b1);
      #1;
      if (flush) flush = 1'b0;
      if (acked && !(act == 3 && fired)) break;
      if (act == 3 && fired && !cpu_req) break;
      if (!fired && cond_hit(kind, k)) begin
        fired = 1'b1; ev_fired = 1'b1;
        case (act)
          0: flush = 1'b1;
          1: begin rst_n = 1'b0; cpu_req = 1'b0; amo_flight = 1'b0; end
          2: cpu_addr = a2;
          3: begin cpu_req = 1'b0; cpu_addr = a2; end
        endcase
        if (act == 1) begin
          repeat (2) @(posedge clk);
          #1; rst_n = 1'b1;
          break;
        end
      end
      g = g + 1;
      if (g > 400000) begin err("TIMEOUT in op_event"); break; end
    end
  endtask

  task automatic chk_fired(input string nm);
    if (!ev_fired) begin
      errors++;
      $display("[%0t] ERROR: TB scenario did not trigger: %s", $time, nm);
    end
  endtask

  // flush pulse in the same cycle as a request (hit write must not be lost)
  task automatic flush_with_op(input logic [63:0] a, input bit we, input logic [63:0] wd, input logic [7:0] st);
    int cyc;
    bit acked;
    #1;
    cpu_addr = a; cpu_we = we; cpu_wdata = wd; cpu_strb = st; cpu_amo = 1'b0;
    cpu_req = 1'b1; amo_flight = 1'b0; flush = 1'b1;
    @(posedge clk);
    acked = (cpu_ack === 1'b1);
    #1; flush = 1'b0;
    cyc = 0;
    while (!acked) begin
      @(posedge clk);
      cyc = cyc + 1;
      acked = (cpu_ack === 1'b1);
      if (cyc > 400000) begin err("TIMEOUT in flush_with_op"); break; end
    end
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
    of = 6'($urandom_range(0, 7)) * 6'd8 + (bytes ? 6'($urandom_range(0, 7)) : 6'd0);
    return {tg, ix, of};
  endfunction

  function automatic logic [7:0] rnd_strb(input logic [2:0] off);
    case ($urandom_range(0, 5))
      0: return 8'hFF;
      1: return 8'h01 << off;
      2: return 8'($urandom_range(1, 255));
      3: return 8'h0F << (off[2] ? 4 : 0);
      4: return 8'h03 << {off[2:1], 1'b0};
      default: return 8'hFF;
    endcase
  endfunction

  // --------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------
  task automatic test_smoke();
    banner("D1 smoke: read miss (8 beats), read hit, write hit, read back");
    set_lat(0, 0, 0, 0);
    rd(64'h0000_0000_8000_0000);
    rd(64'h0000_0000_8000_0000);
    wr(64'h0000_0000_8000_0008, 64'h1122_3344_5566_7788, 8'hFF);
    rd(64'h0000_0000_8000_0008);
  endtask

  task automatic test_offsets();
    logic [63:0] base = 64'h0000_0000_1000_0040;
    banner("D2 all 8 dwords of a line, byte offsets, forward and backward");
    rd(base);
    for (int w = 0; w < 8; w++) wr(base + 64'(w * 8), rnd64(), 8'hFF);
    for (int w = 7; w >= 0; w--) rd(base + 64'(w * 8) + 64'(w % 8));
    for (int w = 0; w < 8; w++) rd(base + 64'(w * 8));
  endtask

  task automatic test_strobes();
    logic [63:0] a = 64'h0000_0000_1100_0100;
    banner("D3 byte strobes: every lane alone, then random masks, merged with older data");
    rd(a);
    wr(a, 64'hAAAA_AAAA_AAAA_AAAA, 8'hFF);
    for (int b = 0; b < 8; b++) begin
      wr(a, rnd64(), 8'h01 << b);
      rd(a);
    end
    for (int i = 0; i < 300; i++) begin
      wr(a + 64'(i % 8) * 8 + 64'($urandom_range(0, 7)), rnd64(), 8'($urandom_range(0, 255)));
      rd(a + 64'(i % 8) * 8);
    end
    wr(a, rnd64(), 8'h00);                     // no lane enabled
    rd(a);
  endtask

  task automatic test_write_alloc();
    logic [63:0] a = 64'h0000_0000_2200_0180;
    banner("D4 write miss = write-allocate (8 refill beats), then hits");
    wr(a + 8, 64'hDEAD_BEEF_0BAD_F00D, 8'hFF);      // miss: refill then write
    rd(a + 8);
    rd(a);                                          // rest of the line came from memory
    wr(a + 16, rnd64(), 8'h0F);
    rd(a + 16);
  endtask

  task automatic test_evict();
    logic [63:0] a = 64'h0000_0000_3000_0200;
    logic [63:0] b = 64'h0000_0000_5000_0200;      // same set (index), different tag
    logic [63:0] c = 64'h0000_0000_7000_0200;
    banner("D5 dirty eviction: 8 write-back beats to the OLD address, then refill; clean eviction: none");
    wr(a, 64'h0123_4567_89AB_CDEF, 8'hFF);          // A dirty
    wr(a + 24, 64'hFEDC_BA98_7654_3210, 8'hFF);
    rd(b);                                          // evict A (16 beats total), refill B
    rd(a);                                          // A again: miss, B is clean -> no write-back
    rd(a + 24);
    rd(c);                                          // clean eviction of A -> no write-back
    wr(c + 8, rnd64(), 8'h3C);
    wr(b + 8, rnd64(), 8'hFF);                      // dirty C evicted, B allocated and written
    rd(c + 8);
  endtask

  task automatic test_index_sweep();
    banner("D6 index sweep: write and read all 256 sets, then conflict all sets");
    for (int i = 0; i < 256; i++) wr(64'h0000_0000_4000_0000 + 64'(i * 64) + 8, rnd64(), 8'hFF);
    for (int i = 0; i < 256; i++) rd(64'h0000_0000_4000_0000 + 64'(i * 64) + 8);
    for (int i = 0; i < 256; i++) rd(64'h0000_0000_4004_0000 + 64'(i * 64));      // evict all dirty lines
    for (int i = 0; i < 256; i++) rd(64'h0000_0000_4000_0000 + 64'(i * 64) + 8);  // must come back from memory
  endtask

  task automatic test_tag_bits();
    logic [63:0] a, b;
    banner("D7 each tag bit [63:14] must be compared (dirty A evicted by A^bit)");
    a = 64'h0000_0000_4000_0040;
    for (int bt = 14; bt < 64; bt++) begin
      b = a ^ (64'd1 << bt);
      wr(a, rnd64(), 8'hFF);
      rd(b);                                        // conflict: must miss, evict A
      rd(a);                                        // must miss again, get A's data from memory
    end
  endtask

  task automatic test_thrash();
    logic [63:0] a = 64'h0000_0000_5000_0280;
    logic [63:0] b = 64'h0000_0000_6000_0280;
    banner("D8 dirty ping-pong A B A B (every access a miss with write-back)");
    wr(a, rnd64(), 8'hFF); wr(b, rnd64(), 8'hFF);
    wr(a, rnd64(), 8'hFF); wr(b, rnd64(), 8'hFF);
    rd(a); rd(b);
    rd(b);
  endtask

  task automatic test_streaming();
    banner("D9 streaming: 4KB sequential read/modify/write");
    for (int w = 0; w < 512; w++) rd(64'h0000_0000_2000_0000 + 64'(w * 8));
    for (int w = 0; w < 512; w++) wr(64'h0000_0000_2000_0000 + 64'(w * 8), rnd64(), 8'hFF);
    for (int w = 0; w < 512; w++) rd(64'h0000_0000_2000_0000 + 64'(w * 8));
  endtask

  task automatic test_edges();
    banner("D10 address edges");
    wr(64'h0, 64'h1, 8'hFF); rd(64'h0);
    wr(64'h0000_0000_1000_3FF8, 64'h2, 8'hFF); rd(64'h0000_0000_1000_3FF8);
    wr(64'hFFFF_FFFF_FFFF_FFF8, 64'h3, 8'hFF); rd(64'hFFFF_FFFF_FFFF_FFF8);
    wr(64'h8000_0000_0000_0000, 64'h4, 8'hFF); rd(64'h8000_0000_0000_0000);
  endtask

  task automatic test_amo();
    logic [63:0] a, b;
    banner("D11 AMO pass-through: uncached, cached clean, cached dirty, read and write");
    // uncached: exactly one beat, no allocation
    a = 64'h0000_0000_A000_0040;
    amo_rd(a);
    amo_wr(a, 64'h1111_2222_3333_4444, 8'hFF);
    amo_rd(a);
    rd(a);                                          // not allocated: must miss (8 beats)
    // cached clean line, AMO write, then normal read must see the AMO value
    a = 64'h0000_0000_A100_0080;
    rd(a);
    amo_rd(a + 16);                                  // AMO read of a cached CLEAN line
    rd(a);                                           // AMO invalidated it: allocate again
    amo_wr(a + 8, 64'h5555_6666_7777_8888, 8'hFF);
    rd(a + 8);
    rd(a);
    // cached dirty line: AMO read must see the store, AMO write must survive
    a = 64'h0000_0000_A200_00C0;
    wr(a, 64'h9999_AAAA_BBBB_CCCC, 8'hFF);
    amo_rd(a);
    wr(a + 16, 64'h0101_0202_0303_0404, 8'hFF);
    amo_wr(a + 16, 64'hEEEE_FFFF_0000_1111, 8'h0F);
    rd(a + 16);
    rd(a);
    // AMO to a set holding a different line: no interference
    b = 64'h0000_0000_A300_00C0;
    wr(a + 32, 64'h7777_7777_7777_7777, 8'hFF);
    amo_rd(b);
    rd(a + 32);
    // byte offsets on AMO
    amo_wr(64'h0000_0000_A400_0003, 64'h0000_0000_00AB_0000, 8'h04);
    amo_rd(64'h0000_0000_A400_0000);
    do_flush_sync();
  endtask

  task automatic test_flush_basic();
    banner("D12 FENCE flush: dirty lines all over, flush, memory must equal CPU view");
    set_lat(0, 0, 0, 0);
    for (int i = 0; i < 40; i++) wr(64'h0000_0000_7000_0000 + 64'(i * 64) + 8, rnd64(), 8'hFF);
    for (int i = 0; i < 20; i++) wr(64'h0000_0000_7000_0000 + 64'(i * 64) + 16, rnd64(), 8'h5A);
    do_flush_sync();
    do_flush_sync();                                // nothing dirty: must still be clean
    for (int i = 0; i < 40; i++) rd(64'h0000_0000_7000_0000 + 64'(i * 64) + 8);   // flushed = invalidated: misses
    // flush with everything clean, then dirty again
    wr(64'h0000_0000_7000_0008, rnd64(), 8'hFF);
    do_flush_sync();
  endtask

  task automatic test_flush_all_sets();
    banner("D13 flush with all 256 sets dirty (both memory personalities)");
    for (int mm = 0; mm < 2; mm++) begin
      mem_mode = mm;
      set_lat(mm == 0 ? 0 : 1, 1, 0, 0);
      for (int i = 0; i < 256; i++) wr(64'h0000_0000_7800_0000 + 64'(i * 64) + 8*(i % 8), rnd64(), 8'hFF);
      do_flush_sync();
    end
    mem_mode = 0; set_lat(0, 0, 0, 0);
  endtask

  task automatic test_flush_write_same_cycle();
    logic [63:0] a = 64'h0000_0000_7A00_0040;
    banner("D14 store acked in the flush cycle must not be lost; read in the flush cycle");
    rd(a);
    wr(a, 64'h0000_0000_0000_0001, 8'hFF);          // line is dirty and cached
    flush_with_op(a + 8, 1'b1, 64'hCAFE_F00D_1234_5678, 8'hFF);   // hit write + flush
    sh_clear();
    do_flush_sync();                                // the store must have survived: now in memory
    rd(a + 8);
    flush_with_op(a + 8, 1'b0, 64'h0, 8'h00);       // read + flush
    sh_clear();
    quiesce();
    check_mem("read in flush cycle");
    // flush pulse together with a MISSING write
    flush_with_op(64'h0000_0000_7B00_0100, 1'b1, 64'h1357_9BDF_0246_8ACE, 8'hFF);
    sh_clear();
    do_flush_sync();
  endtask

  task automatic flush_twice_at(input int k);
    int g = 0;
    for (int i = 0; i < 6; i++) wr(64'h0000_0000_6E00_0000 + 64'(i * 64) + 64'(k * 64 * 8), rnd64(), 8'hFF);
    #1; cpu_req = 1'b0; amo_flight = 1'b0; flush = 1'b1;
    @(posedge clk); #1; flush = 1'b0;
    forever begin
      @(posedge clk); #1;
      if (mem_req === 1'b1 && mem_we === 1'b1 && wb_beat == k) begin
        flush = 1'b1;
        @(posedge clk); #1; flush = 1'b0;
        break;
      end
      g = g + 1;
      if (g > 100000) begin err("flush_twice_at: no write-back beat seen"); break; end
    end
    sh_clear();
    quiesce();
    check_mem("second flush pulse inside a flush write-back beat");
  endtask

  task automatic test_flush_busy();
    logic [63:0] a, b;
    int n = 0;
    banner("D15 flush pulse while the DUT is busy: write-back beat k / refill beat k / AMO / during flush");
    for (int mm = 0; mm < 2; mm++) begin
      mem_mode = mm;
      for (int lm = 0; lm < 2; lm++) begin
        set_lat(lm, 1, 0, 0);
        for (int k = 0; k < 8; k++) begin
          // pulse during a write-back beat (dirty victim) -- the request needs the eviction
          a = 64'h0000_0000_7C00_0000 + 64'(n * 64 * 3);
          b = a ^ 64'h0000_0000_0010_0000;
          n = n + 1;
          wr(a, rnd64(), 8'hFF);
          op_event(b, 1'b0, 64'h0, 8'h00, 1'b0, 1, k, 0, 64'h0);
          chk_fired("D15 write-back beat");
          sh_clear();
          quiesce();
          check_mem("flush pulse in write-back beat");
          // pulse during a refill beat, with a store pending on that line
          a = 64'h0000_0000_7D00_0000 + 64'(n * 64 * 3);
          n = n + 1;
          wr(a ^ 64'h0000_0000_0040_0000, rnd64(), 8'hFF);       // some dirty data elsewhere
          op_event(a, 1'b0, 64'h0, 8'h00, 1'b0, 2, k, 0, 64'h0);
          chk_fired("D15 refill beat");
          sh_clear();
          quiesce();
          check_mem("flush pulse in refill beat");
        end
        // pulse during an AMO (pass-through) with dirty lines around
        a = 64'h0000_0000_7E00_0040 + 64'(lm * 64);
        wr(a + 64'h0000_0000_0100_0000, rnd64(), 8'hFF);
        op_event(a, 1'b1, rnd64(), 8'hFF, 1'b1, 3, 0, 0, 64'h0);
        chk_fired("D15 AMO");
        sh_clear();
        quiesce();
        check_mem("flush pulse during AMO");
      end
    end
    // second pulse on every write-back beat of a running flush (CPU idle)
    mem_mode = 0; set_lat(1, 1, 0, 0);
    for (int k = 0; k < 8; k++) flush_twice_at(k);
    // second pulse while the flush is still running
    mem_mode = 0; set_lat(0, 0, 0, 0);
    for (int i = 0; i < 64; i++) wr(64'h0000_0000_7F00_0000 + 64'(i * 64), rnd64(), 8'hFF);
    #1; cpu_req = 1'b0; flush = 1'b1;
    @(posedge clk); #1; flush = 1'b0;
    repeat (5) @(posedge clk);
    #1; flush = 1'b1;
    @(posedge clk); #1; flush = 1'b0;
    sh_clear();
    quiesce();
    check_mem("second flush pulse while flushing");
    do_flush_sync();
    mem_mode = 0; set_lat(0, 0, 0, 0);
  endtask

  task automatic test_reset_sweep();
    banner("D16 reset clears all 256 sets and drops dirty data (async, mid-cycle)");
    set_lat(0, 0, 0, 0);
    for (int i = 0; i < 256; i++) wr(64'h0000_0000_C000_0000 + 64'(i * 64) + 8, rnd64(), 8'hFF);
    #1; cpu_addr = 64'h0000_0000_C000_0000 + 64'(37 * 64) + 8; cpu_we = 1'b0; cpu_req = 1'b1;
    #3; rst_n = 1'b0;
    repeat (3) @(posedge clk);
    #1; cpu_req = 1'b0;
    @(posedge clk);
    #1; rst_n = 1'b1;
    @(posedge clk);
    arch_sync_to_mem();
    sh_clear();
    for (int i = 0; i < 256; i++) rd(64'h0000_0000_C000_0000 + 64'(i * 64) + 8);   // all miss, memory data
  endtask

  task automatic test_reset_variants();
    logic [63:0] a, b;
    banner("D17 reset in write-back beat / refill beat / AMO / flush");
    set_lat(1, 1, 0, 0);
    for (int k = 0; k < 8; k += 3) begin
      a = 64'h0000_0000_C800_0000 + 64'(k * 64);
      b = a ^ 64'h0000_0000_0020_0000;
      wr(a, rnd64(), 8'hFF);
      op_event(b, 1'b0, 64'h0, 8'h00, 1'b0, 1, k, 1, 64'h0);       // reset during write-back beat k
      chk_fired("D17 reset wb");
      arch_sync_to_mem(); sh_clear();
      rd(a); rd(b);
      wr(a, rnd64(), 8'hFF);
      op_event(b, 1'b0, 64'h0, 8'h00, 1'b0, 2, k, 1, 64'h0);       // reset during refill beat k
      chk_fired("D17 reset refill");
      arch_sync_to_mem(); sh_clear();
      rd(a); rd(b);
    end
    a = 64'h0000_0000_CA00_0040;
    wr(a, rnd64(), 8'hFF);
    op_event(a + 64'h0000_0000_0100_0000, 1'b1, rnd64(), 8'hFF, 1'b1, 3, 0, 1, 64'h0);   // reset during AMO
    chk_fired("D17 reset AMO");
    arch_sync_to_mem(); sh_clear();
    rd(a);
    // reset during the flush scan
    for (int i = 0; i < 20; i++) wr(64'h0000_0000_CB00_0000 + 64'(i * 64), rnd64(), 8'hFF);
    #1; cpu_req = 1'b0; flush = 1'b1;
    @(posedge clk); #1; flush = 1'b0;
    repeat (30) @(posedge clk);
    #3; rst_n = 1'b0;
    repeat (2) @(posedge clk);
    #1; rst_n = 1'b1;
    @(posedge clk);
    arch_sync_to_mem(); sh_clear();
    for (int i = 0; i < 20; i++) rd(64'h0000_0000_CB00_0000 + 64'(i * 64));
    do_reset(); do_reset();
    set_lat(0, 0, 0, 0);
  endtask

  task automatic test_idle_noise();
    int b0, r0, a0;
    banner("D18 idle noise: cpu_req low while addr/data/we/amo wander, no traffic, no ack");
    rd(64'h0000_0000_F000_0000);
    wr(64'h0000_0000_F000_0008, rnd64(), 8'hFF);
    #1;
    b0 = wr_beats; r0 = rd_beats; a0 = n_acks;
    for (int i = 0; i < 400; i++) begin
      #1; cpu_req = 1'b0; cpu_addr = rnd_addr(1'b1, 1'b1); cpu_wdata = rnd64(); cpu_strb = 8'($urandom);
      cpu_we = 1'($urandom); cpu_amo = 1'($urandom); amo_flight = 1'b0;
      @(posedge clk);
    end
    if (wr_beats != b0 || rd_beats != r0) err("memory traffic while cpu_req low");
    if (n_acks != a0)                    err("cpu_ack while cpu_req low");
    #1; cpu_amo = 1'b0;
    rd(64'h0000_0000_F000_0008);
  endtask

  task automatic test_registered_mem();
    banner("D19 registered memory personality (ack with gap), all latency modes");
    mem_mode = 1;
    for (int lm = 0; lm < 3; lm++) begin
      set_lat(lm, 2, 4, 0);
      test_smoke();
      test_offsets();
      test_evict();
      test_thrash();
      test_amo();
    end
    set_lat(0, 0, 0, 0);
    mem_mode = 0;
  endtask

  task automatic test_latency_modes();
    banner("D20 comb memory latency modes incl. spurious mem_ack while mem_req is low");
    for (int m = 0; m < 4; m++) begin
      case (m)
        0: set_lat(0, 0, 0, 0);
        1: set_lat(1, 1, 0, 0);
        2: set_lat(1, 4, 0, 0);
        3: set_lat(2, 0, 6, 1);
      endcase
      test_evict();
      test_amo();
      wr(64'h0000_0000_3300_0000 + 64'(m * 64'h4000), rnd64(), 8'hFF);
      rd(64'h0000_0000_3300_0000 + 64'(m * 64'h4000) ^ 64'h0000_0000_0010_0000);
    end
    set_lat(0, 0, 0, 0);
  endtask

  // Core-like: request always followed by the next after the ack, optional bubbles
  task automatic core_stream(input int n, input int amopct, input int bubblepct, input bit wide);
    logic [63:0] a, prev;
    bit w;
    logic [7:0] s;
    prev = rnd_addr(wide, 1'b0);
    for (int k = 0; k < n; k++) begin
      case ($urandom_range(0, 9))
        0, 1, 2, 3, 4: a = prev + 64'($urandom_range(0, 16)) * 8;       // same line / next lines
        5, 6:          a = {prev[63:14] ^ 50'($urandom_range(1, 3)), prev[13:0]};   // conflict
        default:       a = rnd_addr(wide, 1'b0);
      endcase
      prev = a;
      w = 1'($urandom_range(0, 1));
      s = rnd_strb(a[2:0]);
      if ($urandom_range(0, 99) < bubblepct) idle($urandom_range(1, 3));
      if ($urandom_range(0, 99) < amopct)
        op(a, w, rnd64(), s, 1'b1);
      else
        op(a, w, rnd64(), s, 1'b0);
    end
  endtask

  task automatic test_core_stream();
    banner("D21 core-like stream (req held until ack, bubbles, AMO, conflicts), 2 memories x 3 latencies");
    for (int mm = 0; mm < 2; mm++) begin
      mem_mode = mm;
      for (int lm = 0; lm < 3; lm++) begin
        set_lat(lm, 2, 4, 0);
        core_stream(1200, 3, 15, 1'b0);
        core_stream(1200, 3, 0, 1'b1);
      end
    end
    mem_mode = 0; set_lat(0, 0, 0, 0);
    idle(3);
    do_flush_sync();
  endtask

  // ------------------------------------------------------------------------
  // OPEN ITEM: the CPU does not hold its request while the cache is busy.
  // The core in riscv_core.sv does exactly that today: EX/MEM is not stalled on
  // dmem_ack, so dmem_addr changes one cycle after a miss starts.  The cache
  // must at least stay CONSISTENT (finish the started refill/evict on the
  // original line, never write a wrong line to memory, never hang).
  // ------------------------------------------------------------------------
  task automatic test_hold_open();
    logic [63:0] a, b, c;
    banner("O1 OPEN ITEM: request changes / drops while a refill or write-back is running");
    open_mode = 1'b1; hold_req = 1'b0;
    set_lat(1, 1, 0, 0);

    // (a) address moves to another missing line during a refill
    a = 64'h0000_0000_9000_0040; b = 64'h0000_0000_9100_1080;
    op_event(a, 1'b0, 64'h0, 8'h00, 1'b0, 2, 3, 2, b);
    chk_fired("O1 case");
    wait_quiet(); quiesce(); do_flush_sync(); rd(a); rd(b);

    // (b) request dropped and address scrambled during a refill
    a = 64'h0000_0000_9200_00C0; c = 64'h0000_0000_9300_3F00;
    op_event(a, 1'b0, 64'h0, 8'h00, 1'b0, 2, 3, 3, c);
    chk_fired("O1 case");
    wait_quiet(); idle(3); quiesce(); do_flush_sync(); rd(a);

    // (c) address moves to another set during a write-back (dirty victim)
    a = 64'h0000_0000_9400_0100; b = a ^ 64'h0000_0000_0020_0000; c = 64'h0000_0000_9500_0500;
    wr(a, rnd64(), 8'hFF);
    wr(a + 64'h0000_0000_0000_0040, rnd64(), 8'hFF);
    op_event(b, 1'b0, 64'h0, 8'h00, 1'b0, 1, 3, 2, c);
    chk_fired("O1 case");
    wait_quiet(); quiesce(); do_flush_sync(); rd(a); rd(a + 64);

    // (d) request dropped during a write-back
    a = 64'h0000_0000_9600_0200; b = a ^ 64'h0000_0000_0020_0000;
    wr(a, rnd64(), 8'hFF);
    op_event(b, 1'b1, rnd64(), 8'hFF, 1'b0, 1, 5, 3, 64'h0000_0000_9700_0000);
    chk_fired("O1 case");
    wait_quiet(); idle(3); quiesce(); do_flush_sync(); rd(a);

    // (e) AMO request dropped after it was taken (no hang, no lost line)
    a = 64'h0000_0000_9800_0040;
    wr(a, rnd64(), 8'hFF);
    op_event(a + 64'h0000_0000_0100_0000, 1'b0, 64'h0, 8'h00, 1'b1, 3, 0, 3, 64'h0);
    chk_fired("O1 case");
    wait_quiet(); idle(3); quiesce(); do_flush_sync(); rd(a);

    // (f) AMO to a dirty cached line, request dropped during the write-back beats
    a = 64'h0000_0000_9900_0080;
    wr(a, rnd64(), 8'hFF);
    op_event(a, 1'b0, 64'h0, 8'h00, 1'b1, 1, 2, 3, 64'h0);
    chk_fired("O1 case");
    wait_quiet(); idle(3); quiesce(); do_flush_sync(); rd(a);

    hold_req = 1'b1; open_mode = 1'b0;
    set_lat(0, 0, 0, 0);
  endtask

  task automatic test_random(input int n);
    int r;
    logic [63:0] a, b;
    banner("D22 constrained random (ops, AMO, flush idle/busy, resets, streams, latency/memory changes)");
    for (int i = 0; i < n; i++) begin
      if (i % 200 == 0) begin
        idle(3);
        mem_mode = $urandom_range(0, 1);
        set_lat($urandom_range(0, 2), $urandom_range(1, 4), $urandom_range(1, 6),
                (mem_mode == 0) ? 1'($urandom_range(0, 1)) : 1'b0);
      end
      r = $urandom_range(0, 999);
      a = rnd_addr(($urandom_range(0, 2) == 0), 1'b1);
      if      (r < 10)  do_flush_sync();
      else if (r < 22) begin
        b = rnd_addr(1'b0, 1'b0);
        op_event(a, 1'b0, 64'h0, 8'h00, 1'b0,
                 $urandom_range(1, 2), $urandom_range(0, 7), 0, 64'h0);
        sh_clear(); quiesce();
        if (ev_fired) check_mem("random busy flush");
      end
      else if (r < 60)  idle($urandom_range(1, 5));
      else if (r < 75)  core_stream($urandom_range(10, 60), 3, 10, 1'b1);
      else if (r < 78)  begin
        #1; cpu_req = 1'b0;
        do_reset();
      end
      else if (r < 100) op(a, 1'($urandom_range(0, 1)), rnd64(), rnd_strb(a[2:0]), 1'b1);
      else              op(a, 1'($urandom_range(0, 1)), rnd64(), rnd_strb(a[2:0]), 1'b0);
    end
    mem_mode = 0;
    set_lat(0, 0, 0, 0);
    idle(3);
    do_flush_sync();
  endtask

  // --------------------------------------------------------------------------
  // Coverage closure: a hole is a failure
  // --------------------------------------------------------------------------
  task automatic report_cov();
    int h, nb;
    h = 0;
    for (int i = 0; i < 256; i++) if (cov_set[i] == 0) h++;
    if (h != 0) err($sformatf("COVERAGE: %0d of 256 sets never acked", h));
    h = 0;
    for (int i = 0; i < 8; i++) if (cov_dw[i] == 0) h++;
    if (h != 0) err($sformatf("COVERAGE: %0d of 8 dwords never acked", h));
    for (int i = 0; i < 8; i++)
      if (cov_lane[i] == 0) err($sformatf("COVERAGE: single-lane store %0d never acked", i));
    if (cov_ev_d == 0) err("COVERAGE: no dirty eviction");
    if (cov_ev_c == 0) err("COVERAGE: no clean eviction");
    for (int i = 0; i < 6; i++)
      if (cov_amo[i] == 0) err($sformatf("COVERAGE: AMO bin %0d never hit (0..2 read, 3..5 write; uncached/clean/dirty)", i));
    nb = 0;
    for (int k = 0; k < 8; k++) begin
      if (cov_fl[0][k][0] == 0) err($sformatf("COVERAGE: flush pulse in write-back beat %0d with cpu_req=0", k));
      if (cov_fl[0][k][1] == 0) err($sformatf("COVERAGE: flush pulse in write-back beat %0d with cpu_req=1", k));
      if (cov_fl[1][k][1] == 0) err($sformatf("COVERAGE: flush pulse in refill beat %0d with cpu_req=1", k));
      nb += (cov_fl[0][k][0] != 0) + (cov_fl[0][k][1] != 0) + (cov_fl[1][k][1] != 0);
    end
    if (cov_fl_i[0] == 0 || cov_fl_i[1] == 0) err("COVERAGE: flush pulse in idle x cpu_req bin missing");
    if (cov_fl_a == 0) err("COVERAGE: flush pulse during AMO never hit");
    if (cov_mm[0] == 0 || cov_mm[1] == 0) err("COVERAGE: a memory personality was never used");
    $display(" coverage: sets=256 dwords=8 lanes=8 evict(d/c)=%0d/%0d flush bins=%0d/24+3 acks(mem0/mem1)=%0d/%0d",
             cov_ev_d, cov_ev_c, nb, cov_mm[0], cov_mm[1]);
  endtask

  // --------------------------------------------------------------------------
  initial begin
    @(posedge clk);
    do_reset();

    test_smoke();
    test_offsets();
    test_strobes();
    test_write_alloc();
    test_evict();
    test_index_sweep();
    test_tag_bits();
    test_thrash();
    test_streaming();
    test_edges();
    test_amo();
    test_flush_basic();
    test_flush_all_sets();
    test_flush_write_same_cycle();
    test_flush_busy();
    test_reset_sweep();
    test_reset_variants();
    test_idle_noise();
    test_latency_modes();
    test_registered_mem();
    test_core_stream();
    test_hold_open();
    test_random(6000);

    idle(10);
    $display("==================================================");
    $display(" acks=%0d  read beats=%0d  write beats=%0d  touched dwords=%0d", n_acks, rd_beats, wr_beats, touched.size());
    report_cov();
    $display(" contract errors = %0d   open-item hits = %0d  (request changes/drops while busy)", errors, open_hits);
    if (errors != 0)         $display(" *** RESULT: FAIL ***");
    else if (open_hits != 0) $display(" *** RESULT: PASS WITH OPEN ITEMS (%0d) - not signed off ***", open_hits);
    else                     $display(" *** RESULT: PASS ***");
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

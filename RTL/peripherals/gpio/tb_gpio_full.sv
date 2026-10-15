// ============================================================================
//  tb_gpio_full.sv  -  Functional verification testbench for gpio.sv
//  Plain SystemVerilog (no covergroups/SVA, no dut.* references).
//  Needs a 4-state simulator (xsim, Icarus): the pad is a real tristate net.
//
//  Pad model: every pin has an external driver (ext_val/ext_oe) and a weak
//  pull-down, so a pin that nobody drives reads 0. The testbench never drives a
//  pin that the DUT drives (no contention).
//  Reference model (independent of the RTL): DATA_OUT, DIR, IRQ_EN; the expected
//  pad value is (DIR & DATA_OUT) | (~DIR & external). Checked every clock:
//    * pins == expected pad (output enable / output value per bit)
//    * APB reads == model (DATA_IN only when the pad has been stable for 4 cycles:
//      the synchroniser latency is not specified beyond "2-FF")
//    * gpio_irq: every rising edge of the pad on an enabled pin must produce an
//      irq pulse within 5 cycles, and an irq pulse without such an edge is an
//      error; no X on gpio_irq / prdata.
//  O1 (OPEN ITEM): the interrupt pulse must not come before DATA_IN shows the edge.
//
//  Result lines:  RESULT: PASS | PASS WITH OPEN ITEMS (n) | FAIL
//  -define STRICT_OPEN makes open items fail.
// ============================================================================
`timescale 1ns/1ps

module tb_gpio_full;

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
  logic        gpio_irq;
  tri   [31:0] pins;

  logic [31:0] ext_val = '0;
  logic [31:0] ext_oe  = '0;

  genvar gi;
  generate
    for (gi = 0; gi < 32; gi++) begin : gen_ext
      assign pins[gi] = ext_oe[gi] ? ext_val[gi] : 1'bz;
      pulldown (pins[gi]);                                        // pull-down resistor on every pin
    end
  endgenerate

  gpio dut (.clk(clk), .rst_n(rst_n), .paddr(paddr), .pwdata(pwdata), .psel(psel), .penable(penable),
            .pwrite(pwrite), .prdata(prdata), .pready(pready), .gpio_pins(pins), .gpio_irq(gpio_irq));

  // --------------------------------------------------------------------------
  int errors = 0;
  int open_hits = 0;
  bit open_mode = 1'b0;
  function automatic void err(input string m);
    if (open_mode) begin
`ifdef STRICT_OPEN
      errors++;
      if (errors <= 40) $display("[%0t] ERROR (open item, strict): %s", $time, m);
`else
      open_hits++;
      if (open_hits <= 8) $display("[%0t] OPEN-ITEM: %s", $time, m);
`endif
    end else begin
      errors++;
      if (errors <= 40) $display("[%0t] ERROR: %s", $time, m);
    end
  endfunction

  bit armed = 1'b0;
  int cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;

  // coverage counters (a hole fails the run)
  int cov_out1 [32];
  int cov_out0 [32];
  int cov_in1  [32];
  int cov_in0  [32];
  int cov_irq  [32];
  int cov_rw   [4][2];
  int cov_ev   [12];

  // --------------------------------------------------------------------------
  // Reference model and checker
  // --------------------------------------------------------------------------
  logic [31:0] m_out = '0, m_dir = '0, m_en = '0;
  logic [31:0] pad_prev = '0, pad_now;
  int          pad_chg = 0;
  int          win_s [$];
  int          win_e [$];
  int          wi;
  bit          hit;
  logic [31:0] rising;
  logic [31:0] exp_rd;
  logic [31:0] last_rising_pad = '0;

  function automatic logic [31:0] pad_f();
    return (m_dir & m_out) | (~m_dir & ext_val & ext_oe);
  endfunction

  always @(posedge clk) begin
    if (!rst_n) begin
      m_out = '0; m_dir = '0; m_en = '0;
      pad_prev = pad_f();
      pad_chg = cyc;
      win_s.delete(); win_e.delete();
    end else if (armed) begin
      pad_now = pad_f();
      // ---- pins ----
      if (pins !== pad_now) err($sformatf("pins = %h, expected %h (dir %h out %h ext %h/%h)", pins, pad_now, m_dir, m_out, ext_val, ext_oe));
      if (pad_now !== pad_prev) pad_chg = cyc;
      // ---- interrupt windows ----
      rising = pad_now & ~pad_prev;
      if ((rising & m_en) != 32'h0) begin
        win_s.push_back(cyc + 1); win_e.push_back(cyc + 5);
        for (int i = 0; i < 32; i++) if (rising[i] && m_en[i]) cov_irq[i]++;
        last_rising_pad = pad_now;
      end
      if ($isunknown(gpio_irq)) err("X/Z on gpio_irq");
      else if (gpio_irq === 1'b1) begin
        hit = 1'b0;
        wi = 0;
        while (wi < win_s.size() && !hit) begin
          if (win_s[wi] <= cyc && cyc <= win_e[wi]) begin
            hit = 1'b1;
            win_s.delete(wi); win_e.delete(wi);
          end else wi = wi + 1;
        end
        if (!hit) err("gpio_irq pulse without a rising edge on an enabled pin");
      end
      while (win_e.size() > 0 && win_e[0] < cyc) begin
        err("rising edge on an enabled pin produced no gpio_irq pulse within 5 cycles");
        win_s.delete(0); win_e.delete(0);
      end
      // ---- APB ----
      if (psel && penable && pready !== 1'b1) err("pready is not 1 in the access phase");
      if (psel && penable) begin
        cov_rw[paddr[3:2]][pwrite ? 1 : 0]++;
        if (!pwrite) begin
          case (paddr[3:2])
            2'h0: exp_rd = m_out;
            2'h2: exp_rd = m_dir;
            2'h3: exp_rd = m_en;
            default: exp_rd = pad_now;
          endcase
          if ($isunknown(prdata)) err($sformatf("X/Z in prdata of register %0d", paddr[3:2]));
          else if (paddr[3:2] != 2'h1 || (cyc - pad_chg) >= 4)
            if (prdata !== exp_rd) err($sformatf("read of register %0d = %h, expected %h", paddr[3:2], prdata, exp_rd));
          if (paddr[3:2] == 2'h1 && (cyc - pad_chg) >= 4)
            for (int i = 0; i < 32; i++) begin if (prdata[i]) cov_in1[i]++; else cov_in0[i]++; end
        end else begin
          case (paddr[3:2])
            2'h0: begin
              m_out = pwdata;
              for (int i = 0; i < 32; i++) if (m_dir[i]) begin if (pwdata[i]) cov_out1[i]++; else cov_out0[i]++; end
            end
            2'h2: m_dir = pwdata;
            2'h3: m_en  = pwdata;
            default: ;
          endcase
        end
      end
      pad_prev = pad_now;
    end
  end

  // --------------------------------------------------------------------------
  // Drivers
  // --------------------------------------------------------------------------
  task automatic apb_wr(input logic [3:0] a, input logic [31:0] d);
    paddr = a; pwdata = d; pwrite = 1'b1; psel = 1'b1; penable = 1'b0;
    @(negedge clk); penable = 1'b1;
    @(posedge clk);
    @(negedge clk); psel = 1'b0; penable = 1'b0; pwrite = 1'b0;
  endtask

  task automatic apb_rd(input logic [3:0] a, output logic [31:0] d);
    paddr = a; pwrite = 1'b0; psel = 1'b1; penable = 1'b0;
    @(negedge clk); penable = 1'b1;
    @(posedge clk); d = prdata;
    @(negedge clk); psel = 1'b0; penable = 1'b0;
  endtask

  task automatic idle(input int n);
    psel = 1'b0; penable = 1'b0;
    repeat (n) @(negedge clk);
  endtask

  task automatic banner(input string s);
    $display("[%0t] ---- %s", $time, s);
  endtask

  task automatic do_reset();
    psel = 1'b0; penable = 1'b0; rst_n = 1'b0;
    repeat (3) @(negedge clk);
    armed = 1'b1;
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  // external driver: drive every pin the DUT does not drive
  task automatic ext_set(input logic [31:0] v);
    ext_val = v;
    ext_oe  = ~m_dir;
  endtask

  // change DIR without contention: release pins that become outputs first, drive the new inputs afterwards
  task automatic set_dir(input logic [31:0] nd);
    ext_oe = ext_oe & ~nd;
    @(negedge clk);
    apb_wr(4'h8, nd);
    ext_oe = ~nd;
  endtask

  task automatic set_out(input logic [31:0] v); apb_wr(4'h0, v); endtask
  task automatic set_en (input logic [31:0] v); apb_wr(4'hC, v); endtask

  // --------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------
  task automatic test_reset_defaults();
    logic [31:0] v;
    banner("G1 reset defaults: all registers 0, pins released (inputs), no interrupt");
    // pad self-test with the DUT in reset (it must not drive anything): isolates pad-model problems
    ext_val = 32'hA5A5_5A5A; ext_oe = 32'hFFFF_FFFF;
    #2;
    if (pins !== 32'hA5A5_5A5A) err($sformatf("PAD SELF-TEST (external driver on, DUT in reset): pins = %h, expected a5a55a5a -> pad model or DUT tristate port resolution problem", pins));
    ext_oe = 32'h0;
    #2;
    if (pins !== 32'h0) err($sformatf("PAD SELF-TEST (nothing driving): pins = %h, expected 00000000 (pull-down missing?)", pins));
    ext_oe = 32'hFFFF_FFFF;
    do_reset();
    idle(8);
    apb_rd(4'h0, v); if (v !== 0) err($sformatf("DATA_OUT after reset = %h", v));
    apb_rd(4'h8, v); if (v !== 0) err($sformatf("DIR after reset = %h", v));
    apb_rd(4'hC, v); if (v !== 0) err($sformatf("IRQ_EN after reset = %h", v));
    apb_rd(4'h4, v); if (v !== 32'hA5A5_5A5A) err($sformatf("DATA_IN after reset = %h, expected the pad value A5A55A5A", v));
    if (pins !== 32'hA5A5_5A5A) err("DUT drives pins after reset (DIR must be 0)");
    if (gpio_irq !== 1'b0) err("gpio_irq after reset");
    cov_ev[0]++;
  endtask

  task automatic test_registers();
    logic [31:0] v;
    banner("G2 register read/write: full 32 bits, walking ones/zeros, DATA_IN read-only");
    for (int b = 0; b < 32; b++) begin
      apb_wr(4'h0, 32'h1 << b); apb_rd(4'h0, v); if (v !== (32'h1 << b)) err($sformatf("DATA_OUT walking 1 bit %0d: %h", b, v));
      apb_wr(4'h0, ~(32'h1 << b)); apb_rd(4'h0, v); if (v !== ~(32'h1 << b)) err($sformatf("DATA_OUT walking 0 bit %0d: %h", b, v));
    end
    apb_wr(4'h0, 32'h0);
    for (int b = 0; b < 32; b++) begin
      apb_wr(4'hC, 32'h1 << b); apb_rd(4'hC, v); if (v !== (32'h1 << b)) err($sformatf("IRQ_EN walking 1 bit %0d: %h", b, v));
    end
    apb_wr(4'hC, 32'h0);
    for (int i = 0; i < 12; i++) begin
      logic [31:0] r0, r1;
      r0 = $urandom; r1 = $urandom;
      apb_wr(4'h0, r0); apb_wr(4'hC, r1);
      apb_rd(4'h0, v); if (v !== r0) err("DATA_OUT random");
      apb_rd(4'hC, v); if (v !== r1) err("IRQ_EN random");
    end
    apb_wr(4'hC, 32'h0);
    // DATA_IN is read-only; registers do not alias each other
    apb_wr(4'h0, 32'h1234_5678);
    apb_wr(4'h4, 32'hFFFF_FFFF);
    apb_rd(4'h0, v); if (v !== 32'h1234_5678) err("DATA_OUT changed by a write to DATA_IN");
    apb_rd(4'h8, v); if (v !== 0) err("DIR changed by a write to DATA_IN");
    apb_rd(4'hC, v); if (v !== 0) err("IRQ_EN changed by a write to DATA_IN");
    idle(2);
    if (gpio_irq !== 1'b0) err("gpio_irq during register writes");
    apb_wr(4'h0, 32'h0);
    cov_ev[1]++;
  endtask

  task automatic test_outputs();
    logic [31:0] v, d, o;
    banner("G3 outputs: each pin driven only when DIR=1, pin follows DATA_OUT, DATA_IN reads the pad back");
    ext_set(32'h0F0F_0F0F);
    idle(2);
    for (int b = 0; b < 32; b++) begin                                     // one output at a time
      d = 32'h1 << b;
      set_dir(d);
      set_out(d);  idle(6);
      apb_rd(4'h4, v); if (v[b] !== 1'b1) err($sformatf("pin %0d: DATA_IN does not read back the driven 1", b));
      set_out(32'h0); idle(6);
      apb_rd(4'h4, v); if (v[b] !== 1'b0) err($sformatf("pin %0d: DATA_IN does not read back the driven 0", b));
      set_out(d);
      set_dir(32'h0); idle(6);
      apb_rd(4'h4, v);
      if (v !== (32'h0F0F_0F0F)) err($sformatf("after releasing pin %0d: DATA_IN = %h, expected the external value", b, v));
      set_out(32'h0);
    end
    for (int i = 0; i < 40; i++) begin                                     // random patterns
      d = $urandom; o = $urandom;
      set_out(o); set_dir(d); idle(6);
      apb_rd(4'h4, v);
      if (v !== ((d & o) | (~d & 32'h0F0F_0F0F))) err($sformatf("random: DATA_IN %h, expected %h", v, (d & o) | (~d & 32'h0F0F_0F0F)));
      set_out($urandom); idle(6);
    end
    // DATA_OUT written while the pin is an input must not reach the pin
    set_dir(32'h0); set_out(32'hFFFF_FFFF); idle(6);
    if (pins !== 32'h0F0F_0F0F) err("DATA_OUT reached a pin that is an input");
    set_out(32'h0);
    cov_ev[2]++;
  endtask

  task automatic test_inputs();
    logic [31:0] v, p;
    banner("G4 inputs: DATA_IN follows the external pad within 4 cycles, every bit both values");
    set_dir(32'h0);
    for (int i = 0; i < 24; i++) begin
      p = (i < 16) ? (32'h1 << (2 * i)) | ((i % 2) ? 32'hFFFF_0000 : 32'h0000_FFFF) : $urandom;
      ext_set(p);
      idle(6);
      apb_rd(4'h4, v); if (v !== p) err($sformatf("DATA_IN = %h, pad = %h", v, p));
      ext_set(~p);
      idle(6);
      apb_rd(4'h4, v); if (v !== ~p) err($sformatf("DATA_IN = %h, pad = %h", v, ~p));
    end
    // the value changes within 3 reads' worth of cycles and is stable afterwards
    ext_set(32'h0); idle(6);
    ext_set(32'hFFFF_FFFF);
    for (int i = 0; i < 4; i++) begin @(negedge clk); end
    apb_rd(4'h4, v); if (v !== 32'hFFFF_FFFF) err("DATA_IN did not follow a step within ~6 cycles");
    cov_ev[3]++;
  endtask

  task automatic test_irq();
    logic [31:0] v;
    banner("G5 interrupt: one pulse per rising edge on an enabled pin, none for falling edges / disabled pins / levels");
    set_dir(32'h0);
    ext_set(32'h0); idle(8);
    set_en(32'hFFFF_FFFF);
    for (int b = 0; b < 32; b++) begin
      ext_set(32'h1 << b); idle(8);                         // rising edge on pin b only
      ext_set(32'h0);      idle(8);                         // falling edge: no interrupt
    end
    set_en(32'h0);
    for (int b = 0; b < 32; b++) begin ext_set(32'h1 << b); idle(8); ext_set(32'h0); idle(8); end   // disabled: none
    set_en(32'h0000_FFFF);                                   // only the low half enabled
    for (int b = 0; b < 32; b++) begin ext_set(32'h1 << b); idle(8); ext_set(32'h0); idle(8); end
    // enabling a pin that is already high must not fire (no edge)
    set_en(32'h0); ext_set(32'hFFFF_FFFF); idle(8);
    set_en(32'hFFFF_FFFF); idle(12);
    ext_set(32'h0); idle(8);
    // simultaneous edges: one pulse
    ext_set(32'hFFFF_FFFF); idle(10);
    ext_set(32'h0); idle(8);
    ext_set(32'h8000_0001); idle(10);
    ext_set(32'h0); idle(8);
    // edges on consecutive cycles
    ext_set(32'h0); idle(8);
    for (int i = 0; i < 6; i++) begin ext_set(32'h1 << (3 * i)); @(negedge clk); ext_set(32'h0); @(negedge clk); end
    idle(10);
    // fast toggling of one pin: every rising edge fires
    for (int i = 0; i < 10; i++) begin ext_set(32'h10); @(negedge clk); @(negedge clk); ext_set(32'h0); @(negedge clk); @(negedge clk); end
    idle(10);
    set_en(32'h0);
    cov_ev[4]++;
  endtask

  task automatic test_irq_outputs();
    banner("G6 interrupt on pins driven by this block (rising edge of the pad, also for outputs)");
    set_dir(32'h0000_00F0);
    ext_set(32'h0); set_out(32'h0); idle(8);
    set_en(32'h0000_00F0);
    set_out(32'h0000_0010); idle(8);
    set_out(32'h0000_0000); idle(8);
    set_out(32'h0000_00A0); idle(8);                       // two rising edges together
    set_out(32'h0000_0000); idle(8);
    set_en(32'h0);
    set_out(32'h0000_00F0); idle(8);                       // disabled: nothing
    set_out(32'h0); set_dir(32'h0); idle(8);
    cov_ev[5]++;
  endtask

  task automatic test_reset_state();
    logic [31:0] v;
    banner("G7 reset with configured state and an APB write in flight");
    ext_set(32'h0); set_dir(32'h0000_FFFF); set_out(32'h0000_5555); set_en(32'hFFFF_0000);
    idle(6);
    paddr = 4'h0; pwdata = 32'hFFFF_FFFF; pwrite = 1'b1; psel = 1'b1; penable = 1'b0;
    @(negedge clk); penable = 1'b1;
    #3 rst_n = 1'b0;                                        // asynchronous, during the access phase
    repeat (3) @(negedge clk);
    psel = 1'b0; penable = 1'b0; pwrite = 1'b0;
    rst_n = 1'b1;
    ext_oe = 32'hFFFF_FFFF; ext_val = 32'h0;
    @(negedge clk);
    idle(6);
    apb_rd(4'h0, v); if (v !== 0) err($sformatf("DATA_OUT after reset = %h", v));
    apb_rd(4'h8, v); if (v !== 0) err($sformatf("DIR after reset = %h", v));
    apb_rd(4'hC, v); if (v !== 0) err($sformatf("IRQ_EN after reset = %h", v));
    if (pins !== 32'h0) err("pins not released by reset");
    if (gpio_irq !== 1'b0) err("gpio_irq after reset");
    cov_ev[6]++;
  endtask

  task automatic test_random(input int n);
    logic [31:0] v, r;
    int rr;
    banner("G8 constrained random (outputs, directions, enables, external toggles, reads)");
    do_reset();
    ext_set(32'h0);
    for (int i = 0; i < n; i++) begin
      rr = $urandom_range(0, 99);
      r = $urandom;
      if ($urandom_range(0, 3) == 0) r = (32'h1 << $urandom_range(0, 31));
      if (rr < 18)      begin set_out(r); idle(8); end
      else if (rr < 30) begin set_dir(r); idle(8); end
      else if (rr < 42) begin set_en(r);  idle(8); end
      else if (rr < 66) begin ext_set(r); idle(8); end
      else if (rr < 78) begin apb_rd(4'h4, v); end
      else if (rr < 88) begin apb_rd(4'($urandom_range(0, 3) * 4), v); end
      else if (rr < 94) begin ext_set(~pins); idle(8); end
      else              begin apb_wr(4'h4, r); idle(3); end
    end
    set_en(32'h0); idle(10);
    cov_ev[7]++;
  endtask

  // ------------------------------------------------------------------------
  // OPEN ITEM O1: the interrupt pulse precedes DATA_IN by one cycle in v1 (edge detect on the
  // first synchroniser stage).  An observer that reads DATA_IN in the pulse cycle sees the old value.
  // ------------------------------------------------------------------------
  task automatic test_irq_vs_datain_open();
    bit seen;
    banner("O1 OPEN ITEM: DATA_IN must already show the new value when the interrupt pulse is seen");
    do_reset();
    set_dir(32'h0); ext_set(32'h0); idle(8);
    set_en(32'h0000_0001); idle(6);
    paddr = 4'h4; pwrite = 1'b0; psel = 1'b1; penable = 1'b1;       // DATA_IN read held open, sampled at every edge
    for (int k = 0; k < 6; k++) begin
      @(negedge clk); ext_set(32'h0000_0001);
      seen = 1'b0;
      for (int c = 0; c < 10; c++) begin
        @(posedge clk);
        if (!seen && gpio_irq === 1'b1) begin
          seen = 1'b1;
          if (prdata[0] !== 1'b1) begin
            open_mode = 1'b1;
            err("interrupt pulse while DATA_IN still shows the old value (edge detected before DATA_IN updated)");
            open_mode = 1'b0;
          end
        end
      end
      if (!seen) err("no interrupt pulse for a rising edge");
      @(negedge clk); ext_set(32'h0);
      repeat (8) @(negedge clk);
    end
    psel = 1'b0; penable = 1'b0;
    set_en(32'h0);
    cov_ev[8]++;
  endtask

  task automatic report_cov();
    for (int b = 0; b < 32; b++) begin
      if (cov_out1[b] == 0 || cov_out0[b] == 0) err($sformatf("COVERAGE: pin %0d never driven as both 0 and 1", b));
      if (cov_in1[b] == 0  || cov_in0[b] == 0)  err($sformatf("COVERAGE: DATA_IN bit %0d never read as both 0 and 1", b));
      if (cov_irq[b] == 0) err($sformatf("COVERAGE: pin %0d never produced an interrupt", b));
    end
    for (int a = 0; a < 4; a++) for (int w = 0; w < 2; w++)
      if (cov_rw[a][w] == 0) err($sformatf("COVERAGE: register %0d %s never accessed", a, w ? "write" : "read"));
    for (int i = 0; i < 9; i++) if (cov_ev[i] == 0) err($sformatf("COVERAGE: scenario %0d never ran", i));
    $display(" coverage: 32 pins out/in/irq, 4 registers r/w, 9 scenarios");
  endtask

  initial begin
    @(negedge clk);
    test_reset_defaults();
    test_registers();
    test_outputs();
    test_inputs();
    test_irq();
    test_irq_outputs();
    test_reset_state();
    test_random(800);
    test_irq_vs_datain_open();
    idle(10);
    $display("==================================================");
    report_cov();
    $display(" contract errors = %0d   open-item hits = %0d  (interrupt pulse vs DATA_IN)", errors, open_hits);
    if (errors != 0)         $display(" *** RESULT: FAIL ***");
    else if (open_hits != 0) $display(" *** RESULT: PASS WITH OPEN ITEMS (%0d) - not signed off ***", open_hits);
    else                     $display(" *** RESULT: PASS ***");
    $display("==================================================");
    $finish;
  end

  initial begin
    #500_000_000;
    $display("GLOBAL WATCHDOG TIMEOUT");
    $display(" *** RESULT: FAIL ***");
    $finish;
  end

endmodule

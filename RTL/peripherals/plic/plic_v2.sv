// ============================================================
//  plic.sv (v2)  -  Platform-Level Interrupt Controller
//  Supports NSOURCES interrupt sources (default 64), NCONTEXTS contexts
//  (default 2: context 0 = M-mode, 1 = S-mode)
//  Base address: 0x0C00_0000 (QEMU-compatible); addr is the 28-bit offset
//
//  0x000000 + 4*src              : priority[src]     (3 bits, src 1..N-1; src 0 does not exist)
//  0x001000 + 4*word             : pending[word]     (read-only)
//  0x002000 + 0x80*ctx + 4*word  : enable[ctx][word] (bit 0 = source 0 reads 0)
//  0x200000 + 0x1000*ctx         : threshold[ctx]    (3 bits)
//  0x200004 + 0x1000*ctx         : claim (read) / complete (write source id)
//  everything else (or an index beyond NSOURCES / NCONTEXTS): reads 0, writes ignored
//
//  Interrupt sources are LEVEL sensitive through a gateway per source:
//   * a source that is asserted sets its pending bit (also a short pulse);
//   * reading the claim register returns the highest priority pending+enabled
//     source above the threshold (ties: lowest id, 0 = none), clears its pending
//     bit and marks it "in flight" - the gateway ignores that source until it
//     is completed;
//   * writing the source id to the claim register completes it (ignored if the
//     source is not enabled for that context); if the source is still asserted
//     it becomes pending again.
//
//  Bus: one access is accepted in the first cycle with req && !ack; ack (and
//  rdata) follow one cycle later and ack is one cycle wide.
//
//  Changes against v1:
//   P1  the edge detector compared two copies of the same flop (irq_sync and
//       irq_sync_d were both loaded from irq_sources), so irq_edge was always 0
//       and no interrupt could ever become pending. v2: 2-flop synchroniser and
//       a level gateway with claim/complete.
//   P2  address decode: the priority branch (addr[27:22]==0) also matched the
//       pending (0x1000) and enable (0x2000) and threshold/claim (0x200000)
//       regions, so those registers were unreachable, and their own decode
//       constants were wrong. v2 decodes each region correctly.
//   P3  the claim logic picked the LOWEST numbered eligible source, ignoring
//       priority. v2: highest priority, ties to the lowest id.
//   P4  no bounds checks: priority source / enable word / pending word / context
//       beyond the implemented range returned X, unmapped addresses returned the
//       previous read data. v2: they read 0.
//   P5  priority[0] and enable bit 0 (source 0 does not exist) were writable.
//   P6  claim double execution: a master that holds req until ack executed a
//       claim twice and silently swallowed a second interrupt (side effect on
//       every req cycle). v2: accept once.
// ============================================================
`timescale 1ns/1ps
`default_nettype none

module plic #(
    parameter int NSOURCES  = 64,
    parameter int NCONTEXTS = 2
)(
    input  logic        clk,
    input  logic        rst_n,

    // Interrupt source inputs
    input  logic [NSOURCES-1:0] irq_sources,

    // Memory-mapped register interface
    input  logic [27:0] addr,
    input  logic [31:0] wdata,
    input  logic        req,
    input  logic        we,
    output logic [31:0] rdata,
    output logic        ack,

    // Interrupt outputs
    output logic [NCONTEXTS-1:0] eip   // external interrupt pending per context
);

localparam int NW  = (NSOURCES + 31) / 32;
localparam int IDW = $clog2(NSOURCES);

// mask of implemented enable bits: sources 1 .. NSOURCES-1
localparam logic [NW*32-1:0] ENMASK = (((NW*32)'(1) << NSOURCES) - (NW*32)'(1)) & ~(NW*32)'(1);

logic [NSOURCES-1:0][2:0]           pty;
logic [NCONTEXTS-1:0][2:0]          threshold;
logic [NCONTEXTS-1:0][NW*32-1:0]    enable;
logic [NSOURCES-1:0]                pending, infl;
logic [NSOURCES-1:0]                irq_s1, irq_s2;
logic [NCONTEXTS-1:0][IDW-1:0]      claim;

// ---- synchroniser ----
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        irq_s1 <= '0;
        irq_s2 <= '0;
    end else begin
        irq_s1 <= irq_sources;
        irq_s2 <= irq_s1;
    end
end

// ---- highest-priority eligible source per context (P3) ----
genvar gc;
generate
    for (gc = 0; gc < NCONTEXTS; gc++) begin : gen_ctx
        logic [2:0]       thr_c;
        logic [NW*32-1:0] en_c;
        assign thr_c = threshold[gc];
        assign en_c  = enable[gc];
        always_comb begin
            logic [2:0] bp;
            eip[gc]   = 1'b0;
            claim[gc] = '0;
            bp        = 3'h0;
            for (int s = NSOURCES-1; s >= 1; s--) begin
                if (pending[s] && en_c[s] && (pty[s] > thr_c) && (pty[s] >= bp)) begin
                    eip[gc]   = 1'b1;
                    claim[gc] = IDW'(s);
                    bp        = pty[s];
                end
            end
        end
    end
endgenerate

// ---- decode ----
logic acc;
assign acc = req && !ack;

logic        sel_pri, sel_pend, sel_en, sel_tc;
int          pri_src, pend_w, en_ctx, en_w, tc_ctx;
assign sel_pri  = (addr[27:12] == 16'h0000);
assign sel_pend = (addr[27:7]  == 21'h000020);                                   // 0x1000..0x107F
assign sel_en   = (addr[27:12] >= 16'h0002) && (addr[27:12] < 16'h0200);         // 0x2000..0x1FFFFF
assign sel_tc   = (addr[27:12] >= 16'h0200);                                     // 0x200000..
assign pri_src  = int'(addr[11:2]);
assign pend_w   = int'(addr[6:2]);
assign en_ctx   = int'((addr - 28'h2000) >> 7);
assign en_w     = int'(addr[6:2]);
assign tc_ctx   = int'(addr[27:12] - 16'h0200);

logic [NW*32-1:0] pend_x;
assign pend_x = (NW*32)'(pending);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pty       <= '0;
        threshold <= '0;
        enable    <= '0;
        pending   <= '0;
        infl      <= '0;
        rdata     <= 32'h0;
        ack       <= 1'b0;
    end else begin
        ack <= acc;

        // gateway: an asserted source that is not in flight is pending
        for (int s = 1; s < NSOURCES; s++)
            if (irq_s2[s] && !infl[s]) pending[s] <= 1'b1;

        if (acc) begin
            rdata <= 32'h0;                                   // P4: unmapped / out of range reads 0
            if (sel_pri) begin
                if (pri_src >= 1 && pri_src < NSOURCES) begin                 // P5: source 0 does not exist
                    rdata <= {29'h0, pty[pri_src]};
                    if (we) pty[pri_src] <= wdata[2:0];
                end
            end else if (sel_pend) begin
                if (pend_w < NW) rdata <= pend_x[pend_w*32 +: 32];
            end else if (sel_en) begin
                if (en_ctx < NCONTEXTS && en_w < NW) begin
                    rdata <= enable[en_ctx][en_w*32 +: 32];
                    if (we) enable[en_ctx][en_w*32 +: 32] <= wdata & ENMASK[en_w*32 +: 32];
                end
            end else if (sel_tc) begin
                if (tc_ctx < NCONTEXTS) begin
                    if (addr[11:0] == 12'h000) begin
                        rdata <= {29'h0, threshold[tc_ctx]};
                        if (we) threshold[tc_ctx] <= wdata[2:0];
                    end else if (addr[11:0] == 12'h004) begin
                        if (!we) begin
                            // claim: returns the id, clears pending, source is now in flight
                            rdata <= 32'(claim[tc_ctx]);
                            if (claim[tc_ctx] != '0) begin
                                pending[claim[tc_ctx]] <= 1'b0;
                                infl   [claim[tc_ctx]] <= 1'b1;
                            end
                        end else begin
                            // complete: only for a source that is enabled for this context
                            if (wdata >= 32'd1 && wdata < 32'(NSOURCES))
                                if (enable[tc_ctx][wdata[IDW-1:0]]) infl[wdata[IDW-1:0]] <= 1'b0;
                        end
                    end
                end
            end
        end
    end
end

endmodule
`default_nettype wire

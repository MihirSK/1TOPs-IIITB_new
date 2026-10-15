// ============================================================
//  plic.sv  –  Platform-Level Interrupt Controller
//  Supports 64 interrupt sources, 2 contexts (M+S per hart)
//  Base address: 0x0C00_0000  (QEMU-compatible)
// ============================================================
`timescale 1ns/1ps
//`default_nettype none

module plic #(
    parameter int NSOURCES = 64,
    parameter int NCONTEXTS = 2    // context 0 = M-mode, 1 = S-mode
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

// PLIC memory map:
// 0x000000 + 4*src    : priority[src]       (1 word each)
// 0x001000 + 4*(src/32) : pending[word]     (read-only)
// 0x002000 + 0x80*ctx + 4*word : enable[ctx][word]
// 0x200000 + 0x1000*ctx        : threshold[ctx]
// 0x200004 + 0x1000*ctx        : claim/complete[ctx]

logic [2:0]  pty    [NSOURCES];
logic [2:0]  threshold   [NCONTEXTS];
logic [NSOURCES-1:0] pending;
logic [NSOURCES-1:0] enable [NCONTEXTS];
logic [NSOURCES-1:0] irq_sync_d, irq_sync;

// Edge-detect (level-to-edge conversion via sync)
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        irq_sync_d <= '0;
        irq_sync   <= '0;
    end else begin
        irq_sync_d <= irq_sources;
        irq_sync   <= irq_sources;
    end
end

// Pending is set on rising edge of source
logic [NSOURCES-1:0] irq_edge;
assign irq_edge = irq_sync & ~irq_sync_d;

// Per-context claim registers
logic [$clog2(NSOURCES)-1:0] claim [NCONTEXTS];

// ---- Highest-pty pending interrupt per context ----
always_comb begin
    for (int c = 0; c < NCONTEXTS; c++) begin
        eip[c] = 1'b0;
        claim[c] = '0;
        for (int s = NSOURCES-1; s >= 1; s--) begin
            if (pending[s] && enable[c][s] && pty[s] > threshold[c]) begin
                eip[c]   = 1'b1;
                claim[c] = s[$clog2(NSOURCES)-1:0];
            end
        end
    end
end

// ---- Register access ----
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int s = 0; s < NSOURCES; s++) 
            pty[s] <= 3'h0;
        for (int c = 0; c < NCONTEXTS; c++) begin
            threshold[c] <= 3'h0;
            enable[c]    <= '0;
        end
        pending <= '0;
        rdata   <= 32'h0;
        ack     <= 1'b0;
    end else begin
        ack <= 1'b0;

        // Set pending on edge
        for (int s = 1; s < NSOURCES; s++)
            if (irq_edge[s]) pending[s] <= 1'b1;

        if (req) begin
            ack <= 1'b1;
            if (addr[27:22] == 6'h0) begin
                // pty region
                int src;
                src = addr[11:2];
                rdata <= {29'h0, pty[src]};
                if (we) pty[src] <= wdata[2:0];
            end else if (addr[27:12] == 16'h0001) begin
                // Pending region
                int word;
                word = addr[6:2];
                rdata <= pending[word*32 +: 32];
            end else if (addr[27:16] == 12'h002) begin
                // Enable region
                int ctx, word;
                ctx  = addr[11:7];
                word = addr[4:2];
                if (ctx < NCONTEXTS) begin
                    rdata <= enable[ctx][word*32 +: 32];
                    if (we) enable[ctx][word*32 +: 32] <= wdata;
                end
            end else if (addr[27:22] == 6'h08) begin
                // Threshold / claim
                int ctx;
                ctx = addr[15:12];
                if (addr[2] == 1'b0) begin
                    // Threshold
                    rdata <= {29'h0, threshold[ctx]};
                    if (we) threshold[ctx] <= wdata[2:0];
                end else begin
                    // Claim / complete
                    rdata <= {26'h0, claim[ctx]};
                    if (!we) begin
                        // Claim: clear pending
                        pending[claim[ctx]] <= 1'b0;
                    end else begin
                        // Complete: no action needed (pending re-sets on next edge)
                    end
                end
            end
        end
    end
end

endmodule


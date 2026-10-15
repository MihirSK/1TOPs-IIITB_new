// ============================================================
//  clint.sv (v2)  -  Core-Local Interruptor
//  Implements mtime, mtimecmp, msip  (RISC-V CLINT spec)
//  Base address: 0x0200_0000  (QEMU-compatible layout); only addr[15:0] is decoded
//
//  0x0000 + 4*hart : msip[hart]     (32-bit, only bit 0 is stored)
//  0x4000 + 8*hart : mtimecmp[hart] (64-bit, byte strobes, 32-bit halves at +0/+4)
//  0xBFF8          : mtime          (64-bit, byte strobes, 32-bit halves at +0/+4)
//  everything else : reads 0, writes ignored
//
//  Bus: one access is accepted in the first cycle with req && !ack; ack (and rdata)
//  follow one cycle later and ack is one cycle wide. 32-bit accesses use the byte
//  lane selected by addr[2] (write data on wdata lane addr[2], strobe strb[4*addr[2]+:4]);
//  msip reads return the value on both 32-bit lanes.
//
//  Changes against v1:
//   C1  mtime was driven by two always_ff blocks (tick and bus write): illegal
//       in strict tools and a race when a write and an rtc_tick coincide.
//       v2: one process, a write wins over a coincident tick.
//   C2  reads of a hart that does not exist returned the previous read data
//       (rdata was only assigned inside the hart check). v2: they read 0.
//   C3  mtime half-word access at 0xBFFC was not decoded (only 0xBFF8 matched).
//   C4  msip ignored the write strobes and took data from wdata[31:0] only
//       (wrong lane for odd harts, byte writes to bytes 1..3 changed msip).
//   C5  msip read data was always on the low lane (lw from an odd hart returned 0).
//   C6  region decode covered only 0x0000-0x0FFF (msip) and 0x4000-0x4FFF
//       (mtimecmp); v2 decodes the full 0x0000-0x3FFF and 0x4000-0xBFF7.
//   C7  the handshake accepts a request once (req && !ack): a master that holds
//       req until it sees ack no longer gets two acks.
// ============================================================
`timescale 1ns/1ps
`default_nettype none

module clint #(
    parameter int HARTS = 1
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        rtc_tick,   // real-time clock tick: mtime += 1 in every cycle it is high

    // Memory-mapped register interface
    input  logic [63:0] addr,
    input  logic [63:0] wdata,
    input  logic [7:0]  strb,
    input  logic        req,
    input  logic        we,
    output logic [63:0] rdata,
    output logic        ack,

    // Interrupt outputs per hart
    output logic [HARTS-1:0] msip,    // machine software interrupt
    output logic [HARTS-1:0] mtip     // machine timer interrupt
);

logic [63:0]         mtime;
logic [HARTS*64-1:0] mtimecmp_v;      // mtimecmp[h] = mtimecmp_v[h*64 +: 64]
logic [HARTS-1:0]    msip_q;

assign msip = msip_q;

// timer compare (unsigned)
genvar h;
generate
    for (h = 0; h < HARTS; h++) begin : gen_hart
        assign mtip[h] = (mtime >= mtimecmp_v[h*64 +: 64]);
    end
endgenerate

// ---- decode ----
logic [15:0] off;
logic        sel_msip, sel_cmp, sel_mtime;
int          msip_h, cmp_h;
logic        lane;
assign off       = addr[15:0];
assign lane      = off[2];
assign sel_msip  = (off < 16'h4000);
assign sel_cmp   = (off >= 16'h4000) && (off < 16'hBFF8);
assign sel_mtime = (off >= 16'hBFF8) && (off < 16'hC000);
assign msip_h    = int'(off[13:2]);
assign cmp_h     = int'((off - 16'h4000) >> 3);

function automatic logic [63:0] merge64(input logic [63:0] old, input logic [63:0] nw, input logic [7:0] st);
    logic [63:0] r;
    r = old;
    for (int b = 0; b < 8; b++) if (st[b]) r[b*8 +: 8] = nw[b*8 +: 8];
    return r;
endfunction

logic acc;
assign acc = req && !ack;

logic msip_st, msip_d;
assign msip_st = lane ? strb[4] : strb[0];
assign msip_d  = lane ? wdata[32] : wdata[0];

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mtime      <= 64'h0;
        mtimecmp_v <= '1;
        msip_q     <= '0;
        rdata      <= 64'h0;
        ack        <= 1'b0;
    end else begin
        ack <= acc;                                            // C7

        // mtime: one process; a bus write wins over a coincident tick (C1)
        if (acc && we && sel_mtime) mtime <= merge64(mtime, wdata, strb);
        else if (rtc_tick)          mtime <= mtime + 64'h1;

        if (acc) begin
            rdata <= 64'h0;                                    // C2: unmapped reads give 0
            if (sel_msip) begin
                if (msip_h < HARTS) begin
                    rdata <= {31'h0, msip_q[msip_h], 31'h0, msip_q[msip_h]};   // C5
                    if (we && msip_st) msip_q[msip_h] <= msip_d;                // C4
                end
            end else if (sel_cmp) begin
                if (cmp_h < HARTS) begin
                    rdata <= mtimecmp_v[cmp_h*64 +: 64];
                    if (we) mtimecmp_v[cmp_h*64 +: 64] <= merge64(mtimecmp_v[cmp_h*64 +: 64], wdata, strb);
                end
            end else if (sel_mtime) begin                       // C3: 0xBFF8..0xBFFF
                rdata <= mtime;
            end
        end
    end
end

endmodule
`default_nettype wire

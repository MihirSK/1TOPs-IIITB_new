// ============================================================
//  clint.sv  -  Core-Local Interruptor
//  Implements mtime, mtimecmp, msip  (RISC-V CLINT spec)
//  Base address: 0x0200_0000  (QEMU-compatible layout)
// ============================================================
`timescale 1ns/1ps


module clint #(
    parameter int HARTS = 1
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        rtc_tick,   // real-time clock tick (typically slower)

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
// ============================================================
//  Registers
//  0x0000 + 4*hart : msip[hart]  (32-bit)
//  0x4000 + 8*hart : mtimecmp[hart] (64-bit)
//  0xBFF8          : mtime (64-bit)
// ============================================================

logic [63:0] mtime;
logic [63:0] mtimecmp [HARTS];
logic [31:0] msip_r   [HARTS];

// mtime counter
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)      mtime <= 64'h0;
    else if (rtc_tick) mtime <= mtime + 64'h1;
end

// Timer compare
genvar h;
generate
    for (h = 0; h < HARTS; h++) begin : gen_hart
        assign mtip[h] = (mtime >= mtimecmp[h]);
        assign msip[h] = msip_r[h][0];
    end
endgenerate

// Register read/write
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        foreach (mtimecmp[i]) mtimecmp[i] <= 64'hFFFF_FFFF_FFFF_FFFF;
        foreach (msip_r[i])   msip_r[i]   <= 32'h0;
        rdata <= 64'h0;
        ack   <= 1'b0;
    end else begin
        ack <= 1'b0;
        if (req) begin
            ack <= 1'b1;
            if (addr[15:12] == 4'h0) begin
                // msip region: 0x0000 - 0x3FFF
                int hart_id;
                hart_id = addr[14:2];
                if (hart_id < HARTS) begin
                    rdata <= {32'h0, msip_r[hart_id]};
                    if (we) msip_r[hart_id] <= wdata[31:0] & 32'h1;
                end
            end else if (addr[15:12] == 4'h4) begin
                // mtimecmp: 0x4000 - 0xBFF7
                // hart_id from LOCAL offset: (addr[15:0] - 0x4000) >> 3
                int hart_id;
                hart_id = (addr[15:0] - 16'h4000) >> 3;
                if (hart_id < HARTS) begin
                    rdata <= mtimecmp[hart_id];
                    if (we) begin
                        // 64-bit write with byte enables
                        for (int b = 0; b < 8; b++)
                            if (strb[b]) mtimecmp[hart_id][b*8 +: 8] <= wdata[b*8 +: 8];
                    end
                end
            end else if (addr[15:0] == 16'hBFF8) begin
                // mtime
                rdata <= mtime;
                if (we) begin
                    for (int b = 0; b < 8; b++)
                        if (strb[b]) mtime[b*8 +: 8] <= wdata[b*8 +: 8];
                end
            end else begin
                rdata <= 64'h0;
            end
        end
    end
end

endmodule
`default_nettype wire
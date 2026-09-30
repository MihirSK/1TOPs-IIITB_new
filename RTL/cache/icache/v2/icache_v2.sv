// ============================================================
//  icache.sv  —  16KB direct-mapped instruction cache
//  IIITB 1TOPs SoC spec: 16KB I-Cache
//
//  Config: 256 sets × 1 way × 64B line = 16KB
//  Tag  = PA[39:14]  (26 bits for Sv39)
//  Index= PA[13:6]   (8 bits  = 256 sets)
//  Offset=PA[5:0]    (6 bits  = 64 bytes)
//
//  On miss: fetches full 64-byte line from backing store
//  On FENCE.I: full invalidate
// ============================================================
`timescale 1ns/1ps
`default_nettype none

module icache #(
    parameter LINE_BITS = 512,  // 64 bytes per line
    parameter SETS      = 256
)(
    input  logic        clk,
    input  logic        rst_n,

    // CPU side (word-granular)
    input  logic [63:0] cpu_addr,
    input  logic        cpu_req,
    input  logic        flush,       // FENCE.I
    output logic [31:0] cpu_rdata,
    output logic        cpu_ack,

    // Memory side (64-bit bus to SRAM)
    output logic [63:0] mem_addr,
    output logic        mem_req,
    input  logic [63:0] mem_rdata,
    input  logic        mem_ack
);

localparam WAYS       = 1;
localparam OFFSET_W   = 6;   // byte offset bits (64B line)
localparam INDEX_W    = 8;   // 256 sets
localparam TAG_W      = 64 - OFFSET_W - INDEX_W; // 50 bits (use [49:0])
localparam WORDS_LINE = LINE_BITS / 64; // 8 words per line

// ---- Cache arrays ----
logic [TAG_W-1:0]    tag_arr   [SETS];
logic [LINE_BITS-1:0] data_arr [SETS];
logic                valid_arr [SETS];

// ---- Decode incoming address ----
logic [INDEX_W-1:0]  idx;
logic [TAG_W-1:0]    tag;
logic [OFFSET_W-1:0] off;
assign idx = cpu_addr[OFFSET_W +: INDEX_W];
assign tag = cpu_addr[63:OFFSET_W+INDEX_W];
assign off = cpu_addr[OFFSET_W-1:0];

logic hit;
assign hit = valid_arr[idx] && (tag_arr[idx] == tag);

// ---- Hit path ----
logic [LINE_BITS-1:0] hit_line;
logic [5:0]           word_sel; // which 32-bit word in the line
assign hit_line = data_arr[idx];
assign word_sel = off[5:2]; // word index (offset/4, ignoring bit1:0)

logic [31:0] cpu_rdata_d;
always_comb begin
    cpu_rdata_d = hit_line[word_sel*32 +: 32];
end

// ---- Refill FSM ----
localparam [1:0] IDLE=2'b00, FETCH=2'b01, FILL=2'b10;
logic [1:0] state;
logic [2:0] beat; // 0..7 (8 beats for 64B line)
logic [LINE_BITS-1:0] refill_buf;

logic [63:0] refill_base; // base address of missing line
assign refill_base = {cpu_addr[63:OFFSET_W], {OFFSET_W{1'b0}}};

assign mem_addr = refill_base + {58'h0, beat, 3'b0}; // beat × 8
assign mem_req  = (state == FETCH);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        beat  <= 3'h0;
        for (int i = 0; i < SETS; i++) valid_arr[i] <= 1'b0;
    end else if (flush) begin
        // FENCE.I wins over everything, including a refill completing this cycle
        for (int i = 0; i < SETS; i++) valid_arr[i] <= 1'b0;
        state <= IDLE;
        beat  <= 3'h0;
    end else begin
        case (state)
            IDLE: begin
                beat <= 3'h0;
                if (cpu_req && !hit) state <= FETCH;
            end
            FETCH: if (mem_ack) begin
                refill_buf[beat*64 +: 64] <= mem_rdata;
                if (beat == 3'd7) begin
                    data_arr [idx] <= {mem_rdata, refill_buf[447:0]};  // beats 0..6 + beat 7
                    tag_arr  [idx] <= tag;
                    valid_arr[idx] <= 1'b1;
                    state          <= FILL;
                end else beat <= beat + 3'h1;
            end
            FILL:    state <= IDLE;
            default: state <= IDLE;
        endcase
    end
end

assign cpu_ack   = cpu_req && hit;
assign cpu_rdata = cpu_rdata_d;

endmodule
`default_nettype wire

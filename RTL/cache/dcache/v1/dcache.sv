// ============================================================
//  dcache.sv  —  16KB direct-mapped write-back data cache
//  IIITB 1TOPs SoC spec: 16KB D-Cache
//
//  Config: 256 sets × 1 way × 64B line = 16KB
//  Policy: write-back with dirty bit, write-allocate on miss
//  On AMO: bypass cache (pass-through to memory for atomicity)
// ============================================================
`timescale 1ns/1ps
`default_nettype none

module dcache #(
    parameter SETS = 256
)(
    input  logic        clk,
    input  logic        rst_n,

    // CPU side
    input  logic [63:0] cpu_addr,
    input  logic [63:0] cpu_wdata,
    input  logic [7:0]  cpu_strb,
    input  logic        cpu_req,
    input  logic        cpu_we,
    input  logic        cpu_amo,    // bypass cache for AMO
    output logic [63:0] cpu_rdata,
    output logic        cpu_ack,
    output logic        cpu_err,

    // Memory side (64-bit, 64B line refill/evict)
    output logic [63:0] mem_addr,
    output logic [63:0] mem_wdata,
    output logic [7:0]  mem_strb,
    output logic        mem_req,
    output logic        mem_we,
    input  logic [63:0] mem_rdata,
    input  logic        mem_ack,

    // Flush (FENCE)
    input  logic        flush       // writeback all dirty lines
);

localparam LINE_B   = 64;   // bytes per line
localparam OFFSET_W = 6;    // log2(64)
localparam INDEX_W  = 8;    // log2(256)
localparam TAG_W    = 64 - OFFSET_W - INDEX_W;
localparam WORDS    = LINE_B / 8; // 8 doublewords per line

// ---- Arrays ----
logic [TAG_W-1:0]    tag_arr   [SETS];
logic [511:0]        data_arr  [SETS]; // 64B = 512 bits
logic                valid_arr [SETS];
logic                dirty_arr [SETS];

// ---- Address decode ----
logic [INDEX_W-1:0]  idx;
logic [TAG_W-1:0]    tag;
logic [OFFSET_W-1:0] off;
assign idx = cpu_addr[OFFSET_W +: INDEX_W];
assign tag = cpu_addr[63:OFFSET_W+INDEX_W];
assign off = cpu_addr[OFFSET_W-1:0];

logic hit;
assign hit = valid_arr[idx] && (tag_arr[idx] == tag) && !cpu_amo;

// ---- Read data from cache line ----
logic [2:0] dword_sel;
assign dword_sel = off[5:3]; // which 64-bit word in the 512-bit line

logic [63:0] line_dword;
assign line_dword = data_arr[idx][dword_sel*64 +: 64];

// ---- FSM ----
localparam [2:0]
    ST_IDLE    = 3'b000,
    ST_EVICT   = 3'b001, // write-back dirty line
    ST_REFILL  = 3'b010, // fetch missing line
    ST_AMO     = 3'b011, // pass-through for AMO
    ST_FLUSH   = 3'b100; // flush all dirty lines

logic [2:0]  state;
logic [2:0]  beat;
logic [7:0]  flush_idx;
logic [511:0] refill_buf;

// Eviction address (reconstructed from old tag)
logic [63:0] evict_addr;
assign evict_addr = {tag_arr[idx], idx, {OFFSET_W{1'b0}}};

// Refill base
logic [63:0] miss_base;
assign miss_base = {cpu_addr[63:OFFSET_W], {OFFSET_W{1'b0}}};

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= ST_IDLE; beat <= 0; flush_idx <= 0;
        for (int i=0;i<SETS;i++) begin
            valid_arr[i] <= 0; dirty_arr[i] <= 0;
        end
    end else begin
        case (state)
            ST_IDLE: begin
                beat <= 0;
                if (flush) begin
                    flush_idx <= 0;
                    state <= ST_FLUSH;
                end else if (cpu_req) begin
                    if (cpu_amo) begin
                        state <= ST_AMO;
                    end else if (hit) begin
                        // Hit: handle read/write
                        if (cpu_we) begin
                            // Write to cache line
                            for (int b=0;b<8;b++) begin
                                if (cpu_strb[b])
                                    data_arr[idx][(dword_sel*64 + b*8) +: 8] <= cpu_wdata[b*8 +: 8];
                            end
                            dirty_arr[idx] <= 1'b1;
                        end
                        // Read: combinational via line_dword
                    end else begin
                        // Miss
                        if (valid_arr[idx] && dirty_arr[idx])
                            state <= ST_EVICT; // must write back first
                        else
                            state <= ST_REFILL;
                    end
                end
            end

            ST_EVICT: begin
                // Write back dirty line beat by beat
                if (mem_ack) begin
                    if (beat == 3'd7) begin
                        dirty_arr[idx] <= 1'b0;
                        valid_arr[idx] <= 1'b0;
                        state <= ST_REFILL;
                        beat  <= 0;
                    end else beat <= beat + 1;
                end
            end

            ST_REFILL: begin
                if (mem_ack) begin
                    refill_buf[beat*64 +: 64] <= mem_rdata;
                    if (beat == 3'd7) begin
                        // Commit line (beat 7 word comes in same cycle)
                        data_arr [idx] <= {mem_rdata,
                                           refill_buf[6*64+:64], refill_buf[5*64+:64],
                                           refill_buf[4*64+:64], refill_buf[3*64+:64],
                                           refill_buf[2*64+:64], refill_buf[1*64+:64],
                                           refill_buf[0*64+:64]};
                        tag_arr  [idx] <= tag;
                        valid_arr[idx] <= 1'b1;
                        dirty_arr[idx] <= 1'b0;
                        state <= ST_IDLE;
                        beat  <= 0;
                    end else beat <= beat + 1;
                end
            end

            ST_AMO: begin
                // Single-beat pass-through
                if (mem_ack) state <= ST_IDLE;
            end

            ST_FLUSH: begin
                if (!dirty_arr[flush_idx] || !valid_arr[flush_idx]) begin
                    valid_arr[flush_idx] <= 0;
                    if (flush_idx == 8'hFF) state <= ST_IDLE;
                    else flush_idx <= flush_idx + 1;
                end else if (mem_ack) begin
                    if (beat == 3'd7) begin
                        dirty_arr[flush_idx] <= 0;
                        valid_arr[flush_idx] <= 0;
                        beat <= 0;
                        if (flush_idx == 8'hFF) state <= ST_IDLE;
                        else flush_idx <= flush_idx + 1;
                    end else beat <= beat + 1;
                end
            end

            default: state <= ST_IDLE;
        endcase
    end
end

// ---- Memory bus mux ----
always_comb begin
    mem_addr  = 64'h0;
    mem_wdata = 64'h0;
    mem_strb  = 8'hFF;
    mem_req   = 1'b0;
    mem_we    = 1'b0;
    case (state)
        ST_EVICT: begin
            mem_addr = evict_addr + {58'h0, beat, 3'b0};
            mem_wdata= data_arr[idx][beat*64 +: 64];
            mem_req  = 1'b1;
            mem_we   = 1'b1;
        end
        ST_REFILL: begin
            mem_addr = miss_base + {58'h0, beat, 3'b0};
            mem_req  = 1'b1;
        end
        ST_AMO: begin
            mem_addr  = cpu_addr;
            mem_wdata = cpu_wdata;
            mem_strb  = cpu_strb;
            mem_req   = cpu_req;
            mem_we    = cpu_we;
        end
        ST_FLUSH: begin
            if (dirty_arr[flush_idx] && valid_arr[flush_idx]) begin
                mem_addr = {tag_arr[flush_idx], flush_idx, {OFFSET_W{1'b0}}} + {58'h0, beat, 3'b0};
                mem_wdata= data_arr[flush_idx][beat*64 +: 64];
                mem_req  = 1'b1;
                mem_we   = 1'b1;
            end
        end
        default: ;
    endcase
end

// ---- CPU response ----
assign cpu_rdata = (state == ST_AMO) ? mem_rdata : line_dword;
assign cpu_ack   = (state == ST_IDLE  && cpu_req && hit) ||
                   (state == ST_AMO   && mem_ack);
assign cpu_err   = 1'b0;

endmodule
`default_nettype wire

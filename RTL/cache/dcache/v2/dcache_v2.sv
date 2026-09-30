// ============================================================
//  dcache.sv (v2) -  16KB direct-mapped write-back data cache
//  IIITB 1TOPs SoC spec: 16KB D-Cache
//
//  Config: 256 sets x 1 way x 64B line = 16KB
//  Policy: write-back with dirty bit, write-allocate on miss
//  AMO   : bypass cache (single-beat pass-through to memory);
//          a cached copy of the AMO line is written back (if dirty)
//          and invalidated first, so memory and cache stay coherent
//  Flush : write back every dirty line and invalidate the cache.
//          flush is a 1-cycle pulse; it is remembered if the cache is busy.
//
//  Changes against v1:
//   F1  miss set/tag are latched when the miss is detected; evict, refill
//       and commit use the latched values, so cpu_addr / cpu_req may change
//       while a refill or write-back is running (no wrong line is written).
//   F2  a store that hits in the same cycle as a flush pulse is no longer
//       acked (v1 acked it and dropped it). Requests wait until the flush is
//       done.
//   F3  a flush pulse that arrives while the FSM is busy is latched
//       (flush_pend) and serviced when the FSM is idle (v1 lost it).
//   F4  AMO on a cached line: dirty -> write back then AMO, clean -> invalidate
//       then AMO (v1 bypassed the cache, leaving stale/dirty data behind).
//   F5  the AMO request is latched and mem_req is held until mem_ack even if
//       cpu_req drops (v1 could hang); cpu_ack for AMO requires cpu_req.
//
//  Memory-side contract (same as icache): mem_req is held until mem_ack, the
//  address/data advance on the edge that sees mem_ack, mem_rdata is valid in
//  the mem_ack cycle, the slave answers each beat separately.
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

    // Flush (FENCE): 1-cycle pulse, write back all dirty lines
    input  logic        flush
);

localparam LINE_B   = 64;   // bytes per line
localparam OFFSET_W = 6;    // log2(64)
localparam INDEX_W  = 8;    // log2(256)
localparam TAG_W    = 64 - OFFSET_W - INDEX_W;   // 50
localparam WORDS    = LINE_B / 8; // 8 doublewords per line

// ---- Arrays ----
logic [TAG_W-1:0]    tag_arr   [SETS];
logic [511:0]        data_arr  [SETS]; // 64B = 512 bits
logic [SETS-1:0]     valid_arr;
logic [SETS-1:0]     dirty_arr;

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

// ---- Store merge (byte strobes) ----
logic [63:0] merged_dword;
always_comb begin
    merged_dword = line_dword;
    for (int b = 0; b < 8; b++)
        if (cpu_strb[b]) merged_dword[b*8 +: 8] = cpu_wdata[b*8 +: 8];
end

// ---- FSM ----
localparam [2:0]
    ST_IDLE    = 3'b000,
    ST_EVICT   = 3'b001, // write-back dirty line
    ST_REFILL  = 3'b010, // fetch missing line
    ST_AMO     = 3'b011, // pass-through for AMO
    ST_FLUSH   = 3'b100; // flush all dirty lines

logic [2:0]   state;
logic [2:0]   beat;
logic [7:0]   flush_idx;
logic [511:0] refill_buf;

// F1: request latched at miss detection
logic [INDEX_W-1:0] m_idx;
logic [TAG_W-1:0]   m_tag;
// F5: AMO request latched
logic [63:0] a_addr, a_wdata;
logic [7:0]  a_strb;
logic        a_we;
logic        amo_pend;    // F4: write back the AMO line first, then do the AMO
logic        flush_pend;  // F3

// Eviction address (reconstructed from the old tag of the latched set)
logic [63:0] evict_addr;
assign evict_addr = {tag_arr[m_idx], m_idx, {OFFSET_W{1'b0}}};

// Refill base (latched miss)
logic [63:0] miss_base;
assign miss_base = {m_tag, m_idx, {OFFSET_W{1'b0}}};

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= ST_IDLE; beat <= 0; flush_idx <= 0;
        amo_pend <= 1'b0; flush_pend <= 1'b0;
        valid_arr <= '0; dirty_arr <= '0;
    end else begin
        // F3: remember a flush pulse that cannot be served right now
        if (flush && state != ST_IDLE && state != ST_FLUSH) flush_pend <= 1'b1;

        case (state)
            ST_IDLE: begin
                beat <= 0;
                if (flush || flush_pend) begin
                    flush_idx  <= 0;
                    flush_pend <= 1'b0;
                    state      <= ST_FLUSH;
                end else if (cpu_req) begin
                    if (cpu_amo) begin
                        // F5: latch the request, F4: make the cached copy coherent
                        a_addr <= cpu_addr; a_wdata <= cpu_wdata;
                        a_strb <= cpu_strb; a_we    <= cpu_we;
                        m_idx  <= idx;      m_tag   <= tag;
                        if (valid_arr[idx] && (tag_arr[idx] == tag)) begin
                            if (dirty_arr[idx]) begin
                                amo_pend <= 1'b1;
                                state    <= ST_EVICT;
                            end else begin
                                valid_arr[idx] <= 1'b0;
                                state          <= ST_AMO;
                            end
                        end else state <= ST_AMO;
                    end else if (hit) begin
                        if (cpu_we) begin
                            data_arr[idx][dword_sel*64 +: 64] <= merged_dword;
                            dirty_arr[idx] <= 1'b1;
                        end
                        // Read: combinational via line_dword
                    end else begin
                        // Miss: F1 latch set and tag
                        m_idx <= idx;
                        m_tag <= tag;
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
                        dirty_arr[m_idx] <= 1'b0;
                        valid_arr[m_idx] <= 1'b0;
                        state    <= amo_pend ? ST_AMO : ST_REFILL;
                        amo_pend <= 1'b0;
                        beat     <= 0;
                    end else beat <= beat + 1;
                end
            end

            ST_REFILL: begin
                if (mem_ack) begin
                    refill_buf[beat*64 +: 64] <= mem_rdata;
                    if (beat == 3'd7) begin
                        // Commit line (beat 7 word comes in same cycle)
                        data_arr [m_idx] <= {mem_rdata,
                                             refill_buf[6*64+:64], refill_buf[5*64+:64],
                                             refill_buf[4*64+:64], refill_buf[3*64+:64],
                                             refill_buf[2*64+:64], refill_buf[1*64+:64],
                                             refill_buf[0*64+:64]};
                        tag_arr  [m_idx] <= m_tag;
                        valid_arr[m_idx] <= 1'b1;
                        dirty_arr[m_idx] <= 1'b0;
                        state <= ST_IDLE;
                        beat  <= 0;
                    end else beat <= beat + 1;
                end
            end

            ST_AMO: begin
                // Single-beat pass-through, held until mem_ack
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
            mem_wdata= data_arr[m_idx][beat*64 +: 64];
            mem_req  = 1'b1;
            mem_we   = 1'b1;
        end
        ST_REFILL: begin
            mem_addr = miss_base + {58'h0, beat, 3'b0};
            mem_req  = 1'b1;
        end
        ST_AMO: begin
            mem_addr  = a_addr;
            mem_wdata = a_wdata;
            mem_strb  = a_strb;
            mem_req   = 1'b1;
            mem_we    = a_we;
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
// F2: no ack for a hit in a cycle where a flush is starting (the store would be lost)
assign cpu_ack   = (state == ST_IDLE && cpu_req && hit && !flush && !flush_pend) ||
                   (state == ST_AMO  && mem_ack && cpu_req);
assign cpu_err   = 1'b0;

endmodule
`default_nettype wire

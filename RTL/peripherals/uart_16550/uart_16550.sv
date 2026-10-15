// ============================================================
//  uart_16550.sv  –  Minimal UART compatible with 8250/16550
//  Implements: THR, RBR, LSR, IER, IIR, LCR, MCR, DLL, DLM
//  Linux driver: 8250_core / serial8250
// ============================================================
`timescale 1ns/1ps


module uart_16550 #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  logic       clk,
    input  logic       rst_n,

    // Bus interface (byte-wide registers, 8 addresses)
    input  logic [2:0] addr,
    input  logic [7:0] wdata,
    input  logic       req,
    input  logic       we,
    output logic [7:0] rdata,
    output logic       ack,

    // Serial
    output logic       tx,
    input  logic       rx
);

// ---- Baud rate generator ----
localparam BAUD_DIV = CLK_FREQ / (BAUD_RATE * 16);

logic [15:0] baud_div_r;   // DLL+DLM (loaded by software)
logic [15:0] baud_cnt;
logic        baud_tick;    // 16x oversampling tick

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        baud_cnt  <= 16'h0;
        baud_tick <= 1'b0;
    end else begin
        baud_tick <= 1'b0;
        if (baud_cnt == 16'h0) begin
            baud_cnt  <= baud_div_r == 16'h0 ? BAUD_DIV[15:0] : baud_div_r;
            baud_tick <= 1'b1;
        end else begin
            baud_cnt <= baud_cnt - 16'h1;
        end
    end
end

// ---- Register file ----
logic [7:0] ier, lcr, mcr, fcr;
logic       dlab;      // LCR[7]
logic [7:0] dll, dlm;  // divisor latch

assign dlab      = lcr[7];
assign baud_div_r= {dlm, dll};

// ---- TX FIFO (16 bytes) ----
logic [7:0] tx_fifo [16];
logic [3:0] tx_wptr, tx_rptr;
logic [4:0] tx_count;
logic       tx_full, tx_empty;

assign tx_full  = (tx_count == 5'd16);
assign tx_empty = (tx_count == 5'd0);

// ---- TX shift register ----
logic [9:0] tx_shift;   // start + 8 data + stop
logic [3:0] tx_bit_cnt; // bit position
logic [3:0] tx_tick_cnt;// 16x tick counter
logic       tx_busy;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx          <= 1'b1;
        tx_shift    <= 10'h3FF;
        tx_bit_cnt  <= 4'h0;
        tx_tick_cnt <= 4'h0;
        tx_busy     <= 1'b0;
        tx_wptr     <= 4'h0;
        tx_rptr     <= 4'h0;
        tx_count    <= 5'h0;
    end else begin
        // Load TX FIFO from bus
        if (req && we && !dlab && addr == 3'h0 && !tx_full) begin
            tx_fifo[tx_wptr] <= wdata;
            tx_wptr          <= tx_wptr + 4'h1;
            tx_count         <= tx_count + 5'h1;
        end

        // TX state machine
        if (!tx_busy && !tx_empty) begin
            // Load next byte
            tx_shift   <= {1'b1, tx_fifo[tx_rptr], 1'b0}; // stop, data, start
            tx_rptr    <= tx_rptr + 4'h1;
            tx_count   <= tx_count - 5'h1;
            tx_busy    <= 1'b1;
            tx_bit_cnt <= 4'h0;
            tx_tick_cnt<= 4'h0;
        end else if (tx_busy && baud_tick) begin
            tx_tick_cnt <= tx_tick_cnt + 4'h1;
            if (tx_tick_cnt == 4'hF) begin
                tx         <= tx_shift[0];
                tx_shift   <= {1'b1, tx_shift[9:1]};
                tx_bit_cnt <= tx_bit_cnt + 4'h1;
                if (tx_bit_cnt == 4'h9) begin
                    tx_busy <= 1'b0;
                    tx      <= 1'b1;
                end
            end
        end
    end
end

// ---- RX FIFO (16 bytes) ----
logic [7:0] rx_fifo [16];
logic [3:0] rx_wptr, rx_rptr;
logic [4:0] rx_count;
logic       rx_full, rx_empty;

assign rx_full  = (rx_count == 5'd16);
assign rx_empty = (rx_count == 5'd0);

// ---- RX shift register ----
logic [7:0] rx_shift;
logic [3:0] rx_bit_cnt;
logic [3:0] rx_tick_cnt;
logic       rx_busy;
logic       rx_d1, rx_d2;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_d1    <= 1'b1; rx_d2 <= 1'b1;
        rx_busy  <= 1'b0;
        rx_wptr  <= 4'h0; rx_rptr <= 4'h0;
        rx_count <= 5'h0;
    end else begin
        rx_d1 <= rx;
        rx_d2 <= rx_d1;

        if (!rx_busy && rx_d2 == 1'b0) begin
            // Start bit detected
            rx_busy     <= 1'b1;
            rx_bit_cnt  <= 4'h0;
            rx_tick_cnt <= 4'h0;
        end else if (rx_busy && baud_tick) begin
            rx_tick_cnt <= rx_tick_cnt + 4'h1;
            if (rx_tick_cnt == 4'h7 && rx_bit_cnt == 4'h0) begin
                // middle of start bit — verify
                if (rx_d2 != 1'b0) rx_busy <= 1'b0; // false start
            end else if (rx_tick_cnt == 4'hF) begin
                rx_bit_cnt <= rx_bit_cnt + 4'h1;
                if (rx_bit_cnt > 4'h0 && rx_bit_cnt <= 4'h8) begin
                    rx_shift <= {rx_d2, rx_shift[7:1]};
                end
                if (rx_bit_cnt == 4'h9) begin
                    // Stop bit — push to FIFO
                    rx_busy <= 1'b0;
                    if (!rx_full) begin
                        rx_fifo[rx_wptr] <= rx_shift;
                        rx_wptr          <= rx_wptr + 4'h1;
                        rx_count         <= rx_count + 5'h1;
                    end
                end
            end
        end

        // Pop RBR read
        if (req && !we && !dlab && addr == 3'h0 && !rx_empty) begin
            rx_rptr  <= rx_rptr + 4'h1;
            rx_count <= rx_count - 5'h1;
        end
    end
end

// ---- LSR (Line Status Register) ----
logic [7:0] lsr;
assign lsr = {
    1'b0,           // [7] Error in RX FIFO
    tx_empty,       // [6] TX empty (TEMT)
    tx_empty,       // [5] THR empty (THRE) — ready for next byte
    1'b0,           // [4] Break interrupt
    1'b0,           // [3] Framing error
    1'b0,           // [2] Parity error
    1'b0,           // [1] Overrun error
    ~rx_empty       // [0] Data ready
};

// ---- Register reads ----
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rdata  <= 8'h0;
        ack    <= 1'b0;
        ier    <= 8'h0;
        lcr    <= 8'h03; // 8N1
        mcr    <= 8'h0;
        fcr    <= 8'h0;
        dll    <= BAUD_DIV[7:0];
        dlm    <= BAUD_DIV[15:8];
    end else begin
        ack <= req;
        if (req) begin
            if (!we) begin
                case (addr)
                    3'h0: rdata <= dlab ? dll         : (rx_empty ? 8'h0 : rx_fifo[rx_rptr]);
                    3'h1: rdata <= dlab ? dlm         : ier;
                    3'h2: rdata <= 8'hC1;              // IIR: FIFOs enabled, no interrupt
                    3'h3: rdata <= lcr;
                    3'h4: rdata <= mcr;
                    3'h5: rdata <= lsr;
                    3'h6: rdata <= 8'h00;              // MSR
                    3'h7: rdata <= 8'h00;              // scratch
                    default: rdata <= 8'h0;
                endcase
            end else begin
                case (addr)
                    3'h1: if (!dlab) ier <= wdata; else dlm <= wdata;
                    3'h2: fcr <= wdata;
                    3'h3: lcr <= wdata;
                    3'h4: mcr <= wdata;
                    default: ;
                endcase
                if (dlab && addr == 3'h0) dll <= wdata;
            end
        end
    end
end

endmodule
`default_nettype wire

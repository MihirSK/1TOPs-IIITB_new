// ============================================================
//  uart_16550.sv (v2) -  Minimal UART compatible with 8250/16550
//  Implements: THR, RBR, LSR, IER, IIR, LCR, MCR, SCR, FCR, DLL, DLM
//  Linux driver: 8250_core / serial8250
//
//  Changes against v1:
//   U1  bus: a transaction is accepted once, in the first cycle with
//       req && !ack; ack is one cycle wide. Side effects (THR push, RBR pop,
//       register writes) no longer repeat while req is held.
//   U2  baud: one 16x tick every DIV clocks (v1 gave DIV+1 -> wrong baud).
//   U3  TX: byte is loaded on a baud tick and the start bit begins at once,
//       every bit lasts exactly 16 ticks; the transmitter stays busy until
//       the stop bit is complete.
//   U4  TEMT = TX FIFO empty AND shifter idle (v1 set it while shifting).
//   U5  RX: start bit is verified at its middle and every following bit is
//       sampled at its middle (v1 sampled at the bit boundary).
//   U6  FIFO counters: push and pop in the same cycle are handled together
//       (v1 lost a byte when both happened).
//   U7  FCR[1] / FCR[2] clear the RX / TX FIFO (v1 ignored them).
//   U8  SCR (address 7) is a real read/write register (v1 always read 0).
//   U9  after a reset or a bad stop bit the receiver waits for the line to
//       be high before it looks for a new start bit.
//
//  Not implemented (unchanged): parity / word length / stop-bit select,
//  framing/overrun/break flags in LSR, interrupt output, modem lines, loopback.
// ============================================================
`timescale 1ns/1ps
`default_nettype none

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

// ---- Bus accept (U1) ----
logic acc, wr_acc, rd_acc;
assign acc    = req && !ack;
assign wr_acc = acc && we;
assign rd_acc = acc && !we;

// ---- Register file ----
logic [7:0] ier, lcr, mcr, fcr, scr;
logic       dlab;      // LCR[7]
logic [7:0] dll, dlm;  // divisor latch
logic [15:0] baud_div_r;

assign dlab       = lcr[7];
assign baud_div_r = {dlm, dll};

// ---- Baud rate generator (U2): one tick every div_eff clocks ----
localparam BAUD_DIV = CLK_FREQ / (BAUD_RATE * 16);

logic [15:0] div_eff;
logic [15:0] baud_cnt;
logic        baud_tick;    // 16x oversampling tick

assign div_eff = (baud_div_r == 16'h0) ? BAUD_DIV[15:0] : baud_div_r;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        baud_cnt  <= 16'h0;
        baud_tick <= 1'b0;
    end else begin
        baud_tick <= 1'b0;
        if (baud_cnt == 16'h0 || baud_cnt >= div_eff) begin
            baud_cnt  <= div_eff - 16'h1;
            baud_tick <= 1'b1;
        end else begin
            baud_cnt <= baud_cnt - 16'h1;
        end
    end
end

// ---- TX FIFO (16 bytes) ----
logic [7:0] tx_fifo [16];
logic [3:0] tx_wptr, tx_rptr;
logic [4:0] tx_count;
logic       tx_full, tx_empty;

assign tx_full  = (tx_count == 5'd16);
assign tx_empty = (tx_count == 5'd0);

// ---- TX shift register ----
logic [8:0] tx_shift;    // stop + 8 data (start bit is driven at load)
logic [3:0] tx_bit_cnt;
logic [3:0] tx_tick_cnt;
logic       tx_busy;

logic tx_clr, tx_push, tx_pop;
assign tx_clr  = wr_acc && (addr == 3'h2) && wdata[2];              // U7
assign tx_push = wr_acc && !dlab && (addr == 3'h0) && !tx_full;
assign tx_pop  = !tx_busy && !tx_empty && baud_tick && !tx_clr;    // U3

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx          <= 1'b1;
        tx_shift    <= 9'h1FF;
        tx_bit_cnt  <= 4'h0;
        tx_tick_cnt <= 4'h0;
        tx_busy     <= 1'b0;
        tx_wptr     <= 4'h0;
        tx_rptr     <= 4'h0;
        tx_count    <= 5'h0;
    end else begin
        // FIFO bookkeeping (U6: push and pop together)
        if (tx_clr) begin
            tx_wptr  <= 4'h0;
            tx_rptr  <= 4'h0;
            tx_count <= 5'h0;
        end else begin
            if (tx_push) begin
                tx_fifo[tx_wptr] <= wdata;
                tx_wptr          <= tx_wptr + 4'h1;
            end
            if (tx_pop) tx_rptr <= tx_rptr + 4'h1;
            tx_count <= tx_count + {4'h0, tx_push} - {4'h0, tx_pop};
        end

        // TX state machine (U3)
        if (tx_pop) begin
            tx_shift    <= {1'b1, tx_fifo[tx_rptr]};
            tx          <= 1'b0;               // start bit now
            tx_busy     <= 1'b1;
            tx_bit_cnt  <= 4'h0;
            tx_tick_cnt <= 4'h0;
        end else if (tx_busy && baud_tick) begin
            tx_tick_cnt <= tx_tick_cnt + 4'h1;
            if (tx_tick_cnt == 4'hF) begin
                if (tx_bit_cnt == 4'h9) begin
                    tx_busy <= 1'b0;           // stop bit complete
                end else begin
                    tx         <= tx_shift[0];
                    tx_shift   <= {1'b1, tx_shift[8:1]};
                    tx_bit_cnt <= tx_bit_cnt + 4'h1;
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
logic       rx_armed;
logic       rx_d1, rx_d2;

logic rx_clr, rx_push, rx_pop;
assign rx_clr = wr_acc && (addr == 3'h2) && wdata[1];                // U7
assign rx_pop = rd_acc && !dlab && (addr == 3'h0) && !rx_empty;
// stop bit sampled at its middle: byte complete
assign rx_push = rx_busy && baud_tick && (rx_tick_cnt == 4'h7) && (rx_bit_cnt == 4'h9) && !rx_full;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_d1       <= 1'b1;
        rx_d2       <= 1'b1;
        rx_busy     <= 1'b0;
        rx_armed    <= 1'b0;
        rx_shift    <= 8'h0;
        rx_bit_cnt  <= 4'h0;
        rx_tick_cnt <= 4'h0;
        rx_wptr     <= 4'h0;
        rx_rptr     <= 4'h0;
        rx_count    <= 5'h0;
    end else begin
        rx_d1 <= rx;
        rx_d2 <= rx_d1;

        // FIFO bookkeeping (U6)
        if (rx_clr) begin
            rx_wptr  <= 4'h0;
            rx_rptr  <= 4'h0;
            rx_count <= 5'h0;
        end else begin
            if (rx_push) begin
                rx_fifo[rx_wptr] <= rx_shift;
                rx_wptr          <= rx_wptr + 4'h1;
            end
            if (rx_pop) rx_rptr <= rx_rptr + 4'h1;
            rx_count <= rx_count + {4'h0, rx_push} - {4'h0, rx_pop};
        end

        // Receiver (U5, U9)
        if (!rx_busy) begin
            if (rx_d2) rx_armed <= 1'b1;
            else if (rx_armed) begin
                rx_busy     <= 1'b1;
                rx_armed    <= 1'b0;
                rx_bit_cnt  <= 4'h0;
                rx_tick_cnt <= 4'h0;
            end
        end else if (baud_tick) begin
            rx_tick_cnt <= rx_tick_cnt + 4'h1;
            if (rx_tick_cnt == 4'h7) begin
                if (rx_bit_cnt == 4'h0) begin
                    if (rx_d2) rx_busy <= 1'b0;                 // false start
                end else if (rx_bit_cnt <= 4'h8) begin
                    rx_shift <= {rx_d2, rx_shift[7:1]};
                end else begin
                    rx_busy <= 1'b0;                            // stop bit: byte done
                end
            end
            if (rx_tick_cnt == 4'hF) rx_bit_cnt <= rx_bit_cnt + 4'h1;
        end
    end
end

// ---- LSR (Line Status Register) ----
logic [7:0] lsr;
assign lsr = {
    1'b0,                       // [7] Error in RX FIFO
    tx_empty & ~tx_busy,        // [6] TEMT: FIFO empty and shifter idle (U4)
    tx_empty,                   // [5] THRE: FIFO empty
    1'b0,                       // [4] Break interrupt
    1'b0,                       // [3] Framing error
    1'b0,                       // [2] Parity error
    1'b0,                       // [1] Overrun error
    ~rx_empty                   // [0] Data ready
};

// ---- Register access ----
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rdata  <= 8'h0;
        ack    <= 1'b0;
        ier    <= 8'h0;
        lcr    <= 8'h03; // 8N1
        mcr    <= 8'h0;
        fcr    <= 8'h0;
        scr    <= 8'h0;
        dll    <= BAUD_DIV[7:0];
        dlm    <= BAUD_DIV[15:8];
    end else begin
        ack <= acc;                                              // U1
        if (rd_acc) begin
            case (addr)
                3'h0: rdata <= dlab ? dll : (rx_empty ? 8'h0 : rx_fifo[rx_rptr]);
                3'h1: rdata <= dlab ? dlm : ier;
                3'h2: rdata <= 8'hC1;              // IIR: FIFOs enabled, no interrupt
                3'h3: rdata <= lcr;
                3'h4: rdata <= mcr;
                3'h5: rdata <= lsr;
                3'h6: rdata <= 8'h00;              // MSR
                3'h7: rdata <= scr;                // U8
                default: rdata <= 8'h0;
            endcase
        end
        if (wr_acc) begin
            case (addr)
                3'h0: if (dlab) dll <= wdata;
                3'h1: if (!dlab) ier <= wdata; else dlm <= wdata;
                3'h2: fcr <= wdata;
                3'h3: lcr <= wdata;
                3'h4: mcr <= wdata;
                3'h7: scr <= wdata;
                default: ;
            endcase
        end
    end
end

endmodule
`default_nettype wire

// ============================================================
//  gpio.sv  —  Simple 32-bit GPIO (APB-mapped)
//  IIITB 1TOPs SoC spec: Timer and GPIOs
//
//  Registers (APB byte offsets):
//  0x00 DATA_OUT [31:0] — output value
//  0x04 DATA_IN  [31:0] — sampled input value (read-only)
//  0x08 DIR      [31:0] — 1=output, 0=input
//  0x0C IRQ_EN   [31:0] — 1=enable rising-edge interrupt
// ============================================================
`timescale 1ns/1ps
`default_nettype none

module gpio (
    input  logic        clk,
    input  logic        rst_n,

    // APB
    input  logic [3:0]  paddr,
    input  logic [31:0] pwdata,
    input  logic        psel,
    input  logic        penable,
    input  logic        pwrite,
    output logic [31:0] prdata,
    output logic        pready,

    // GPIO pins
    inout  logic [31:0] gpio_pins,

    // Interrupt (rising edge detect)
    output logic        gpio_irq
);

logic [31:0] data_out, dir_reg, irq_en;
logic [31:0] gpio_in_sync0, gpio_in_sync1;
logic [31:0] rising_edge;

assign pready = 1'b1;

// Synchronise inputs (2-FF)
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        gpio_in_sync0 <= 32'h0; gpio_in_sync1 <= 32'h0;
    end else begin
        gpio_in_sync0 <= gpio_pins;
        gpio_in_sync1 <= gpio_in_sync0;
    end
end

assign rising_edge = gpio_in_sync0 & ~gpio_in_sync1;
assign gpio_irq    = |(rising_edge & irq_en);

// APB writes
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        data_out <= 32'h0; dir_reg <= 32'h0; irq_en <= 32'h0;
    end else if (psel && penable && pwrite) begin
        case (paddr[3:2])
            2'h0: data_out <= pwdata;
            2'h2: dir_reg  <= pwdata;
            2'h3: irq_en   <= pwdata;
            default: ;
        endcase
    end
end

// Tristate drive
genvar g;
generate
    for (g=0;g<32;g++) begin : gen_gpio
        assign gpio_pins[g] = dir_reg[g] ? data_out[g] : 1'bZ;
    end
endgenerate

// APB reads
always_comb begin
    case (paddr[3:2])
        2'h0: prdata = data_out;
        2'h1: prdata = gpio_in_sync1;
        2'h2: prdata = dir_reg;
        2'h3: prdata = irq_en;
        default: prdata = 32'h0;
    endcase
end

endmodule
`default_nettype wire

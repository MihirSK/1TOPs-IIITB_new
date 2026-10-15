// ============================================================
//  spi_master.sv  —  APB-mapped SPI master  (for BootROM/SPI)
//  IIITB 1TOPs SoC spec: BootROM (4KB) with SPI
//
//  Registers (APB byte offsets):
//  0x00 CTRL  [0]=EN, [1]=CPOL, [2]=CPHA, [3]=CSn_POL
//  0x04 DIV   [15:0] = clk divider (SCK = clk / (2*(DIV+1)))
//  0x08 DATA  [7:0]  write=TX, read=RX (self-clearing busy)
//  0x0C STATUS[0]=BUSY, [1]=RX_VALID
// ============================================================
`timescale 1ns/1ps
`default_nettype none

module spi_master (
    input  logic        clk,
    input  logic        rst_n,

    // APB slave interface
    input  logic [3:0]  paddr,
    input  logic [31:0] pwdata,
    input  logic        psel,
    input  logic        penable,
    input  logic        pwrite,
    output logic [31:0] prdata,
    output logic        pready,

    // SPI pins
    output logic        spi_sck,
    output logic        spi_mosi,
    input  logic        spi_miso,
    output logic        spi_csn
);

// Registers
logic        en, cpol, cpha, csn_pol;
logic [15:0] div;
logic [7:0]  tx_data, rx_data;
logic        busy, rx_valid;

// Divider counter
logic [15:0] div_cnt;
logic        sck_tick;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        div_cnt  <= 16'h0;
        sck_tick <= 1'b0;
    end else begin
        sck_tick <= 1'b0;
        if (div_cnt == div) begin
            div_cnt  <= 16'h0;
            sck_tick <= 1'b1;
        end else div_cnt <= div_cnt + 1;
    end
end

// Shift register
logic [7:0] shift_reg;
logic [2:0] bit_cnt;
logic       sck_r;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        busy     <= 1'b0; rx_valid <= 1'b0;
        sck_r    <= 1'b0; spi_csn  <= 1'b1;
        bit_cnt  <= 3'h0; shift_reg<= 8'h0;
        spi_mosi <= 1'b0;
    end else if (busy && sck_tick) begin
        sck_r <= ~sck_r;
        if (sck_r == cpol) begin // falling edge — shift out
            spi_mosi    <= shift_reg[7];
            shift_reg   <= {shift_reg[6:0], 1'b0};
        end else begin // rising edge — sample in
            shift_reg[0] <= spi_miso;
            if (bit_cnt == 3'd7) begin
                busy     <= 1'b0;
                rx_data  <= {shift_reg[6:0], spi_miso};
                rx_valid <= 1'b1;
                sck_r    <= cpol;
            end else bit_cnt <= bit_cnt + 1;
        end
    end
end

assign spi_sck = en ? sck_r : cpol;

// APB interface
assign pready = 1'b1;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        en<=0; cpol<=0; cpha<=0; csn_pol<=0; div<=16'd7;
    end else if (psel && penable && pwrite) begin
        case (paddr[3:2])
            2'h0: {csn_pol,cpha,cpol,en} <= pwdata[3:0];
            2'h1: div <= pwdata[15:0];
            2'h2: begin
                tx_data   <= pwdata[7:0];
                shift_reg <= pwdata[7:0];
                bit_cnt   <= 3'h0;
                busy      <= 1'b1;
                rx_valid  <= 1'b0;
                spi_csn   <= csn_pol;
            end
            default: ;
        endcase
    end
end

always_comb begin
    prdata = 32'h0;
    case (paddr[3:2])
        2'h0: prdata = {28'h0, csn_pol, cpha, cpol, en};
        2'h1: prdata = {16'h0, div};
        2'h2: prdata = {24'h0, rx_data};
        2'h3: prdata = {30'h0, rx_valid, busy};
        default: prdata = 32'h0;
    endcase
end

endmodule
`default_nettype wire

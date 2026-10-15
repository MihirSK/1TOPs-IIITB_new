// ============================================================
//  spi_master.sv (v2)  -  APB-mapped SPI master  (for BootROM/SPI)
//  IIITB 1TOPs SoC spec: BootROM (4KB) with SPI
//
//  Registers (APB byte offsets):
//  0x00 CTRL  [0]=EN, [1]=CPOL, [2]=CPHA, [3]=CSn_POL, [4]=CS_HOLD (new)
//  0x04 DIV   [15:0] = clk divider (SCK = clk / (2*(DIV+1)))
//  0x08 DATA  [7:0]  write=TX (starts a transfer), read=RX (last received byte)
//  0x0C STATUS[0]=BUSY, [1]=RX_VALID
//
//  Behaviour (v2):
//   * SPI modes 0..3 (CPOL/CPHA) with standard timing: CPHA=0 samples on the
//     leading edge, CPHA=1 on the trailing edge; MOSI changes only on the
//     other edge (CPHA=0: bit 7 is presented when the transfer starts).
//   * SCK idles at CPOL. Every half period is exactly DIV+1 clocks, including
//     the first (the timer restarts at every transfer).
//   * CSn is active (low for CSn_POL=0, high for CSn_POL=1) while a transfer
//     runs or CS_HOLD=1; it is asserted one half period before the first SCK
//     edge and released one half period after the last one.
//   * Writes to CTRL / DIV / DATA are ignored while BUSY, DATA writes are also
//     ignored while EN=0. RX_VALID is cleared when a transfer starts and set
//     when it completes; reading DATA does not clear it.
//
//  Changes against v1:
//   V1  SPI timing: v1 always used mode-1 style timing, MOSI changed at the
//       sampling edge and CPHA was ignored.
//   V2  RX data assembly lost bit 7 and inserted a 0 (rx_data <= {sr[6:0],miso}
//       after the last shift).
//   V3  SCK idle level: v1 idled low after reset even with CPOL=1.
//   V4  divider free-ran: random first half period, and a DIV write below the
//       running count stalled the shifter for up to 65536 clocks.
//   V5  CSn was never released (and no setup time before the first edge);
//       CS_HOLD added for multi-byte transactions.
//   V6  DATA/CTRL/DIV writes during a transfer corrupted it.
//   V7  EN=0 still ran the shifter and asserted CSn.
//   V8  rx_data had no reset value.
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
logic        en, cpol, cpha, csn_pol, cs_hold;
logic [15:0] div;
logic [7:0]  tx_byte, rx_data, rx_sh;
logic        busy, rx_valid;

// Engine
localparam [1:0] ST_IDLE = 2'd0, ST_LEAD = 2'd1, ST_XFER = 2'd2, ST_TRAIL = 2'd3;
logic [1:0]  st;
logic [15:0] hp_cnt;      // half-period timer
logic [3:0]  edge_i;      // index of the SCK edge to generate (0..15)
logic [2:0]  bit_i;       // index of the current bit (0 = bit 7)
logic        sck_r;
logic        lead_edge;

assign lead_edge = ~edge_i[0];

// APB
assign pready = 1'b1;

logic wr_acc, start;
assign wr_acc = psel && penable && pwrite;
assign start  = wr_acc && (paddr[3:2] == 2'h2) && !busy && en;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        en <= 1'b0; cpol <= 1'b0; cpha <= 1'b0; csn_pol <= 1'b0; cs_hold <= 1'b0;
        div      <= 16'd7;
        busy     <= 1'b0;
        rx_valid <= 1'b0;
        rx_data  <= 8'h0;
        rx_sh    <= 8'h0;
        tx_byte  <= 8'h0;
        st       <= ST_IDLE;
        hp_cnt   <= 16'h0;
        edge_i   <= 4'h0;
        bit_i    <= 3'h0;
        sck_r    <= 1'b0;
        spi_mosi <= 1'b0;
    end else begin
        // ---- register writes (ignored while a transfer runs) ----
        if (wr_acc && !busy) begin
            case (paddr[3:2])
                2'h0: {cs_hold, csn_pol, cpha, cpol, en} <= pwdata[4:0];
                2'h1: div <= pwdata[15:0];
                default: ;
            endcase
        end

        // ---- transfer start (V6, V7: only when idle and enabled) ----
        if (start) begin
            tx_byte  <= pwdata[7:0];
            rx_valid <= 1'b0;
            busy     <= 1'b1;
            st       <= ST_LEAD;
            hp_cnt   <= 16'h0;          // V4: first half period is exact
            edge_i   <= 4'h0;
            bit_i    <= 3'h0;
            sck_r    <= cpol;           // V3
            if (!cpha) spi_mosi <= pwdata[7];   // mode 0/2: bit 7 before the first edge
        end else if (st != ST_IDLE) begin
            // ---- half-period timer ----
            if (hp_cnt == div) begin
                hp_cnt <= 16'h0;
                if (st == ST_TRAIL) begin
                    busy     <= 1'b0;
                    rx_valid <= 1'b1;
                    rx_data  <= rx_sh;
                    st       <= ST_IDLE;
                end else begin
                    // one SCK edge
                    sck_r  <= ~sck_r;
                    edge_i <= edge_i + 4'h1;
                    if (!cpha) begin
                        if (lead_edge) rx_sh <= {rx_sh[6:0], spi_miso};        // sample
                        else begin                                               // shift
                            if (bit_i != 3'd7) spi_mosi <= tx_byte[3'd6 - bit_i];
                            bit_i <= bit_i + 3'h1;
                        end
                    end else begin
                        if (lead_edge) spi_mosi <= tx_byte[3'd7 - bit_i];        // shift
                        else begin                                               // sample
                            rx_sh <= {rx_sh[6:0], spi_miso};
                            bit_i <= bit_i + 3'h1;
                        end
                    end
                    st <= (edge_i == 4'd15) ? ST_TRAIL : ST_XFER;
                end
            end else hp_cnt <= hp_cnt + 16'h1;
        end
    end
end

// ---- pins ----
logic cs_act;
assign cs_act  = en && (busy || cs_hold);
assign spi_csn = csn_pol ? cs_act : ~cs_act;
assign spi_sck = busy ? sck_r : cpol;

// ---- APB read ----
always_comb begin
    prdata = 32'h0;
    case (paddr[3:2])
        2'h0: prdata = {27'h0, cs_hold, csn_pol, cpha, cpol, en};
        2'h1: prdata = {16'h0, div};
        2'h2: prdata = {24'h0, rx_data};
        2'h3: prdata = {30'h0, rx_valid, busy};
        default: prdata = 32'h0;
    endcase
end

endmodule
`default_nettype wire

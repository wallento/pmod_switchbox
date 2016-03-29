// Copyright 2016 by the authors
//
// Copyright and related rights are licensed under the Solderpad
// Hardware License, Version 0.51 (the "License"); you may not use
// this file except in compliance with the License. You may obtain a
// copy of the License at http://solderpad.org/licenses/SHL-0.51.
// Unless required by applicable law or agreed to in writing,
// software, hardware and materials distributed under this License is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS
// OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the
// License.
//
// Authors:
//    Stefan Wallentowitz <stefan@wallentowitz.de>

module pmod_switchbox
  #(parameter N_PMOD = 4,
    parameter N_GPIO = 8,
    parameter N_SPI = 2,
    parameter N_UART = 2,
    parameter INITIAL = {N_PMOD*(N_GPIO+N_SPI+N_UART){1'b0}})
   (input clk,
    input                 rst,

    // Pmods
    inout [N_PMOD*4-1:0]  pmod,

    // GPIOs
    output [N_GPIO*4-1:0] gpio_in,
    input [N_GPIO*4-1:0]  gpio_out,
    input [N_GPIO*4-1:0]  gpio_oe,

    // SPIs
    input [N_SPI-1:0]     spi_ss,
    input [N_SPI-1:0]     spi_mosi,
    output [N_SPI-1:0]    spi_miso,
    input [N_SPI-1:0]     spi_sck,

    // UARTs
    output [N_UART-1:0]   uart_cts,
    input [N_UART-1:0]    uart_txd,
    output [N_UART-1:0]   uart_rxd,
    input [N_UART-1:0]    uart_rts,

    // Status and Control
    input [7:0]           ctrl_awaddr,
    input [2:0]           ctrl_awprot,
    input                 ctrl_awvalid,
    output                ctrl_awready,

    input [31:0]          ctrl_wdata,
    input [3:0]           ctrl_wstrb,
    input                 ctrl_wvalid,
    output                ctrl_wready,

    output [1:0]          ctrl_bresp,
    output                ctrl_bvalid,
    input                 ctrl_bready,

    input [7:0]           ctrl_araddr,
    input [2:0]           ctrl_arprot,
    input                 ctrl_arvalid,
    output                ctrl_arready,

    output [31:0]         ctrl_rdata,
    output [1:0]          ctrl_resp,
    output                ctrl_rvalid,
    input                 ctrl_rready
    );

   localparam N_PERIPH = N_GPIO+N_SPI+N_UART;
   localparam SPI_BASE = N_GPIO;
   localparam UART_BASE = SPI_BASE+N_SPI;
   
   reg [N_PERIPH-1:0]     mapping[N_PMOD];
   logic [N_PMOD-1:0]     mapping_transposed[N_PERIPH];
   logic                  mapping_error[N_PERIPH];
   
   logic [3:0]            pmod_in [N_PMOD];
   logic [3:0]            pmod_out [N_PMOD];
   logic [3:0]            pmod_oe [N_PMOD];
   
   logic [3:0]            periph_in [N_PERIPH];
   logic [3:0]            periph_out [N_PERIPH];
   logic [3:0]            periph_oe [N_PERIPH];

   genvar                 v,p;
   
   generate
     for (v=0; v<N_PMOD; v=v+1) begin
         for (p=0; p<4; p=p+1) begin
            assign pmod[v*4+p] = pmod_oe[v][p] ? pmod_out[v][p] : 1'bz;
            assign pmod_in[v][p] = pmod[v*4+p];
         end
     end
   endgenerate
   
   generate
     for (v=0; v<N_GPIO; v=v+1) begin : gpio
        for (p=0; p<4; p=p+1) begin
           assign periph_out[v][p] = gpio_out[v*4+p];
           assign periph_oe[v][p] = gpio_oe[v*4+p];
           assign gpio_in[v*4+p] = periph_in[v][p];
        end
     end
      
      for (v=0; v<N_SPI; v=v+1) begin : spi
         assign periph_oe[SPI_BASE+v] = 4'b1011;
         assign periph_out[SPI_BASE+v][0] = spi_ss[v];
         assign periph_out[SPI_BASE+v][1] = spi_mosi[v];
         assign spi_miso[v] = periph_in[SPI_BASE+v][2];
         assign periph_out[SPI_BASE+v][3] = spi_sck[v];
      end
      
      for (v=0; v<N_SPI; v=v+1) begin : uart
         assign periph_oe[UART_BASE+v] = 4'b1010;
         assign uart_cts[v] = periph_in[UART_BASE+v][0];
         assign periph_out[UART_BASE+v][1] = uart_txd[v];
         assign uart_rxd[v] = periph_in[UART_BASE+v][2];
         assign periph_out[UART_BASE+v][3] = uart_rts[v];
      end     
   endgenerate
   
   always @(*) begin : mux_pmod
      integer pm, periph;
      // Defaults
      for (pm=0; pm<N_PMOD; pm=pm+1) begin
         pmod_out[pm] = 4'h0;
         pmod_oe[pm] = 4'h0;
      end

      // Actual muxing
      for (pm=0; pm<N_PMOD; pm=pm+1) begin
         if (^mapping[pm]) begin
            for (periph = 0; periph<N_PERIPH; periph=periph+1) begin
               if (mapping[pm][periph] & !mapping_error[periph]) begin
                  pmod_out[pm] |= periph_out[periph];
                  pmod_oe[pm] |= periph_oe[periph];
               end
            end
         end
      end
   end // block: muxing

   always @(*) begin : mux_periph
      integer pm, periph;
      // Defaults
      for (periph=0; periph<N_PERIPH; periph=periph+1) begin
         periph_in[periph] = 4'h0;
      end

      // Actual muxing
      for (pm=0; pm<N_PMOD; pm=pm+1) begin
         if (^mapping[pm]) begin
            for (periph = 0; periph<N_PERIPH; periph=periph+1) begin
               if (mapping[pm][periph] & !mapping_error[periph]) begin
                  periph_in[periph] |= pmod_in[pm];
               end
            end
         end
      end
   end // block: muxing
   
   always @(*) begin : transpose_mapping
      integer pm, periph;
      for (periph=0; periph<N_PERIPH; periph=periph+1) begin
         for (pm=0; pm<N_PMOD; pm=pm+1) begin
            mapping_transposed[periph][pm] = mapping[pm][periph];
         end

         mapping_error[periph] = |mapping_transposed[periph] &
                                 ~^mapping_transposed[periph];
      end
   end

   logic [N_PMOD*N_PERIPH-1:0] initial_mapping = INITIAL;
   
   always @(posedge clk) begin
      integer pm, periph;
      if (rst) begin
         for (pm=0; pm<N_PMOD; pm=pm+1) begin
            mapping[pm] <= initial_mapping[(pm+1)*N_PERIPH-1 -: N_PERIPH];
         end
      end else begin
         // TODO: Control
      end
   end
   
endmodule // pmod_switchbox

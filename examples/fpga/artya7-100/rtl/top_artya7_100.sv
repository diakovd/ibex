// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
 `include "D:/Work/test_pr/ibex-master/examples/fpga/artya7-100/rtl/IOmodule/defInterface.sv"

module top_artya7_100 (
    input               IO_CLK,
    input               IO_RST_N,
    output [31:0]        LED
);

  parameter int          MEM_SIZE  = 64 * 1024; // 64 kB
  parameter logic [31:0] MEM_START = 32'h00000000;
  parameter logic [31:0] MEM_MASK  = MEM_SIZE-1;

  logic clk_sys, rst_sys_n;

  // Instruction connection to SRAM
  logic        instr_req;
  logic        instr_gnt;
  logic        instr_rvalid;
  logic [31:0] instr_addr;
  logic [31:0] instr_rdata;

  // Data connection to SRAM
  logic        data_req;
  logic        data_gnt;
  logic        data_rvalid;
  logic        data_we;
  logic  [3:0] data_be;
  logic [31:0] data_addr;
  logic [31:0] data_wdata;
  logic [31:0] data_rdata;

  // SRAM arbiter
  logic [31:0] mem_addr;
  logic        mem_req;
  logic        mem_write;
  logic  [3:0] mem_be;
  logic [31:0] mem_wdata;
  logic        mem_rvalid;
  logic [31:0] mem_rdata;
  

 CPUdataMemBus 	CPUdataMem();
 RAMbus  		Data_RAMbus();
 AXI4bus 		AXI4busIO();

  ibex_core #(
     .PMPEnable                ( 0            ),
     .PMPGranularity           ( 0            ),
     .PMPNumRegions            ( 4            ),
     .MHPMCounterNum           ( 0            ),
     .MHPMCounterWidth         ( 40           ),
     .RV32E                    ( 0            ),
     .RV32M                    ( 1            ),
     .MultiplierImplementation ( "fast"       ),  
     .DmHaltAddr(32'h00000000),
     .DmExceptionAddr(32'h00000000)
  ) u_core (
     .clk_i                 (clk_sys),
     .rst_ni                (rst_sys_n),

     .test_en_i             ('b0),

     .hart_id_i             (32'b0),
     // First instruction executed is at 0x0 + 0x80
     .boot_addr_i           (32'h00000000),

     .instr_req_o           (instr_req),
     .instr_gnt_i           (instr_gnt),
     .instr_rvalid_i        (instr_rvalid),
     .instr_addr_o          (instr_addr),
     .instr_rdata_i         (instr_rdata),
     .instr_err_i           ('b0),

     .data_req_o            (CPUdataMem.data_req),
     .data_gnt_i            (CPUdataMem.data_gnt),
     .data_rvalid_i         (CPUdataMem.data_rvalid),
     .data_we_o             (CPUdataMem.data_we),
     .data_be_o             (CPUdataMem.data_be),
     .data_addr_o           (CPUdataMem.data_addr),
     .data_wdata_o          (CPUdataMem.data_wdata),
     .data_rdata_i          (CPUdataMem.data_rdata),
     .data_err_i            ('b0),

     .irq_software_i        (1'b0),
     .irq_timer_i           (1'b0),
     .irq_external_i        (1'b0),
     .irq_fast_i            (15'b0),
     .irq_nm_i              (1'b0),

     .debug_req_i           ('b0),

     .fetch_enable_i        ('b1),
     .core_sleep_o          ()
  );

 AXI4_bus_mux AXI4_bus_mux_inst(
    //CPU BUS
	.CPUdataMem(CPUdataMem),
	
	.Data_RAMbus(Data_RAMbus),
	
	.AXI4busIO(AXI4busIO),
	
	.Clk(IO_CLK)
 );

  // Connect Ibex to SRAM
  // always_comb begin
    // mem_req        = 1'b0;
    // mem_addr       = 32'b0;
    // mem_write      = 1'b0;
    // mem_be         = 4'b0;
    // mem_wdata      = 32'b0;
    // if (instr_req) begin
      // mem_req        = (instr_addr & ~MEM_MASK) == MEM_START;
      // mem_addr       = instr_addr;
    // end else if (data_req) begin
      // mem_req        = (data_addr & ~MEM_MASK) == MEM_START;
      // mem_write      = data_we;
      // mem_be         = data_be;
      // mem_addr       = data_addr;
      // mem_wdata      = data_wdata;
    // end
  // end
 
  // SRAM block for instruction 
  ram_1p #(
    .Depth(MEM_SIZE / 4)
  ) u_ram_instr (
    .clk_i     ( clk_sys        ),
    .rst_ni    ( rst_sys_n      ),
    .req_i     ( instr_req      ),
    .we_i      ( 1'b0 	        ),
    .be_i      ( 4'hf	        ),
    .addr_i    ( instr_addr     ),
    .wdata_i   ( 32'h00000000   ),
    .rvalid_o  ( instr_rvalid   ),
    .rdata_o   ( instr_rdata    )
  );

  // SRAM block for data storage
  ram_1p #(
    .Depth(MEM_SIZE / 4)
  ) u_ram_data (
    .clk_i     ( clk_sys        ),
    .rst_ni    ( rst_sys_n      ),
    .req_i     ( Data_RAMbus.req),
    .we_i      ( Data_RAMbus.we ),
    .be_i      ( Data_RAMbus.be ),
    .addr_i    ( Data_RAMbus.addr),
    .wdata_i   ( Data_RAMbus.wdata),
    .rvalid_o  ( Data_RAMbus.rvalid),
    .rdata_o   ( Data_RAMbus.rdata)
  );

  // SRAM to Ibex
  // assign instr_rdata    = mem_rdata;
  // assign data_rdata     = mem_rdata;
  // assign instr_rvalid   = mem_rvalid;
  
  assign mem_req = (instr_addr & ~MEM_MASK) == MEM_START;
  always_ff @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      instr_gnt    <= 'b0;
      // data_gnt     <= 'b0;
      // data_rvalid  <= 'b0;
    end else begin
      instr_gnt    <= instr_req && mem_req;
      // data_gnt     <= ~instr_req && data_req && mem_req;
      // data_rvalid  <= ~instr_req && data_req && mem_req;
    end
  end


  // Connect the LED output to the lower four bits of the most significant
  // byte
  // logic [3:0] leds;
  // always_ff @(posedge clk_sys or negedge rst_sys_n) begin
    // if (!rst_sys_n) begin
      // leds <= 4'b0;
    // end else begin
      // if (mem_req && data_req && data_we) begin
        // for (int i = 0; i < 4; i = i + 1) begin
          // if (data_be[i] == 1'b1) begin
            // leds <= data_wdata[i*8 +: 4];
          // end
        // end
      // end
    // end
  // end
  // assign LED = leds;

  // Clock and reset
  // clkgen_xil7series
    // clkgen(
      // .IO_CLK,
      // .IO_RST_N,
      // .clk_sys,
      // .rst_sys_n
    // );

 assign clk_sys = IO_CLK;
 assign rst_sys_n = IO_RST_N;
 
 IOmodule IOmodule_inst(
   // AXI4 lite slave interface
  .AXI4s(AXI4busIO),	

   //IO out
   .IO(LED),
   
   .Rst(!IO_RST_N),	
   .Clk(IO_CLK)
 );
 
 
endmodule

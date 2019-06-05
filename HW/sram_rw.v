// SRAM read/write module   sram_rw.v
// for altera DE2 board
// Ozgun Ayaz, Yildiz Technical University 2011

module sram_read_write(
	inout	   reg [15:0]	SRAM_DQ,				//	SRAM Data bus 16 Bits
	output   reg [17:0]	SRAM_ADDR,			//	SRAM Address bus 18 Bits
	output	reg		   SRAM_WE_N,			//	SRAM Write Enable
	output	reg	   	SRAM_CE_N,			//	SRAM Chip Enable
	output	reg	    	SRAM_OE_N,			//	SRAM Output Enable
	inout 	    [15:0]  data,
	input 		 [17:0]  addr,
	input 				   operation, 			//0 write 1 rd
	input 				   enable,
	output 	reg 			busy);

	reg [15:0] data_write_latch;
	assign data = enable ? (operation ? 16'bz : data_write_latch) : 16'bz;	
	always @( enable or ~enable ) begin
		case ( {operation, enable} )
		2'b01: begin
			busy <= 1;
			SRAM_ADDR <= addr;
			SRAM_DQ <= data;
			SRAM_WE_N <= 0;
			busy <= 0;
			SRAM_WE_N <= 1;
			end
		2'b11: begin
			busy <= 1;
			SRAM_ADDR <= addr;
			SRAM_OE_N <= 0;
			data_write_latch <= SRAM_DQ;
			busy <= 0;
			SRAM_OE_N <= 1;
			end
		2'bx0: begin
			busy <= 0;
			SRAM_WE_N <= 1;
			SRAM_OE_N <= 1;
			 end
		endcase
		
	end
 endmodule
 
 
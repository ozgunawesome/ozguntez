module DE2_USB_API
	(
		////////////////////	Clock Input	 	////////////////////	 
		OSC_27,							//	27 MHz
		OSC_50,							//	50 MHz
		EXT_CLOCK,						//	External Clock
		////////////////////	Push Button		////////////////////
		KEY,							//	Button[3:0]
		////////////////////	DPDT Switch		////////////////////
		DPDT_SW,						//	DPDT Switch[17:0]
		////////////////////	7-SEG Dispaly	////////////////////
		HEX0,							//	Seven Segment Digital 0
		HEX1,							//	Seven Segment Digital 1
		HEX2,							//	Seven Segment Digital 2
		HEX3,							//	Seven Segment Digital 3
		HEX4,							//	Seven Segment Digital 4
		HEX5,							//	Seven Segment Digital 5
		HEX6,							//	Seven Segment Digital 6
		HEX7,							//	Seven Segment Digital 7
		////////////////////////	LED		////////////////////////
		LED_GREEN,					//	LED Green[8:0]
		LED_RED,						//	LED Red[17:0]
		////////////////////////	UART	////////////////////////
		UART_TXD,					//	UART Transmitter
		UART_RXD,					//	UART Rceiver
		////////////////////////	IRDA	////////////////////////
		IRDA_TXD,					//	IRDA Transmitter
		IRDA_RXD,					//	IRDA Rceiver
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,							//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 12 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask 
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 0
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	Flash Interface		////////////////
		FL_DQ,							//	FLASH Data bus 8 Bits
		FL_ADDR,						//	FLASH Address bus 22 Bits
		FL_WE_N,						//	FLASH Write Enable
		FL_RST_N,						//	FLASH Reset
		FL_OE_N,						//	FLASH Output Enable
		FL_CE_N,						//	FLASH Chip Enable
		////////////////////	SRAM Interface		////////////////
		SRAM_DQ,						//	SRAM Data bus 16 Bits
		SRAM_ADDR,						//	SRAM Address bus 18 Bits
		SRAM_UB_N,						//	SRAM High-byte Data Mask 
		SRAM_LB_N,						//	SRAM Low-byte Data Mask 
		SRAM_WE_N,						//	SRAM Write Enable
		SRAM_CE_N,						//	SRAM Chip Enable
		SRAM_OE_N,						//	SRAM Output Enable
		////////////////////	ISP1362 Interface	////////////////
		OTG_DATA,						//	ISP1362 Data bus 16 Bits
		OTG_ADDR,						//	ISP1362 Address 2 Bits
		OTG_CS_N,						//	ISP1362 Chip Select
		OTG_RD_N,						//	ISP1362 Write
		OTG_WR_N,						//	ISP1362 Read
		OTG_RST_N,						//	ISP1362 Reset
		OTG_FSPEED,						//	USB Full Speed,	0 = Enable, Z = Disable
		OTG_LSPEED,						//	USB Low Speed, 	0 = Enable, Z = Disable
		OTG_INT0,						//	ISP1362 Interrupt 0
		OTG_INT1,						//	ISP1362 Interrupt 1
		OTG_DREQ0,						//	ISP1362 DMA Request 0
		OTG_DREQ1,						//	ISP1362 DMA Request 1
		OTG_DACK0_N,					//	ISP1362 DMA Acknowledge 0
		OTG_DACK1_N,					//	ISP1362 DMA Acknowledge 1
		////////////////////	LCD Module 16X2		////////////////
		LCD_ON,							//	LCD Power ON/OFF
		LCD_BLON,						//	LCD Back Light ON/OFF
		LCD_RW,							//	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN,							//	LCD Enable
		LCD_RS,							//	LCD Command/Data Select, 0 = Command, 1 = Data
		LCD_DATA,						//	LCD Data bus 8 bits
		////////////////////	SD_Card Interface	////////////////
		SD_DAT,							//	SD Card Data
		SD_DAT3,						//	SD Card Data 3
		SD_CMD,							//	SD Card Command Signal
		SD_CLK,							//	SD Card Clock
		////////////////////	USB JTAG link	////////////////////
		TDI,  							// CPLD -> FPGA (data in)
		TCK,  							// CPLD -> FPGA (clk)
		TCS,  							// CPLD -> FPGA (CS)
	    TDO,  							// FPGA -> CPLD (data out)
		////////////////////	I2C		////////////////////////////
		I2C_SDAT,						//	I2C Data
		I2C_SCLK,						//	I2C Clock
		////////////////////	PS2		////////////////////////////
		PS2_DAT,						//	PS2 Data
		PS2_CLK,						//	PS2 Clock
		////////////////////	VGA		////////////////////////////
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,  						//	VGA Blue[9:0]
		////////////	Ethernet Interface	////////////////////////
		ENET_DATA,						//	DM9000A DATA bus 16Bits
		ENET_CMD,						//	DM9000A Command/Data Select, 0 = Command, 1 = Data
		ENET_CS_N,						//	DM9000A Chip Select
		ENET_WR_N,						//	DM9000A Write
		ENET_RD_N,						//	DM9000A Read
		ENET_RST_N,						//	DM9000A Reset
		ENET_INT,						//	DM9000A Interrupt
		ENET_CLK,						//	DM9000A Clock 25 MHz
		////////////////	Audio CODEC		////////////////////////
		AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
		AUD_ADCDAT,						//	Audio CODEC ADC Data
		AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
		AUD_DACDAT,						//	Audio CODEC DAC Data
		AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
		AUD_XCK,						//	Audio CODEC Chip Clock
		////////////////	TV Decoder		////////////////////////
		TD_DATA,    					//	TV Decoder Data bus 8 bits
		TD_HS,							//	TV Decoder H_SYNC
		TD_VS,							//	TV Decoder V_SYNC
		TD_RESET,						//	TV Decoder Reset
		////////////////////	GPIO	////////////////////////////
		GPIO_0,							//	GPIO Connection 0
		GPIO_1							//	GPIO Connection 1
	);

////////////////////////	Clock Input	 	////////////////////////
input			OSC_27;					//	27 MHz
input			OSC_50;					//	50 MHz
input			EXT_CLOCK;				//	External Clock
////////////////////////	Push Button		////////////////////////
input	[3:0]	KEY;					//	Button[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	[17:0]	DPDT_SW;				//	DPDT Switch[17:0]
////////////////////////	7-SEG Dispaly	////////////////////////
output	[6:0]	HEX0;					//	Seven Segment Digital 0
output	[6:0]	HEX1;					//	Seven Segment Digital 1
output	[6:0]	HEX2;					//	Seven Segment Digital 2
output	[6:0]	HEX3;					//	Seven Segment Digital 3
output	[6:0]	HEX4;					//	Seven Segment Digital 4
output	[6:0]	HEX5;					//	Seven Segment Digital 5
output	[6:0]	HEX6;					//	Seven Segment Digital 6
output	[6:0]	HEX7;					//	Seven Segment Digital 7
////////////////////////////	LED		////////////////////////////
output	[8:0]	LED_GREEN;				//	LED Green[8:0]
output	[17:0]	LED_RED;				//	LED Red[17:0]
////////////////////////////	UART	////////////////////////////
output			UART_TXD;				//	UART Transmitter
input			UART_RXD;				//	UART Rceiver
////////////////////////////	IRDA	////////////////////////////
output			IRDA_TXD;				//	IRDA Transmitter
input			IRDA_RXD;				//	IRDA Rceiver
///////////////////////		SDRAM Interface	////////////////////////
inout	[15:0]	DRAM_DQ;				//	SDRAM Data bus 16 Bits
output	[11:0]	DRAM_ADDR;				//	SDRAM Address bus 12 Bits
output			DRAM_LDQM;				//	SDRAM Low-byte Data Mask 
output			DRAM_UDQM;				//	SDRAM High-byte Data Mask
output			DRAM_WE_N;				//	SDRAM Write Enable
output			DRAM_CAS_N;				//	SDRAM Column Address Strobe
output			DRAM_RAS_N;				//	SDRAM Row Address Strobe
output			DRAM_CS_N;				//	SDRAM Chip Select
output			DRAM_BA_0;				//	SDRAM Bank Address 0
output			DRAM_BA_1;				//	SDRAM Bank Address 0
output			DRAM_CLK;				//	SDRAM Clock
output			DRAM_CKE;				//	SDRAM Clock Enable
////////////////////////	Flash Interface	////////////////////////
inout	[7:0]	FL_DQ;					//	FLASH Data bus 8 Bits
output	[21:0]	FL_ADDR;				//	FLASH Address bus 22 Bits
output			FL_WE_N;				//	FLASH Write Enable
output			FL_RST_N;				//	FLASH Reset
output			FL_OE_N;				//	FLASH Output Enable
output			FL_CE_N;				//	FLASH Chip Enable
////////////////////////	SRAM Interface	////////////////////////
inout	[15:0]	SRAM_DQ;				//	SRAM Data bus 16 Bits
output	[17:0]	SRAM_ADDR;				//	SRAM Address bus 18 Bits
output			SRAM_UB_N;				//	SRAM High-byte Data Mask 
output			SRAM_LB_N;				//	SRAM Low-byte Data Mask 
output			SRAM_WE_N;				//	SRAM Write Enable
output			SRAM_CE_N;				//	SRAM Chip Enable
output			SRAM_OE_N;				//	SRAM Output Enable
////////////////////	ISP1362 Interface	////////////////////////
inout	[15:0]	OTG_DATA;				//	ISP1362 Data bus 16 Bits
output	[1:0]	OTG_ADDR;				//	ISP1362 Address 2 Bits
output			OTG_CS_N;				//	ISP1362 Chip Select
output			OTG_RD_N;				//	ISP1362 Write
output			OTG_WR_N;				//	ISP1362 Read
output			OTG_RST_N;				//	ISP1362 Reset
output			OTG_FSPEED;				//	USB Full Speed,	0 = Enable, Z = Disable
output			OTG_LSPEED;				//	USB Low Speed, 	0 = Enable, Z = Disable
output			OTG_INT0;				//	ISP1362 Interrupt 0
output			OTG_INT1;				//	ISP1362 Interrupt 1
output			OTG_DREQ0;				//	ISP1362 DMA Request 0
output			OTG_DREQ1;				//	ISP1362 DMA Request 1
output			OTG_DACK0_N;			//	ISP1362 DMA Acknowledge 0
output			OTG_DACK1_N;			//	ISP1362 DMA Acknowledge 1
////////////////////	LCD Module 16X2	////////////////////////////
inout	[7:0]	LCD_DATA;				//	LCD Data bus 8 bits
output			LCD_ON;					//	LCD Power ON/OFF
output			LCD_BLON;				//	LCD Back Light ON/OFF
output			LCD_RW;					//	LCD Read/Write Select, 0 = Write, 1 = Read
output			LCD_EN;					//	LCD Enable
output			LCD_RS;					//	LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////	SD Card Interface	////////////////////////
inout			SD_DAT;					//	SD Card Data
inout			SD_DAT3;				//	SD Card Data 3
inout			SD_CMD;					//	SD Card Command Signal
output			SD_CLK;					//	SD Card Clock
////////////////////////	I2C		////////////////////////////////
inout			I2C_SDAT;				//	I2C Data
output			I2C_SCLK;				//	I2C Clock
////////////////////////	PS2		////////////////////////////////
input		 	PS2_DAT;				//	PS2 Data
input			PS2_CLK;				//	PS2 Clock
////////////////////	USB JTAG link	////////////////////////////
input  			TDI;					// CPLD -> FPGA (data in)
input  			TCK;					// CPLD -> FPGA (clk)
input  			TCS;					// CPLD -> FPGA (CS)
output 			TDO;					// FPGA -> CPLD (data out)
////////////////////////	VGA			////////////////////////////
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
////////////////	Ethernet Interface	////////////////////////////
inout	[15:0]	ENET_DATA;				//	DM9000A DATA bus 16Bits
output			ENET_CMD;				//	DM9000A Command/Data Select, 0 = Command, 1 = Data
output			ENET_CS_N;				//	DM9000A Chip Select
output			ENET_WR_N;				//	DM9000A Write
output			ENET_RD_N;				//	DM9000A Read
output			ENET_RST_N;				//	DM9000A Reset
input			ENET_INT;				//	DM9000A Interrupt
output			ENET_CLK;				//	DM9000A Clock 25 MHz
////////////////////	Audio CODEC		////////////////////////////
output			AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			AUD_ADCDAT;				//	Audio CODEC ADC Data
inout			AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output			AUD_DACDAT;				//	Audio CODEC DAC Data
inout			AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output			AUD_XCK;				//	Audio CODEC Chip Clock
////////////////////	TV Devoder		////////////////////////////
input	[7:0]	TD_DATA;    			//	TV Decoder Data bus 8 bits
input			TD_HS;					//	TV Decoder H_SYNC
input			TD_VS;					//	TV Decoder V_SYNC
output			TD_RESET;				//	TV Decoder Reset
////////////////////////	GPIO	////////////////////////////////
inout	[35:0]	GPIO_0;					//	GPIO Connection 0
inout	[35:0]	GPIO_1;					//	GPIO Connection 1
////////////////////////////////////////////////////////////////////


//	USB JTAG
wire [7:0] mRXD_DATA,mTXD_DATA;
wire mRXD_Ready,mTXD_Done,mTXD_Start;
wire mTCK;
//	FLASH
wire [21:0] mFL_ADDR;
wire [7:0] mFL2RS_DATA,mRS2FL_DATA;
wire [2:0] mFL_CMD;
wire mFL_Ready,mFL_Start;
//	SRAM
wire [17:0]	mSR_ADDR;
wire [15:0]	mSR2RS_DATA,mRS2SR_DATA;
wire		mSR_OE,mSR_WE;
//	SEG7
wire [31:0] mSEG7_DIG;
//	Async Port Select
wire [2:0] mSDR_Select;
wire [2:0] mFL_Select;
wire [2:0] mSR_Select;
//	External IO
wire [7:0] mExt_IO;
//	FLASH Async Port
wire [21:0] mFL_AS_ADDR_1;
wire [21:0] mFL_AS_ADDR_2;
wire [21:0] mFL_AS_ADDR_3;
wire [7:0]	mFL_AS_DATA_1;
wire [7:0]	mFL_AS_DATA_2;
wire [7:0]	mFL_AS_DATA_3;
//	All inout port turn to tri-state
assign	OTG_DATA	=	16'hzzzz;
assign	SD_DAT		=	1'bz;
assign	ENET_DATA	=	16'hzzzz;
assign	GPIO_0		=	36'hzzzzzzzzz;
assign	GPIO_1		=	36'hzzzzzzzzz;

CLK_LOCK 			p0	(	.inclk(TCK),.outclk(mTCK)	);

Reset_Delay			d0	(	.iCLK(OSC_50),.oRESET(DLY_RST)	);

SEG7_LUT_8 			u0	(	HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7,mSEG7_DIG );

USB_JTAG			u1	(	//	HOST
							.iTxD_DATA(mTXD_DATA),.oTxD_Done(mTXD_Done),.iTxD_Start(mTXD_Start),
							.oRxD_DATA(mRXD_DATA),.oRxD_Ready(mRXD_Ready),.iRST_n(KEY[0]),.iCLK(OSC_50),
							//	JTAG
							.TDO(TDO),.TDI(TDI),.TCS(TCS),.TCK(mTCK)	);

Multi_Flash			u2	(	//	Host Side
							mFL2RS_DATA,mRS2FL_DATA,mFL_ADDR,mFL_CMD,mFL_Ready,mFL_Start,
							//	Async Side 1
							mFL_AS_DATA_1,mFL_AS_ADDR_1,
							//	Async Side 2
							mFL_AS_DATA_2,mFL_AS_ADDR_2,
							//	Async Side 3
							mFL_AS_DATA_3,mFL_AS_ADDR_3,
							//	Control Signals
							mFL_Select,	OSC_50, KEY[0],
							//	Flash Interface
							FL_DQ,FL_ADDR,FL_WE_N,FL_CE_N,FL_OE_N,FL_RST_N);

CMD_Decode			u5	(	//	USB JTAG
							.iRXD_DATA(mRXD_DATA),.iRXD_Ready(mRXD_Ready),
						 	.oTXD_DATA(mTXD_DATA),.oTXD_Start(mTXD_Start),.iTXD_Done(mTXD_Done),
						 	//	FLASH
							.iFL_DATA(mFL2RS_DATA),.oFL_DATA(mRS2FL_DATA),
						 	.oFL_ADDR(mFL_ADDR),.iFL_Ready(mFL_Ready),
						 	.oFL_Start(mFL_Start),.oFL_CMD(mFL_CMD),
							//	SRAM
							.iSR_DATA(mSR2RS_DATA),.oSR_DATA(mRS2SR_DATA),
							.oSR_ADDR(mSR_ADDR),
							.oSR_WE_N(mSR_WE),.oSR_OE_N(mSR_OE),
							// Yesil LED
							.oLED_GREEN(LED_GREEN),
							//	Async Port Select
							.oSDR_Select(mSDR_Select),
							.oFL_Select(out_mFL_Select),
							.oSR_Select(out_mSR_Select),
							//	Ext Control Signals
							.oExt_IO(mExt_IO),
							//	Control
						 	.iCLK(OSC_50),.iRST_n(KEY[0])	);

Multi_Sram			u6	(	//	Host
							.oHS_DATA(mSR2RS_DATA),.iHS_DATA(mRS2SR_DATA),.iHS_ADDR(mSR_ADDR),
							.iHS_WE_N(mSR_WE),.iHS_OE_N(mSR_OE),
							.oAS1_DATA(OZGUN_SRAM_DOUT),.iAS1_DATA(OZGUN_SRAM_DIN),
							.iAS1_ADDR(OZGUN_SRAM_ADDR),
							.iAS1_WE_N(OZGUN_SRAM_WE),.iAS1_OE_N(OZGUN_SRAM_OE),
							//	Control Signals
							.iSelect(mSR_Select),.iRST_n(KEY[0]),
							//	SRAM
							.SRAM_DQ(SRAM_DQ),
							.SRAM_ADDR(SRAM_ADDR),
							.SRAM_UB_N(SRAM_UB_N),
							.SRAM_LB_N(SRAM_LB_N),
							.SRAM_WE_N(SRAM_WE_N),
							.SRAM_CE_N(SRAM_CE_N),
							.SRAM_OE_N(SRAM_OE_N));
							
matrix_mult matrix1 (	// reset ve clock kontrol
								ClkSelect, matrix_reset,						
								// kontrol giris cikislari
							   matrix_enable, matrix_busy, matrix_ready,
								// islem yapilacak bellek adresleri
							   flash_addr, sram_addr, sram_store,
								//sram arayuzu
							   OZGUN_SRAM_DOUT, OZGUN_SRAM_DIN, OZGUN_SRAM_ADDR, OZGUN_SRAM_WE, OZGUN_SRAM_OE,   
							   //flash arayuzu
								mFL_AS_DATA_1, mFL_AS_ADDR_1, 
								//debug
								mSEG7_DIG[31:16], DPDT_SW[7:0]);	

// OZGUN'S SRAM CONTROL INTERFACE SIGNALS
wire [15:0]OZGUN_SRAM_DOUT;
wire [15:0]OZGUN_SRAM_DIN;
wire [17:0]OZGUN_SRAM_ADDR;
wire OZGUN_SRAM_WE, OZGUN_SRAM_OE;

// SRAM & FLASH access multiplexer
assign mSR_Select = matrix_busy ? 1 : out_mSR_Select;
assign mFL_Select = matrix_busy ? 1 : out_mFL_Select;

// clock multiplexer
assign ClkSelect = DPDT_SW[17] ? OSC_50: OSC_27;

// islem adres tanimlari
reg [21:0] flash_addr;
reg [17:0] sram_addr;
reg [17:0] sram_store;

reg matrix_enable;
reg matrix_mult_enabled;
reg matrix_reset;
reg sistem_ready;
reg [7:0] iteration_count;

assign LED_RED[0] = matrix_enable;
assign LED_RED[1] = matrix_busy;
assign LED_RED[2] = matrix_ready;
assign LED_RED[3] = sistem_ready;
assign mSEG7_DIG[7:0] = iteration_count;
reg [2:0] sss;
assign mSEG7_DIG[15:8] = sss;

always @ (posedge OSC_50) begin
	if (!KEY[0]) sss <= 0;
	case (sss)
		0:
		begin
			if (!KEY[1]) begin
				matrix_mult_enabled <= 1;
				sistem_ready <= 0;
				sss <= 1;
			end else begin
				matrix_reset <= 1;
				matrix_enable <= 0;
				matrix_mult_enabled <= 0;
				iteration_count <= 0;
				flash_addr <= 0;
				sram_addr <= 18'h10000;
				sram_store <= 18'h20000;
			end
		end
		1:
		begin 
			if(iteration_count == 64) begin
				sss <= 6;
			end else begin
				sss <= 2;
				matrix_reset <= 1;
			end
		end
		2:
		begin
			matrix_reset <= 0;
			sss <= 3;
		end
		3:
		begin
			matrix_enable <= 1;
			sss <= 4;
		end
		4:
		begin 
			if (matrix_ready) begin
				sss <= 5;
				matrix_enable <= 0;
			end
		end
		5:
		begin
			flash_addr <= flash_addr + 22'h10000;
			sram_store <= sram_store + 18'h200; // h400 degil neden degil ? cunku sram wordleri 16-bit
			iteration_count <= iteration_count + 1;
			sss <= 1;
		end
		6:
		begin
			matrix_mult_enabled <= 0;
			sistem_ready <= 1;
			sss <= 0;
		end
	endcase
end



endmodule


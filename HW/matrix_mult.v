// A X B matrix multiplier module
// in which A is n*n square matrix and
// B is a n-element column vector
// read A from flash B from SRAM, then
// multiply A with B and store in SRAM.
// Ozgun Ayaz, Yildiz Technical University 2011

module matrix_mult(
	// Control I/O
	input clk,
	input reset,
	input enable,
	output reg busy,
	output reg ready,

	// Memory Addresses
	input [21:0] flashStartAddr,
	input [17:0] sramStartAddr,
	input [17:0] sramStoreAddr,
	
	// Memory 1 Access I/O
	input      [15:0] sramDataRead,
	output reg [15:0] sramDataWrite,
	output reg [17:0]  sramAddr,
	output reg sramWriteEnable,
	output reg sramOutEnable,
	
	// Memory 2 Access I/O
	input      [7:0]  flashData,
	output reg [21:0] flashAddr,
	
	// Debugging code
	output [15:0] debug,
	input [7:0] DelayAmount
	);
		
		
	// Debug icin konuldu	
	assign debug[7:0] = state;
	assign debug[15:8] = DelayAmount;
	// Debug icin konuldu
	
	
	// Matris boyutu icin N parametresi
	parameter sizeN = 64;  
		
	// Ic degiskenler
	reg [7:0] state;
	reg [127:0] temp;
	reg [127:0] temp2;
	reg [7:0] i;
	reg [7:0] j;
	reg [7:0] delay;
	reg [7:0] cursor;

	// tasarim begin
	always @ (posedge clk or posedge reset) begin
	if (reset) begin
		state <= 0;
		busy <= 0;
		ready <= 0;
		enable_fpu <= 0;
	end else begin
		case (state)
			0:  // durum makinesinin rolanti konumu 
			if (enable) begin
				busy <= 1;
				sramOutEnable <= 1;
				sramWriteEnable <= 1;
				state <= 1;
				ready <= 0;
				enable_fpu <= 0;
				fpu_op <= 0;
			end
			1:  // isaret okuma basla
			begin
				ready <= 0;
				i <= 0;
				state <= 2;
			end
			2: // matrix B ana durum satir okumaya gidecek
			begin
				if (i == sizeN) begin
					state <= 3; //islem tamam begin read A
				end else begin
					temp <= 0;	  //B[i] <= 0;
					state <= 100; //satir oku
					sramOutEnable <= 0;
				end
			end
			100: //matrix b
			begin
				if (i == sizeN) begin
					state <= 3;  // read A begin from FLASH
				end else begin
					sramAddr <= sramStartAddr + ((i + 1) * 8) - 1; // oh i have a baaaad feelin about this
					cursor <= 8;
					state <= 101;
				end
			end
			101: //set cursor
			begin
				if (cursor == 0) begin
					Bwe <= 1;
					Bwrite <= temp;
					Baddr <= i;
					state <= 90;
				end else begin
					temp <= temp << 16;
					state <= 102;
				end
			end			
			90:
			begin
				i <= i + 1;
				state <= 100;
				Bwe <= 0;
			end
			102:
			begin 				
				temp <= temp + sramDataRead; //blockram [i]
				state <= 103;
				cursor <= cursor - 1;
			end
			103: //read from buffer
			begin
				sramAddr <= sramAddr - 1; // burada kesinlikle 1 clk delay koymus olduk hic gerek yoktu ama boku bokuna overwrite etmeyelim
				state <= 101;
			end
			3: // begin read matrix A from FLASH
			begin
				i <= 0;
				j <= 0;
				state <= 4;
			end
			4: // matrix A ana durum satir okumaya gidecek
			begin
				if (j == sizeN) begin
					i <= 0;
					j <= 0;
					state <= 6; //islem tamam
				end else begin
					Xaddr <= j; //blockram X[j] <= 0;
					Xwe <= 1;
					Xwrite <= 0;
					state <= 249; //satir oku
				end
			end
			249:
			begin
			//opb <= B[i];  //B[i] load from blockram
				Xwe <= 0;
				state <= 5;
			end
			5: //matrix a carpmaya baslaniyor
			begin
				Xwe <= 0;
				enable_fpu <= 0;
				if (i == sizeN) begin
					i <= 0;
					j <= j + 1;
					state <= 4;  //satir tamam, bir artir kontrole git.
				end else begin
					flashAddr <= flashStartAddr + (((j * sizeN) + i + 1) * 16) - 1;
					cursor <= 16;
					state <= 120;
					temp <= 0;
				end
			end
			120: // cursor belirle
			begin
				if (cursor == 0) begin
					state <= 124; 
					// element buffera geldi
				end else begin
					temp  <= temp << 8;
					delay <= DelayAmount;
					state <= 121;
				end
			end
			121:
			begin //delay for N clks					
				if (delay == 0) begin
					temp <= temp | flashData;
					state <= 122;
					cursor <= cursor - 1;
				end else begin
					delay <= delay - 1;
				end
			end
			122: //read from buffer
			begin
				flashAddr <= flashAddr - 1;
				state <= 120;
			end
			124:
			begin
				// multiply flash data with B[i]
				//temp2 <= temp * B[i]; complex multiply ieee standards
				//opb <= B[i];  //B[i] load from blockram
				opa <= temp;
				Baddr <= i;
				fpu_op <= 1;
				fpu_reset <= 1;
				state <= 210;//61;
			end
			210:
			begin
				fpu_reset <= 0;
				opb <= Bread; //aslinda B
				state <=70;
			end
			70:
			begin
				enable_fpu <= 1;
				state <= 71;
			end
			71:
			begin
				if (fpu_ready) begin
					temp2 <= fpu_out;
					state <= 13;
				end
			end
			13:
			begin
				//X[j] <= X[j] + temp2; // Complex toplama operation'u
				enable_fpu <= 0;
				fpu_reset <= 1;
				//opa <= X[j]; //BlockRAM fetch X[j]
				Xaddr <= j;
				opb <= temp2;
				i <= i + 1;
				fpu_op <= 0;
				state <= 62;
			end
			62: 
			begin
				opa <= Xread;
				state <= 80;
				fpu_reset <= 0;
			end
			80:
			begin
				enable_fpu <= 1;
				state <= 81;
			end
			81:
			begin // X[j] yeni degeri yaz.
				if (fpu_ready) begin
					Xwe <= 1;
					Xwrite <= fpu_out;
					state <= 5;  
				end
			end
			6:  // ama once modulus sq lerinin alinmasi
			begin				
				if (i == sizeN) begin
					i <= 0;
					state <= 7;
				end else begin
					Xaddr <= i;
					state <= 211;
					fpu_reset <= 1;
				end
			end
			211: 
			begin
				fpu_reset <= 0;
				opa <= Xread;
				opb <= Xread;
				fpu_op <= 2'b10;  // complex modulus square operation
				state <= 216;
			end
			216:
			begin
				enable_fpu <= 1;
				state <= 213;
			end
			213:
			begin
				if (fpu_ready) begin
					Xwrite <= fpu_out; // x(j)
					Xwe <= 1;
					state <= 212;  // isimiz bitti mi bakmaya gidelim
					i <= i + 1;
				end
			end
			212:
			begin
				Xwe <= 0;
				enable_fpu <= 0;
				state <= 6;
			end
			7: //burdan itibaren sonucu bellege yaziyorus
			begin
				if (i == sizeN) begin
					state <= 14;
				end else begin
					sramAddr <= sramStoreAddr + (i * 8);
					Xaddr <= i;
					cursor <= 8;
					state <= 89;
				end
			end
			89:
			begin
				temp <= Xread;
				state <= 50;
			end
			50: //matrix writings
			begin
				if (cursor == 0) begin
					state <= 7;  // element is written, continue
					i <= i + 1;
				end else begin
					sramDataWrite <= temp;
					state <= 54;
				end
			end
			54:
			begin
				sramWriteEnable <= 1;
				state <= 51;
			end
			51:
			begin
				sramWriteEnable <= 0;
				state <= 52;
			end
			52:
			begin
				sramAddr <= sramAddr + 1;
				cursor <= cursor - 1;
				temp <= temp >> 16;
				state <= 50;
			end
			14:
			begin // 1 clk delay
				busy <= 0;
				state <= 15;
			end
			15:  // pizza pi$ti afiyet olsun
			begin
				ready <= 1;
				state <= 8'hFF; // icinden cikilmaz bir duruma git
			end
		endcase
	end
	end
	// dahil edilen moduller
	// OZGUN SUPER MEGA KOMPLEKS DOUBLE FLOAT PROCESSING MODULE
	reg enable_fpu, fpu_reset;
	reg [1:0] fpu_op;
	reg [127:0] opa;
	reg [127:0] opb;
	wire [127:0] fpu_out;
	wire fpu_ready;
	complex_ops c1(clk, fpu_reset, enable_fpu, fpu_op, opa, opb, fpu_out, fpu_ready);
	// isaret matrisinin bufferi icin block RAM
	reg [5:0] Baddr;
	reg [127:0] Bwrite;
	reg Bwe;
	wire [127:0] Bread;
	ozgun_block_ram ram1(clk, Baddr, Bwe, Bwrite, Bread);
	// sonucun bufferi icin block RAM
	reg [5:0] Xaddr;
	reg [127:0] Xwrite;
	reg Xwe;
	wire [127:0] Xread;
	ozgun_block_ram ram2(clk, Xaddr, Xwe, Xwrite, Xread);
	// dahil edilen moduller bu kadardi
endmodule	
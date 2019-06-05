// Verilog implemetation for builtin Altera Cyclone II block RAM
// Ozgun Ayaz, Yildiz Technical University 2011


module ozgun_block_ram #(parameter ADDR_WIDTH=6, DATA_WIDTH=128, DEPTH=4096)
(
 input                   clk,       // clock 
 input  [ADDR_WIDTH-1:0] addr,      // adres 
 input                   we,        // write enable signal
 input  [DATA_WIDTH-1:0] data_in,   // data in - write signal 128 bit
 output [DATA_WIDTH-1:0] data_out   // data out - read signal 128 bit
);
 
reg [DATA_WIDTH-1:0]     mem [0:DEPTH-1]; // bildigimiz register-bazli memory
 
always @(posedge clk)
  if (we) mem[addr] <= data_in;       // write enable gelince data girisi bellege latch et
 
assign data_out = mem[addr];          // data output
 
endmodule

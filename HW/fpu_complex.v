// double precision complex add multiply and subtract module
// conforms to all worldwide standards
// i can spew out this crap all the way i want because there ARE no standards.
// ozgun ayaz, yildiz technical university 2011
//cOper : 2 modulus square 1 multiply 0 add
module complex_ops(cClock, cReset, cEnable, cOper, cOp1, cOp2, cOut, cReady);
input cClock;
input cReset;
input cEnable;
input [1:0] cOper;
input [127:0] cOp1;
input [127:0] cOp2;
output [127:0] cOut;
output cReady;
assign clk = cClock;
assign fpu_reset = cReset;

assign fpu_op1 = cOper==2'b01 ? 1 : 0;
assign fpu_op2 = 0;

assign add_e1 = cOper==2'b00 ? cEnable : (mul_r1 & mul_r2);
assign add_e2 = cOper==2'b00 ? cEnable : (mul_r3 & mul_r4);

assign add_a1 = cOper==2'b00 ? cOp1[63:0] : mul_o1[63:0] ; //A
assign add_b1 = cOper==2'b00 ? cOp2[63:0] : mul_o2[63:0] ; //C
assign add_a2 = cOper==2'b00 ? cOp1[127:64] : mul_o3[63:0]; //B
assign add_b2 = cOper==2'b00 ? cOp2[127:64] : mul_o4[63:0]; //D

assign cOut[127:64] = cOper==2'b10 ? 64'd0 : add_o2[63:0];
assign cOut[63:0] = add_o1[63:0];

assign cReady = add_r1 & add_r2;

assign mul_a1 = cOp1[63:0];
assign mul_b1 = cOp2[63:0];
assign mul_a2 = cOp1[127:64];
assign mul_b2 = cOp2[127:64];
assign mul_a3 = cOp1[63:0];
assign mul_b3 = cOp2[127:64];
assign mul_a4 = cOp2[63:0];
assign mul_b4 = cOp1[127:64];

assign mul_e1 = (cOper==2'b00 ? 0 : cEnable);
assign mul_e2 = (cOper==2'b00 ? 0 : cEnable);
assign mul_e3 = (cOper==2'b00 ? 0 : cEnable);
assign mul_e4 = (cOper==2'b00 ? 0 : cEnable);

wire [63:0] add_a1;
wire [63:0] add_a2;
wire [63:0] add_b1;
wire [63:0] add_b2;
wire [63:0] add_o1;
wire [63:0] add_o2;
wire [63:0] mul_a1;
wire [63:0] mul_a2;
wire [63:0] mul_a3;
wire [63:0] mul_a4;
wire [63:0] mul_o1;
wire [63:0] mul_o2;
wire [63:0] mul_o3;
wire [63:0] mul_o4;
wire [63:0] mul_b1;
wire [63:0] mul_b2;
wire [63:0] mul_b3;
wire [63:0] mul_b4;

fpu_addsub add1(clk, fpu_reset, add_e1, fpu_op1, add_a1, add_b1, add_o1, add_r1);
fpu_addsub add2(clk, fpu_reset, add_e2, fpu_op2, add_a2, add_b2, add_o2, add_r2);
fpu_mul mul1( clk, fpu_reset, mul_e1, mul_a1, mul_b1, mul_o1, mul_r1);
fpu_mul mul2( clk, fpu_reset, mul_e2, mul_a2, mul_b2, mul_o2, mul_r2);
fpu_mul mul3( clk, fpu_reset, mul_e3, mul_a3, mul_b3, mul_o3, mul_r3);
fpu_mul mul4( clk, fpu_reset, mul_e4, mul_a4, mul_b4, mul_o4, mul_r4);
endmodule

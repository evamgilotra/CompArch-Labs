module ctrlCkt(input [6:0] opcode, input [2:0] funct3_32,
	output reg [2:0] aluOp, output reg aluSrcA, output reg [2:0] aluSrcB, output reg [1:0] aluOut);
	
	//Write your code here
	always@(opcode,funct3_32)
	begin
	if((opcode == 7'b0110011)&(funct3_32 == 3'b000))
	begin
	aluOp = 3'b001;
	aluSrcA = 1'b0;
	aluSrcB = 3'b000;
	aluOut = 2'b00;
	end
	
	else if((opcode == 7'b0110011)&(funct3_32 == 3'b110))
	begin
	aluOp = 3'b010;
	aluSrcA = 1'b0;
	aluSrcB = 3'b000;
	aluOut = 2'b00;
	end
	
	else if((opcode == 7'b0010011)&(funct3_32 == 3'b111))
	begin
	aluOp = 3'b011;
	aluSrcA = 1'b0;
	aluSrcB = 3'b001;
	aluOut = 2'b00;
	end
	
	else if((opcode == 7'b0000011)&(funct3_32 == 3'b010))
	begin
	aluOp = 3'b000;
	aluSrcA = 1'b0;
	aluSrcB = 3'b001;
	aluOut = 2'b00;
	end
	
	else if((opcode == 7'b0010011)&(funct3_32 == 3'b101))
	begin
	aluOp = 3'b100;
	aluSrcA = 1'b0;
	aluSrcB = 3'b010;
	aluOut = 2'b00;
	end
	
	else if(opcode == 7'b0110111)
	begin
	aluOp = 3'b000;
	aluSrcA = 1'b0;
	aluSrcB = 3'b000;
	aluOut = 2'b01;
	end
	
	else if((opcode == 7'b0100011)&(funct3_32 == 3'b010))
	begin
	aluOp = 3'b000;
	aluSrcA = 1'b0;
	aluSrcB = 3'b011;
	aluOut = 2'b00;
	end
	
	else if(opcode == 7'b1101111)
	begin
	aluOp = 3'b000;
	aluSrcA = 1'b1;
	aluSrcB = 3'b100;
	aluOut = 2'b00;
	end
	
	else if(opcode[1:0] == 2'b01)
	begin
	aluOp = 3'b000;
	aluSrcA = 1'b0;
	aluSrcB = 3'b101;
	aluOut = 2'b00;
	end
	
	else if (opcode[1:0] == 2'b10)
	begin
	aluOp = 3'b000;
	aluSrcA = 1'b0;
	aluSrcB = 3'b000;
	aluOut = 2'b10;
	end
	
	end
	
endmodule

module ALU(input signed [31:0] aluIn1, input signed [31:0] aluIn2, input [2:0] aluOp, output reg [31:0] aluOut);
		
	//Write your code here
	always@(aluIn1,aluIn2,aluOp)
	begin
	
	if(aluOp == 3'b000)
	begin
	aluOut = aluIn1 + aluIn2;
	end
	
	if(aluOp == 3'b001)
	begin
	aluOut = aluIn1 - aluIn2;
	end
	
	if(aluOp == 3'b010)
	begin
	aluOut = aluIn1 | aluIn2;
	end
	
	if(aluOp == 3'b011)
	begin
	aluOut = aluIn1 & aluIn2;
	end
	
	if(aluOp == 3'b100)
	begin
	aluOut = aluIn1 >>> aluIn2;
	end
	end
	
endmodule

module signExt12to32(input [11:0] in, output reg [31:0] signExtin);
		
	//Write your code here
	always@(in)
	begin
	signExtin[0] = in[0];
	signExtin[1] = in[1];
	signExtin[2] = in[2];
	signExtin[3] = in[3];
	signExtin[4] = in[4];
	signExtin[5] = in[5];
	signExtin[6] = in[6];
	signExtin[7] = in[7];
	signExtin[8] = in[8];
	signExtin[9] = in[9];
	signExtin[10] = in[10];
	signExtin[11] = in[11];
	signExtin[12] = in[11];
	signExtin[13] = in[11];
	signExtin[14] = in[11];
	signExtin[15] = in[11];
	signExtin[16] = in[11];
	signExtin[17] = in[11];
	signExtin[18] = in[11];
	signExtin[19] = in[11];
	signExtin[20] = in[11];
	signExtin[21] = in[11];
	signExtin[22] = in[11];
	signExtin[23] = in[11];
	signExtin[24] = in[11];
	signExtin[25] = in[11];
	signExtin[26] = in[11];
	signExtin[27] = in[11];
	signExtin[28] = in[11];
	signExtin[29] = in[11];
	signExtin[30] = in[11];
	signExtin[31] = in[11];
	end
	
	
endmodule

module zeroExt5to32(input [4:0] in, output reg [31:0] zeroExtin);
		
	//Write your code here
	always@(in)
	begin
	zeroExtin[0] = in[0];
	zeroExtin[1] = in[1];
	zeroExtin[2] = in[2];
	zeroExtin[3] = in[3];
	zeroExtin[4] = in[4];
	zeroExtin[5] = 0;
	zeroExtin[6] = 0;
	zeroExtin[7] = 0;
	zeroExtin[8] = 0;
	zeroExtin[9] = 0;
	zeroExtin[10] = 0;
	zeroExtin[11] = 0;
	zeroExtin[12] = 0;
	zeroExtin[13] = 0;
	zeroExtin[14] = 0;
	zeroExtin[15] = 0;
	zeroExtin[16] = 0;
	zeroExtin[17] = 0;
	zeroExtin[18] = 0;
	zeroExtin[18] = 0;
	zeroExtin[19] = 0;
	zeroExtin[20] = 0;
	zeroExtin[21] = 0;
	zeroExtin[22] = 0;
	zeroExtin[23] = 0;
	zeroExtin[24] = 0;
	zeroExtin[25] = 0;
	zeroExtin[26] = 0;
	zeroExtin[27] = 0;
	zeroExtin[28] = 0;
	zeroExtin[29] = 0;
	zeroExtin[30] = 0;
	zeroExtin[31] = 0;
	end
endmodule

module signExt21to32(input [20:0] in, output reg [31:0] signExtin);
		
	//Write your code here
	always@(in)
	begin
	signExtin[0] = in[0];
	signExtin[1] = in[1];
	signExtin[2] = in[2];
	signExtin[3] = in[3];
	signExtin[4] = in[4];
	signExtin[5] = in[5];
	signExtin[6] = in[6];
	signExtin[7] = in[7];
	signExtin[8] = in[8];
	signExtin[9] = in[9];
	signExtin[10] = in[10];
	signExtin[11] = in[11];
	signExtin[12] = in[12];
	signExtin[13] = in[13];
	signExtin[14] = in[14];
	signExtin[15] = in[15];
	signExtin[16] = in[16];
	signExtin[17] = in[17];
	signExtin[18] = in[18];
	signExtin[19] = in[19];
	signExtin[20] = in[20];
	signExtin[21] = in[20];
	signExtin[22] = in[20];
	signExtin[23] = in[20];
	signExtin[24] = in[20];
	signExtin[25] = in[20];
	signExtin[26] = in[20];
	signExtin[27] = in[20];
	signExtin[28] = in[20];
	signExtin[29] = in[20];
	signExtin[30] = in[20];
	signExtin[31] = in[20];
	end
	
endmodule

module signExt6to32(input [5:0] in, output reg [31:0] signExtin);
		
	//Write your code here
	always@(in)
	begin
	signExtin[0] = in[0];
	signExtin[1] = in[1];
	signExtin[2] = in[2];
	signExtin[3] = in[3];
	signExtin[4] = in[4];
	signExtin[5] = in[5];
	signExtin[6] = in[5];
	signExtin[7] = in[5];
	signExtin[8] = in[5];
	signExtin[9] = in[5];
	signExtin[10] = in[5];
	signExtin[11] = in[5];
	signExtin[12] = in[5];
	signExtin[13] = in[5];
	signExtin[14] = in[5];
	signExtin[15] = in[5];
	signExtin[16] = in[5];
	signExtin[17] = in[5];
	signExtin[18] = in[5];
	signExtin[19] = in[5];
	signExtin[20] = in[5];
	signExtin[21] = in[5];
	signExtin[22] = in[5];
	signExtin[23] = in[5];
	signExtin[24] = in[5];
	signExtin[25] = in[5];
	signExtin[26] = in[5];
	signExtin[27] = in[5];
	signExtin[28] = in[5];
	signExtin[29] = in[5];
	signExtin[30] = in[5];
	signExtin[31] = in[5];
	end
	
endmodule

module mux2to1_32bits(input [31:0] in0, input [31:0] in1, input sel, output reg [31:0] muxOut);
    	
	//Write your code here
	always@(sel,in0,in1)
	begin
	
	case(sel)
	1'b0 : begin
	muxOut = in0;
	end
	1'b1 : begin
	muxOut = in1;
	end
	endcase
	end
	
endmodule

module mux4to1_32bits(input [31:0] in0, input [31:0] in1, input [31:0] in2, input [31:0] in3, input [1:0] sel, output reg [31:0] muxOut);
    	
	//Write your code here
	always@(sel,in0,in2,in2,in3)
	begin
	
	case(sel)
	2'b00 : begin
	muxOut = in0;
	end
	
	2'b01 : begin
	muxOut = in1;
	end
	
	2'b10 : begin
	muxOut = in2;
	end
	
	2'b11 : begin
	muxOut = in3;
	end
	
	endcase
	end
	
	
endmodule

module mux8to1_32bits(input [31:0] in0, input [31:0] in1, input [31:0] in2, input [31:0] in3,
	input [31:0] in4, input [31:0] in5, input [31:0] in6, input [31:0] in7, input [2:0] sel, output reg [31:0] muxOut);
   	
	//Write your code here
	always@(sel,in0,in1,in2,in3,in4,in5,in6,in7)
	begin
	
	case(sel)
	3'b000 : begin
	muxOut = in0;
	end
	
	3'b001 : begin
	muxOut = in1;
	end
	
	3'b010 : begin
	muxOut = in2;
	end
	
	3'b011 : begin
	muxOut = in3;
	end
	
	3'b100 : begin
	muxOut = in4;
	end
	
	3'b101 : begin
	muxOut = in5;
	end
	
	3'b110 : begin
	muxOut = in6;
	end
	
	3'b111 : begin
	muxOut = in7;
	end
	endcase
	end
endmodule

module decodeExecute(input [31:0] busA, input [31:0] busB, input [31:0] IR, output [31:0] outPut, output [31:0] busBout);
		
	//Write your code here
	wire [2:0] aluOp;
	wire aluSrcA;
	wire [2:0] aluSrcB;
	wire [1:0] aluSel;
	wire [31:0] signExt1;
	wire [31:0] signExt2;
	wire [31:0] signExt3;
	wire [31:0] signExt4;
	wire [31:0] signExt5;
	wire [31:0] muxOut1;
	wire [31:0] muxOut2;
	wire [31:0] aluOut;
	ctrlCkt circuit(IR[6:0],IR[14:12],aluOp,aluSrcA,aluSrcB,aluSel);
	signExt12to32 sign1(IR[31:20],signExt1);
	zeroExt5to32 sign2(IR[24:20],signExt2);
	signExt12to32 sign3({IR[31:25],IR[11:7]},signExt3);
	signExt21to32 sign4({IR[31:12],1'b0},signExt4);
	signExt6to32 sign5({IR[12],IR[6:2]},signExt5);
	mux8to1_32bits mux1(busB,signExt1,signExt2,signExt3,signExt4,signExt5,32'b0,32'b0,aluSrcB,muxOut2);
	mux2to1_32bits mux2(busA,32'b0,aluSrcA,muxOut1);
	ALU alu(muxOut1,muxOut2,aluOp,aluOut);
	mux4to1_32bits mux3(aluOut,{IR[31:12],12'b0},busB,32'b0,aluSel,outPut);
	assign busBout = busB;
	
	
	
		
endmodule

module testbench;
	reg [31:0] busA, busB, IR;
	wire [31:0] outPut, busBout;
	
	decodeExecute DE(busA, busB, IR, outPut, busBout);
	
	initial
	begin
		#00 busA = 32'h82345678; busB = 32'h12345678; IR = 32'h40000033; // sub
		#10 busA = 32'h3F82A814; busB = 32'h5E518C31; IR = 32'h00006033; // or
		#10 busA = 32'hABD31352; busB = 32'hBABABABA; IR = 32'h36A37413; // andi
		#10 busA = 32'h10000000; busB = 32'hDCDCDCDC; IR = 32'hC0062303; // lw
		#10 busA = 32'hE0000000; busB = 32'h00000005; IR = 32'h40205013; // srai
		#10 busA = 32'hFFFFFFFF; busB = 32'hFFFFFFFF; IR = 32'h0F300637; // lui
		#10 busA = 32'h00000000; busB = 32'hFFFFFFFF; IR = 32'hF0C52023; // sw
		#10 busA = 32'hFFFFFFFF; busB = 32'hFFFFFFFF; IR = 32'hF00001EF; // j
		#10 busA = 32'h0000000F; busB = 32'h11111111; IR = 32'h00001581; // c.addi
		#10 busA = 32'h22222222; busB = 32'h00DEAD00; IR = 32'h00008DBA; // c.mv
		#10 $finish;
	end
endmodule

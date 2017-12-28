`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    15:47:15 08/17/2017 
// Design Name: 
// Module Name:    registerFile 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module D_ff(input clk, input reset, input regWrite, input decOut1b , input d, output reg q);
	always @ (negedge clk)
	begin
	if(reset==1)
		q=0;
	else
		if(regWrite == 1 && decOut1b==1)
		begin
			q=d;
		end
	end
endmodule

module register32bit(input clk, input reset, input writeLow, input writeHigh, input decOut1b, input [31:0] writeData, output  [31:0] outR);
	
	//Write your code here
	
	D_ff D00(clk,reset,writeLow,decOut1b,writeData[0],outR[0]);
	D_ff D01(clk,reset,writeLow,decOut1b,writeData[1],outR[1]);
	D_ff D02(clk,reset,writeLow,decOut1b,writeData[2],outR[2]);
	D_ff D03(clk,reset,writeLow,decOut1b,writeData[3],outR[3]);
	D_ff D04(clk,reset,writeLow,decOut1b,writeData[4],outR[4]);
	D_ff D05(clk,reset,writeLow,decOut1b,writeData[5],outR[5]);
	D_ff D06(clk,reset,writeLow,decOut1b,writeData[6],outR[6]);
	D_ff D07(clk,reset,writeLow,decOut1b,writeData[7],outR[7]);
	D_ff D08(clk,reset,writeLow,decOut1b,writeData[8],outR[8]);
	D_ff D09(clk,reset,writeLow,decOut1b,writeData[9],outR[9]);
	D_ff D10(clk,reset,writeLow,decOut1b,writeData[10],outR[10]);
	D_ff D11(clk,reset,writeLow,decOut1b,writeData[11],outR[11]);
	D_ff D12(clk,reset,writeLow,decOut1b,writeData[12],outR[12]);
	D_ff D13(clk,reset,writeLow,decOut1b,writeData[13],outR[13]);
	D_ff D14(clk,reset,writeLow,decOut1b,writeData[14],outR[14]);
	D_ff D15(clk,reset,writeLow,decOut1b,writeData[15],outR[15]);
	D_ff D16(clk,reset,writeHigh,decOut1b,writeData[16],outR[16]);
	D_ff D17(clk,reset,writeHigh,decOut1b,writeData[17],outR[17]);
	D_ff D18(clk,reset,writeHigh,decOut1b,writeData[18],outR[18]);
	D_ff D19(clk,reset,writeHigh,decOut1b,writeData[19],outR[19]);
	D_ff D20(clk,reset,writeHigh,decOut1b,writeData[20],outR[20]);
	D_ff D21(clk,reset,writeHigh,decOut1b,writeData[21],outR[21]);
	D_ff D22(clk,reset,writeHigh,decOut1b,writeData[22],outR[22]);
	D_ff D23(clk,reset,writeHigh,decOut1b,writeData[23],outR[23]);
	D_ff D24(clk,reset,writeHigh,decOut1b,writeData[24],outR[24]);
	D_ff D25(clk,reset,writeHigh,decOut1b,writeData[25],outR[25]);
	D_ff D26(clk,reset,writeHigh,decOut1b,writeData[26],outR[26]);
	D_ff D27(clk,reset,writeHigh,decOut1b,writeData[27],outR[27]);
	D_ff D28(clk,reset,writeHigh,decOut1b,writeData[28],outR[28]);
	D_ff D29(clk,reset,writeHigh,decOut1b,writeData[29],outR[29]);
	D_ff D30(clk,reset,writeHigh,decOut1b,writeData[30],outR[30]);
	D_ff D31(clk,reset,writeHigh,decOut1b,writeData[31],outR[31]);
	
endmodule

module registerSet(input clk, input reset, input writeLow, input writeHigh, input [15:0] decOut, input [31:0] writeData,
	output [31:0] outR0, output [31:0] outR1, output [31:0] outR2, output [31:0] outR3,
	output [31:0] outR4, output [31:0] outR5, output [31:0] outR6, output [31:0] outR7,
	output [31:0] outR8, output [31:0] outR9, output [31:0] outR10, output [31:0] outR11,
	output [31:0] outR12, output [31:0] outR13, output [31:0] outR14, output [31:0] outR15);

		//Write your code here
		
		register32bit R00(clk,reset,writeLow,writeHigh,decOut[0],writeData,outR0);
		register32bit R01(clk,reset,writeLow,writeHigh,decOut[1],writeData,outR1);
		register32bit R02(clk,reset,writeLow,writeHigh,decOut[2],writeData,outR2);
		register32bit R03(clk,reset,writeLow,writeHigh,decOut[3],writeData,outR3);
		register32bit R04(clk,reset,writeLow,writeHigh,decOut[4],writeData,outR4);
		register32bit R05(clk,reset,writeLow,writeHigh,decOut[5],writeData,outR5);
		register32bit R06(clk,reset,writeLow,writeHigh,decOut[6],writeData,outR6);
		register32bit R07(clk,reset,writeLow,writeHigh,decOut[7],writeData,outR7);
		register32bit R08(clk,reset,writeLow,writeHigh,decOut[8],writeData,outR8);
		register32bit R09(clk,reset,writeLow,writeHigh,decOut[9],writeData,outR9);
		register32bit R10(clk,reset,writeLow,writeHigh,decOut[10],writeData,outR10);
		register32bit R11(clk,reset,writeLow,writeHigh,decOut[11],writeData,outR11);
		register32bit R12(clk,reset,writeLow,writeHigh,decOut[12],writeData,outR12);
		register32bit R13(clk,reset,writeLow,writeHigh,decOut[13],writeData,outR13);
		register32bit R14(clk,reset,writeLow,writeHigh,decOut[14],writeData,outR14);
		register32bit R15(clk,reset,writeLow,writeHigh,decOut[15],writeData,outR15);
		
endmodule

module decoder4to16(input [3:0] destReg, output reg [15:0] decOut);
	
	//Write your code here
	
	always@(destReg)
	begin
	case(destReg)
	
	4'b0000:begin
	decOut = 16'b0000000000000001;
	end
	
	4'b0001:begin
	decOut = 16'b0000000000000010;
	end
	
	4'b0010:begin
	decOut = 16'b0000000000000100;
	end
	
	4'b0011:begin
	decOut = 16'b0000000000001000;
	end
	
	4'b0100:begin
	decOut = 16'b0000000000010000;
	end
	
	4'b0101:begin
	decOut = 16'b0000000000100000;
	end
	
	4'b0110:begin
	decOut = 16'b0000000001000000;
	end
	
	4'b0111:begin
	decOut = 16'b0000000010000000;
	end
	
	4'b1000:begin
	decOut = 16'b0000000100000000;
	end
	
	4'b1001:begin
	decOut = 16'b0000001000000000;
	end
	
	4'b1010:begin
	decOut = 16'b0000010000000000;
	end
	
	4'b1011:begin
	decOut = 16'b0000100000000000;
	end
	
	4'b1100:begin
	decOut = 16'b0001000000000000;
	end
	
	4'b1101:begin
	decOut = 16'b0010000000000000;
	end
	
	4'b1110:begin
	decOut = 16'b0100000000000000;
	end
	
	4'b1111:begin
	decOut = 16'b1000000000000000;
	end
	endcase
	end
	
endmodule

module mux16to1(input [31:0] outR0, input [31:0] outR1, input [31:0] outR2, input [31:0] outR3,
	input [31:0] outR4, input [31:0] outR5, input [31:0] outR6, input [31:0] outR7,
	input [31:0] outR8, input [31:0] outR9, input [31:0] outR10, input [31:0] outR11,
	input [31:0] outR12, input [31:0] outR13, input [31:0] outR14, input [31:0] outR15,
	input [3:0] Sel, output reg [31:0] outBus);

	//Write your code here
	
	always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,Sel)
	begin
	case(Sel)
	
	4'b0000:begin
	outBus = outR0;
	end
	
	4'b0001:begin
	outBus = outR1;
	end
	
	4'b0010:begin
	outBus = outR2;
	end
	
	4'b0011:begin
	outBus = outR3;
	end
	
	4'b0100:begin
	outBus = outR4;
	end
	
	4'b0101:begin
	outBus = outR5;
	end
	
	4'b0110:begin
	outBus = outR6;
	end
	
	4'b0111:begin
	outBus = outR7;
	end
	
	4'b1000:begin
	outBus = outR8;
	end
	
	4'b1001:begin
	outBus = outR9;
	end
	
	4'b1010:begin
	outBus = outR10;
	end
	
	4'b1011:begin
	outBus = outR11;
	end
	
	4'b1100:begin
	outBus = outR12;
	end
	
	4'b1101:begin
	outBus = outR13;
	end
	
	4'b1110:begin
	outBus = outR14;
	end
	
	4'b1111:begin
	outBus = outR15;
	end
	
	endcase
	end
endmodule

module mux2to1_16bits(input [15:0] in0, input [15:0] in1, input Sel, output reg [15:0] outBus);
	
	//Write your code here
	
	always@(in0,in1,Sel)
	begin
	case(Sel)
	
	1'b0:begin
	outBus = in0;
	end
	
	1'b1:begin
	outBus = in1;
	end 
	endcase
	
	end
	
endmodule

module busOutput(input [31:0] reg_rx, input mode, input rx3, output [31:0] rxOut);
	
	//Write your code here
	
	mux2to1_16bits mux1(16'b0, reg_rx[31:16], mode, rxOut[31:16]);
	mux2to1_16bits mux2(16'b0,reg_rx[15:0], mode||(~rx3), rxOut[15:0]);
	
	
	
endmodule

module registerFile(input clk, input reset, input regWrite, input mode, input [3:0] rs, input [3:0] rt,input [3:0] rd, 
input [31:0] writeData, output [31:0] rsOut, output [31:0] rtOut);
	
	//Write your code here
	
	wire [15:0] decOut;
	wire [31:0] reg_rs,reg_rt;
	wire [31:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15;
	wire writeLow,writeHigh;
	
	writeLow = ((~rd[3] || mode) && regWrite);
	writeHigh = mode && regWrite;

	
	
	
	decoder4to16 decoder(rd,decOut);
	registerSet rs1(clk,reset,writeLow,writeHigh,decOut,writeData,outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15);
	mux16to1 mux1(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,rs,reg_rs);
	mux16to1 mux2(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,rt,reg_rt);
	busOutput bus1(reg_rs,mode,rs[3],rsOut);
	busOutput bus2(reg_rt,mode,rt[3],rtOut);
	
	
	
	
endmodule

module testbench;
	// Input
	reg clk, reset, regWrite, mode;
	reg [3:0] rs, rt, rd;
	reg [31:0] writeData;
	// Output
	wire [31:0] rsOut, rtOut;
	
	registerFile RF1(clk,reset,regWrite,mode,rs,rt,rd,writeData,rsOut,rtOut);
	
	always
		#5 clk=~clk;
	initial
		begin
			clk=0;reset=1; rs=4'd15; rt=4'd14; rd=4'd15; regWrite=1; mode=1; writeData=32'hAC0AC0AC;
			#5 reset=0;
			#10 rs=4'd15; rt=4'd14; rd=4'd14; regWrite=1; mode=1; writeData=32'hDEADBEEF;
			#10 rs=4'd15; rt=4'd14; rd=4'd14; regWrite=1; mode=0; writeData=32'h12345678;
			#10 rs=4'd15; rt=4'd14; rd=4'd1; regWrite=1; mode=1; writeData=32'hDEADBEEF;
			#10 rs=4'd15; rt=4'd1; rd=4'd1; regWrite=1; mode=0; writeData=32'h12345678;
			#10 rs=4'd15; rt=4'd1; rd=4'd2; regWrite=1; mode=1; writeData=32'hBABABABE;
			#10 rs=4'd2; rt=4'd1; rd=4'd2; regWrite=0; mode=1; writeData=32'h12345678;
			#10 rs=4'd2; rt=4'd1; rd=4'd2; regWrite=0; mode=0; writeData=32'h12345678;
			#10 $finish;
		end
endmodule

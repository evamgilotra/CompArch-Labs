module D_ff(input clk, input reset, input regWrite, input d, output reg q);
	always@(negedge clk)
		begin
			if(reset)
				q=0;
			else
				if(regWrite == 1) begin q=d; end
		end
endmodule

//viArray[i] stores i_th v/i bit. 
//we is used for regWrite; d of D_ff is always 1'b1.
module viArray(input clk, input reset, input [7:0] we, output [7:0] valid);
	
	//Write your code here
	D_ff DFF0(clk,reset,we[0],1'b1,valid[0]);
	D_ff DFF1(clk,reset,we[1],1'b1,valid[1]);
	D_ff DFF2(clk,reset,we[2],1'b1,valid[2]);
	D_ff DFF3(clk,reset,we[3],1'b1,valid[3]);
	D_ff DFF4(clk,reset,we[4],1'b1,valid[4]);
	D_ff DFF5(clk,reset,we[5],1'b1,valid[5]);
	D_ff DFF6(clk,reset,we[6],1'b1,valid[6]);
	D_ff DFF7(clk,reset,we[7],1'b1,valid[7]);
endmodule

//tagRegister stores 4-bit tag
module tagRegister(input clk, input reset, input set, input [3:0] inputTag, output [3:0] outputTag);
		
	//Write your code here
	D_ff DFF0(clk,reset,set,inputTag[0],outputTag[0]);
	D_ff DFF1(clk,reset,set,inputTag[1],outputTag[1]);
	D_ff DFF2(clk,reset,set,inputTag[2],outputTag[2]);
	D_ff DFF3(clk,reset,set,inputTag[3],outputTag[3]);
endmodule

//tagArray[i] stores i_th four bit tag
module tagArray(input clk, input reset, input [7:0] we, input [3:0] inputTag,
	output [3:0] Tag0, output [3:0] Tag1, output [3:0] Tag2, output [3:0] Tag3,
	output [3:0] Tag4, output [3:0] Tag5, output [3:0] Tag6, output [3:0] Tag7);
	
	//Write your code here
	tagRegister TR0(clk,reset,we[0],inputTag,Tag0);
	tagRegister TR1(clk,reset,we[1],inputTag,Tag1);
	tagRegister TR2(clk,reset,we[2],inputTag,Tag2);
	tagRegister TR3(clk,reset,we[3],inputTag,Tag3);
	tagRegister TR4(clk,reset,we[4],inputTag,Tag4);
	tagRegister TR5(clk,reset,we[5],inputTag,Tag5);
	tagRegister TR6(clk,reset,we[6],inputTag,Tag6);
	tagRegister TR7(clk,reset,we[7],inputTag,Tag7);
endmodule

//comparator module compares input tag with tagRegister tag.
//It returns 1 if tags matches otherwise it returns 0.
module comparator(input [3:0] in1, input [3:0] in2, output reg equal);
		
	//Write your code here
	always@(in1,in2)
		begin
			if(in1==in2)
				equal = 1'b1;
			else
				equal = 1'b0;
		end
endmodule

//encoder module encodes 8 bits input in1 to 3 bits output op
//Eg. if in1 = 8'b00000010 then op = 3'b001
module encoder(input [7:0] in1, output reg [2:0] op);
		
	//Write your code here
	always@(in1)
		begin
			case(in1)
				8'b0000_0001 : op = 3'b000;
				8'b0000_0010 : op = 3'b001;
				8'b0000_0100 : op = 3'b010;
				8'b0000_1000 : op = 3'b011;
				8'b0001_0000 : op = 3'b100;
				8'b0010_0000 : op = 3'b101;
				8'b0100_0000 : op = 3'b110;
				8'b1000_0000 : op = 3'b111;
				8'b0 			 : op = 3'b0;
			endcase
		end
endmodule

// counter changes its value on miss so that it can select the
//next cache line to be used for replacement.
//Implement the given counter circuit here. 
module counter(input clk, input reset, input miss,  output [2:0] curState);
	
	//Write your code here

			D_ff DFF0(curState[2], reset,miss,~curState[0],curState[0]);
			D_ff DFF1(clk, reset,miss,~curState[1],curState[1]);
			D_ff DFF2(curState[1], reset,miss,~curState[2],curState[2]);
		
	
endmodule

//decoder module decodes 3 bits input in1 to 8 bits output op.
//Eg. if in1 = 3'b101 then op = 8'b00100000  
module decoder(input [2:0] in1, output reg [7:0] op);
		
	//Write your code here
	always@(in1)
		begin
			case(in1)
			3'd0: op =8'b0000_0001;
			3'd1: op =8'b0000_0010;
			3'd2: op =8'b0000_0100;
			3'd3: op =8'b0000_1000;
			3'd4: op =8'b0001_0000;
			3'd5: op =8'b0010_0000;
			3'd6: op =8'b0100_0000;
			3'd7: op =8'b1000_0000;
			3'd0: op =8'b0;
			endcase
		end	
	
	
endmodule

module fullyAssociativeCache(input clk, input reset, input [3:0] tag, output hit, output [2:0] lineNo);
		
	//Write your code here
	wire [7:0] in1,equal,valid,we,decoderOut;
	wire [2:0] decoderIn;
	wire counterIn;
	wire [3:0] Tag0,Tag1,Tag2,Tag3,Tag4,Tag5,Tag6,Tag7;
	comparator CR0(tag,Tag0,equal[0]);
	comparator CR1(tag,Tag1,equal[1]);
	comparator CR2(tag,Tag2,equal[2]);
	comparator CR3(tag,Tag3,equal[3]);
	comparator CR4(tag,Tag4,equal[4]);
	comparator CR5(tag,Tag5,equal[5]);
	comparator CR6(tag,Tag6,equal[6]);
	comparator CR7(tag,Tag7,equal[7]);
	
	counter counter(clk,reset,counterIn,decoderIn);
	decoder decoder(decoderIn,decoderOut);
	encoder encoder(in1,lineNo);
	
		
	assign we[0] = decoderOut[0] & counterIn;
	assign we[1] = decoderOut[1] & counterIn;
	assign we[2] = decoderOut[2] & counterIn;
	assign we[3] = decoderOut[3] & counterIn;
	assign we[4] = decoderOut[4] & counterIn;
	assign we[5] = decoderOut[5] & counterIn;
	assign we[6] = decoderOut[6] & counterIn;
	assign we[7] = decoderOut[7] & counterIn;
	
	assign counterIn = ~(in1[0] | in1[1] | in1[2] | in1[3] | in1[4] | in1[5] | in1[6] | in1[7]);
	assign hit = in1[0] | in1[1] | in1[2] | in1[3] | in1[4] | in1[5] | in1[6] | in1[7];
	
	assign in1[0] = equal[0] & valid[0];
	assign in1[1] = equal[1] & valid[1];
	assign in1[2] = equal[2] & valid[2];
	assign in1[3] = equal[3] & valid[3];
	assign in1[4] = equal[4] & valid[4];
	assign in1[5] = equal[5] & valid[5];
	assign in1[6] = equal[6] & valid[6];
	assign in1[7] = equal[7] & valid[7];
	
	viArray va(clk,reset,we,valid);
	tagArray ta(clk,reset,we,tag, Tag0, Tag1, Tag2, Tag3, Tag4, Tag5, Tag6, Tag7);
endmodule

module testbench;
	reg clk;
	reg reset;
	reg [3:0] tag;
	wire hit;
	wire [2:0] lineNo;
	fullyAssociativeCache FAC(clk, reset, tag, hit, lineNo);

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;tag = 4'd15;
		#5  reset=0; 
		#10  tag = 4'd1;
		#10  tag = 4'd2;
		#10  tag = 4'd1;
		#10  tag = 4'd3;
		#10  tag = 4'd4;
		#10  tag = 4'd5;
		#10  tag = 4'd6;
		#10  tag = 4'd7;
		#10  tag = 4'd8;
		#10  tag = 4'd15;
		#10  tag = 3'd4;
		#10 $finish;
	end
endmodule

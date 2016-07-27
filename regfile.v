module regfile(clock, ctrl_writeEnable, ctrl_reset, ctrl_writeReg, ctrl_readRegA, ctrl_readRegB, data_writeReg, data_readRegA, data_readRegB);
   input clock, ctrl_writeEnable, ctrl_reset;
   input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
   input [31:0] data_writeReg;
   output [31:0] data_readRegA, data_readRegB;
	
	wire [31:0] choosenReg; 
	decoder32bit findWriteRegister (.out(choosenReg), .enable(ctrl_writeEnable), .select(ctrl_writeReg));
	
	wire flipReset;
	assign flipReset = ~ctrl_reset;
	
	wire [31:0] registerToTBuff [31:0];
	genvar k;
	generate
		for(k = 0 ; k < 32 ; k = k + 1 ) begin: loop3
			register a_reg (.writeIn(data_writeReg), .clock(clock), .writeEnable(choosenReg[k]), .reset(flipReset), .readOut(registerToTBuff[k]));
		end
	endgenerate
	
	wire [31:0] decoderToReadPortA;
	decoder32bit readPortOne (.out(decoderToReadPortA), .select(ctrl_readRegA), .enable(1'b1));
	
	wire [31:0] decoderToReadPortB;
	decoder32bit readPortTwo (.out(decoderToReadPortB), .select(ctrl_readRegB), .enable(1'b1));
	
	genvar j;
	generate
		for( j = 0 ; j < 32 ; j = j + 1) begin: loop
			tbuff32 a_tbuff32ReadPortOne (.in(registerToTBuff[j]), .oe(decoderToReadPortA[j]), .out(data_readRegA));
		end
	endgenerate
	
	genvar y;
	generate
		for (y = 0 ; y < 32 ; y = y + 1) begin: loop5
			tbuff32 a_tbuff32ReadPortTwo (.in(registerToTBuff[y]), .oe(decoderToReadPortB[y]), .out(data_readRegB));
		end
	endgenerate

endmodule

module decoder32bit(out,select,enable);
	input [4:0] select;
	input enable;
	output [31:0] out;
	assign out = enable << select;
endmodule


module register(writeIn, clock, writeEnable, reset, readOut);
	input [31:0] writeIn;
	input clock, reset, writeEnable;
	output [31:0] readOut;
	
	genvar x;
	generate
		for(x = 0 ; x < 32 ; x = x +1) begin: loop1
			DFFE reg_dff(.d(writeIn[x]), .clk(clock), .ena(writeEnable), .q(readOut[x]), .clrn(reset));
		end
	endgenerate
endmodule

//module tbuff(in, oe, out);
//	input in, oe;
//	output out;
//	
//	assign out = oe ? in : 1'bz;
//	
//endmodule


module registerFive(writeIn, clock, writeEnable, reset, readOut);
	input [4:0] writeIn;
	input clock, reset, writeEnable;
	output [4:0] readOut;
	
	genvar x;
	generate
		for(x = 0 ; x < 5 ; x = x +1) begin: loop1
			DFFE reg_dff(.d(writeIn[x]), .clk(clock), .ena(writeEnable), .q(readOut[x]), .clrn(reset));
		end
	endgenerate
endmodule

module registerTwentySeven(writeIn, clock, writeEnable, reset, readOut);
	input [26:0] writeIn;
	input clock, reset, writeEnable;
	output [26:0] readOut;
	
	genvar x;
	generate
		for(x = 0 ; x < 27 ; x = x +1) begin: loop1
			DFFE reg_dff(.d(writeIn[x]), .clk(clock), .ena(writeEnable), .q(readOut[x]), .clrn(reset));
		end
	endgenerate
endmodule

module registerSeventeen(writeIn, clock, writeEnable, reset, readOut);
	input [16:0] writeIn;
	input clock, reset, writeEnable;
	output [16:0] readOut;
	
	genvar x;
	generate
		for(x = 0 ; x < 17 ; x = x +1) begin: loop1
			DFFE reg_dff(.d(writeIn[x]), .clk(clock), .ena(writeEnable), .q(readOut[x]), .clrn(reset));
		end
	endgenerate
endmodule

module tbuff(in, oe, out);
	input in, oe;
	output out;
	
	assign out = oe ? in : 1'bz;
	
endmodule

module tbuff32(in, oe, out);
	input [31:0] in; 
	input oe;
	output [31:0] out;
	
	genvar i;
	generate
		for(i = 0 ; i < 32 ; i = i + 1) begin: loop2
			tbuff a_tbuff(.in(in[i]),.oe(oe),.out(out[i]));
		end
	endgenerate
endmodule



module registerOne(writeIn, clock, writeEnable, reset, readOut);
	input  writeIn;
	input clock, reset, writeEnable;
	output  readOut;
	

			DFFE reg_dff(.d(writeIn), .clk(clock), .ena(writeEnable), .q(readOut), .clrn(reset));
	
endmodule




	
	
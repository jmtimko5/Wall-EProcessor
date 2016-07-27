module timer (clock, reset, out, test, testincrement);
	input clock, reset;
	output [31:0] out, testincrement;
	output test;
	
	wire [31:0] one, five, clockperiod, alu_out, alu_in, mod_counter_in, mod_counter_out;
	assign one  = 32'b00000000000000000000000000000001;
	assign five = 32'b00000000000000000000000000000101;
	assign clockperiod = 32'b00000000000000001100001101001111;
	
	register inputtomodcounter (.writeIn(mod_counter_out), .clock(clock), .reset(clear_out), .writeEnable(1'b1), .readOut(mod_counter_in));
	
	alu modcounter(.data_operandA(mod_counter_in), .data_operandB(one), .ctrl_ALUopcode(5'b00000), .data_result(mod_counter_out));
	assign out = mod_counter_out;
	wire clear;
	alu checkmod (.data_operandA(mod_counter_out), .data_operandB(clockperiod), .ctrl_ALUopcode(5'b00001), .isNotEqual(clear));
	
	wire clear_out;
	registerOne clearreg (.writeIn(clear), .clock(clock), .reset(1'b1), .writeEnable(1'b1), .readOut(clear_out));
	assign test = clear_out;
	
//	assign alu_out = 32'b00000000000000000000000000000101;
	register inputtoincrementcounter (.writeIn(alu_out), .clock(clock), .reset(1'b1), .writeEnable(!clear_out), .readOut(alu_in));
	assign testincrement = alu_in;
	alu incrementcounter (.data_operandA(alu_in), .data_operandB(one), .ctrl_ALUopcode(5'b00000), .data_result(alu_out));

endmodule

//module timer (clock, reset, out, test);
//	input clock, reset;
//	output [31:0] out, test;
//	
//	wire [31:0] one, alu_out, alu_in, sub_counter_in, sub_counter_out;
//	assign one  = 32'b00000000000000000000000000000001;
//	
//	register inputalu (.writeIn(alu_out), .clock(clock), .reset(reset), .writeEnable(doshit), .readOut(alu_in));
//	
//	alu timeralu (.data_operandA(alu_in), .data_operandB(one), .ctrl_ALUopcode(5'b00000), .data_result(alu_out));
//	
//	register inputsubalu (.writeIn(sub_counter_out), .clock(clock), .reset(!doshit), .writeEnable(1'b1), .readOut(sub_counter_in));
//
//	
//	wire [31:0] fiftythousand;
//	assign fiftythousand = 32'b00000000000000001100001101010000;
//	wire increment;
//	alu subcounter (.data_operandA(sub_counter_in), .data_operandB(one), .ctrl_ALUopcode(5'b00000), .data_result(sub_counter_out));
//	alu checkincrement (.data_operandA(sub_counter_in), .data_operandB(fiftythousand), .ctrl_ALUopcode(5'b00001), .isNotEqual(increment));
//	
//	wire doshit = !increment;
//	
//	assign test = sub_counter_out;
//	assign out = alu_in;
//	
//	
//endmodule

module alu(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan, testopcode);
   input [31:0] data_operandA, data_operandB;
   input [4:0] ctrl_ALUopcode, ctrl_shiftamt;
   output [31:0] data_result;
   output isNotEqual, isLessThan;
	
	
	
	wire [31:0] nottedInputB;
	wire [5:0] selectedOpcode;
	wire [31:0] operationOutput [5:0];
	
	output [5:0] testopcode;
	assign testopcode = selectedOpcode;
	
	
	
	opcodeDecoder myOpcodeDecoder (.out(selectedOpcode), .select(ctrl_ALUopcode), .enable(1'b1));
	
	andOperation myAnd (.inputA(data_operandA), .inputB(data_operandB), .out(operationOutput[2]));
	
	orOperation myOr(.inputA(data_operandA), .inputB(data_operandB), .out(operationOutput[3]));
	
	CLAAdder32Bit myAdd(.inputA(data_operandA), .inputB(data_operandB), .carryIn(1'b0), .sumFinal(operationOutput[0]));
	
	subtractionPrep myalusubprep (.inputB(data_operandB), .notInputB(nottedInputB));
	CLAAdder32Bit mySubtract (.inputA(data_operandA), .inputB(nottedInputB), .carryIn(1'b1), .sumFinal(operationOutput[1]));
	
	
	checkEqual myEqual (.subResult(operationOutput[1]), .equalOut(isNotEqual));
	
	shiftBLL myShiftLeft (.inputValue(data_operandA), .shiftNum(ctrl_shiftamt), .shifted(operationOutput[4]));
	
	shiftBRA myShiftRight(.inputValue(data_operandA), .shiftNum(ctrl_shiftamt), .shifted(operationOutput[5]));
	
	lessThan myLessThan (.inputA(data_operandA[31]), .inputB(data_operandB[31]), .subResult(operationOutput[1][31]), .result(isLessThan));
	
	
	
	genvar i;
	generate 
		for (i = 0 ; i < 6 ; i = i + 1) begin: loop1
			tbuff32 operationBuffer(.in(operationOutput[i]), .oe(selectedOpcode[i]), .out(data_result));
		end
	endgenerate
endmodule

module checkEqual(subResult, equalOut);
	input [31:0] subResult;
	output equalOut;
	
	assign equalOut = ( subResult[0] | subResult[1] | subResult[2] | subResult[3] | subResult[4] | subResult[5] | subResult[6] | subResult[7] | subResult[8] | subResult[9] | subResult[10] | subResult[11] | subResult[12] | subResult[13] | subResult[14] | subResult[15] | subResult[16] | subResult[17] | subResult[18] | subResult[19] | subResult[20] | subResult[21] | subResult[22] | subResult[23] | subResult[24] | subResult[25] | subResult[26] | subResult[27] | subResult[28] | subResult[29] | subResult[30] | subResult[31]);
endmodule


module opcodeDecoder(out,select,enable);
	input [4:0] select;
	input enable;
	output [5:0] out;
	assign out = enable << select;
endmodule

module lessThan(inputA, inputB, subResult, result);
	input inputA, inputB, subResult;
	output result;
	
	wire xorResult;
	assign xorResult = inputA ^ inputB;
	
	wire noSubCase;
	assign noSubCase = xorResult & inputA;
	
	wire isBothAsserted;
	assign isBothAsserted = inputA & inputB;
	
	wire isNeitherAsserted;
	assign isNeitherAsserted = (!inputA) & (!inputB);
	
	wire lookAtSub;
	assign lookAtSub = isBothAsserted ^ isNeitherAsserted;
	
	wire resultIsSubMSB;
	assign resultIsSubMSB = lookAtSub & subResult;
	
	assign result = resultIsSubMSB | noSubCase;
endmodule


module andOperation(inputA, inputB, out);
	input [31:0] inputA;
	input [31:0] inputB;
	output [31:0] out;
	
	genvar v;
	generate
		for (v = 0 ; v < 32 ; v = v + 1) begin: loop3
			assign out[v] = inputA[v] & inputB[v];
		end
	endgenerate
endmodule


module orOperation(inputA, inputB, out);
	input [31:0] inputA;
	input [31:0] inputB;
	output [31:0] out;
	
	genvar o;
	generate
		for (o = 0 ; o < 32 ; o = o + 1) begin: loop4
			assign out[o] = inputA[o] | inputB[o];
		end
	endgenerate
endmodule

module CLAAdder32Bit (inputA, inputB, carryIn, sumFinal);
	input [31:0] inputA, inputB;
	input carryIn;
	output [31:0] sumFinal;
	
	wire [3:0] G;
	wire [3:0] P;
	wire [3:0] carries;
	
	EightBitAdderForCLA first (.cin(carryIn), .a(inputA[7:0]), .b(inputB[7:0]), .bigG(G[0]), .bigP(P[0]), .sum(sumFinal[7:0]));
	firstCarryLogic firstLogic (.cEight(carries[0]), .gZero(G[0]), .pZero(P[0]), .cZero(carryIn));
	
	EightBitAdderForCLA second (.cin(carries[0]), .a(inputA[15:8]), .b(inputB[15:8]), .bigG(G[1]), .bigP(P[1]), .sum(sumFinal[15:8]));
	secondCarryLogic secondLogic (.cSixteen(carries[1]), .gZero(G[0]), .pZero(P[0]), .cZero(carryIn), .gOne(G[1]), .pOne(P[1]));
	
	EightBitAdderForCLA third (.cin(carries[1]), .a(inputA[23:16]), .b(inputB[23:16]), .bigG(G[2]), .bigP(P[2]), .sum(sumFinal[23:16]));
	thirdCarryLogic thirdLogic (.cTwentyFour(carries[2]), .gZero(G[0]), .pZero(P[0]), .cZero(carryIn), .gOne(G[1]), .pOne(P[1]), .gTwo(G[2]), .pTwo(P[2]));
	
	EightBitAdderForCLA fourth (.cin(carries[2]), .a(inputA[31:24]), .b(inputB[31:24]), .bigG(G[3]), .bigP(P[3]), .sum(sumFinal[31:24]));
	fourthCarryLogic fourthLogic (.cThirtyTwo(carries[3]), .gZero(G[0]), .pZero(P[0]), .cZero(carryIn), .gOne(G[1]), .pOne(P[1]), .gTwo(G[2]), .pTwo(P[2]), .gThree(G[3]), .pThree(P[3]));
	
	
	
endmodule

module EightBitAdderForCLA(a, b, cin, bigG, bigP, sum);
	input cin;
	input [7:0] a, b;
	output bigG, bigP;
	output [7:0] sum;
	
	wire [7:0] littleGs;
	wire [7:0] littlePs;
	
	wire [7:0] connectCarries;
	
	FullAdderForCLA firstAdder (.a(a[0]), .b(b[0]), .cin(cin), .g(littleGs[0]), .p(littlePs[0]), .s(sum[0]), .cout(connectCarries[0]));
	
	genvar j;
	generate
		for(j = 1 ; j < 8 ; j = j + 1) begin: loop
			FullAdderForCLA adder (.a(a[j]), .b(b[j]), .cin(connectCarries[j - 1]), .g(littleGs[j]), .p(littlePs[j]), .s(sum[j]), .cout(connectCarries[j]));
		end
	endgenerate
	
	assign bigP = littlePs[0] & littlePs[1] & littlePs[2] & littlePs[3] & littlePs[4] & littlePs[5] & littlePs[6] & littlePs[7];
	assign bigG = littleGs[7] | (littleGs[6] & littlePs[7]) | (littleGs[5] & littlePs[6] & littlePs[7]) | (littleGs[4] & littlePs[5] & littlePs[6] & littlePs[7]) | (littleGs[3] & littlePs[4] & littlePs[5] & littlePs[6] & littlePs[7]) | (littleGs[2] & littlePs[3] & littlePs[4] & littlePs[5] & littlePs[6] & littlePs[7]) | (littleGs[1] & littlePs[2] & littlePs[3] & littlePs[4] & littlePs[5] & littlePs[6] & littlePs[7]) | (littleGs[0] & littlePs[1] & littlePs[2] & littlePs[3] & littlePs[4] & littlePs[5] & littlePs[6] & littlePs[7]);
	
endmodule

module FullAdderForCLA(a, b, cin, g, p, s, cout);
	input a, b, cin;
	output g, p, s, cout;
	
	assign g = a & b;
	assign p = a ^ b;
	assign cout = g | (cin & p);
	assign s = p ^ cin;
	
endmodule

module firstCarryLogic(gZero, pZero, cZero, cEight);
	input gZero, pZero, cZero;
	output cEight;
	
	assign cEight = gZero | (pZero & cZero);

endmodule

module checkBitsEqual(inputA, inputB, result);
	input [31:0] inputA, inputB;
	output result;
	
	
endmodule

module secondCarryLogic(gZero, pZero, cZero, gOne, pOne, cSixteen);
	input gZero, pZero, cZero, gOne, pOne;
	output cSixteen;
	
	assign cSixteen = gOne | (pOne & gZero) | (pZero & pOne & cZero);
	
endmodule

module thirdCarryLogic(gZero, pZero, cZero, gOne, pOne, gTwo, pTwo, cTwentyFour);
	input gZero, pZero, cZero, gOne, pOne, gTwo, pTwo;
	output cTwentyFour;
	
	assign cTwentyFour = gTwo | (pTwo & gOne) | (pTwo & pOne & gZero) | (pTwo & pOne & pZero & cZero);
endmodule

module fourthCarryLogic(gZero, pZero, cZero, gOne, pOne, gTwo, pTwo, gThree, pThree, cThirtyTwo);
	input gZero, pZero, cZero, gOne, pOne, gTwo, pTwo, gThree, pThree;
	output cThirtyTwo;
	
//	assign cThirtyTWo = gThree | (pThree & gTwo) | (pThree & pTwo & gOne) | (pThree & pTwo & pOne & gZero) | (pThree & pTwo & pOne & pZero & cZero);
	assign cThirtyTwo = 1'b0;
	
endmodule

module subtractionPrep(inputB, notInputB);
	input [31:0] inputB;
	output [31:0] notInputB;
	
	genvar u;
	generate
		for(u = 0 ; u < 32 ; u = u + 1) begin: loop19
			not a_not_gate(notInputB[u], inputB[u]); 
		end
	endgenerate
	
endmodule

module shiftBLL(inputValue, shiftNum, shifted);
	input [31:0] inputValue;
	input [4:0] shiftNum;
	output [31:0] shifted;
	
	wire [31:0] barrel_bus [4:0];

	assign barrel_bus[4] = shiftNum[4] ? (inputValue << 16) : inputValue;
	assign barrel_bus[3] = shiftNum[3] ? (barrel_bus[4] << 8) : barrel_bus[4];
	assign barrel_bus[2] = shiftNum[2] ? (barrel_bus[3] << 4) : barrel_bus[3];
	assign barrel_bus[1] = shiftNum[1] ? (barrel_bus[2] << 2) : barrel_bus[2];
	assign barrel_bus[0] = shiftNum[0] ? (barrel_bus[1] << 1) : barrel_bus[1];
	assign shifted = barrel_bus[0];
endmodule

module shiftBRA(inputValue, shiftNum, shifted);
	input [31:0] inputValue;
	input [4:0] shiftNum;
	output [31:0] shifted;
	
	wire signed [31:0] barrel_bus [4:0];
	
	assign barrel_bus[4] = shiftNum[4] ? (inputValue >>> 16) : inputValue;
	assign barrel_bus[3] = shiftNum[3] ? (barrel_bus[4] >>> 8) : barrel_bus[4];
	assign barrel_bus[2] = shiftNum[2] ? (barrel_bus[3] >>> 4) : barrel_bus[3];
	assign barrel_bus[1] = shiftNum[1] ? (barrel_bus[2] >>> 2) : barrel_bus[2];
	assign barrel_bus[0] = shiftNum[0] ? (barrel_bus[1] >>> 1) : barrel_bus[1];
	assign shifted = barrel_bus[0];
	
	
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


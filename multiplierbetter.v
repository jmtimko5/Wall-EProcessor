module multiplierbetter(clock, multiplicand, multiplier, ctrl_MULT, dataException, data_RDY, result);
	input [31:0] multiplicand;
	input [15:0] multiplier;
	input ctrl_MULT;
	input clock;
	output data_RDY;
	output dataException;
	output [31:0] result;
	
	
	
	
	wire [31:0] busA [7:0];
	wire [31:0] busIntermediate [7:0];
	wire [15:0] busB [7:0];
	wire [7:0] busWE;
	wire [7:0] busNegCand;
	wire [7:0] busNegLier;
	
	wire [31:0] initialIntermediate;
	assign initialIntermediate = 32'b00000000000000000000000000000000;
	
	
	aluFirstStage stageOne (.clock(clock), .inputA(multiplicand), .inputB(multiplier), .inputIntermediate(initialIntermediate), .inputWE(ctrl_MULT), .outputA(busA[0]), .outputB(busB[0]), .outputIntermediate(busIntermediate[0]), .outputWE(busWE[0]), .negCand(multiplicand[31]), .negLier(multiplier[15]), .outNegCand(busNegCand[0]), .outNegLier(busNegLier[0]));
	
	aluStage stageTwo(.clock(clock), .inputA(busA[0]), .inputB(busB[0]), .inputIntermediate(busIntermediate[0]), .inputWE(busWE[0]),        .outputA(busA[1]), .outputB(busB[1]), .outputIntermediate(busIntermediate[1]), .outputWE(busWE[1]), .negCand(busNegCand[0]), .negLier(busNegLier[0]), .outNegCand(busNegCand[1]), .outNegLier(busNegLier[1]));

	aluStage stageThree (.clock(clock), .inputA(busA[1]), .inputB(busB[1]), .inputIntermediate(busIntermediate[1]), .inputWE(busWE[1]),        .outputA(busA[2]), .outputB(busB[2]), .outputIntermediate(busIntermediate[2]), .outputWE(busWE[2]), .negCand(busNegCand[1]), .negLier(busNegLier[1]), .outNegCand(busNegCand[2]), .outNegLier(busNegLier[2]));
		
	aluStage stageFour (.clock(clock), .inputA(busA[2]), .inputB(busB[2]), .inputIntermediate(busIntermediate[2]), .inputWE(busWE[2]),        .outputA(busA[3]), .outputB(busB[3]), .outputIntermediate(busIntermediate[3]), .outputWE(busWE[3]), .negCand(busNegCand[2]), .negLier(busNegLier[2]), .outNegCand(busNegCand[3]), .outNegLier(busNegLier[3]));
				
	aluStage stageFive (.clock(clock), .inputA(busA[3]), .inputB(busB[3]), .inputIntermediate(busIntermediate[3]), .inputWE(busWE[3]),        .outputA(busA[4]), .outputB(busB[4]), .outputIntermediate(busIntermediate[4]), .outputWE(busWE[4]), .negCand(busNegCand[3]), .negLier(busNegLier[3]), .outNegCand(busNegCand[4]), .outNegLier(busNegLier[4]));
		
	aluStage stageSix (.clock(clock), .inputA(busA[4]), .inputB(busB[4]), .inputIntermediate(busIntermediate[4]), .inputWE(busWE[4]),        .outputA(busA[5]), .outputB(busB[5]), .outputIntermediate(busIntermediate[5]), .outputWE(busWE[5]), .negCand(busNegCand[4]), .negLier(busNegLier[4]), .outNegCand(busNegCand[5]), .outNegLier(busNegLier[5]));
	
	aluStage stageSeven (.clock(clock), .inputA(busA[5]), .inputB(busB[5]), .inputIntermediate(busIntermediate[5]), .inputWE(busWE[5]),        .outputA(busA[6]), .outputB(busB[6]), .outputIntermediate(busIntermediate[6]), .outputWE(busWE[6]), .negCand(busNegCand[5]), .negLier(busNegLier[5]), .outNegCand(busNegCand[6]), .outNegLier(busNegLier[6]));
	
	aluStage stageEight (.clock(clock), .inputA(busA[6]), .inputB(busB[6]), .inputIntermediate(busIntermediate[6]), .inputWE(busWE[6]),        .outputA(busA[7]), .outputB(busB[7]), .outputIntermediate(busIntermediate[7]), .outputWE(busWE[7]), .negCand(busNegCand[6]), .negLier(busNegLier[6]), .outNegCand(busNegCand[7]), .outNegLier(busNegLier[7]));
	
//	assign result = busIntermediate[7];
//	
//	assign data_RDY = busWE[7];
//	
//	assign testcand = busNegCand[7];
//	assign testlier = busNegLier[7];
	
	wire [31:0] endResult;
	
	finalStageMult lastStage (.clock(clock), .cand(busNegCand[7]), .lier(busNegLier[7]), .readyBit(busWE[7]), .tempResult(busIntermediate[7]), .data_ready(data_RDY), .answer(endResult), .exception(dataException)); 
	
	assign result = endResult;


endmodule


module finalStageMult(clock, cand, lier, tempResult,  readyBit, data_ready, answer, exception);
	input cand, lier, readyBit, clock;
	input [31:0] tempResult;
	output data_ready, exception;
	output [31:0] answer;
	
	wire sign;
	assign sign = tempResult[31];
	
	wire exceptionInput;
	assign exceptionInput = ( (cand ^ lier) & ~sign ) | (~(cand ^ lier) & sign);
	
	registerOne exceptionRegister (.writeIn(exceptionInput), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(exception));
	
	registerOne readyReg(.writeIn(readyBit) , .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(data_ready));
	
	register answerReg(.writeIn(tempResult ), .clock(clock), .writeEnable(1'b1), .reset(1'b1), .readOut(answer));
	
	



endmodule

module aluStage(clock, inputA, inputB, inputIntermediate, inputWE, outputA, outputB, outputIntermediate, outputWE, negCand, negLier, outNegCand, outNegLier);
	input [31:0] inputA, inputIntermediate;
	input [15:0] inputB;
	input inputWE;
	input clock;
	input negCand, negLier;
	output [31:0] outputA, outputIntermediate;
	output [15:0] outputB;
	output outputWE;
	output outNegCand, outNegLier;
	
	wire [31:0] toRegsTO [1:0];
	wire [31:0] inputShiftedBBus;
	wire [31:0] shiftedBBus;
	wire [15:0] toRegsFifteen;
	wire toRegsOne;
	
	aluMod myALU (.data_operandA(inputIntermediate), .data_operandB(inputA), .ctrl_ALUopcode(inputB[2:0]), .data_result(toRegsTO[1])); 
	
	//shiftBLL shifterA (.inputValue(inputA), .shiftNum(1'b00100), .shifted(toRegsTO[0]));
	
	//assign inputShiftedBBus = {1'b0000000000000000,inputB};
	//shiftBRA shifterB (.inputValue(inputShiftedBBus), .shiftNum(1'b00100), .shifted(shiftedBBus));
	//assign toRegsFifteen = shiftedBBus[15:0];
	shiftA myShiftA (.inputA(inputA), .outputA(toRegsTO[0]));
	shiftB myShiftB (.inputB(inputB), .outputB(toRegsFifteen));
	
	
	
	register a (.writeIn(toRegsTO[0]), .clock(clock), .writeEnable(clock),.reset(1'b1),.readOut(outputA));
	register i (.writeIn(toRegsTO[1]), .clock(clock), .writeEnable(clock),.reset(1'b1),.readOut(outputIntermediate));
	registerFifteen b (.writeIn(toRegsFifteen), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outputB));
	registerOne we (.writeIn(inputWE), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outputWE));
	
	registerOne cand (.writeIn(negCand), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outNegCand));
	
	registerOne lier (.writeIn(negLier), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outNegLier));
	
	

endmodule

module aluFirstStage(clock, inputA, inputB, inputIntermediate, inputWE, outputA, outputB, outputIntermediate, outputWE, negCand, negLier, outNegCand, outNegLier);
	input [31:0] inputA, inputIntermediate;
	input [15:0] inputB;
	input inputWE;
	input clock;
	input negCand, negLier;
	output [31:0] outputA, outputIntermediate;
	output [15:0] outputB;
	output outputWE;
	output outNegCand, outNegLier;
	
	wire [31:0] toRegsTOf [1:0];
	wire [31:0] inputShiftedBBusf;
	wire [31:0] shiftedBBusf;
	wire [15:0] toRegsFifteenf;
	wire toRegsOnef;
	
	wire [2:0] firstOpcode;
	assign firstOpcode = {inputB[1:0], 1'b0};
	
	aluMod myALU (.data_operandA(inputIntermediate), .data_operandB(inputA), .ctrl_ALUopcode(firstOpcode), .data_result(toRegsTOf[1])); 
	
	//shiftBLL shifterA (.inputValue(inputA), .shiftNum(1'b00100), .shifted(toRegsTO[0]));
	
	//assign inputShiftedBBus = {1'b0000000000000000,inputB};
	//shiftBRA shifterB (.inputValue(inputShiftedBBus), .shiftNum(1'b00100), .shifted(shiftedBBus));
	//assign toRegsFifteen = shiftedBBus[15:0];
	shiftA myShiftAFirstStage (.inputA(inputA), .outputA(toRegsTOf[0]));
	shiftBFirstStage myShiftBFirstStage (.inputB(inputB), .outputB(toRegsFifteenf));
//	assign toRegsFifteen = inputB;
	
	
	register af (.writeIn(toRegsTOf[0]), .clock(clock), .writeEnable(clock),.reset(1'b1),.readOut(outputA));
	register If (.writeIn(toRegsTOf[1]), .clock(clock), .writeEnable(clock),.reset(1'b1),.readOut(outputIntermediate));
	registerFifteen bf (.writeIn(toRegsFifteenf), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outputB));
	registerOne wef (.writeIn(inputWE), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outputWE));
	
	registerOne candf (.writeIn(negCand), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outNegCand));
	
	registerOne lierf (.writeIn(negLier), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outNegLier));

endmodule

module shiftA(inputA, outputA);
	input [31:0] inputA;
	output [31:0] outputA;
	
	assign outputA = inputA << 2;
endmodule

module shiftB (inputB, outputB);
	input [15:0] inputB;
	output [15:0] outputB;
	assign outputB = inputB >> 2;
endmodule

module shiftBFirstStage (inputB, outputB);
	input [15:0] inputB;
	output [15:0] outputB;
	assign outputB = inputB >> 1;
endmodule

	

module registerFifteen(writeIn, clock, writeEnable, reset, readOut);
	input [15:0] writeIn;
	input clock, reset, writeEnable;
	output [15:0] readOut;
	
	genvar x;
	generate
		for(x = 0 ; x < 16 ; x = x +1) begin: loop1
			DFFE reg_dff(.d(writeIn[x]), .clk(clock), .ena(writeEnable), .q(readOut[x]), .clrn(reset));
		end
	endgenerate
endmodule



module aluMod(data_operandA, data_operandB, ctrl_ALUopcode, data_result);
   input [31:0] data_operandA, data_operandB;
	input [2:0] ctrl_ALUopcode;
   output [31:0] data_result;
	
	wire [7:0] oneHot;
	
	wire [31:0] nottedInputB;
	wire [4:0] selectedOpcode;
	wire [31:0] operationOutput [4:0];
	
	
	
	opcodeDecoderMod myOpcodeDecoderMod (.out(oneHot), .select(ctrl_ALUopcode), .enable(1'b1));
	
	assign selectedOpcode[0] = oneHot[1] | oneHot[2];
	assign selectedOpcode[1] = oneHot[6] | oneHot[5];
	assign selectedOpcode[2] = oneHot[0] | oneHot[7];
	assign selectedOpcode[3] = oneHot[3];
	assign selectedOpcode[4] = oneHot[4];
	
	
	
	wire [31:0] shiftedMBus;
	wire [31:0] shiftedMBusNeg;
	
	
	
	
	
	CLAAdder32Bit myAdd(.inputA(data_operandA), .inputB(data_operandB), .carryIn(1'b0), .sumFinal(operationOutput[0]));
	
	subtractionPrep subPrepOne (.inputB(data_operandB), .notInputB(nottedInputB));
	CLAAdder32Bit mySubtract (.inputA(data_operandA), .inputB(nottedInputB), .carryIn(1'b1), .sumFinal(operationOutput[1]));
	
	assign operationOutput[2] = data_operandA;
	
	shiftBLL shiftedMultiplier (.inputValue(data_operandB), .shiftNum(5'b00001), .shifted(shiftedMBus));
	CLAAdder32Bit myAddShift(.inputA(data_operandA), .inputB(shiftedMBus), .carryIn(1'b0), .sumFinal(operationOutput[3]));
	
	subtractionPrep subPrepTwo (.inputB(shiftedMBus), .notInputB(shiftedMBusNeg));

	CLAAdder32Bit mySubtractShift (.inputA(data_operandA), .inputB(shiftedMBusNeg), .carryIn(1'b1), .sumFinal(operationOutput[4]));
	
	genvar i;
	generate 
		for (i = 0 ; i < 5 ; i = i + 1) begin: loop1
			tbuff32 operationBuffer(.in(operationOutput[i]), .oe(selectedOpcode[i]), .out(data_result));
		end
	endgenerate
endmodule



module opcodeDecoderMod(out,select,enable);
	input [2:0] select;
	input enable;
	output [7:0] out;
	assign out = enable << select;
endmodule




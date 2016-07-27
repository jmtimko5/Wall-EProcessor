module dividerbetter (clock, ctrl_DIV, dividend, divisor, answer, exception, result_RDY);
	input clock, ctrl_DIV;
	input [31:0] dividend;
	input [15:0] divisor;
	output [31:0] answer;
	output exception, result_RDY;
	
	wire [31:0] dividendBus [33:0];
	wire [31:0] divisorBus [33:0];
	wire [31:0] origDivisorBus [33:0];
	wire [31:0] interBus [33:0];
	wire [31:0] resultBus [33:0];
	wire [33:0] weBus;
	wire [33:0] foundBus;
	wire[33:0] negDividendBus;
	wire [33:0] negDivisorBus;
	wire [33:0] exceptionBus;
	
	
	
	wire [15:0] divisorComp;
	wire [15:0] divisorInput;
	getTwosCompFifteen getDivisorComp (.inputA(divisor), .outputA(divisorComp));
	
	assign divisorInput = divisor[15] ? divisorComp : divisor;
	
	wire [31:0] shiftedDivisor;
	shiftDivisor divisorShifter(.inputA(divisorInput), .outputA(shiftedDivisor));
	
	
	wire [31:0] oDivisor;
	assign oDivisor = {16'b000000000000000, divisorInput};
	
	wire [31:0] dividendComp;
	wire [31:0] dividendInput;
	getTwosComp getDividendComp (.inputA(dividend) , .outputA(dividendComp));
	
	
	
	assign dividendInput = dividend[31] ? dividendComp : dividend;
	
	wire exceptionInput;
	checkException checkDivisor (.divisor(divisor), .gotException(exceptionInput)); 
	
	//EXCEPTION IS HARDCODED, KEEP SHIFT RIGHT AS ONE
	divStage first (.clock(clock), .dividend(dividendInput), .divisor(shiftedDivisor >> 1), .inter(32'b00000000000000000000000000000000), .we(ctrl_DIV), .origDivisor(oDivisor), . outDividend(dividendBus[0]), .outDivisor(divisorBus[0]), .outOrigDivisor(origDivisorBus[0]), .outInter(interBus[0]), .outWE(weBus[0]), .result(32'b00000000000000000000000000000000), .outResult(resultBus[0]), .found(1'b0), .outFound(foundBus[0]), .exception(exceptionInput), .negDividend(dividend[31]), .negDivisor(divisor[15]), .outNegDividend(negDividendBus[0]), .outNegDivisor(negDivisorBus[0]), .outException(exceptionBus[0]));

//		divStage first (.clock(clock), .dividend(dividend), .divisor(oDivisor), .inter(32'b00000000000000000000000000000000), .we(ctrl_DIV), .origDivisor(oDivisor), . outDividend(dividendBus[0]), .outDivisor(divisorBus[0]), .outOrigDivisor(origDivisorBus[0]), .outInter(interBus[0]), .outWE(weBus[0]), .result(32'b00000000000000000000000000000000), .outResult(resultBus[0]), .found(1'b0), .outFound(foundBus[0]), .exception(1'b0), .negDividend(dividend[31]), .negDivisor(divisor[15]), .outNegDividend(negDividendBus[0]), .outNegDivisor(negDivisorBus[0]), .outException(exceptionBus[0]));

	
//	divStage second (.clock(clock), .dividend(dividendBus[0]), .divisor( divisorBus[0]), .inter(interBus[0]), .we(weBus[0]), .origDivisor(origDivisorBus[0]), . outDividend(dividendBus[1]), .outDivisor(divisorBus[1]), .outOrigDivisor(origDivisorBus[1]), .outInter(interBus[1]), .outWE(weBus[1]), .result(resultBus[0]), .outResult(resultBus[1]), .found(foundBus[0]), .outFound(foundBus[1]), .exception(exceptionBus[0]), .negDividend(negDividendBus[0]), .negDivisor(negDivisorBus[0]), .outNegDividend(negDividendBus[1]), .outNegDivisor(negDivisorBus[1]), .outException(exceptionBus[1]));
	
//		divStage third (.clock(clock), .dividend(dividendBus[1]), .divisor( divisorBus[1]), .inter(interBus[1]), .we(weBus[1]), .origDivisor(origDivisorBus[1]), . outDividend(dividendBus[2]), .outDivisor(divisorBus[2]), .outOrigDivisor(origDivisorBus[2]), .outInter(interBus[2]), .outWE(weBus[2]), .result(resultBus[1]), .outResult(resultBus[2]), .found(foundBus[1]), .outFound(foundBus[2]), .exception(exceptionBus[1]), .negDividend(negDividendBus[1]), .negDivisor(negDivisorBus[1]), .outNegDividend(negDividendBus[2]), .outNegDivisor(negDivisorBus[2]), .outException(exceptionBus[2]));

	
	

	genvar c;
	generate
		for(c = 1 ; c < 34 ; c = c + 1 ) begin: looppp
			divStage myStage (.clock(clock), .dividend(dividendBus[c-1]), .divisor(divisorBus[c-1]), .inter(interBus[c-1]), .we(weBus[c-1]), .origDivisor(origDivisorBus[c-1]), .found(foundBus[c-1]), . outDividend(dividendBus[c]), .outDivisor(divisorBus[c]), .outOrigDivisor(origDivisorBus[c]), .outInter(interBus[c]), .outWE(weBus[c]), .result(resultBus[c-1]), .outResult(resultBus[c]), .outFound(foundBus[c]),  .exception(exceptionBus[c-1]), .negDividend(negDividendBus[c-1]), .negDivisor(negDivisorBus[c-1]), .outNegDividend(negDividendBus[c]), .outNegDivisor(negDivisorBus[c]), .outException(exceptionBus[c]));
		end
	endgenerate
	
//	assign result_RDY = weBus[33];
//	assign answer = resultBus[33];
//	assign exception = exceptionBus[33];

	finalStage last (.clock(clock), .negDividend(negDividendBus[33]), .negDivisor(negDivisorBus[33]), .exception(exceptionBus[33]), .tempResult(resultBus[33]), .readyBit(weBus[33]), .finalResult(answer), .finalException(exception), .finalReadyBit(result_RDY));
	

endmodule

module divStage(clock, dividend, divisor, found, result , inter, we, origDivisor, exception, negDividend, negDivisor, outDividend, outFound ,outDivisor, outOrigDivisor, outInter, outWE, outException, outNegDividend, outNegDivisor, outResult);

//LESS THAN WORKS BACKWARDS HAD TO SWITCH INPUTS FOR CHECK IF WE GOT RESULT
//THIS IS INCONSISTENT CODING!!!
	input [31:0] dividend, divisor, inter, origDivisor, result;
	input we, clock, found, negDividend, negDivisor, exception;
	output [31:0] outDividend, outDivisor, outOrigDivisor, outInter, outResult;
	output outWE, outFound, outNegDividend, outNegDivisor, outException; 
	
	
	wire less;
	wire notEq;
	wire lessThanOrEqual;
	wire [31:0] subtractToMux;
	
	
	alu compDivDiv (.data_operandA(divisor), .data_operandB(dividend), .ctrl_ALUopcode(5'b00001), .isNotEqual(notEq), .isLessThan(less));
	
	alu subDivisorFromDividend (.data_operandA(dividend), .data_operandB(divisor), .ctrl_ALUopcode(5'b00001), .data_result(subtractToMux));
	assign lessThanOrEqual = less | ~notEq;
	
	wire [31:0] dividendResult;
	
	assign dividendResult = lessThanOrEqual ? subtractToMux : dividend;
//	assign dividendResult = lessThanOrEqual ? dividend : subtractToMux;
	register dividendReg (.writeIn(dividendResult), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outDividend));
	
	register divisorReg (.writeIn(divisor >> 1), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outDivisor));
	register origDivisorReg(.writeIn(origDivisor), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outOrigDivisor));
	
	wire [31:0] one;
	assign one =  32'b00000000000000000000000000000001;
	wire [31:0] zero;
	assign zero = 32'b00000000000000000000000000000000;
	wire [31:0] addToInter;
	assign addToInter = lessThanOrEqual ? one : zero;
//	assign addToInter = lessThanOrEqual ? zero : one;
	
	wire [31:0] interToReg;
	alu compInter (.data_operandA(inter << 1), .data_operandB(addToInter), .ctrl_ALUopcode(5'b00000), .data_result(interToReg));
	
	
	register interReg(.writeIn(interToReg), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outInter));
	
	wire stop;
//	alu checkStop (.data_operandA(origDivisor), .data_operandB(divisor), .ctrl_ALUopcode(5'b00001), .isLessThan(stop));
		alu checkStop (.data_operandA(divisor), .data_operandB(origDivisor), .ctrl_ALUopcode(5'b00001), .isLessThan(stop));
		
	wire thisIsTheOutput;
	assign thisIsTheOutput = stop & ~found;
	wire [31:0] thisResult;
	
	
	tbuff32 potentialResult (.in(inter), .oe(thisIsTheOutput), .out(thisResult));
	
	wire [31:0] resultRegInput;
	assign resultRegInput = thisIsTheOutput ? thisResult : result;
	
	register resultReg (.writeIn(resultRegInput), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outResult));
	
	registerOne weReg (.writeIn(we), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outWE)); 
	
	wire foundIt;
	assign foundIt = stop | found;
	registerOne foundReg (.writeIn(foundIt), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outFound)); 
	
	registerOne negDividendReg(.writeIn(negDividend), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outNegDividend));
	
	registerOne negDivisorReg(.writeIn(negDivisor), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outNegDivisor));
	
	registerOne exceptionREg(.writeIn(exception), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(outException));
	

endmodule






	
	
	
	
	
module shiftDivisor(inputA, outputA);
	input [15:0] inputA;
	
	output [31:0] outputA;
	wire [15:0] result;
	wire [20:0] foundBus;
	wire [20:0] hfBus;

	compStage firstStage(.hfIn(1'b0), .x2(1'b0), .x1(inputA[15]), .found(foundBus[0]), .hfOut(hfBus[0]));
	
	genvar i;
	generate
		for(i = 15 ; i > 0 ; i = i - 1) begin: loop
			compStage stage (.hfIn(hfBus[(0 + (15 - i))]), .x2(inputA[i]), .x1(inputA[i-1]), .found(foundBus[(1 + (15 - i))]), .hfOut(hfBus[(1 + (15 - i ))]));
		end
	endgenerate
	
	genvar y;
	generate
		for(i = 0 ; i < 16 ; i = i + 1) begin: loopp
			tbuff32 buffGUY (.in(inputA << i), .oe(foundBus[i]), .out(result));	
		end
	endgenerate
	
	assign outputA = {result, 16'b0000000000000000};
	
endmodule

module compStage(hfIn, x2, x1, found, hfOut);
	input hfIn, x2, x1;
	output found, hfOut;
	
	assign found = ~hfIn & ~x2 & x1;
	assign hfOut = hfIn | (~hfIn & ~x2 & x1);
endmodule

module checkException (divisor, gotException);
	input [15:0] divisor;
	output gotException;
	
	wire notEqualToZero;
	alu checkZero (.data_operandA(divisor), .data_operandB(32'b00000000000000000000000000000000), .ctrl_ALUopcode(5'b00001), .isNotEqual(notEqualToZero));
	
	assign gotException = ~notEqualToZero;
	
endmodule

module getTwosComp(inputA, outputA);
	input [31:0] inputA;
	output [31:0] outputA;
	
	wire [31:0] notted;
	assign notted = ~inputA;
	
	alu addOne (.data_operandA(notted), .data_operandB(32'b00000000000000000000000000000001), .ctrl_ALUopcode(5'b00000), .data_result(outputA));
	
endmodule

module getTwosCompFifteen(inputA, outputA);
	input [15:0] inputA;
	output [15:0] outputA;
	
	wire [15:0] notted;
	assign notted = ~inputA;
	
	alu addOne (.data_operandA(notted), .data_operandB(16'b0000000000000001), .ctrl_ALUopcode(5'b00000), .data_result(outputA));
	
endmodule

module finalStage (clock, negDividend, negDivisor, tempResult, exception, readyBit, finalResult, finalException, finalReadyBit);
	input negDividend, negDivisor, readyBit, exception, clock;
	input [31:0] tempResult;
	output [31:0] finalResult;
	output finalReadyBit, finalException;
	
	wire invertIt;
	assign invertIt = negDividend ^ negDivisor;
	
	wire [31:0] compedResult;
	getTwosComp mytwoscompGUY (.inputA(tempResult), .outputA(compedResult));
	
	wire [31:0] finalResultToReg;
	assign finalResultToReg = invertIt ? compedResult : tempResult;
	
	register finalResultReg(.writeIn(finalResultToReg), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(finalResult));
	
	registerOne finalReadyReg(.writeIn(readyBit), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(finalReadyBit));
	
	registerOne finalExceptionReg (.writeIn(exception), .clock(clock), .writeEnable(clock), .reset(1'b1), .readOut(finalException));
	
endmodule

module multdivbetter(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, clock, data_result, data_exception, data_inputRDY, data_resultRDY, testAssertDiv, testAssertMult, testMultiplicandIn, testDivisorIn, testDivResult, testMultResult, testMultiplierIn, testMultReady, testDivReady);
   input [31:0] data_operandA;
   input [15:0] data_operandB;
   input ctrl_MULT, ctrl_DIV, clock;
   output [31:0] data_result;
   output data_exception, data_inputRDY, data_resultRDY;

	wire assertMultiplication;
	assign assertMultiplication = ctrl_MULT & ~ctrl_DIV;

	wire assertDivide;
	assign assertDivide = ctrl_DIV & ~ctrl_MULT;

	output testAssertDiv, testAssertMult;
	assign testAssertDiv = assertDivide;
	assign testAssertMult = assertMultiplication;


	wire [31:0] multiplicandIn;
	wire [15:0] multiplierIn;
	wire ctrl_MULTIn;

	wire [31:0] mult_result;
	wire [31:0] div_result;

	wire multReady;
	wire divReady;

	wire multEx;
	wire divEx;

	assign data_inputRDY = 1'b1;

	output [31:0] testMultiplicandIn, testDivResult, testMultResult;



	tbuff32 multiplicandBuff (.in(data_operandA), .oe(assertMultiplication), .out(multiplicandIn));

	tbuff32 multiplierBuff (.in(data_operandB), .oe(assertMultiplication), .out(multiplierIn));
	tbuff ctrl_MULTBuff (.in(ctrl_MULT), .oe(assertMultiplication), .out(ctrl_MULTIn));

	assign testMultiplicandIn = multiplicandIn;

	multiplierbetter myMultiplier (.clock(clock), .multiplicand(multiplicandIn), .multiplier(multiplierIn), .ctrl_MULT(ctrl_MULTIn), .dataException(multEx), .data_RDY(multReady), .result(mult_result));



	wire [31:0] dividendIn;
	wire [15:0] divisorIn;
	wire ctrl_DIVIn;



	tbuff32 dividendBuff (.in(data_operandA), .oe(assertDivide), .out(dividendIn));
	tbuff32 divisorBuff (.in(data_operandB), .oe(assertDivide), .out(divisorIn));
	tbuff ctrl_DIVBuff (.in(ctrl_DIV), .oe(assertDivide), .out(ctrl_DIVIn));

	output [15:0] testDivisorIn, testMultiplierIn;
	assign testDivisorIn = divisorIn;
	assign testMultiplierIn = multiplierIn;

	dividerbetter myDivider (.clock(clock), .ctrl_DIV(ctrl_DIVIn), .dividend(dividendIn), .divisor(divisorIn), .answer(div_result), .exception(divEx), .result_RDY(divReady));

	assign testDivResult = div_result;
	assign testMultResult = mult_result;

	wire showDiv;
	assign showDiv = (divReady ^ multReady) & divReady;

	wire showMult;
	assign showMult = (divReady ^ multReady) & multReady;

	wire [31:0] multdivResult;

	tbuff32 divResultBuff (.in(div_result), .oe(showDiv), .out(multdivResult));
	tbuff32 multResultBuff (.in(mult_result), .oe(showMult), .out(multdivResult));

	assign data_result = multdivResult;

	wire multdivReady;

	tbuff divReadyBuff (.in(divReady), .oe(showDiv), .out(multdivReady));
	tbuff multReadyBuff (.in(multReady), .oe(showMult), .out(multdivReady));
	tbuff nothingReadyBuff (.in(1'b0), .oe(~showDiv & ~showMult), .out(multdivReady));

	assign data_resultRDY = multdivReady;

	wire multdivException;

	//is this potentiall redundant?
	tbuff divExBuff (.in(divEx), .oe(showDiv), .out(multdivException));
	tbuff multExBuff (.in(multEx), .oe(showMult), .out(multdivException));
	tbuff neitherExceptionBuff (.in(1'b0), .oe(~showDiv & ~showMult), .out(multdivException));

	assign data_exception = multdivException;

	output testDivReady, testMultReady;
	assign testDivReady = divReady;
	assign testMultReady = multReady;

//	assign data_result = divReady ? div_result : mult_result;
//	assign data_resultRDY = divReady ? divReady : multReady;
//	assign data_exception = divReady ? divEx : multEx;

endmodule

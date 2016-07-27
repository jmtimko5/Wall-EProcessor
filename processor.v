module processor(clock, reset, ps2_key_pressed, ps2_out, lcd_write, lcd_data, debug_data, debug_addr, testInstructionIn, testWEOut, testDataOut, testRegOut, testALUOutput, testMuxB, testDataInMemory,testWEMEM, testNOP, testFlush, testOpcode, testMuxA, pc_reference, instruction_reference, testRX_Execute, testFetchNOP, testStall);
	//if active high pipe reset, if low pipe ~reset
	output [31:0] testALUOutput;
	output [31:0] testMuxB;
	output [31:0] testDataInMemory;
	output [4:0] testOpcode, testRX_Execute;
	output [31:0] testMuxA;
	output [31:0] pc_reference;
	output [31:0] instruction_reference;
	output testWEMEM, testNOP, testFlush, testFetchNOP;
	output testStall;
	
	
	input [31:0] testInstructionIn;
	output testWEOut;
	output [31:0] testDataOut;
	output [4:0] testRegOut;

	input 			clock, reset, ps2_key_pressed;
	input 	[7:0]	ps2_out;
	
	output 			lcd_write;
	output 	[31:0] 	lcd_data;
	
	// GRADER OUTPUTS - YOU MUST CONNECT TO YOUR DMEM
	output 	[31:0] 	debug_data;
	output	[11:0]	debug_addr;
	
	
	// your processor here
	//
	
	//////////////////////////////////////
	////// THIS IS REQUIRED FOR GRADING
	// CHANGE THIS TO ASSIGN YOUR DMEM WRITE ADDRESS ALSO TO debug_addr
//	assign debug_addr = (12'b000000000001);
	assign debug_addr = alu_output_memory[11:0];
	// CHANGE THIS TO ASSIGN YOUR DMEM DATA INPUT (TO BE WRITTEN) ALSO TO debug_data
//	assign debug_data = (12'b000000000001);
 //WHY IS IS THE LOWER 12 bits?!?!?!
	wire [31:0] dmem_data_in_memory;
	wire xm_bypass;
	assign dmem_data_in_memory = xm_bypass ? data_writeback : rtrd_value_memory;
//	assign debug_data = rtrd_value_memory; 
	assign debug_data = dmem_data_in_memory;
	////////////////////////////////////////////////////////////
	
		
	// You'll need to change where the dmem and imem read and write...
	dmem mydmem(	.address	(debug_addr),
					.clock		(~clock), //TODO CHECK IF THIS FIXING THE PROBLEM
					.data		(debug_data), //CHANGE THIS TO DEBUG_DATA
					.wren		(we_memory) ,	
					.q			(dmem_output_memory) // change where output q goes...
	);
	
	
	
	wire [31:0] readInstruction;
	wire [31:0] pc;
	wire [31:0] pcPlusOne;
	////TODO MUST CHANGE TO ALLOW FOR NEW PC VALUES, WRITE ENABLE CONNECTED TO STALL LOGIC
	register programCounter (.writeIn(pc_next), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(pc));
	
	wire [31:0] stall_pc_fetch;
	assign stall_pc_fetch = pc;
	
	assign testDataInMemory = dmem_data_in_memory;
	
	alu pcIncrementOne (.data_operandA(pc), .data_operandB(16'b0000000000000001), .ctrl_ALUopcode(5'b00000), .data_result(pcPlusOne));
	
	wire [31:0] pcplusone_fetch;
	wire [31:0] pcplusone_buffer_in;
	assign pcplusone_buffer_in = stall_needed ? pcplusone_fetch : pcPlusOne;
	
	register pcPlusOneBuffer (.writeIn(pcplusone_buffer_in), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(pcplusone_fetch));
	
	wire [31:0] pc_fetch_reference;
	wire [31:0] pc_buffer_in;
	assign pc_buffer_in = stall_needed ? pc_fetch_reference : pc;
	register pcBuffer (.writeIn(pc_buffer_in), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(pc_fetch_reference));
	
	
	wire [31:0] pc_in_imem;
	assign pc_in_imem = stall_needed ? pc_fetch_reference : pc;
	
	imem myimem(	.address 	(pc_in_imem[11:0]), //gonna change this to PC_NEXT because i think i created an extra stage?
					.clken		(1'b1),
					.clock		(clock), 
					.q			(readInstruction) // change where output q goes...
	); 
	
	wire [31:0] nop_instruction_in_fetch;
	assign nop_instruction_in_fetch = 32'b00000000000000000000000000000000;
	
	wire [31:0] pcInDecode, instructionInDecode;
	 //THE REGFILE RESET IS NOW ACTIVE HIGH, BUT REGISTERS ARE RESET ON ACTIVE LOW
	
	///////TODO MUST ASSIGN FETCH NOP
	wire ir_NOP_fd;
	wire ir_NOP_dx;
	nopMod fd_nop (.clock(clock), .reset(reset), .flushSignal(flush_execute), .assertNOP(ir_NOP_fd));
	nopMod dx_nop (.clock(clock), .reset(reset), .flushSignal(flush_execute), .stallSignal(stall_needed), .assertNOP(ir_NOP_dx));
	
	wire fetch_NOP;
	registerOne fetchClear (.writeIn(ir_NOP_fd), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(fetch_NOP));
	
	stallLogic myStallLogic (.clock(clock), .opcode_dx(opcode_execute), .opcode_fd(opcodeDecode), .rd_dx(rd_execute), .rs_fd(rsDecode), .rt_fd(rtDecode), .stall_output(stall_needed));
	
//	wire clear_stalled_instruction;
//	assign clear_stalled_instruction = stall_needed;
//	registerOne holdStall (.writeIn(stall_needed), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(clear_stalled_instruction));
//TODO DECIDE IF WE WANT TO CLEAR OUT STALLED INSTRUCTIONS
	wire fetch_NOP_neg;
	registerOne holdFetchNOP (.writeIn(fetch_NOP), .clock(~clock), .writeEnable(1'b1), .reset(~reset), .readOut(fetch_NOP_neg));

	
	
//	/
//	/
//	/
//	/
//	/
//	/	/
//
//	/
//	/
	
	wire [31:0] instruction_nop_or_read;
	wire [31:0] instruction_fetch_output;
	wire zeroInstruction;
	assign zeroInstruction = fetch_NOP | fetch_NOP_neg;
	assign instruction_nop_or_read = zeroInstruction ? nop_instruction_in_fetch : readInstruction;
	assign instruction_fetch_output = stall_needed ? instructionInDecode : instruction_nop_or_read;
	
	irFetchDecode myFDReg (.clock(clock), .reset(reset), .pcIn(pcplusone_fetch), .instructionIn(instruction_fetch_output), .pcOut(pcInDecode), .instructionOut(instructionInDecode), .nop(ir_NOP_fd));

//	/
//	/
//	/
//	/
//	/
//	/
//	/
//	/
//	/
//	/
//	/
	
	////TODO ASSIGN THESE WB WIRES
	wire we_writeback;
	wire [31:0] data_writeback;
	wire [4:0] reg_number_writeback;
	
	wire [4:0] rdDecode, rsDecode, rtDecode, shiftamtDecode, aluopDecode, opcodeDecode;
	wire [16:0] imedDecode;
	wire [26:0] targetDecode;
	
	assign opcodeDecode = instructionInDecode[31:27];
	assign rdDecode = instructionInDecode[26:22];
	assign rsDecode = instructionInDecode[21:17];
	assign rtDecode = instructionInDecode[16:12];
	assign shiftamtDecode = instructionInDecode[11:7];
	assign aluopDecode = instructionInDecode[6:2];
	assign imedDecode = instructionInDecode[16:0];
	assign targetDecode = instructionInDecode[26:0];
	
	wire [31:0] rsValueDecode, rtrdValueDecode;
	
	wire [4:0] secondReadRegDecode;
	
	wire selectRD;
	assign selectRD = opcodeDecode[0] | opcodeDecode[1] | opcodeDecode[2] | opcodeDecode[3] | opcodeDecode[4];
	
	assign secondReadRegDecode = selectRD ? rdDecode : rtDecode;
	
	wire [31:0] rs_value_read_regfile, rtrd_value_read_regfile;
	wire rs_bypass_decode, rtrd_bypass_decode;
	
	checkEqualStall checkRSRead (.inputA(rsDecode), .inputB(rd_writeback), .out(rs_bypass_decode));
	
	checkEqualStall CheckRRTRDRead (.inputA(secondReadRegDecode), .inputB(rd_writeback), .out(rtrd_bypass_decode));
	
	
	regfile myRegFile (.clock(clock), .ctrl_writeEnable(we_writeback), .ctrl_reset(reset), .ctrl_writeReg(reg_number_writeback), .ctrl_readRegA(rsDecode), .ctrl_readRegB(secondReadRegDecode), .data_writeReg(data_writeback), .data_readRegA(rs_value_read_regfile), .data_readRegB(rtrd_value_read_regfile));
	
	assign rsValueDecode = rs_bypass_decode ? data_writeback : rs_value_read_regfile;
	assign rtrdValueDecode = rtrd_bypass_decode ? data_writeback : rtrd_value_read_regfile;
//	/
//	/
//	/
//	/
//	/
//	/
//	/
//	/
//	/
//	/
//	
//	/
	wire [31:0] pc_execute,rs_value_execute, rtrd_value_execute;
	wire [4:0] rd_execute, rs_execute, rt_execute, rx_execute, shiftamt_execute, aluop_execute, opcode_execute;
	wire [16:0] imed_execute;
	wire [26:0] target_execute;
	
	irDecodeExecute myIRDX ( .clock(clock), .reset(reset), .nop(ir_NOP_dx), .pcIn(pcInDecode), .rdIn(rdDecode), .rsIn(rsDecode), .rtIn(rtDecode), .rsValueIn(rsValueDecode), .rtrdValueIn(rtrdValueDecode), .shiftamtIn(shiftamtDecode), .aluopIn(aluopDecode), .imedIn(imedDecode), .targetIn(targetDecode), .opcodeIn(opcodeDecode), .pcOut(pc_execute), .rdOut(rd_execute), .rsOut(rs_execute), .rtOut(rt_execute), .rsValueOut(rs_value_execute), .rtrdValueOut(rtrd_value_execute), .shiftamtOut(shiftamt_execute), .aluopOut(aluop_execute), .imedOut(imed_execute), .targetOut(target_execute), .opcodeOut(opcode_execute), .rxIn(secondReadRegDecode), .rxOut(rx_execute));
//	/
//	/
//	/
//	/
//	/
//	/
//	/
//	/
//	/
	wire overflow_multdiv_writeback;
	wire status_execute;
	//TODO FIND OUT HOW TO DEAL WITH SPURIOS LAGGING SETX INSTRUCTION
	registerOne STATUS_REGISTER (.writeIn(overflow_multdiv_writeback), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(status_execute));
	
	
//HERE BEGINS BRANCHING AND JUMPING LOGIC/////////////////////////////////////////////////////////////////////	

	
	wire branch_less_execute, branch_notequal_execute;
	alu branchALU (.data_operandA(rtrd_value_execute), .data_operandB(rs_value_execute), .ctrl_ALUopcode(5'b00001), .isLessThan(branch_less_execute), .isNotEqual(branch_notequal_execute));
	
	wire branch_execute;
	branchLogic branch_logic_execute (.opcode(opcode_execute), .branchLessResult(branch_less_execute), .branchNotEqualResult(branch_notequal_execute), .currentStatus(status_execute), .shouldBranch(branch_execute));
	
	wire is_jump_execute;
	checkJumpInstruction check_jump_execute (.opcode(opcode_execute), .out(is_jump_execute));
	
	wire jump_or_branch, flush_execute;
	assign jump_or_branch = is_jump_execute | branch_execute;
	assign flush_execute = jump_or_branch;
	
	wire [31:0] branch_address;
	alu jump_address_alu (.data_operandA(pc_execute), .data_operandB(imed_extended_execute), .ctrl_ALUopcode(5'b00000), .data_result(branch_address));
	
	wire is_jr;
	isJumpReturn explicitcheckjr (.in(opcode_execute), .out(is_jr));
	
	wire [31:0] return_imed_mux;
	assign return_imed_mux = is_jr ? rtrd_value_execute : imed_extended_execute;
	
	wire [31:0] jumpalu_returnimed_mux;
	assign jumpalu_returnimed_mux = branch_execute ? branch_address : return_imed_mux; 
	
	wire [31:0] pc_calculated;
	wire [31:0] pc_next;
	wire stall_needed;
	assign pc_calculated = jump_or_branch ? jumpalu_returnimed_mux : pcPlusOne;
	assign pc_next = stall_needed ? stall_pc_fetch : pc_calculated;
		
//  HERE ENDS BRANCHING AND JUMPING LOGIC ////////////////////////////////////////////////////////////////////////////////////	
	
	wire [31:0] imed_extended_execute;
	signExtender myExtender_execute (.inputImed(imed_execute), .outputImed(imed_extended_execute));
	
	
	wire notRType;
	assign notRType = (|opcode_execute); 
	
	wire [31:0] muxed_rtrd_value_execute;
	aluBypassMux mux_rtrd_value (.input_zero(rtrd_value_execute), .input_one(alu_output_memory), .input_two(data_writeback), .rd_xm(rd_memory), .rx_dx(rx_execute), .rd_mw(rd_writeback), .opcode_xm(opcode_memory), .opcode_mw(opcode_writeback), .alu_input(muxed_rtrd_value_execute));
	
	
	//todo change input to reg file, and change it in ternary statement below
	wire [31:0] alu_input_B_execute;
//	assign alu_input_B_execute = notRType ? imed_extended_execute : rtrd_value_execute;
	assign alu_input_B_execute = notRType ? imed_extended_execute : muxed_rtrd_value_execute;
	
	wire [4:0] hardcode_add;
	assign hardcode_add = 5'b00000;
	
	wire [4:0] alu_input_opcode;
	assign alu_input_opcode = notRType ? hardcode_add : aluop_execute;
	
	
	wire [31:0] muxed_alu_input_A;
	aluBypassMux mux_input_A (.input_zero(rs_value_execute), .input_one(alu_output_memory), .input_two(data_writeback), .rd_xm(rd_memory), .rx_dx(rs_execute), .rd_mw(rd_writeback), .opcode_xm(opcode_memory), .opcode_mw(opcode_writeback), .alu_input(muxed_alu_input_A));
	
	wire [31:0] alu_output_execute;
	alu alu_execute (.data_operandA(muxed_alu_input_A), .data_operandB(alu_input_B_execute), .ctrl_shiftamt(shiftamt_execute), .ctrl_ALUopcode(alu_input_opcode), .data_result(alu_output_execute));
//	alu alu_execute (.data_operandA(rs_value_execute), .data_operandB(alu_input_B_execute), .ctrl_shiftamt(shiftamt_execute), .ctrl_ALUopcode(alu_input_opcode), .data_result(alu_output_execute));
	
//
//
//
//
//
//
//
//
//
//
	wire [4:0] opcode_memory, rd_memory, rs_memory, rt_memory;
	wire [31:0] pc_memory, alu_output_memory, rtrd_value_memory;
	irExecuteMemory myIRXM (.clock(clock), .reset(reset), .pcIn(pc_execute), .pcOut(pc_memory), .rdIn(rd_execute), .rdOut(rd_memory), .rsIn(rs_execute), .rsOut(rs_memory), .rtIn(rt_execute), .rtOut(rt_memory), .opcodeIn(opcode_execute), .opcodeOut(opcode_memory), .alu_outputIn(alu_output_execute), .alu_outputOut(alu_output_memory), .rtrd_valueIn(muxed_rtrd_value_execute), .rtrd_valueOut(rtrd_value_memory));
//
//
//
//
//
//
//
//
//
//
//
	wire we_memory;
	assign we_memory = ~opcode_memory[4] & ~opcode_memory[3] & opcode_memory[2] & opcode_memory[1] & opcode_memory[0]; //00111
	//TODO must sign extend this value if its only 12 bits... assuming it's 32 bits
	wire [31:0] dmem_output_memory;
//
///
//
//
//
//
//
//
//
//
//
//
	wire [4:0] opcode_writeback, rd_writeback, rs_writeback, rt_writeback;
	wire [31:0] pc_writeback, alu_output_writeback, dmem_output_writeback;
	
	irMemoryWriteback myIRMW (.clock(clock), .reset(reset), .pcIn(pc_memory), .rdIn(rd_memory), .rsIn(rs_memory), .rtIn(rt_memory), .opcodeIn(opcode_memory), .alu_outputIn(alu_output_memory), .mem_outputIn(dmem_output_memory), .pcOut(pc_writeback), .rdOut(rd_writeback), .rsOut(rs_writeback), .rtOut(rt_writeback), .opcodeOut(opcode_writeback), .alu_outputOut(alu_output_writeback), .mem_outputOut(dmem_output_writeback));
//
//
//
//
//
//
//
//
//
//

	xmBypassLogic myXMBypass(.rd_xm(rd_memory), .rd_mw(rd_writeback), .opcode_xm(opcode_memory), .xm_bypass_select(xm_bypass));
	
	wire [31:0] alu_or_mem_writeback;
	wire isLW_writeback;
	assign isLW_writeback = ~opcode_writeback[4] & opcode_writeback[3] & ~opcode_writeback[2] & ~opcode_writeback[1] & ~opcode_writeback[0];
	assign alu_or_mem_writeback = isLW_writeback ? dmem_output_writeback : alu_output_writeback;
	
	wire isJAL_writeback;
	isJumpAndLink myJALcheck_writeback (.in(opcode_writeback), .out(isJAL_writeback));
	
	wire [31:0] pc_or_alumem_writeback;
	assign pc_or_alumem_writeback = isJAL_writeback ? pc_writeback : alu_or_mem_writeback;
	
	assign data_writeback = pc_or_alumem_writeback;
	
	wire [4:0] thirtyOne_writeback;
	assign thirtyOne_writeback = 5'b11111;
	assign reg_number_writeback = isJAL_writeback ? thirtyOne_writeback : rd_writeback;
	
	wire isADDI_writeback;
	assign isADDI_writeback = ~opcode_writeback[4] & ~opcode_writeback[3] & opcode_writeback[2] & ~opcode_writeback[1] & opcode_writeback[0];
	
	assign we_writeback = isJAL_writeback | isLW_writeback | !(|opcode_writeback) | isADDI_writeback;
///
///
//
//
//
//
//
//
//
//
//
//
//
//
	
	assign testWEOut = we_writeback;
//	assign testWEOut = (|opcodeDecode);


	assign testDataOut = data_writeback;
//	assign testDataOut = alu_output_memory;
//	assign testDataOut = pc_execute;
	
	assign testRegOut = reg_number_writeback;
//	assign testRegOut = rd_execute;
	
	assign testALUOutput = alu_output_execute;
//	assign testALUOutput = pc_memory;

	assign testMuxB = muxed_rtrd_value_execute;
	
	assign testWEMEM = we_memory;
	assign testNOP = ir_NOP_dx;
	assign testFlush = flush_execute;
//	assign testFlush = fetch_NOP_neg;
	assign testOpcode = opcodeDecode;
	assign testMuxA = muxed_alu_input_A;
	assign pc_reference = pc_fetch_reference;
	assign instruction_reference = instruction_fetch_output;
	assign testRX_Execute = rx_execute;
	assign testFetchNOP = fetch_NOP;
	assign testStall = stall_needed;
	
	
endmodule

module stallLogic (clock, opcode_dx, opcode_fd, rd_dx, rs_fd, rt_fd, stall_output);
	input clock;
	input [4:0] opcode_dx, opcode_fd, rd_dx, rs_fd, rt_fd;
	output stall_output;
	
	wire load_dx;
	assign load_dx = ~opcode_dx[4] & opcode_dx[3] & ~opcode_dx[2] & ~opcode_dx[1] & ~opcode_dx[0];
	
	wire save_dx; 
	assign save_dx = ~opcode_dx[4] & ~opcode_dx[3] & opcode_dx[2] & opcode_dx[1] & opcode_dx[0];
	
	wire branch_dx;
	assign branch_dx = (~opcode_dx[3] & opcode_dx[1] & ~opcode_dx[0]) & (~opcode_dx[4] | (opcode_dx[4] & opcode_dx[2]));
	
	wire save_fd; 
	assign save_fd = ~opcode_fd[4] & ~opcode_fd[3] & opcode_fd[2] & opcode_fd[1] & opcode_fd[0];
	
	wire rd_dx_equals_rs_fd; 
	checkEqualStall rd_dx_rs_fd (.inputA(rd_dx), .inputB(rs_fd), .out(rd_dx_equals_rs_fd));
	
	wire rd_dx_equals_rt_fd; 
	checkEqualStall rd_dx_rt_fd (.inputA(rd_dx), .inputB(rt_fd), .out(rd_dx_equals_rt_fd));
	
	
	
	wire case_one;
	assign case_one = load_dx & (rd_dx_equals_rs_fd | rd_dx_equals_rt_fd) & !save_dx;
	
	wire case_two;
	assign case_two = load_dx & rd_dx_equals_rs_fd & save_fd;
	
	wire assertStall;
	assign assertStall = case_one | case_two;
//	assign assertStall = case_one;
//	assign assertStall = load_dx;
//	registerOne flushStallStored (.writeIn(assertStall), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(stall_output));
	assign stall_output = assertStall;
	
endmodule

module aluBypassMux (input_zero, input_one, input_two, rd_xm, rx_dx, rd_mw, opcode_xm, opcode_mw, alu_input);
	input [31:0] input_zero, input_one, input_two;
	input [4:0] rd_xm, rx_dx;
	input [4:0] rd_mw, opcode_xm, opcode_mw;
	output [31:0] alu_input;
	
	wire select_one, select_two;
	
	wire rd_xm_equals_rx_dx, rd_mw_equals_rx_dx, rtype_xm, load_or_rtype_mw, load_mw, addi_xm, addi_mw;
	checkEqualStall checkrdxmrxdx (.inputA(rd_xm), .inputB(rx_dx), .out(rd_xm_equals_rx_dx));
	checkEqualStall checkrdmwrxdx (.inputA(rd_mw), .inputB(rx_dx), .out(rd_mw_equals_rx_dx));
	
	assign load_mw = ~opcode_mw[4] & opcode_mw[3] & ~opcode_mw[2] & ~opcode_mw[1] & ~opcode_mw[0];
	
	assign addi_xm = ~opcode_xm[4] & ~opcode_xm[3] & opcode_xm[2] & ~opcode_xm[1] & opcode_xm[0];
	assign addi_mw = ~opcode_mw[4] & ~opcode_mw[3] & opcode_mw[2] & ~opcode_mw[1] & opcode_mw[0];
	
	///CHANGED SO THAT ADDI CONSIDERED A PSUEDO RTYPE FOR THIS PURPOSE
	assign rtype_xm = !(|opcode_xm) | addi_xm;
	assign load_or_rtype_mw = !(|opcode_mw) | load_mw | addi_mw;
	
	assign select_one = rd_xm_equals_rx_dx & rtype_xm;
	assign select_two = rd_mw_equals_rx_dx & load_or_rtype_mw;
	
	wire [31:0] zero_two_bus;
	
	assign zero_two_bus = select_two ? input_two : input_zero;
	assign alu_input = select_one ? input_one : zero_two_bus;

endmodule


module checkEqualStall (inputA, inputB, out);
	input [4:0] inputA, inputB;
	output out;
	
	wire [31:0] inA, inB;
	assign inA = {27'b000000000000000000000000000,inputA};
	assign inB = {27'b000000000000000000000000000, inputB};
	
	wire noteq;
	alu checkequalalu (.data_operandA(inA), .data_operandB(inB), .ctrl_ALUopcode(5'b00001), .isNotEqual(noteq));
	
	assign out = ~noteq;

endmodule

module branchLogic (opcode, branchLessResult, branchNotEqualResult, currentStatus, shouldBranch);
	input [4:0] opcode;
	input branchLessResult, branchNotEqualResult, currentStatus;
	output shouldBranch;
	
	wire branchBNE, branchBLT, branchX;
	assign branchBNE = branchNotEqualResult & ~opcode[4] & ~opcode[3] & ~opcode[2] & opcode[1] & ~opcode[0];
	
	assign branchBLT = branchLessResult & ~opcode[4] & ~opcode[3] & opcode[2] & opcode[1] & ~opcode[0];
	
	assign branchX = currentStatus & opcode[4] & ~opcode[3] & opcode[2] & opcode[1] & ~opcode[0];
	
	
	assign shouldBranch = branchBNE | branchBLT | branchX;
	
endmodule

module irFetchDecode ( clock, pcIn, reset, instructionIn, pcOut, instructionOut, nop);
	input clock, nop, reset;
	input [31:0] pcIn, instructionIn;
	output [31:0] pcOut, instructionOut;
	
	wire [31:0] nop_pc;
	assign nop_pc = 32'b00000000000000000000000000000000;
	wire [31:0] nop_instruction;
	assign nop_instruction = 32'b00000000000000000000000000000000;
	
	wire [31:0] pc_in_reg;
	assign pc_in_reg = nop ? nop_pc : pcIn; ///because NOP Is assuming active low 
	
	wire [31:0] instruction_in_reg;
	assign instruction_in_reg = nop ? nop_instruction : instructionIn;
	
	register pcRegFD (.writeIn(pc_in_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(pcOut));
	
	register instructionRegFD (.writeIn(instruction_in_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(instructionOut));
	
	
endmodule


module irDecodeExecute( clock, nop, reset, pcIn, rdIn, rsIn, rtIn, rxIn, rsValueIn, rtrdValueIn, shiftamtIn, aluopIn, imedIn, targetIn, opcodeIn, pcOut, rdOut, rsOut, rtOut, rxOut, rsValueOut, rtrdValueOut, shiftamtOut, aluopOut, imedOut, targetOut, opcodeOut);

	input clock, nop, reset;
	input [4:0] rdIn, rsIn, rtIn, rxIn, shiftamtIn, aluopIn, opcodeIn;
	wire [4:0] rd_reg, rs_reg, rt_reg, rx_reg, shift_reg, aluop_reg, opcode_reg;
	input [16:0] imedIn;
	wire [16:0] imed_reg;
	input [26:0] targetIn;
	wire [26:0] target_reg;
	input [31:0] pcIn, rsValueIn, rtrdValueIn;
	wire [31:0] pc_reg, rs_value_reg, rtrd_value_reg;
	
	output [4:0] rdOut, rsOut, rtOut, rxOut, shiftamtOut, aluopOut, opcodeOut;
	output [16:0] imedOut;
	output [26:0] targetOut;
	output [31:0] pcOut, rsValueOut, rtrdValueOut;
	
	wire [4:0] nop_five;
	wire [16:0] nop_imed;
	wire [26:0] nop_target;
	wire [31:0] nop_thirtytwo;
	assign nop_five = 5'b00000;
	assign nop_imed = 17'b00000000000000000;
	assign nop_target = 27'b000000000000000000000000000;
	assign nop_thirtytwo = 32'b00000000000000000000000000000000;
	
	assign rd_reg = nop ? nop_five : rdIn;
	registerFive rdRegDX (.writeIn(rd_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(rdOut));
	
	assign rs_reg = nop ? nop_five : rsIn;
	registerFive rsRegDX (.writeIn(rs_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(rsOut));
	
	assign rt_reg = nop ? nop_five : rtIn;
	registerFive rtRegDX (.writeIn(rt_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(rtOut));
	
	assign rx_reg = nop ? nop_five : rxIn;
	registerFive rxRegDX (.writeIn(rx_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(rxOut));
	
	assign shift_reg = nop ? nop_five : shiftamtIn;
	registerFive shiftRegDX (.writeIn(shift_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(shiftamtOut));
	
	assign aluop_reg = nop ? nop_five : aluopIn;
	registerFive aluopRegDX (.writeIn(aluop_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(aluopOut));
			
	assign opcode_reg = nop ? nop_five : opcodeIn;
	registerFive opcodeRegDX (.writeIn(opcode_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(opcodeOut));
				
	assign imed_reg = nop ? nop_imed : imedIn;
	registerSeventeen imedRegDX (.writeIn(imed_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(imedOut));
		
	assign target_reg = nop ? nop_target : targetIn;
	registerTwentySeven targetRegDX (.writeIn(target_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(targetOut));

	assign pc_reg = nop ? nop_thirtytwo : pcIn;					
	register pcRegDX (.writeIn(pc_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(pcOut));
	
	assign rs_value_reg = nop ? nop_thirtytwo : rsValueIn;
	register rsValueRegDX (.writeIn(rs_value_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(rsValueOut));
		
	assign rtrd_value_reg = nop ? nop_thirtytwo : rtrdValueIn;
	register rtrdValueRegDX (.writeIn(rtrd_value_reg), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(rtrdValueOut));

endmodule
 
module signExtender(inputImed, outputImed);
	input [16:0] inputImed;
	output [31:0] outputImed;
	wire [31:0] positive, negative;
	
	assign positive = {15'b000000000000000, inputImed};
	assign negative = {15'b111111111111111, inputImed};
	
	wire sign;
	assign sign = inputImed[16];
	
	assign outputImed = sign ? negative : positive;


endmodule

module checkJumpInstruction ( opcode , out);
	input [4:0] opcode;
	output out;
	
	wire j, jal, jr;
	
	isJump myj (.in(opcode), .out(j));
	isJumpAndLink myjal (.in(opcode), .out(jal));
	isJumpReturn myjr (.in(opcode), .out(jr));
	
	assign out = j | jal | jr;

endmodule

module isJump (in , out);
	input [4:0] in;
	output out;
	
	assign out = ~in[4] & ~in[3] & ~in[2] & ~in[1] & in[0];

endmodule

module isJumpAndLink (in , out);
	input [4:0] in;
	output out;
	
	assign out = ~in[4] & ~in[3] & ~in[2] & in[1] & in[0];
	
endmodule

module isJumpReturn (in , out);
	input [4:0] in;
	output out;
	
	assign out = ~in[4] & ~in[3] & in[2] & ~in[1] & ~in[0];

endmodule

module irExecuteMemory (clock, reset, pcIn, rdIn, rsIn, rtIn, opcodeIn, alu_outputIn, rtrd_valueIn, pcOut, rdOut, rsOut, rtOut, opcodeOut, alu_outputOut, rtrd_valueOut);
	input clock, reset;
	input [4:0] rdIn, rsIn, rtIn, opcodeIn;
	input [31:0] pcIn, alu_outputIn, rtrd_valueIn;
	
	output [4:0] rdOut, rsOut, rtOut, opcodeOut;
	output [31:0] pcOut, alu_outputOut, rtrd_valueOut;
	
	registerFive rdRegXM (.writeIn(rdIn), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(rdOut));
	
	registerFive rsRegXM (.writeIn(rsIn), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(rsOut));
	
	registerFive rtRegXM (.writeIn(rtIn), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(rtOut));
	
	registerFive opcodeRegXM (.writeIn(opcodeIn), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(opcodeOut));
	
	register pcRegXM (.writeIn(pcIn), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(pcOut));
	
	register alu_outputRegXM (.writeIn(alu_outputIn), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(alu_outputOut));
	
	register rtrd_valueRegXM (.writeIn(rtrd_valueIn), .clock(clock), .writeEnable(1'b1), .reset(~reset), .readOut(rtrd_valueOut));
	
endmodule

module irMemoryWriteback (clock, reset, pcIn, rdIn, rsIn, rtIn, opcodeIn, alu_outputIn, mem_outputIn, pcOut, rdOut, rsOut, rtOut, opcodeOut, alu_outputOut, mem_outputOut);
	input clock, reset;
	input [4:0] rdIn, rsIn, rtIn, opcodeIn;
	input [31:0] pcIn, alu_outputIn, mem_outputIn;
	
	output [4:0] rdOut, rsOut, rtOut, opcodeOut;
	output [31:0] pcOut, alu_outputOut, mem_outputOut;
	
	wire one;
	assign one = 1'b1;
	
	
	register pcRegMW (.writeIn(pcIn), .clock(clock), .writeEnable(one), .reset(one), .readOut(pcOut));
	
	registerFive rdRegMW (.writeIn(rdIn), .clock(clock), .writeEnable(one), .reset(one), .readOut(rdOut));
	
	registerFive rsRegMW (.writeIn(rsIn), .clock(clock), .writeEnable(one), .reset(one), .readOut(rsOut));
	
	registerFive rtRegMW (.writeIn(rtIn), .clock(clock), .writeEnable(one), .reset(one), .readOut(rtOut));
	
	registerFive opcodeRegMW (.writeIn(opcodeIn), .clock(clock), .writeEnable(one), .reset(one), .readOut(opcodeOut));
	
	register alu_outputRegMW (.writeIn(alu_outputIn), .clock(clock), .writeEnable(one), .reset(one), .readOut(alu_outputOut));
	
	register mem_outputRegMW (.writeIn(mem_outputIn), .clock(clock), .writeEnable(one), .reset(one), .readOut(mem_outputOut));


endmodule

module nopMod(clock, reset, flushSignal, stallSignal, assertNOP);
	input flushSignal, clock, stallSignal, reset;
	output assertNOP;
	
	wire notted;
	assign notted = flushSignal | stallSignal;
	
	registerOne nopReg (.writeIn(notted), .clock(~clock), .writeEnable(1'b1), .reset(~reset), .readOut(assertNOP));
	
endmodule

module xmBypassLogic (rd_xm, rd_mw, opcode_xm, xm_bypass_select);
	input [4:0] rd_xm, rd_mw, opcode_xm;
	output xm_bypass_select;
	
	wire rd_equal;
	checkEqualStall xmbypasscheck (.inputA(rd_xm), .inputB(rd_mw), .out(rd_equal));
	
	wire save_memory;
	//redudant and could cause problems but fuck it? let's go!!!
	assign save_memory = ~opcode_xm[4] & ~opcode_xm[3] & opcode_xm[2] & opcode_xm[1] & opcode_xm[0];
	
	assign xm_bypass_select = rd_equal & save_memory;

endmodule

# Wall-EProcessor
This project is the Verilog HDL processor that drove an Wall-E looking obstacle avoiding robot.  This 32-bit processor ran on an Altera FPGA and had several custom instructions to intake data the data necessary for navigation and output the necessary data to control servo motors.  

The main file that ran on the FPGA was skeleton.v, which contains the logic for a register file, an ALU, a multiplier and divider unit, and other custom instructions.  

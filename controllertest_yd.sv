module testbench();

logic clk, reset;//Input logic
logic[31:12] Instr;
logic[3:0] ALUFlags; 
logic PCWrite,MemWrite,RegWrite,IRWrite,AdrSrc; //Output logic
logic[1:0] RegSrc,ALUSrcA, ALUSrcB,ResultSrc,ImmSrc,ALUControl;

controller control(.clk, .reset, .Instr, .ALUFlags, .PCWrite, .MemWrite,
                   .RegWrite, .IRWrite, .AdrSrc, .RegSrc, .ALUSrcA, .ALUSrcB,
                   .ResultSrc, .ImmSrc, .ALUControl);
									
									
initial 
    forever begin
	       clk = 0;#20; clk = 1; #20;
	 end
	 
initial 
    begin
	 reset = 1; 
	 #10;
	 //ADD R5,R6,(R7)
	 reset = 0;
	 ALUFlags = 4'b0000;
	 Instr = 20'b1110_0000_1000_0110_0101;
	 #160;
	 
	 //ADD R0, R1,(#42)
	 Instr = 20'b1110_0010_1000_0001_0000;
	 #160;
	 
	 //SUB R8,R9,(R10)
	 Instr = 20'b1110_0000_0100_1001_1000;
	 #160;
	 
	 //SUB R2,R3,(#0xFF0)
	 Instr = 20'b1110_0010_0100_0011_0010;
	 #160;
	 
	 //AND R3,R1,(R2)
	 Instr = 20'b1110_0000_0000_0001_0011;
	 #160; 
	 
	 //AND R3,R1,(#4)
    Instr = 20'b1110_0010_0000_0001_0011;
	 #160;
	 
	 //ORR R3,R1,(R2)
	 Instr = 20'b1110_0001_1000_0001_0011;
	 #160;
	 
	 //ORR R3,R1,(#4)
	 Instr = 20'b1110_0011_1000_0001_0011;
	 #160;
	 
	 //LDR R3,(#4)
	 Instr = 20'b1110_0100_1011_0001_0011;
	 #200;
	 
	 //STR R3,(#4)
	 Instr = 20'b1110_0100_1010_0001_0011;
	 #160;
	 
	 //BNE PC+20 Branch Taken
	 ALUFlags = 4'b0000;
	 Instr = 20'b0001_1010_0000_0000_0000;
	 #80;
	 
	 //BNE PC+20 Branch Not Taken
	 ALUFlags = 4'b0100;
	 Instr = 20'b0001_1010_0000_0000_0000;
	 #120;
	
    end

endmodule
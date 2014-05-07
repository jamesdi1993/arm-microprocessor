module alu(input logic [31:0] a,b,
           input logic [1:0] ALUControl,
			  output logic [31:0] Result,
			  output logic [3:0] Flags);
		  
logic [31:0] c;
always_comb
 if(ALUControl[0]) c = ~b;
 else              c = b;

always_comb

  case(ALUControl[1])
   1'b1: if(ALUControl[0]) Result = a | b;
	      else              Result = a & b;
   1'b0: Result = a + c + ALUControl;
  endcase
  
assign Flags[0] = ((~a[31]) &(~b[31]) & Result[31])|((a[31]) &(b[31]) & (~Result[31]));
assign Flags[2] = (Result ==0);
assign Flags[3] = (Result[31] == 1);

always_comb
  if(ALUControl[1]) Flags[1] = 0;
  else              Flags[1] =  ((a[31]& c[31])|((a[31]^c[31])&(~Result[31])));

endmodule
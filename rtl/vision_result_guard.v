// Separate from raw VIS validity. Only real qualified frames refresh this timer.
module vision_result_guard #(parameter integer MS_CYCLES=50000,LOSS_GRACE_MS=500)(
 input wire clk,rst_n,clear,frame,valid,
 output wire hold_valid
);
 localparam integer LIMIT=MS_CYCLES*LOSS_GRACE_MS;
 reg seen=0;reg [31:0] elapsed=0;
 assign hold_valid=rst_n && !clear && seen && elapsed<LIMIT;
 always @(posedge clk)begin
  if(!rst_n || clear)begin seen<=0;elapsed<=0;end
  else if(frame && valid)begin seen<=1;elapsed<=0;end
  else if(seen && elapsed<LIMIT)elapsed<=elapsed+1'b1;
 end
endmodule

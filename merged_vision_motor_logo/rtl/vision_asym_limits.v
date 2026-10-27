// One mechanical reference per load. Later K3 changes the control center, never
// moves this envelope. Counts use4096/rev; MC uses3200pulses/rev.
module vision_asym_limits(
 input wire clk,rst_n,capture,ready,motor_en,input wire signed [31:0] position,command_position,
 output reg qualified=0,output reg signed [31:0] anchor=0,command_anchor=0,
 output wire signed [31:0] work_min,work_max,feedback_min,feedback_max,command_min,command_max
);
 assign work_min=qualified ? anchor-182 : -64;
 assign work_max=qualified ? anchor+364 : 64;
 assign feedback_min=qualified ? anchor-204 : -64;
 assign feedback_max=qualified ? anchor+398 : 64;
 assign command_min=qualified ? command_anchor-159 : -64;
 assign command_max=qualified ? command_anchor+310 : 64;
 always @(posedge clk)begin
  if(!rst_n)begin qualified<=0;anchor<=0;command_anchor<=0;end
  else if(capture && ready && motor_en && !qualified && position>=-64 && position<=64)begin
   anchor<=position;command_anchor<=command_position;qualified<=1;
  end
 end
endmodule

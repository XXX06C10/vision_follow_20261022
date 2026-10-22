// Debounced K3 edge, release-qualified after reset. A held key never replays.
module vision_center_key #(parameter integer DEBOUNCE_CYCLES=1000000)(
 input wire clk,rst_n,key_n,output reg press_event=0
);
 reg sync1=0,sync2=0,candidate=0,stable_pressed=0,stable_valid=0;
 reg previous_pressed=0,armed=0;reg [31:0] debounce_count=0;
 always @(posedge clk)begin
  if(!rst_n)begin
   sync1<=0;sync2<=0;candidate<=0;stable_pressed<=0;stable_valid<=0;
   previous_pressed<=0;armed<=0;debounce_count<=0;press_event<=0;
  end else begin
   sync1<=~key_n;sync2<=sync1;press_event<=0;
   if(sync2!=candidate)begin candidate<=sync2;debounce_count<=0;stable_valid<=0;end
   else if(debounce_count<DEBOUNCE_CYCLES-1)debounce_count<=debounce_count+1'b1;
   else begin stable_pressed<=candidate;stable_valid<=1;end
   if(stable_valid)begin
    previous_pressed<=stable_pressed;
    if(!stable_pressed)armed<=1;
    else if(!previous_pressed)begin
     if(armed)press_event<=1;
     armed<=0;
    end
   end
  end
 end
endmodule

// Coalescing configuration mailbox: transfer once per changed value, keep bus held until ack.
module vision_value_cdc #(parameter WIDTH=48)(
 input wire src_clk,src_rst_n,input wire [WIDTH-1:0] src_value,
 input wire dst_clk,dst_rst_n,output reg [WIDTH-1:0] dst_value=0,output reg dst_valid=0
);
 reg [WIDTH-1:0] held=0,last_sent=0;
 reg initialized=0,request=0,ack=0,seen=0;
 reg [1:0] ack_sync=0,req_sync=0;
 always @(posedge src_clk or negedge src_rst_n)begin
  if(!src_rst_n)begin held<=0;last_sent<=0;initialized<=0;request<=0;ack_sync<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(request==ack_sync[1] && (!initialized || last_sent!=src_value))begin
    held<=src_value;last_sent<=src_value;initialized<=1;request<=~request;
   end
  end
 end
 always @(posedge dst_clk or negedge dst_rst_n)begin
  if(!dst_rst_n)begin req_sync<=0;seen<=0;ack<=0;dst_value<=0;dst_valid<=0;end
  else begin
   req_sync<={req_sync[0],request};
   if(req_sync[1]!=seen)begin dst_value<=held;dst_valid<=1;seen<=req_sync[1];ack<=req_sync[1];end
  end
 end
endmodule

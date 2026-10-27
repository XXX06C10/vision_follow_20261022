// Additive C01 + fixed25-byte payload + XOR8 as hex + LF (54 bytes).
// P/V/S remain owned by the unchanged legacy receiver. Only line-start C accepted.
module vision_cascade_config #(parameter UART_DIV=434,TIMEOUT_CYCLES=5000000,DEFAULT_SELECTION=0,DEFAULT_PRESET=0)(
 input wire clk,rst_n,rx,
 output reg applied=0,output wire commit_now,output reg [15:0] config_seq=0,errors=0,
 output reg [7:0] selection=DEFAULT_SELECTION,output reg diagnostics=DEFAULT_PRESET,polarity=0,
 output reg [15:0] kp=(DEFAULT_PRESET ? 819 : (DEFAULT_SELECTION==0 ? 512 : 0)),kv=(DEFAULT_PRESET ? 64 : (DEFAULT_SELECTION==0 ? 128 : 0)),ki=(DEFAULT_PRESET ? 131 : 0),inner_kp=8192,feedforward=200,max_hz=(DEFAULT_PRESET ? 200 : 50),slew_rate=(DEFAULT_PRESET ? 200 : 80),
 output reg signed [15:0] bias=0,
 output reg [7:0] delay_ms=(DEFAULT_PRESET ? 20 : 0),amplitude=4
);
 reg s1=1,s2=1;reg [1:0] state=0;reg [15:0] baud=0;reg [2:0] bitno=0;
 reg [7:0] shift=0,value=0;reg received=0;
 always @(posedge clk)begin
  if(!rst_n)begin s1<=1;s2<=1;state<=0;baud<=0;bitno<=0;shift<=0;value<=0;received<=0;end
  else begin
   s1<=rx;s2<=s1;received<=0;
   case(state)
    0:if(!s2)begin baud<=UART_DIV/2-1;state<=1;end
    1:if(baud==0)begin if(!s2)begin state<=2;baud<=UART_DIV-1;end else state<=0;end else baud<=baud-1'b1;
    2:if(baud==0)begin shift[bitno]<=s2;baud<=UART_DIV-1;if(bitno==7)begin bitno<=0;state<=3;end else bitno<=bitno+1'b1;end else baud<=baud-1'b1;
    3:if(baud==0)begin state<=0;if(s2)begin value<=shift;received<=1;end end else baud<=baud-1'b1;
   endcase
  end
 end
 function [4:0] hex;
  input [7:0] c;
  begin if(c>="0" && c<="9")hex={1'b1,c[3:0]};else if(c>="A" && c<="F")hex={1'b1,c[3:0]+4'd9};else if(c>="a" && c<="f")hex={1'b1,c[3:0]+4'd9};else hex=0;end
 endfunction
 reg line_start=1,collecting=0,discard=0;reg [5:0] digits=0;reg [207:0] packet=0;reg [31:0] age=0;
 wire [4:0] digit=hex(value);integer j;reg [7:0] checksum;
 always @*begin checksum=0;for(j=1;j<26;j=j+1)checksum=checksum^packet[j*8+:8];end
 wire signed [15:0] packet_bias=packet[119:104];
 wire good=digits==52 && packet[207:200]==1 && packet[183:176]<=3 && packet[175:168]<=1 &&
 packet[167:152]<=16384 && packet[151:136]<=2048 && packet[135:120]<=3277 &&
 packet_bias>=-2048 && packet_bias<=2048 && packet[103:88]>=256 && packet[103:88]<=32768 &&
 packet[87:72]<=256 && (packet[71:56]==50 || packet[71:56]==100 || packet[71:56]==200) &&
 packet[55:48]<=120 && (packet[47:40]==4 || packet[47:40]==8 || (packet[47:40]==16 && packet[183:176]!=3)) &&
 packet[39:24]>=8 && packet[39:24]<=256 && packet[23:16]<=1 && packet[15:8]==0 && checksum==packet[7:0];
 assign commit_now=received && value==10 && collecting && age<TIMEOUT_CYCLES-1 && good;
 always @(posedge clk)begin
  if(!rst_n)begin applied<=0;config_seq<=0;errors<=0;selection<=DEFAULT_SELECTION;diagnostics<=DEFAULT_PRESET;polarity<=0;
   kp<=DEFAULT_PRESET ? 819 : (DEFAULT_SELECTION==0 ? 512 : 0);kv<=DEFAULT_PRESET ? 64 : (DEFAULT_SELECTION==0 ? 128 : 0);ki<=DEFAULT_PRESET ? 131 : 0;bias<=0;inner_kp<=8192;feedforward<=200;max_hz<=DEFAULT_PRESET ? 200 : 50;slew_rate<=DEFAULT_PRESET ? 200 : 80;delay_ms<=DEFAULT_PRESET ? 20 : 0;amplitude<=4;
   line_start<=1;collecting<=0;discard<=0;digits<=0;packet<=0;age<=0;
  end else begin
   applied<=0;
   if(collecting)begin
    if(age>=TIMEOUT_CYCLES-1)begin collecting<=0;discard<=1;age<=0;if(errors!=65535)errors<=errors+1'b1;end
    else age<=age+1'b1;
   end
   if(received)begin
    if(value==10)begin
     line_start<=1;discard<=0;
     if(collecting && age<TIMEOUT_CYCLES-1)begin
      collecting<=0;age<=0;
      if(good)begin applied<=1;config_seq<=packet[199:184];selection<=packet[183:176];diagnostics<=packet[168];
       kp<=packet[167:152];kv<=packet[151:136];ki<=packet[135:120];bias<=packet_bias;inner_kp<=packet[103:88];
       feedforward<=packet[87:72];max_hz<=packet[71:56];delay_ms<=packet[55:48];amplitude<=packet[47:40];slew_rate<=packet[39:24];polarity<=packet[16];
      end else if(errors!=65535)errors<=errors+1'b1;
     end
    end else if(line_start)begin
     line_start<=0;
     if(value=="C")begin collecting<=1;discard<=0;digits<=0;packet<=0;age<=0;end
    end else if(collecting && !discard && age<TIMEOUT_CYCLES-1)begin
     if(digit[4] && digits<52)begin packet<={packet[203:0],digit[3:0]};digits<=digits+1'b1;end
     else begin collecting<=0;discard<=1;if(errors!=65535)errors<=errors+1'b1;end
    end
   end
  end
 end
endmodule

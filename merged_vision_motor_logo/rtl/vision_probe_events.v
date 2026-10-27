// E1:26 payload bytes + XOR8, hex, CRLF =58 wire bytes. FIFO overflow is sticky.
// Sequence increments on every capture attempt, including dropped events.
module vision_probe_events #(parameter UART_DIV=434,MS_CYCLES=50000)(
 input wire clk,rst_n,enabled,grant,input wire [31:0] time_ms,
 input wire step,dir,input wire signed [31:0] position,command_position,
 input wire [3:0] state,input wire [7:0] pending,input wire signed [15:0] commands,delta,
 input wire [7:0] diagnostic_flags,input wire [3:0] phase,status,
 input wire [7:0] selection,input wire [3:0] reason,input wire changed,
 output wire tx,output reg busy=0,output wire wants,output reg overflow=0
);
 reg [207:0] memory[0:127];
 reg [7:0] wr=0,rd=0;reg [15:0] event_seq=0;
 reg old_step=0,old_dir=0;reg [31:0] old_position=0;
 reg [3:0] old_state=0,old_phase=0,old_status=0,old_reason=0;
 reg [7:0] old_pending=0,old_diag=0,old_selection=0;
 reg [31:0] heartbeat=0;
 wire [7:0] events={dir!=old_dir,changed,time_ms-heartbeat>=100,
     pending!=old_pending || diagnostic_flags!=old_diag,
     phase!=old_phase || status!=old_status || reason!=old_reason || selection!=old_selection,
     state!=old_state,position!=old_position,step && !old_step};
 wire capture=enabled && events!=0;
 wire full=wr[7]!=rd[7] && wr[6:0]==rd[6:0];
 assign wants=enabled && wr!=rd;
 wire [207:0] payload={event_seq,time_ms,events,position,command_position,{4'd0,state},pending,commands,delta,diagnostic_flags,{4'd0,phase},{4'd0,status},selection,{4'd0,reason}};
 reg [207:0] snapshot=0;reg [7:0] checksum=0;
 reg loading=0,active=0;reg [5:0] index=0;reg [3:0] bit_index=0;
 reg [15:0] baud=0;reg [9:0] shift=1023;
 assign tx=shift[0];
 integer j;reg [7:0] folded;
 always @*begin folded=0;for(j=0;j<26;j=j+1)folded=folded^snapshot[j*8+:8];end
 function [7:0] hex_char;input [3:0] n;begin hex_char=n<10 ? "0"+n : "A"+n-10;end endfunction
 function [7:0] byte_at;input [5:0] at;begin
  if(at==0)byte_at="E";else if(at==1)byte_at="1";
  else if(at<54)byte_at=hex_char(snapshot[207-(at-2)*4 -:4]);
  else if(at==54)byte_at=hex_char(checksum[7:4]);else if(at==55)byte_at=hex_char(checksum[3:0]);
  else if(at==56)byte_at=13;else byte_at=10;
 end endfunction
 always @(posedge clk)begin
  if(!rst_n)begin wr<=0;rd<=0;event_seq<=0;overflow<=0;old_step<=0;old_dir<=0;old_position<=0;old_state<=0;old_phase<=0;old_status<=0;old_reason<=0;old_pending<=0;old_diag<=0;old_selection<=0;heartbeat<=0;busy<=0;loading<=0;active<=0;index<=0;bit_index<=0;baud<=0;shift<=1023;snapshot<=0;checksum<=0;end
  else begin
   old_step<=step;old_dir<=dir;old_position<=position;old_state<=state;old_phase<=phase;old_status<=status;old_reason<=reason;old_pending<=pending;old_diag<=diagnostic_flags;old_selection<=selection;
   if(capture)begin
    event_seq<=event_seq+1'b1;heartbeat<=time_ms;
    if(full)overflow<=1;
    else begin memory[wr[6:0]]<=payload;wr<=wr+1'b1;end
   end
   // Registered read, separate write/read addresses: block RAM implementation.
   if(!busy && wants && grant)begin snapshot<=memory[rd[6:0]];rd<=rd+1'b1;busy<=1;loading<=1;index<=0;end
   else if(loading)begin checksum<=folded;loading<=0;end
   else if(busy)begin
    if(!active)begin shift<={1'b1,byte_at(index),1'b0};baud<=0;bit_index<=0;active<=1;end
    else if(baud==UART_DIV-1)begin
     baud<=0;shift<={1'b1,shift[9:1]};
     if(bit_index==9)begin active<=0;if(index==57)busy<=0;else index<=index+1'b1;end
     else bit_index<=bit_index+1'b1;
    end else baud<=baud+1'b1;
   end
  end
 end
endmodule


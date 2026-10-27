// Dedicated finite measurement image. C selection 2=static,3=neutral,1=response.
// Configuration arms a profile only; every start needs a physical K2 edge.
module vision_region_probe #(parameter MS_CYCLES=50000)(
 input wire clk,rst_n,key_event,k4_n,enable_request,settings_ready,valid,stale,frame,changed,
 input wire center_confirmed,neutral_ready,core_ready,core_busy,event_overflow,
 input wire [7:0] selection,input wire signed [7:0] neutral,
 input wire signed [31:0] position,input wire [3:0] fault,
 input wire [31:0] time_ms,input wire [10:0] x,y,
 input wire [15:0] center,roi_left,roi_right,roi_top,roi_bottom,
 output reg run=0,request=0,output wire cancel,
 output reg [3:0] status=0,phase=0,reason=0,reject=0,
 output reg signed [7:0] target=0
);
 reg [31:0] mc=0;reg [13:0] age_ms=0;reg [9:0] band_ms=0,hold_ms=0;
 reg issued=0,holding=0,previous_run=0,velocity_ready=0;
 reg [31:0] previous_frame=0;reg [10:0] previous_x=0;
 wire tick=mc==MS_CYCLES-1;
 wire static_profile=selection==2;
 wire response_profile=selection==1;
 wire signed [16:0] pe=$signed(position)-$signed(target);
 wire in_band=pe<=2 && pe>=-2;
 wire signed [16:0] xe=$signed({1'b0,x})-$signed({1'b0,center});
 wire signed [12:0] dx=$signed({1'b0,x})-$signed({1'b0,previous_x});
 wire [12:0] abs_dx=dx<0 ? -dx : dx;
 wire [31:0] dt=time_ms-previous_frame;
 wire excessive_speed=frame && velocity_ready && dt>0 && abs_dx*32'd1000>dt*32'd150;
 // Ball telemetry is1920x1080; ROI commands are1280x720 sensor pixels.
 wire [16:0] x2={6'd0,x}<<1,y2={6'd0,y}<<1;
 wire [16:0] rl3=roi_left*17'd3,rr3=roi_right*17'd3,rt3=roi_top*17'd3,rb3=roi_bottom*17'd3;
 wire near_edge=x2<=rl3+48 || x2+48>=rr3 || y2<=rt3+48 || y2+48>=rb3;
 wire ball_guard=!static_profile && (!valid || stale || near_edge || xe>120 || xe< -120 || excessive_speed);
 wire abort_now=run && (!k4_n || (!static_profile && !enable_request) || fault!=0 || changed || event_overflow || ball_guard || !settings_ready || key_event);
 assign cancel=(previous_run && !run) || abort_now;
 // Twelve legs: +/0/-/0, -/0/+/0, +/0/-/0. Each new leg has its own finite request.
 function signed [7:0] leg_target;
  input [3:0] leg;input signed [7:0] n;
  begin case(leg)
   0,6,8:leg_target=n+8;
   2,4,10:leg_target=n-8;
   default:leg_target=n;
  endcase end
 endfunction
 always @(posedge clk)begin
  if(!rst_n)begin run<=0;request<=0;status<=0;phase<=0;reason<=0;reject<=0;target<=0;mc<=0;age_ms<=0;band_ms<=0;hold_ms<=0;issued<=0;holding<=0;previous_run<=0;velocity_ready<=0;previous_frame<=0;previous_x<=0;end
  else begin
   request<=0;previous_run<=run;
   if(frame)begin previous_frame<=time_ms;previous_x<=x;velocity_ready<=valid && !stale;end
   if(tick)begin
    mc<=0;
    if(run)begin
     if(age_ms<16383)age_ms<=age_ms+1'b1;
     if(!response_profile)begin
      if(age_ms>=9999)begin run<=0;status<=2;reason<=10;end
     end else if(!holding)begin
      if(issued && core_ready && !core_busy && in_band)begin if(band_ms<100)band_ms<=band_ms+1'b1;end else band_ms<=0;
      if(band_ms>=99 && issued && core_ready && in_band)begin holding<=1;hold_ms<=0;end
      else if(age_ms>=999)begin run<=0;status<=4;reason<=11;end
     end else begin
      if(!in_band || !core_ready)begin run<=0;status<=4;reason<=12;end
      else if(hold_ms>=(phase[0] ? 599 : 399))begin
       if(phase==11)begin run<=0;status<=2;reason<=10;end
       else begin phase<=phase+1'b1;target<=leg_target(phase+1'b1,neutral);issued<=0;holding<=0;age_ms<=0;band_ms<=0;hold_ms<=0;end
      end else hold_ms<=hold_ms+1'b1;
     end
    end
   end else mc<=mc+1'b1;
   if(run && response_profile && !issued && core_ready && !abort_now)begin request<=1;issued<=1;end
   if(key_event && !run && !changed)begin
    reject<=0;
    if(selection==0 || selection>3)reject<=12;
    else if(fault!=0)reject<=9;
    else if(event_overflow)reject<=14;
    else if(!static_profile && (!enable_request || !core_ready))reject<=1;
    else if(!settings_ready)reject<=5;
    else if(!static_profile && (!center_confirmed || !neutral_ready))reject<=10;
    else if(!static_profile && ball_guard)reject<=3;
    else if(response_profile && (neutral>52 || neutral< -52 || position>60 || position< -60))reject<=13;
    else begin run<=1;status<=1;phase<=0;reason<=0;age_ms<=0;band_ms<=0;hold_ms<=0;issued<=0;holding<=0;target<=response_profile ? leg_target(0,neutral) : position[7:0];end
   end
   if(abort_now)begin
    run<=0;request<=0;status<=4;
    if(fault!=0)reason<=6;else if(!k4_n)reason<=2;else if(!enable_request)reason<=1;
    else if(changed || !settings_ready)reason<=5;else if(event_overflow)reason<=14;
    else if(key_event)reason<=7;else if(stale)reason<=4;else if(!valid)reason<=3;
    else if(near_edge)reason<=13;else if(excessive_speed)reason<=15;else reason<=9;
   end
   // A fresh configuration/reference never inherits a previous DONE.
   if(changed)begin run<=0;request<=0;status<=0;phase<=0;reason<=5;reject<=0;issued<=0;holding<=0;age_ms<=0;band_ms<=0;hold_ms<=0;velocity_ready<=0;end
  end
 end
endmodule

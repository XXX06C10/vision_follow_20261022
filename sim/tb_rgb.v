`timescale 1ns/1ps
// Real packed RGB -> six pixel classifier -> actual bbox -> coherent CDC -> AUTO.
module tb_rgb;
 reg clk=0;always #5 clk=~clk;reg dst=0;always #7 dst=~dst;
 reg rst=0,valid=0,sof=0,last=0;reg [127:0] data=0;
 reg [10:0] roi_right=1279;wire [5:0] mask;wire [4:0] meta;
 wire done,ball,bad,track;wire [10:0] x,y,track_x,track_y;wire [3:0] quality;wire [1:0] error;
 wire [127:0] video;wire vv,vs,vl;
 steel_ball_detect #(.IMG_WIDTH(1280),.IMG_HEIGHT(720),.COLOR_MODE(1),.COLOR_MIN(45),.COLOR_RATIO_LOG2(1),
 .TRACK_EN(1),.TRACK_COLOR_MIN(35),.TRACK_RADIUS(48),.OVERLAY_EN(1),.EXTERNAL_BOX(1),.LABEL_BITS(6),.FIFO_BITS(12)) pixels(
 .I_clk(clk),.I_rst_n(rst),.S_AXIS_tdata(data),.S_AXIS_tvalid(valid),.S_AXIS_tuser(sof),.S_AXIS_tlast(last),
 .I_box_valid(1'b0),.I_box_left(11'd0),.I_box_right(11'd0),.I_box_top(11'd0),.I_box_bottom(11'd0),.I_box_confirmed(1'b0),
 .I_roi_left(11'd0),.I_roi_top(11'd0),.I_roi_right(roi_right),.I_roi_bottom(11'd719),
 .I_track_valid(track),.I_track_x(track_x),.I_track_y(track_y),.O_color_mask(mask),.O_color_meta(meta),
 .M_AXIS_tdata(video),.M_AXIS_tvalid(vv),.M_AXIS_tuser(vs),.M_AXIS_tlast(vl));
 steel_ball_stream_bbox #(.IMG_WIDTH(1280),.IMG_HEIGHT(720),.MIN_AREA(8),.BRIGHT_MIN(20),.MAX_AREA(90000),
 .CONFIRM_FRAMES(2),.DOMINANCE_PERCENT(40),.ASPECT_PERCENT(35),.CLUSTER_RADIUS(8),.MAX_JUMP(147),.EDGE_MARGIN(2),.MAX_DIM(267),.ROI_EN(1)) boxes(
 .I_clk(clk),.I_rst_n(rst),.I_mask(mask),.I_meta(meta),.I_roi_left(11'd0),.I_roi_top(11'd0),.I_roi_right(roi_right),.I_roi_bottom(11'd719),
 .O_frame_done(done),.O_ball_detected(ball),.O_bad_frame(bad),.O_error_code(error),.O_ball_x(x),.O_ball_y(y),.O_confidence(quality),
 .O_track_valid(track),.O_track_x(track_x),.O_track_y(track_y));
 wire [10:0] dx=(x*3)>>1,dy=(y*3)>>1;wire [27:0] result;wire pulse;
 vision_cdc_bridge #(.WIDTH(28)) bridge(clk,rst,done,{dx,dy,ball,(bad || error!=0),quality},dst,rst,pulse,result);
 reg [3:0] keys=15;reg [1:0] enc=0,phase=0;wire en,dir,step,tx;wire [3:0] led;wire [15:0] cs,ae,ag,rl,rt,rr,rb;
 vision_monitor #(.MS_CYCLES(6000),.DEBOUNCE_CYCLES(4),.ENABLE_CYCLES(16),.POR_CYCLES(8),.UART_DIV(4),
 .SETUP_CYCLES(20),.HALF_CYCLES(5),.SETTLE_CYCLES(100),.TIMEOUT_CYCLES(10000000)) controller(
 dst,rst,keys,enc,2'b0,pulse,result[27:17],result[16:6],result[5],result[4],result[3:0],1'b1,tx,en,dir,step,led,
 16'd0,16'd0,1'b0,cs,ae,ag,rl,rt,rr,rb);
 integer commands=0,encoded=0,goal=0,difference=0,n=0,pulses=0;
 always @(posedge step)begin
  pulses=pulses+1;commands=commands+(dir ? 1 : -1);goal=commands*32/25;difference=goal-encoded;
  for(n=0;n<(difference<0 ? -difference : difference);n=n+1)begin repeat(4)@(negedge dst);phase=phase+(difference>0 ? 2'd3 : 2'd1);case(phase)0:enc=0;1:enc=1;2:enc=3;3:enc=2;endcase end
  encoded=goal;
 end
 integer fr,yy,group_index,ph,p,px,cx,cy;reg [383:0] packed_rgb;reg [23:0] color;reg expect_ball;
 task tick(input integer cycles);begin repeat(cycles)@(negedge clk);end endtask
 task click(input integer key);begin keys[key]=0;repeat(20)@(negedge dst);keys[key]=1;repeat(30)@(negedge dst);end endtask
 function [23:0] pixel(input integer f,xx,zz);
  integer xx0,zz0;reg [23:0] v;begin
   xx0=f>=13 ? (f==15 ? 1000 : 900) : (f>=8 ? 640 : f>=5 ? 620 : 600);zz0=f>=13 ? 400 : 350;
   v=24'h101010;
   if(xx>=150 && xx<=180 && zz>=90 && zz<=120)v=24'hFFFFFF; // bright white is not a red ball
   if((xx==100 && zz==100) || (xx==1100 && zz==600))v=24'h641010; // sparse red noise, area<8
   if((xx-xx0)*(xx-xx0)+(zz-zz0)*(zz-zz0)<=25)begin
    case(f)
     0:v=24'h2C0A0A; // R44 below the global45 floor; no prior track
     1,2,13,14,16:v=24'h2D1414; // R45 is included; R>2G/B
     4:v=24'h241818; // 2R==3G/B, strictly excluded by weak ratio
     6:v=24'h220A0A; // R34 below weak35 floor
     7,9,10,11,17:v=24'h101010;
     default:v=24'h231414; // R35 weak, ratio1.75, fails old ratio2
    endcase
   end
   pixel=v;
  end
 endfunction
 initial begin
  tick(5);rst=1;tick(10);sof=1;tick(1);sof=0;tick(4);
  for(fr=0;fr<18;fr=fr+1)begin
   if(fr==16)roi_right=800;
   for(yy=0;yy<720;yy=yy+1)begin
    for(group_index=0;group_index<80;group_index=group_index+1)begin
     for(p=0;p<16;p=p+1)packed_rgb[383-p*24 -:24]=pixel(fr,group_index*16+p,yy);
     for(ph=0;ph<3;ph=ph+1)begin
      @(negedge clk);valid=1;data=packed_rgb[383-ph*128 -:128];last=(group_index==79 && ph==1);
     end
    end
    // Fr17 has a malformed first line: remove the final transfer before input sampling.
    if(fr==17 && yy==0)valid=0;
   end
   @(negedge clk);valid=0;last=0;tick(6);sof=1;tick(1);sof=0;
   wait(done);#1;
   expect_ball=(fr==2 || fr==3 || fr==5 || fr==8 || fr==14);
   if(ball!==expect_ball)$fatal(1,"RGB frame%0d ball=%b expected=%b Q=%h center=%d,%d track=%b miss=%d",fr,ball,expect_ball,quality,x,y,track,boxes.missed);
   if((fr==4 || fr==6 || fr==7 || fr==9 || fr==10) && !track)$fatal(1,"short miss lost track history");
   if(fr==11 && track)$fatal(1,"third miss failed to expire track");
   if(fr!=17 && (bad || error!=0))$fatal(1,"RGB stream packing/line integrity wrong frame%0d",fr);
   if(fr==17 && (!bad || error!=3))$fatal(1,"malformed frame not rejected");
   repeat(30)@(negedge dst);
   if(fr==2)begin click(2);tick(30);click(0);wait(controller.request_ready);click(1);if(!controller.auto_run)$fatal(1,"RGB AUTO start");end
   if(fr>=3 && fr<=8 && !controller.auto_run)$fatal(1,"real RGB short drop cancelled AUTO");
   if(controller.fault!=0)$fatal(1,"RGB model false actuator fault=%0d",controller.fault);
   $display("RGB_FRAME_PASS frame=%0d ball=%b track=%b Q=%h AR=%b P=%0d",fr,ball,track,quality,controller.auto_run,$signed(controller.position));
  end
  if(pulses==0 || controller.errors1!=0)$fatal(1,"RGB->AUTO real encoder feedback missing");
  $display("RGB_INTEGRATION_PASS: actual1280x720 RGB5/5/6,45/35 and ratio boundaries, local recovery1/2 and expiry3, white/dark/sparse noise, outside-neighborhood, ROI, bad-frame, CDC and1.28AB AUTO");$finish;
 end
 initial begin #100000000;$fatal(1,"RGB timeout");end
endmodule

`timescale 1ns/1ps
module retarget_guard #(parameter KIND=5)(output reg done=0);
 reg clk=0;always #5 clk=~clk;reg rst=0,request=0;
 reg signed [31:0] encoder=0;reg signed [7:0] target=60;
 wire en,dir,step,ready,busy;wire [3:0] fault;wire [7:0] spent;
 vision_servo_core #(.EXTERNAL_CONTROL(1),.ENABLE_CYCLES(8),.SETUP_CYCLES(20),.HALF_CYCLES(5),.SETTLE_CYCLES(20),.TIMEOUT_CYCLES(KIND==4 ? 2000 : 100000)) dut(
 .clk(clk),.rst_n(rst),.enable_request(1'b1),.k4_n(1'b1),.k2_n(1'b1),.request(request),.following(1'b1),.external_cancel(1'b0),.external_cancel_reason(4'd0),
 .encoder_count(encoder),.encoder_errors(16'd0),.visual_valid(1'b1),.visual_stale(1'b0),.settings_ready(1'b1),.config_changed(1'b0),.visual_target(target),
 .motor_en(en),.motor_dir(dir),.motor_step(step),.ready(ready),.busy(busy),.fault(fault),.spent(spent));
 always @(posedge step)begin encoder<=encoder+(dir ? -1 : 1);target<=dir ? -60 : 60;end
 always @(dir)if(step)$fatal(1,"DIR changed during high STEP");
 initial begin repeat(4)@(negedge clk);rst=1;wait(ready);@(negedge clk);request=1;@(negedge clk);request=0;
  wait(fault!=0);#1;if(fault!=KIND || en || step)$fatal(1,"retarget reset/bypassed burst guard %0d",KIND);
  if(KIND==5 && spent!=64)$fatal(1,"retarget bypassed64 pulse cap");
  $display("RETARGET_GUARD_PASS fault=%0d spent=%0d",fault,spent);done=1;
 end
endmodule
// Slow movement must survive velocity filtering; all VE values are Q4.
module pd_arithmetic(output reg done=0);
 reg clk=0;always #5 clk=~clk;
 reg rst=0,key=0,frame=0,valid=1,changed=0;reg [10:0] x=960;
 reg [7:0] gain=0;wire run;wire signed [15:0] velocity;wire signed [7:0] candidate;
 vision_follow_control #(.MS_CYCLES(1),.CONTROL_MS(1000)) dut(
 .clk(clk),.rst_n(rst),.key_event(key),.k4_n(1'b1),.enable_request(1'b1),.config_changed(changed),
 .visual_valid(valid),.follow_valid(valid),.visual_stale(1'b0),.settings_ready(1'b1),.center_confirmed(1'b1),.frame(frame),
 .x(x),.center(16'd960),.deadzone(16'd0),.runtime_seconds(16'd0),.gain(gain),.limit(8'd60),.mode(8'd1),.damping(8'd12),.slew(8'd2),
 .manual_target(8'sd0),.core_ready(1'b1),.core_busy(1'b0),.fault(4'd0),.step(1'b0),.position(32'sd0),
 .run(run),.candidate(candidate),.velocity(velocity));
 task tick;begin @(negedge clk);end endtask
 task sample(input [10:0] px);begin x=px;frame=1;tick();frame=0;tick();end endtask
 initial begin
  tick();rst=1;tick();key=1;tick();key=0;tick();sample(960);sample(961);
  if(!run || velocity!=4 || candidate!=4)$fatal(1,"subpixel positive damping lost: VE=%d T=%d",velocity,candidate);
  sample(961);if(velocity!=3 || candidate!=3)$fatal(1,"subpixel decay truncated prematurely");
  gain=128;#1;if(candidate!=-3)$fatal(1,"automatic direction inversion incorrect");
  gain=0;sample(959);if(velocity!=-5 || candidate!=-4)$fatal(1,"signed negative damping incorrect: VE=%d T=%d",velocity,candidate);
  valid=0;#1;if(candidate!=0)$fatal(1,"invalid PD target nonzero");tick();valid=1;tick();
  if(run)$fatal(1,"valid recovery resumed PD");
  $display("PD_ARITHMETIC_PASS: Q4 slow positive/negative velocity, decay, damping, inversion and invalid/no-resume");done=1;
 end
endmodule
module tb_follow;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,rx=1,frame=0,auto_frames=1,confirmed=1,bad=0,settings_busy=0;
 reg [3:0] keys=4'b1101,confidence=15;reg [10:0] bx=960,by=500;reg [1:0] enc1=0,phase=0;
 wire tx,en,dir,step;wire [3:0] led;wire [15:0] cs,ae,ag,rl,rt,rr,rb;
 vision_monitor #(.DEBOUNCE_CYCLES(4),.ENABLE_CYCLES(16),.POR_CYCLES(8),.UART_DIV(8),.REPORT_CYCLES(15000),
 .MS_CYCLES(10),.CONTROL_MS(10),.PACKET_TIMEOUT_CYCLES(5000),.SETUP_CYCLES(30),.HALF_CYCLES(5),.SETTLE_CYCLES(20),.TIMEOUT_CYCLES(10000)) dut(
 clk,rst,keys,enc1,2'b0,frame,bx,by,confirmed,bad,confidence,rx,tx,en,dir,step,led,
 16'd0,16'd0,settings_busy,cs,ae,ag,rl,rt,rr,rb);
 integer fc=0,pulses=0;
 always @(negedge clk)begin frame<=0;if(auto_frames)begin if(fc==39)begin fc<=0;frame<=1;end else fc<=fc+1;end else fc<=0;end
 always @(posedge step)begin pulses=pulses+1;phase=phase+(dir ? 2'd3 : 2'd1);case(phase)0:enc1=0;1:enc1=1;2:enc1=3;3:enc1=2;endcase end
 task tick(input integer n);begin repeat(n)@(negedge clk);end endtask
 task check(input integer ok,input string msg);begin if(!ok)$fatal(1,"FAIL follow: %s",msg);end endtask
 task click(input integer key);begin keys[key]=0;tick(24);keys[key]=1;tick(30);end endtask
 task sendbyte(input [7:0] b);integer i;begin rx=0;tick(8);for(i=0;i<8;i=i+1)begin rx=b[i];tick(8);end rx=1;tick(8);end endtask
 task packet(input [15:0] seq,input [7:0] mode,kd,sl,input [15:0] runtime,input integer corrupt,lf);
  reg [7:0] sum;integer i;string s;begin sum=seq[15:8]^seq[7:0]^mode^kd^sl^runtime[15:8]^runtime[7:0];if(corrupt)sum=sum^1;
   s=$sformatf("S%04h%02h%02h%02h%04h%02h",seq,mode,kd,sl,runtime,sum);for(i=0;i<s.len();i=i+1)sendbyte(s[i]);if(lf)sendbyte(10);tick(20);end endtask
 integer before_count,saved_pulses,errs;reg control_done=0;string last_line="";
 wire guard4,guard5,pd_done;retarget_guard #(.KIND(4)) g4(guard4);retarget_guard #(.KIND(5)) g5(guard5);pd_arithmetic pd_unit(pd_done);
 initial begin
  tick(4);rst=1;tick(100);keys[0]=0;tick(24);keys[0]=1;tick(100);
  check(en && !dut.auto_run && pulses==0,"held K2 through enable never starts");keys[1]=1;tick(30);
  click(1);check(!dut.auto_run && dut.supervisor_reject==10,"AUTO requires confirmed visual center");
  click(2);tick(100);check(dut.center_confirmed && dut.center_count==1,"K3 confirms center without motion");
  bx=1192;tick(100);keys[1]=0;tick(600);check(dut.auto_run,"long K2 starts exactly once");keys[1]=1;
  wait(dut.position>=10);tick(100);check(dut.fault==0 && dut.auto_run && dut.target==12,"continuous right target tracks while bounded");
  click(2);check(dut.center_status==4 && dut.center_count==1,"K3 rejected even between automatic bursts");
  bx=728;tick(100);wait(dut.position<=-10);check(dut.fault==0 && dut.auto_run,"continuous target reverses without new K2");
  bx=960;wait(dut.position>=-2 && dut.position<=2);tick(300);check(dut.auto_run && dut.target==0,"ball center and zero velocity return software reference");
  click(1);check(!dut.auto_run && en && dut.stop_reason==7,"K2 toggles stop with holding EN");saved_pulses=pulses;tick(300);check(pulses==saved_pulses,"stop never replays");
  click(1);check(dut.auto_run,"new K2 restarts");confirmed=0;tick(5100);check(!dut.auto_run && en && dut.stop_reason==3,"loss cancels entire automatic session");
  confirmed=1;tick(100);check(!dut.auto_run,"vision recovery cannot auto resume");click(1);auto_frames=0;tick(2050);check(!dut.auto_run && dut.stop_reason==4,"watchdog stops session");
  auto_frames=1;tick(100);click(1);settings_busy=1;tick(10);check(!dut.auto_run && dut.stop_reason==5,"camera reconfiguration stops session");settings_busy=0;tick(100);
  click(1);packet(1,2,6,2,30,0,1);check(!dut.auto_run && dut.operating_mode==2 && dut.follow_sequence==1,"S applies atomically and never starts motion");
  before_count=dut.completed_count;click(1);wait(dut.completed_count==before_count+1);check(!dut.auto_run && dut.locked_target==0,"explicit reference-return mode uses K2 single request");
  errs=dut.rx_errors;packet(2,3,6,2,30,0,1);packet(2,1,17,2,30,0,1);packet(2,1,6,0,30,0,1);packet(2,1,6,9,30,0,1);packet(2,1,6,2,301,0,1);packet(2,1,6,2,30,1,1);
  check(dut.follow_sequence==1 && dut.rx_errors==errs+6,"S bounds/checksum reject without commit");
  packet(2,1,6,2,1,0,0);check(dut.follow_sequence==1,"S does not commit without LF");sendbyte(10);tick(40);check(dut.follow_sequence==2 && dut.runtime_seconds==1,"S LF commit and readback");
  click(1);wait(!dut.auto_run);check(dut.stop_reason==8 && en && dut.elapsed_seconds==1,"configured runtime stops cleanly without fault/homing");
  packet(3,1,6,2,0,0,1);tick(100);click(1);tick(10500);check(dut.auto_run && dut.elapsed_seconds>=1,"runtime0 permits ongoing control");
  keys[3]=0;#1;check(!en && !step,"K4 raw stop");tick(10);keys[3]=1;tick(100);check(!en && !dut.auto_run,"K4 clears EN and run latch");
  click(0);tick(100);check(en && !dut.auto_run,"reenable never resumes session");click(1);click(0);tick(30);check(!en && !dut.auto_run,"K1-off clears automatic run");click(0);tick(100);
  // Keep a stable automatic report for actual waveform/parser verification.
  click(1);tick(15000);wait(last_line.len()==429 && last_line.substr(349,349)=="1");$display("UART_FOLLOW %s",last_line);
  saved_pulses=pulses;enc1=~enc1;tick(30);check(dut.fault==3 && !en && !dut.auto_run,"invalid encoder latches fault and stops session");click(1);check(!dut.auto_run && pulses==saved_pulses,"K2 cannot bypass fault");
  $display("FOLLOW_CONTROL_PASS: automatic PD/retarget/return, held/start-stop, center qualifier, S atomic bounds/checksum, runtime, loss/watchdog/config/K1/K4/fault and no auto-resume");control_done=1;
 end
 initial begin : capture
  reg [7:0] b;integer i;string line;line="";forever begin @(negedge tx);repeat(12)@(posedge clk);#1;
   for(i=0;i<8;i=i+1)begin b[i]=tx;repeat(8)@(posedge clk);#1;end check(tx==1,"UART stop bit");line={line,b};if(b==10)begin last_line=line;line="";end
  end
 end
 initial begin wait(control_done && guard4 && guard5 && pd_done);$display("FOLLOW_ALL_PASS");$finish;end
 initial begin #15000000;$display("DEBUG AR=%b P=%d T=%d AT=%d F=%d SR=%d AJ=%d",dut.auto_run,dut.position,dut.target,dut.drive_target,dut.fault,dut.stop_reason,dut.supervisor_reject);$fatal(1,"follow timeout");end
endmodule

`timescale 1ns/1ps
module fault_case #(parameter CASE=1)(output reg finished=0);
 reg clk=0;always #5 clk=~clk;
 reg rst=0,en_req=1,k4=1,k2=1;reg signed [31:0] encoder=0;reg [15:0] errors=0;
 wire en,dir,step,ready,busy;wire [3:0] fault;wire [7:0] spent;
 vision_servo_core #(.EXTERNAL_CONTROL(1),.DEBOUNCE_CYCLES(3),.ENABLE_CYCLES(8),.SETUP_CYCLES(6),.HALF_CYCLES(4),.SETTLE_CYCLES(5),
 .TIMEOUT_CYCLES(CASE==4 ? 1 : 2000),.MOVE_BUDGET(CASE==5 ? 1 : 64)) dut(
 .clk(clk),.rst_n(rst),.enable_request(en_req),.k4_n(k4),.k2_n(k2),.request(~k2),.following(1'b0),.external_cancel(1'b0),.external_cancel_reason(4'd0),.encoder_count(encoder),.encoder_errors(errors),
 .visual_valid(1'b1),.visual_stale(1'b0),.settings_ready(1'b1),.config_changed(1'b0),.visual_target(8'sd5),
 .motor_en(en),.motor_dir(dir),.motor_step(step),.ready(ready),.busy(busy),.fault(fault),.spent(spent));
 always @(posedge step)begin
  if(CASE==2)encoder<=encoder+(dir ? 1 : -1);
  else if(CASE==5)encoder<=encoder+(dir ? -1 : 1);
 end
 task tick(input integer n);begin repeat(n)@(negedge clk);end endtask
 initial begin
  tick(4);rst=1;tick(60);wait(ready);tick(12);
  if(CASE==6)dut.command_position=64;
  k2=0;tick(15);k2=1;
  if(CASE==3)begin wait(busy);errors=1;end
  if(CASE==7)begin wait(busy);encoder=-65;end
  if(CASE==8)begin wait(busy);force dut.state=15;tick(1);release dut.state;end
  wait(fault!=0);#1;
  if(fault!=CASE || en || step)$fatal(1,"fault fixture %0d got fault=%0d en=%b",CASE,fault,en);
  if(CASE==1 && spent!=3)$fatal(1,"no-response budget changed");
  if(CASE==2 && spent!=2)$fatal(1,"wrong direction threshold changed");
  tick(10);en_req=0;tick(10);en_req=1;k2=0;tick(30);
  if(fault!=CASE || en || step)$fatal(1,"fault %0d bypassed by re-enable",CASE);
  rst=0;tick(3);if(fault!=0)$fatal(1,"reset did not clear fault");
  $display("PASS fault %0d: latched, output stopped, no re-enable bypass",CASE);finished=1;
 end
endmodule

module normal_case(output reg finished=0);
 reg clk=0;always #5 clk=~clk;
 reg rst=0,en_req=0,k4=1,k2=0,valid=1,stale=0,settings=1,changed=0;
 reg signed [31:0] encoder=0;reg signed [7:0] target=5;
 wire en,dir,step,ready,busy,ok,z;wire signed [31:0] pos,mt,cmd,err;
 wire [3:0] fault,cancel,reject;wire [15:0] accepted,completed;wire [7:0] spent;
 vision_servo_core #(.DEBOUNCE_CYCLES(3),.ENABLE_CYCLES(8),.SETUP_CYCLES(30),.HALF_CYCLES(10),.SETTLE_CYCLES(20),.TIMEOUT_CYCLES(2000)) dut(
 .clk(clk),.rst_n(rst),.enable_request(en_req),.k4_n(k4),.k2_n(k2),.encoder_count(encoder),.encoder_errors(16'd0),
 .visual_valid(valid),.visual_stale(stale),.settings_ready(settings),.config_changed(changed),.visual_target(target),
 .motor_en(en),.motor_dir(dir),.motor_step(step),.ready(ready),.busy(busy),.reached(ok),.origin_valid(z),
 .position(pos),.locked_target(mt),.command_position(cmd),.error(err),.fault(fault),.cancel_reason(cancel),.reject_reason(reject),
 .accepted_count(accepted),.completed_count(completed),.spent(spent));
 integer pulses=0;time dir_changed=0,high_started=0;
 always @(dir)dir_changed=$time;
 always @(posedge step)begin
  if($time-dir_changed<300)$fatal(1,"DIR setup interval shortened");
  high_started=$time;pulses=pulses+1;encoder<=encoder+(dir ? -2 : 2);
 end
 always @(negedge step)if(rst && en && valid && !stale && settings && !changed && high_started!=0 && $time-high_started<100)$fatal(1,"normal high pulse shortened");
 task tick(input integer n);begin repeat(n)@(negedge clk);end endtask
 task check(input integer pass,input string text);begin if(!pass)$fatal(1,"FAIL normal: %s",text);end endtask
 task click;begin k2=1;tick(15);k2=0;tick(15);k2=1;end endtask
 integer count_before,pulse_before;reg signed [31:0] saved_cmd;
 initial begin
  tick(4);rst=1;tick(20);en_req=1;tick(100);check(ready && !busy && accepted==0,"held K2 at boot/enable cannot replay");
  k2=1;tick(20);click;wait(busy);target=-5;click;wait(ok);tick(10);
  check(mt==5 && pos==4 && accepted==1 && completed==1 && reject==6,"snapshot frozen, busy command rejected, feedback reached");
  tick(100);check(accepted==1 && pos==4,"no continuous following after completion");
  target=0;click;wait(ok && completed==2);check(pos<=2 && pos>=-2,"center zero reached in deadband");
  target=-5;click;wait(busy);k2=0;@(posedge step);#1;valid=0;#1;check(!step && en,"loss gates STEP but keeps holding EN");
  tick(5);check(!busy && cancel==3 && mt==-5,"loss cancels without changing locked target to zero");
  count_before=accepted;pulse_before=pulses;valid=1;tick(100);check(accepted==count_before && pulses==pulse_before,"no replay on recovery while held");
  k2=1;tick(20);click;wait(ok);check(mt==-5 && pos>=-7 && pos<=-3,"fresh press after recovery reaches negative target");
  target=5;click;wait(busy);stale=1;tick(5);check(!busy && en && cancel==4,"stale cancels, no return-to-zero");
  stale=0;k2=1;tick(20);click;wait(busy);changed=1;tick(3);changed=0;tick(10);check(!busy && cancel==5,"parameter commit cancels");
  click;wait(busy);settings=0;tick(5);check(!busy && cancel==5,"configuration busy cancels");settings=1;tick(20);
  click;wait(busy);saved_cmd=cmd;en_req=0;#1;check(!en && !step,"K1-off raw gate");tick(5);check(cancel==1,"K1-off cancellation reason");
  en_req=1;tick(60);check(z && ready && !busy && cmd==saved_cmd,"re-enable preserves origin/CMD, cannot resume");
  click;wait(busy);k4=0;#1;check(!en && !step,"K4 immediate");tick(5);check(cancel==2,"K4 reason");k4=1;tick(60);check(!busy,"K4 cannot replay command");
  valid=0;tick(5);pulse_before=pulses;click;tick(30);check(reject==3 && pulses==pulse_before,"invalid command rejected");
  valid=1;target=61;tick(20);click;tick(30);check(reject==7 && !busy,"out-of-range command rejected");
  check(fault==0,"normal/cancel tests do not invent faults");
  $display("PASS finite motion: snapshot, positive/zero/negative feedback, busy/held rejection, all cancellation paths and no auto-resume");finished=1;
 end
endmodule

module tb_motion;
 wire [8:0] done;
 normal_case normal(done[0]);
 genvar i;generate for(i=1;i<=8;i=i+1)begin: faults fault_case #(.CASE(i)) f(done[i]);end endgenerate
 initial begin wait(&done);$display("MOTION_GUARDS_PASS");$finish;end
 initial begin #2000000;$fatal(1,"motion test timeout");end
endmodule

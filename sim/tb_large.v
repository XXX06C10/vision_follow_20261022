`timescale 1ns/1ps
// Production phase ratios /500: 20ms setup, 2.5ms high/low, 50ms feedback,
// and 5s timeout. Model an encoder with exactly32/25 counts per pulse.
module tb_large;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,k2=1;reg signed [7:0] target=60;reg signed [31:0] encoder=0;
 wire en,dir,step,ready,busy,ok;wire [3:0] fault;wire [7:0] spent;
 wire signed [31:0] position,mt,cmd,err;wire [15:0] ac,dc;
 vision_servo_core #(.DEBOUNCE_CYCLES(10),.ENABLE_CYCLES(20),.SETUP_CYCLES(2000),
 .HALF_CYCLES(250),.SETTLE_CYCLES(5000),.TIMEOUT_CYCLES(500000)) dut(
 .clk(clk),.rst_n(rst),.enable_request(1'b1),.k4_n(1'b1),.k2_n(k2),
 .encoder_count(encoder),.encoder_errors(16'd0),.visual_valid(1'b1),.visual_stale(1'b0),
 .settings_ready(1'b1),.config_changed(1'b0),.visual_target(target),
 .motor_en(en),.motor_dir(dir),.motor_step(step),.ready(ready),.busy(busy),.reached(ok),
 .fault(fault),.spent(spent),.position(position),.locked_target(mt),.command_position(cmd),.error(err),
 .accepted_count(ac),.completed_count(dc));
 integer physical_pulses=0,pulses=0,start_pulses=0,n=0;
 always @(posedge step)begin
  physical_pulses=physical_pulses+(dir ? 1 : -1);pulses=pulses+1;
  encoder<=-(physical_pulses*32/25);
 end
 task tick(input integer count);begin repeat(count)@(negedge clk);end endtask
 task check(input integer good,input string text);begin if(!good)$fatal(1,"FAIL large: %s",text);end endtask
 task start(input signed [7:0] value);begin
  target=value;k2=1;tick(40);start_pulses=pulses;k2=0;tick(40);k2=1;
 end endtask
 task finish(input signed [7:0] value,input integer count);begin
  wait(ok || fault!=0);tick(2);
  check(fault==0 && mt==value && err<=2 && err>=-2 && dc==count && spent<=64,"large target reaches with intact limits");
  check(cmd>=-64 && cmd<=64 && position>=-64 && position<=64,"position and command boundaries respected");
  $display("LARGE_REACHED target=%0d P=%0d error=%0d pulses=%0d elapsed_scaled=%0d",mt,position,err,spent,dut.action_timer);
 end endtask
 initial begin
  tick(5);rst=1;tick(120);check(ready && pulses==0,"enable does not move");
  start(60);wait(busy);target=-60;finish(60,1);
  check(dut.action_timer>200000 && dut.action_timer<500000,"old2s would timeout; new5s permits physical phase timings");
  n=pulses;tick(200);check(pulses==n && !busy,"new live target never follows automatically");
  start(0);finish(0,2);start(-60);finish(-60,3);start(0);finish(0,4);
  $display("LARGE_NORMAL_PASS: +60/0/-60/0 at1.28 counts per pulse, production phase ratios, five-second timeout, no false faults");
  start(60);finish(60,5);start(-60);wait(fault!=0);tick(2);
  check(fault==5 && spent==64 && pulses-start_pulses==64 && !en && !step,"oversized direct reversal stops at64-pulse budget before5s timeout");
  check(dut.action_timer<500000,"budget fault not disguised as timeout");
  $display("LARGE_BUDGET_PASS: direct +60 to -60 exceeds requested64-pulse budget and is stopped intentionally");
  $display("LARGE_TARGET_PASS");$finish;
 end
 initial begin #30000000;$fatal(1,"large simulation timeout");end
endmodule

`timescale 1ns/1ps
module tb_loss;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,rx=1,frame=0,auto_frames=1,manual_frame=0,confirmed=1,bad=0,settings=0;
 reg [3:0] keys=15,q=15;reg [10:0] bx=960,by=500;reg [1:0] enc=0,phase=0;
 wire tx,en,dir,step;wire [3:0] led;wire [15:0] cs,ae,ag,rl,rt,rr,rb;
 vision_monitor #(.DEBOUNCE_CYCLES(20),.ENABLE_CYCLES(10000),.POR_CYCLES(8),.UART_DIV(4),.REPORT_CYCLES(20000),
 .MS_CYCLES(100),.SETUP_CYCLES(2000),.HALF_CYCLES(250),.SETTLE_CYCLES(5000),.TIMEOUT_CYCLES(500000)) dut(
 clk,rst,keys,enc,2'b0,frame,bx,by,confirmed,bad,q,rx,tx,en,dir,step,led,16'd0,16'd0,settings,cs,ae,ag,rl,rt,rr,rb);
 integer fc=0,pulses=0,commands=0,encoded=0,goal=0,diff=0,j=0,prior=0,k=0;
 reg signed [7:0] frozen_t,frozen_at;reg signed [31:0] saved_origin;
 always @(negedge clk)begin frame<=manual_frame;if(auto_frames)begin if(fc==3299)begin fc<=0;frame<=1;end else fc<=fc+1;end else fc<=0;end
 always @(posedge step)begin
  pulses=pulses+1;commands=commands+(dir ? 1 : -1);goal=commands*32/25;diff=goal-encoded;
  for(j=0;j<(diff<0 ? -diff : diff);j=j+1)begin
   repeat(8)@(negedge clk);phase=phase+(diff>0 ? 2'd3 : 2'd1);
   case(phase)0:enc=0;1:enc=1;2:enc=3;3:enc=2;endcase
  end
  encoded=goal;
 end
 task tick(input integer n);begin repeat(n)@(negedge clk);end endtask
 task check(input integer ok,input string msg);begin if(!ok)$fatal(1,"LOSS: %s AR=%b V=%b P=%d SR=%d age=%d good_cycles=%d",msg,dut.auto_run,dut.valid,dut.position,dut.stop_reason,dut.age,dut.loss_guard.elapsed);end endtask
 task click(input integer key);begin keys[key]=0;tick(60);keys[key]=1;tick(70);end endtask
 task next_result;begin @(posedge dut.result_frame);tick(3);end endtask
 task restore;begin confirmed=1;bad=0;q=15;next_result;tick(100);end endtask
 task start;begin restore;click(1);check(dut.auto_run,"new released K2 starts");end endtask
 task miss(input integer count);begin confirmed=0;bx=0;by=0;repeat(count)next_result;end endtask
 task sendbyte(input [7:0] b);integer i;begin rx=0;tick(4);for(i=0;i<8;i=i+1)begin rx=b[i];tick(4);end rx=1;tick(4);end endtask
 task mode_packet(input [7:0] md,input [15:0] seq);integer i;reg [7:0] sum;string packet;begin
  sum=seq[15:8]^seq[7:0]^md^8'd12^8'd2^8'd30;packet=$sformatf("S%04h%02h0C02001E%02h",seq,md,sum);
  for(i=0;i<packet.len();i=i+1)sendbyte(packet[i]);sendbyte(10);tick(20);
 end endtask
 initial begin
  tick(5);rst=1;tick(4000);click(2);tick(4000);click(0);wait(dut.request_ready);saved_origin=dut.motor_core.origin;
  bx=1192;next_result;click(1);wait(step);frozen_t=dut.target;frozen_at=dut.drive_target;
  // One invalid result arrives during HIGH: raw V must not gate/truncate STEP.
  auto_frames=0;confirmed=0;bx=0;by=0;@(negedge clk);manual_frame=1;tick(2);manual_frame=0;tick(5);
  check(!dut.valid && dut.auto_run && step && dut.target==frozen_t && dut.drive_target==frozen_at,"one bad frame preserves session/reference and high STEP");
  tick(3000);confirmed=1;bx=1180;by=500;manual_frame=1;tick(2);manual_frame=0;tick(3);check(dut.velocity==0,"first recovered frame resets derivative");
  auto_frames=1;restore;check(dut.auto_run && dut.fault==0,"one frame recovery");
  miss(1);frozen_t=dut.target;frozen_at=dut.drive_target;miss(1);tick(500);
  check(dut.auto_run && !dut.valid && dut.target==frozen_t && dut.drive_target==frozen_at && dut.velocity==0,"two frames freeze AT/T and velocity");
  bx=1160;by=500;restore;check(dut.auto_run && dut.velocity==0,"two frame recovery no velocity spike");
  for(k=0;k<6;k=k+1)begin miss(2);bx=1160;by=500;restore;check(dut.auto_run,"repeated losses do not restart session");end
  // Keep invalid completed results flowing; their ages remain fresh, but the
  // independent valid-ball deadline must still expire at exactly500ms.
  bx=728;by=500;restore;wait(dut.drive_target<0 && dut.busy);prior=pulses;miss(1);frozen_t=dut.target;frozen_at=dut.drive_target;
  wait(dut.loss_guard.elapsed==49999);#1;check(dut.auto_run && dut.hold_valid && dut.drive_target==frozen_at,"499.99ms still held");
  tick(4);check(!dut.auto_run && !step && en && dut.stop_reason==3 && dut.age<200,"500ms invalid result flow stops, retains EN");
  check(pulses>prior,"executor issued real feedback-controlled steps while V0");
  $display("LOSS500_BOUNDARY_PASS elapsed=%0d pulses_during_loss=%0d",dut.loss_guard.elapsed,pulses-prior);
  bx=1192;by=500;restore;check(!dut.auto_run,"recovery after deadline never auto resumes");
  start;auto_frames=0;wait(dut.age==199);check(dut.auto_run,"199ms no-result still held");wait(dut.age==200);tick(4);
  check(!dut.auto_run && en && !step && dut.stop_reason==4,"200ms no-result watchdog unchanged");auto_frames=1;
  start;miss(1);click(1);check(!dut.auto_run && en && dut.stop_reason==7,"K2 stops while coasting");click(1);check(!dut.auto_run && dut.supervisor_reject==3,"K2 cannot start from held target");
  click(2);check(dut.center_status==2,"K3 cannot capture held coordinates");
  bx=1192;by=500;start;miss(1);settings=1;tick(3);check(!dut.auto_run && dut.stop_reason==5,"settings stop during coast");settings=0;
  bx=1192;by=500;start;miss(1);keys[3]=0;#1;check(!en && !step,"K4 remains immediate during coast");tick(10);keys[3]=1;tick(100);check(!dut.auto_run && !en,"K4 clears run and EN");
  bx=1192;by=500;restore;click(0);wait(dut.request_ready);start;miss(1);click(0);check(!dut.auto_run && !en,"K1 off during coast");
  bx=1192;by=500;restore;click(0);wait(dut.request_ready);start;miss(1);mode_packet(0,1);check(!dut.auto_run,"S configuration cancels coast");
  bx=1192;by=500;restore;click(1);wait(dut.position>=6 && dut.busy);miss(1);tick(5);check(!dut.busy && !step && dut.cancel_reason==3,"manual remains strict");
  mode_packet(2,2);bx=960;by=500;restore;click(1);wait(dut.busy);miss(1);tick(5);check(!dut.busy && !step && dut.cancel_reason==3,"reference return remains strict");
  mode_packet(1,3);bx=1192;by=500;start;bad=1;q=4;next_result;check(dut.auto_run && !dut.valid,"one bad/low-confidence frame coasts");
  restore;check(dut.auto_run,"bad-frame recovery");
  check(dut.fault==0 && dut.errors1==0 && dut.motor_core.origin==saved_origin,"real1.28AB feedback no spurious faults or reference reset");
  $display("LOSS_CONTROL_PASS: one/two/repeated losses, true STEP/AB, frozen reference and reset velocity,500/200ms boundaries, strict starts/K3/manual/return, K1/K2/K4/config, no resume");tick(20000);$finish;
 end
 initial begin : uart
  reg [7:0] b;integer i;string line;line="";
  forever begin @(negedge tx);repeat(6)@(posedge clk);#1;for(i=0;i<8;i=i+1)begin b[i]=tx;repeat(4)@(posedge clk);#1;end
   if(tx!=1)$fatal(1,"UART loss stop bit");line={line,b};if(b==10)begin if(line.len()!=429)$fatal(1,"loss wire length");$display("UART_LOSS %s",line);line="";end
  end
 end
 initial begin #30000000;$fatal(1,"loss timeout");end
endmodule

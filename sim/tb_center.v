`timescale 1ns/1ps
module tb_center;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,rx=1,frame=0,auto_frames=1,confirmed=1,bad=0,settings_busy=0;
 reg [3:0] keys=4'b1011,confidence=15;
 reg [10:0] bx=812,by=500;reg [1:0] enc1=0,phase=0;
 wire tx,en,dir,step;wire [3:0] led;wire [15:0] cs,ae,ag,rl,rt,rr,rb;
 vision_monitor #(.DEFAULT_MODE(0),.DEFAULT_GAIN(7),.DEFAULT_LIMIT(5),.DEBOUNCE_CYCLES(4),.ENABLE_CYCLES(16),.POR_CYCLES(8),
 .UART_DIV(8),.REPORT_CYCLES(40000),.MS_CYCLES(50),.PACKET_TIMEOUT_CYCLES(5000),
 .SETUP_CYCLES(3000),.HALF_CYCLES(5),.SETTLE_CYCLES(20),.TIMEOUT_CYCLES(100000)) dut(
 clk,rst,keys,enc1,2'b0,frame,bx,by,confirmed,bad,confidence,rx,tx,en,dir,step,led,
 16'd0,16'd0,settings_busy,cs,ae,ag,rl,rt,rr,rb);
 integer frame_cycle=0,pulses=0;
 always @(negedge clk)begin
  frame<=0;
  if(auto_frames)begin if(frame_cycle==39)begin frame<=1;frame_cycle<=0;end else frame_cycle<=frame_cycle+1;end
  else frame_cycle<=0;
 end
 always @(posedge step)begin
  pulses=pulses+1;phase=phase+(dir ? 2'd3 : 2'd1);
  case(phase)0:enc1=0;1:enc1=1;2:enc1=3;3:enc1=2;endcase
 end
 task tick(input integer n);begin repeat(n)@(negedge clk);end endtask
 task check(input integer ok,input string text);begin if(!ok)$fatal(1,"FAIL center: %s",text);end endtask
 task click(input integer key);begin keys[key]=0;tick(24);keys[key]=1;tick(50);end endtask
 task sendbyte(input [7:0] b);integer i;begin rx=0;tick(8);for(i=0;i<8;i=i+1)begin rx=b[i];tick(8);end rx=1;tick(8);end endtask
 task packet(input [15:0] seq,c,input integer lf);
 reg [7:0] sum;string s;integer i;begin sum=seq[15:8]^seq[7:0]^c[15:8]^c[7:0]^8'd0^8'd20^8'd7^8'd5;
  s=$sformatf("P%04h%04h00140705%02h",seq,c,sum);for(i=0;i<s.len();i=i+1)sendbyte(s[i]);if(lf)sendbyte(10);tick(16);end endtask
 integer saved_pulses,saved_count;reg signed [31:0] saved_cmd,saved_origin;
 string last_line="";
 initial begin
  tick(4);rst=1;tick(100);check(dut.center==960 && dut.center_count==0,"held K3 at boot cannot capture");
  keys[2]=1;tick(24);keys[2]=0;tick(1);keys[2]=1;tick(24);check(dut.center_count==0,"short bounce ignored");
  auto_frames=0;click(2);check(dut.center==812 && dut.center_count==1 && dut.center_status==1 && dut.target==0,"disabled motor can set actual ball center");
  check(!dut.settings_ready,"K3 commit cannot reuse pre-capture frame for a new action");
  auto_frames=1;tick(100);check(dut.settings_ready,"post-K3 valid new frame requalifies action conditions");
  check(!en && pulses==0 && dut.accepted_count==0 && !dut.origin_valid,"K3 does not enable/move/establish encoder origin");
  bx=1000;tick(100);keys[2]=0;tick(100);check(dut.center==1000 && dut.center_count==2,"fresh K3 captures once");
  bx=1200;tick(150);check(dut.center==1000 && dut.center_count==2,"long hold does not follow ball");keys[2]=1;tick(30);
  confirmed=0;tick(100);click(2);check(dut.center==1000 && dut.center_status==2,"lost ball rejected");
  confirmed=1;bad=1;tick(100);click(2);check(dut.center_count==2 && dut.center_status==2,"bad frame rejected");
  bad=0;confidence=7;tick(100);click(2);check(dut.center_count==2,"weak confidence rejected");confidence=15;
  bx=1920;tick(100);click(2);check(dut.center_count==2,"out-of-screen frame rejected");
  bx=700;tick(100);auto_frames=0;tick(10030);click(2);check(dut.center_count==2 && dut.center_status==2,"200ms stale result rejected");
  auto_frames=1;settings_busy=1;tick(100);click(2);check(dut.center_count==2 && dut.center_status==3,"camera config rejected");
  settings_busy=0;auto_frames=0;tick(60);click(2);check(dut.center_count==2 && dut.center_status==3,"old frame after settings cannot capture");
  auto_frames=1;tick(100);click(2);check(dut.center==700 && dut.center_count==3,"fresh frame after config captures");
  keys[3]=0;click(2);check(dut.center_count==3 && dut.center_status==6,"K4 blocks capture");keys[3]=1;tick(30);
  click(0);tick(100);check(en && dut.origin_valid && pulses==0,"K1 reference only");
  bx=1400;tick(100);click(1);wait(dut.busy);click(2);
  check(dut.center_count==3 && dut.center==700 && dut.center_status==4 && dut.locked_target==5,"busy K3 rejected without changing frozen motor target");
  wait(dut.reached);check(dut.position==3 && dut.completed_count==1,"existing actual AB feedback still reaches target");
  saved_pulses=pulses;saved_cmd=dut.command_position;saved_origin=dut.motor_core.origin;
  keys[1]=0;keys[2]=0;tick(24);keys[1]=1;keys[2]=1;tick(100);
  check(dut.center==1400 && dut.center_count==4 && dut.accepted_count==1,"simultaneous K3/K2 captures but never launches motion");
  check(pulses==saved_pulses && dut.position==3 && dut.command_position==saved_cmd && dut.motor_core.origin==saved_origin,"center capture preserves motor position/origin/command");
  check(dut.target==0 && !dut.busy,"capture does not drive back to zero");
  click(1);wait(dut.completed_count==2);check(dut.position<=2 && dut.position>=-2,"new K2 needed to execute center-zero target");
  packet(1,900,1);tick(100);check(dut.center==900 && dut.cfg_sequence==1 && dut.center_count==4,"legacy P overrides center explicitly; hardware capture counter separate");
  packet(2,850,0);tick(100);saved_count=dut.center_count;
  fork
   begin sendbyte(10);end
   begin wait(dut.config_rx.byte_valid && dut.config_rx.byte_value==10);force dut.center_event=1;tick(2);release dut.center_event;end
  join
  tick(50);check(dut.center==850 && dut.cfg_sequence==2 && dut.center_count==saved_count && dut.center_status==7,"exact UART/K3 collision has serial priority and visible rejection");
  bx=0;tick(100);click(2);check(dut.center==0 && dut.target==0,"left boundary zero is valid");
  bx=1919;tick(100);click(2);check(dut.center==1919 && dut.target==0,"right boundary is valid");
  bx=500;tick(100);click(2);saved_count=dut.center_count;
  wait(last_line.len()==429 && last_line.substr(331,334)==$sformatf("%04h",saved_count[15:0]) && last_line.substr(339,339)=="1");
  $display("UART_CENTER %s",last_line);
  saved_pulses=pulses;enc1=~enc1;tick(30);check(dut.fault==3,"actual illegal AB produces retained fault");click(2);
  check(dut.center_count==saved_count && dut.center_status==5 && !en && pulses==saved_pulses,"K3 cannot clear/bypass fault or emit pulses");
  rst=0;tick(20);check(dut.center==960 && dut.center_count==0 && dut.center_status==0,"reload/reset returns default; no false persistence");
  $display("CENTER_CAPTURE_PASS: debounce/release/hold, valid-current X, no motion/origin reset, invalid/stale/config/busy/K4/fault rejection, K2 collision, serial arbitration, bounds, reset and wire status");$finish;
 end
 initial begin : capture
  reg [7:0] b;integer i;string line;line="";
  forever begin @(negedge tx);repeat(12)@(posedge clk);#1;
   for(i=0;i<8;i=i+1)begin b[i]=tx;repeat(8)@(posedge clk);#1;end
   check(tx==1,"UART stop bit");line={line,b};if(b==10)begin last_line=line;line="";end
  end
 end
 initial begin #3000000;$display("DEBUG C=%d CC=%d KC=%d F=%d AC=%d DC=%d MB=%b T=%d P=%d last=%s",dut.center,dut.center_count,dut.center_status,dut.fault,dut.accepted_count,dut.completed_count,dut.busy,dut.target,dut.position,last_line);$fatal(1,"center test timeout");end
endmodule

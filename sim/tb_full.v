`timescale 1ns/1ps
module tb_full;
reg clk=0;always #5 clk=~clk;
reg rst=0,rx=1,frame=0,auto_frames=1,confirmed=1,bad=0,settings_busy=0;
reg [3:0] keys=4'b1110,confidence=15;
reg [10:0] bx=1660,by=500;reg [1:0] enc1=0,phase=0;
wire tx,en,dir,step;wire [3:0] led;
wire [15:0] cs,ae,ag,rl,rt,rr,rb;
vision_monitor #(.DEFAULT_MODE(0),.DEFAULT_GAIN(7),.DEFAULT_LIMIT(5),.DEBOUNCE_CYCLES(4),.ENABLE_CYCLES(16),.POR_CYCLES(8),
 .UART_DIV(8),.REPORT_CYCLES(40000),.MS_CYCLES(50),.PACKET_TIMEOUT_CYCLES(5000),
 .SETUP_CYCLES(3000),.HALF_CYCLES(5),.SETTLE_CYCLES(20),.TIMEOUT_CYCLES(500000)) dut(
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
task check(input integer ok,input string text);begin if(!ok)$fatal(1,"FAIL integrated: %s",text);end endtask
task click(input integer key);begin keys[key]=0;tick(24);keys[key]=1;tick(30);end endtask
task sendbyte(input [7:0] b);integer i;begin rx=0;tick(8);for(i=0;i<8;i=i+1)begin rx=b[i];tick(8);end rx=1;tick(8);end endtask
task textline(input string s);integer i;begin for(i=0;i<s.len();i=i+1)sendbyte(s[i]);end endtask
task packet(input [15:0] seq,c,d,input [7:0] g,l,input integer corrupt,lf);
reg [7:0] sum;string s;begin sum=seq[15:8]^seq[7:0]^c[15:8]^c[7:0]^d[15:8]^d[7:0]^g^l;
 if(corrupt)sum=sum^1;s=$sformatf("P%04h%04h%04h%02h%02h%02h",seq,c,d,g,l,sum);
 textline(s);if(lf)sendbyte(10);tick(16);end endtask
integer before_count,saved_pulses,errs;
integer monitor_lines=0;string monitor_last="";reg monitor_done=0;
initial begin
 tick(4);rst=1;tick(100);check(!en && pulses==0,"held K1 cannot enable");
 keys[0]=1;tick(30);click(1);check(dut.reject_reason==1 && pulses==0,"K2 while disabled rejected");
 click(0);tick(100);check(en && dut.request_ready && dut.origin_valid,"K1 latch, alignment and fresh vision ready");
 tick(100);check(pulses==0,"no enable-only movement");
 click(1);check(dut.busy && dut.locked_target==5,"K2 locks right visual target");
 bx=260;tick(100);check(dut.target==-5 && dut.locked_target==5,"live and locked targets differ during action");
 wait(dut.reached);check(dut.position==3 && dut.errors1==0 && dut.completed_count==1,"actual AB feedback reaches right target");
 saved_pulses=pulses;tick(1000);check(pulses==saved_pulses,"no continuous follow");
 bx=960;tick(100);click(1);wait(dut.reached && dut.completed_count==2);check(dut.position<=2 && dut.position>=-2,"center position deadband");
 bx=260;tick(100);click(1);wait(dut.reached && dut.completed_count==3);check(dut.position<=-3 && dut.position>=-7,"left target real AB feedback");
 bx=960;tick(100);click(1);wait(dut.reached && dut.completed_count==4);
 // Exercise real serial validation and unchanged candidate mapping at the new bounds.
 packet(16'h0ffd,960,20,8'h80,60,0,1);bx=1040;tick(100);check(dut.gain==128 && dut.limit==60 && dut.target==-60,"inverted gain0/limit60 accepted end-to-end");
 packet(16'h0ffe,960,20,1,60,0,1);tick(100);check(dut.target==30,"gain1 accepted");
 packet(16'h0fff,960,20,2,60,0,1);tick(100);check(dut.target==15,"gain2 accepted");
 packet(16'h1000,960,20,0,60,0,1);tick(100);check(dut.target==60,"gain0/limit60 accepted");
 click(1);wait(dut.reached && dut.completed_count==5);check(dut.position>=58 && dut.position<=62 && dut.spent<=64 && dut.fault==0,"large +60 real AB feedback reaches");
 bx=960;tick(100);click(1);wait(dut.reached && dut.completed_count==6);check(dut.position<=2 && dut.position>=-2,"large positive returns center");
 bx=880;tick(100);check(dut.target==-60,"large signed negative candidate");click(1);wait(dut.reached && dut.completed_count==7);check(dut.position<=-58 && dut.position>=-62 && dut.spent<=64 && dut.fault==0,"large -60 real AB feedback reaches");
 bx=960;tick(100);click(1);wait(dut.reached && dut.completed_count==8);check(dut.position<=2 && dut.position>=-2 && dut.fault==0,"large negative returns center");
 $display("LARGE_INTEGRATION_PASS: real P gain0/1/2/invert and limit60 -> candidate +/-60 -> K2 -> real AB feedback, center returns, guards intact");
 packet(0,960,20,7,5,0,1);bx=1660;tick(100);
 bx=1660;tick(100);click(1);wait(dut.busy);confirmed=0;tick(100);
 check(!dut.busy && en && dut.cancel_reason==3,"lost frame cancels while holding EN");
 confirmed=1;tick(100);before_count=dut.accepted_count;tick(2000);check(dut.accepted_count==before_count && !dut.busy,"recovery requires new K2");
 click(1);wait(dut.busy);auto_frames=0;tick(10030);check(!dut.busy && dut.cancel_reason==4,"missing frames cancel at watchdog");
 auto_frames=1;tick(100);click(1);wait(dut.busy);packet(1,1000,10,8'h87,3,0,1);
 check(!dut.busy && dut.cancel_reason==5,"actual P commit cancels before a new step");
 auto_frames=0;packet(2,960,20,7,5,0,1);tick(20);check(!dut.settings_ready,"commit cannot reuse an older frame");
 auto_frames=1;tick(100);check(dut.settings_ready,"new valid frame requalifies settings");
 click(1);wait(dut.busy);settings_busy=1;tick(4);check(!dut.busy && en,"camera settings busy cancels");
 settings_busy=0;auto_frames=0;tick(40);check(!dut.settings_ready,"camera completion requires a later frame");auto_frames=1;tick(100);
 click(1);wait(dut.busy);keys[3]=0;#1;check(!en && !step,"K4 immediate gate");tick(5);keys[3]=1;tick(100);check(!en && !dut.busy,"latch cleared, no replay");
 click(0);tick(100);check(en && !dut.busy,"fresh K1 enables holding only");click(0);check(!en,"second K1 click releases");
 errs=dut.rx_errors;packet(3,960,20,7,5,1,1);check(dut.cfg_sequence==2 && dut.rx_errors==errs+1,"checksum rejection");
 packet(3,1920,20,7,5,0,1);packet(3,960,201,7,5,0,1);packet(3,960,20,255,5,0,1);packet(3,960,20,13,5,0,1);packet(3,960,20,7,61,0,1);
 check(dut.cfg_sequence==2,"FPGA parameter bounds preserved");textline("P00");tick(5100);sendbyte(10);tick(20);
 packet(4,960,20,7,5,0,0);check(dut.cfg_sequence==2,"no partial commit");sendbyte(10);tick(20);check(dut.cfg_sequence==4,"LF atomic commit");
 textline("MA\nM+\nMS\n");check(!en && !dut.busy,"legacy motor messages ignored");
 wait(monitor_last.len()==429 && monitor_last.substr(141,144)=="0004");
 $display("UART_MONITOR_FINAL %s",monitor_last);
 enc1=~enc1;tick(30);check(dut.fault==3 && !en,"E1 invalid AB fault inhibits enable");click(0);check(!en,"fault cannot be bypassed");
 $display("PASS monitor integration: K1/K4, real AB feedback, fresh-frame qualification, P commits, snapshot targets and all cancellation causes");monitor_done=1;
end
initial begin : monitor_capture
reg [7:0] b;integer i;string line;line="";
forever begin @(negedge tx);repeat(12)@(posedge clk);#1;
 for(i=0;i<8;i=i+1)begin b[i]=tx;repeat(8)@(posedge clk);#1;end
 check(tx==1,"UART stop bit");line={line,b};if(b==10)begin monitor_last=line;monitor_lines=monitor_lines+1;$display("UART %s",line);line="";end
end end
// Separate serializer test changes its live inputs while the first snapshot is in flight.
reg report_rst=0;reg [10:0] report_x=123,report_y=456;wire report_tx;
vision_report_tx #(.UART_DIV(8),.REPORT_CYCLES(15000)) reporter(
 .clk(clk),.rst_n(report_rst),.x(report_x),.y(report_y),.valid(1'b1),.age(16'd1),.frames(16'd2),
 .enabled(1'b0),.origin_valid(1'b1),.fault(4'd0),.count1(-32'sd4),.count2(32'd0),
 .errors1(16'd0),.errors2(16'd0),.position(32'sd4),.target(-8'sd5),
 .center(16'd960),.deadzone(16'd20),.gain(8'd7),.limit(8'd5),.cfg_sequence(16'h1234),.rx_errors(16'd0),
 .locked_target(-32'sd5),.move_error(-32'sd9),.move_state(4'd7),.move_busy(1'b1),.reached(1'b0),.request_ready(1'b0),.spent(8'd3),.command_position(-32'sd4),.cancel_reason(4'd0),.reject_reason(4'd6),.accepted_count(16'd2),.completed_count(16'd1),
 .operating_mode(8'd1),.auto_run(1'b1),.damping(8'd6),.slew(8'd2),.runtime_seconds(16'd30),.follow_sequence(16'h1234),.center_confirmed(1'b1),.velocity(-16'sd4),.drive_target(-8'sd5),.stop_reason(4'd7),.auto_reject(4'd10),.elapsed_seconds(16'd3),.total_pulses(32'h12345678),.center_count(16'habcd),.center_status(4'd1),.quality(4'hf),.bad(1'b0),.cam_sequence(16'h1234),.exposure(16'd3600),.sensor_gain(16'd143),
 .roi_left(16'd100),.roi_top(16'd200),.roi_right(16'd1200),.roi_bottom(16'd700),
 .camera_applied_seq(16'h1233),.roi_applied_seq(16'h1234),.settings_busy(1'b1),.tx(report_tx));
reg report_done=0;
initial begin : report_capture
 reg [7:0] b;integer i,j;string line,expected;
 expected="VIS=5 BX=007B BY=01C8 V=1 AGE=0001 FR=0002 EN=0 Z=1 F=0 E1=FFFFFFFC E2=00000000 X1=0000 X2=0000 P=00000004 T=FB C=03C0 DZ=0014 G=07 L=05 SEQ=1234 RXE=0000 Q=F B=0 CS=1234 AE=0E10 AG=008F RL=0064 RT=00C8 RR=04B0 RB=02BC CA=1233 RA=1234 CB=1 MT=FFFFFFFB ME=FFFFFFF7 MS=7 MB=1 OK=0 RD=0 SP=03 MC=FFFFFFFC CR=0 RJ=6 AC=0002 DC=0001 CC=ABCD KC=1 MD=1 AR=1 KD=06 SL=02 RU=001E DS=1234 CZ=1 VE=FFFC AT=FB SR=7 AJ=A EL=0003 TS=12345678\015\n";
 tick(6);report_rst=1;
 for(j=0;j<2;j=j+1)begin
 line="";
 repeat(429)begin
 @(negedge report_tx);repeat(12)@(posedge clk);#1;
 for(i=0;i<8;i=i+1)begin b[i]=report_tx;repeat(8)@(posedge clk);#1;end
 check(report_tx==1,"serializer UART stop bit");line={line,b};
 report_x=1000;report_y=700;
 end
 if(j==0)begin $display("LENGTHS actual=%0d expected=%0d",line.len(),expected.len());for(i=0;i<line.len();i=i+1)if(line[i]!=expected[i])$display("DIFF %0d %02h %02h",i,line[i],expected[i]);check(line==expected,"all fields match one coherent snapshot");end
 else begin expected="VIS=5 BX=03E8 BY=02BC V=1 AGE=0001 FR=0002 EN=0 Z=1 F=0 E1=FFFFFFFC E2=00000000 X1=0000 X2=0000 P=00000004 T=FB C=03C0 DZ=0014 G=07 L=05 SEQ=1234 RXE=0000 Q=F B=0 CS=1234 AE=0E10 AG=008F RL=0064 RT=00C8 RR=04B0 RB=02BC CA=1233 RA=1234 CB=1 MT=FFFFFFFB ME=FFFFFFF7 MS=7 MB=1 OK=0 RD=0 SP=03 MC=FFFFFFFC CR=0 RJ=6 AC=0002 DC=0001 CC=ABCD KC=1 MD=1 AR=1 KD=06 SL=02 RU=001E DS=1234 CZ=1 VE=FFFC AT=FB SR=7 AJ=A EL=0003 TS=12345678\015\n";
 check(line==expected,"next report receives updated snapshot");end
 $display("UART_FIXTURE %s",line);
 end
 $display("PASS serializer: actual 8N1 waveform, all 429 bytes, signed fields and snapshot isolation");report_done=1;
end
// Unequal source/destination clocks, real request/ack and source mutation during transfer.
reg sc=0;always #7 sc=~sc;
reg dc=0;always #11 dc=~dc;
reg cr=0,sf=0;reg [31:0] sd=0;wire df;wire [31:0] dd;
vision_cdc_bridge #(.WIDTH(32)) cdc(sc,cr,sf,sd,dc,cr,df,dd);
integer deliveries=0;reg cdc_done=0;
always @(posedge dc)begin #1;if(df)begin
 deliveries=deliveries+1;
 if(deliveries==1)check(dd==32'h1234abcd,"CDC held data not torn while live source changes");
 if(deliveries==2)check(dd==32'hface5678,"CDC later accepted frame");
 end end
initial begin
 #20;cr=1;@(negedge sc);sd=32'h1234abcd;sf=1;@(negedge sc);sd=32'hdeadbeef;
 @(negedge sc);sf=0;repeat(15)@(negedge sc);check(deliveries==1,"busy CDC frame dropped safely");
 sd=32'hface5678;sf=1;@(negedge sc);sf=0;repeat(15)@(negedge sc);check(deliveries==2,"second CDC transfer");
 cr=0;#30;check(dd==0 && !df,"CDC common reset clears pending snapshot");
 $display("PASS CDC: unequal clocks, held snapshots, busy drop, subsequent frame and reset");cdc_done=1;
end
initial begin wait(monitor_done && report_done && cdc_done);$display("ALL_TESTS_PASS");$finish;end
initial begin #15000000;$fatal(1,"simulation timeout");end
endmodule

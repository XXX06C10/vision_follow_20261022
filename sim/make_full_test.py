from pathlib import Path
root=Path(__file__).resolve().parents[1]
old=(root.parent/'vision_control_20261020/sim/tb_full.v').read_text(encoding='utf-8')
tail=old[old.index('// Separate serializer test'):]
tail=tail.replace('VIS=2','VIS=3').replace('repeat(241)','repeat(329)').replace('all 241 bytes','all 329 bytes')
tail=tail.replace('CB=1\\015\\n','CB=1 MT=FFFFFFFB ME=FFFFFFF7 MS=7 MB=1 OK=0 RD=0 SP=03 MC=FFFFFFFC CR=0 RJ=6 AC=0002 DC=0001\\015\\n')
tail=tail.replace(".quality(4'hf),.bad(1'b0)",".locked_target(-32'sd5),.move_error(-32'sd9),.move_state(4'd7),.move_busy(1'b1),.reached(1'b0),.request_ready(1'b0),.spent(8'd3),.command_position(-32'sd4),.cancel_reason(4'd0),.reject_reason(4'd6),.accepted_count(16'd2),.completed_count(16'd1),\n .quality(4'hf),.bad(1'b0)")
head='''`timescale 1ns/1ps
module tb_full;
reg clk=0;always #5 clk=~clk;
reg rst=0,rx=1,frame=0,auto_frames=1,confirmed=1,bad=0,settings_busy=0;
reg [3:0] keys=4'b1110,confidence=15;
reg [10:0] bx=1660,by=500;reg [1:0] enc1=0,phase=0;
wire tx,en,dir,step;wire [3:0] led;
wire [15:0] cs,ae,ag,rl,rt,rr,rb;
vision_monitor #(.DEBOUNCE_CYCLES(4),.ENABLE_CYCLES(16),.POR_CYCLES(8),
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
 click(2);tick(100);check(pulses==0,"K3 has no motion function");
 click(1);check(dut.busy && dut.locked_target==5,"K2 locks right visual target");
 bx=260;tick(100);check(dut.target==-5 && dut.locked_target==5,"live and locked targets differ during action");
 wait(dut.reached);check(dut.position==3 && dut.errors1==0 && dut.completed_count==1,"actual AB feedback reaches right target");
 saved_pulses=pulses;tick(1000);check(pulses==saved_pulses,"no continuous follow");
 bx=960;tick(100);click(1);wait(dut.reached && dut.completed_count==2);check(dut.position<=2 && dut.position>=-2,"center position deadband");
 bx=260;tick(100);click(1);wait(dut.reached && dut.completed_count==3);check(dut.position<=-3 && dut.position>=-7,"left target real AB feedback");
 bx=960;tick(100);click(1);wait(dut.reached && dut.completed_count==4);
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
 packet(3,1920,20,7,5,0,1);packet(3,960,201,7,5,0,1);packet(3,960,20,3,5,0,1);packet(3,960,20,13,5,0,1);packet(3,960,20,7,6,0,1);
 check(dut.cfg_sequence==2,"FPGA parameter bounds preserved");textline("P00");tick(5100);sendbyte(10);tick(20);
 packet(4,960,20,7,5,0,0);check(dut.cfg_sequence==2,"no partial commit");sendbyte(10);tick(20);check(dut.cfg_sequence==4,"LF atomic commit");
 textline("MA\\nM+\\nMS\\n");check(!en && !dut.busy,"legacy motor messages ignored");
 wait(monitor_last.len()==329 && monitor_last.substr(141,144)=="0004");
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
'''
(root/'sim/tb_full.v').write_text(head+tail,encoding='utf-8')
print('Built real-AB motion integration and 329-byte wire snapshot fixtures')

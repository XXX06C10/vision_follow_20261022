`timescale 1ns/1ps
module tb_camera;
reg clk=0,camclk=0,roiclk=0;always #5 clk=~clk;always #11 camclk=~camclk;always #7 roiclk=~roiclk;
reg rst=0,rx=1,sof=0;
wire [15:0] cs,ae,ag,rl,rt,rr,rb;wire [15:0] errors;
wire req,change,busy;wire [15:0] sensor_ae,sensor_ag,ca,ra;
wire [10:0] left,top,right,bottom;
wire cam_done,ae_done,scl;tri1 sda;
reg [3:0] keys=15;wire uart_tx,motor_en,motor_dir,motor_step;
vision_monitor #(.DEFAULT_MODE(0),.DEFAULT_GAIN(7),.DEFAULT_LIMIT(5),.UART_DIV(8),.PACKET_TIMEOUT_CYCLES(5000),.DEBOUNCE_CYCLES(4),
 .ENABLE_CYCLES(16),.POR_CYCLES(8),.REPORT_CYCLES(25000)) monitor(
 .clk(clk),.external_rst_n(rst),.key_n(keys),.encoder_j1(2'b0),.encoder_j2(2'b0),
 .frame(1'b0),.ball_x(11'd0),.ball_y(11'd0),.confirmed(1'b0),.bad_frame(1'b0),.confidence(4'd0),
 .rx(rx),.tx(uart_tx),.motor_en(motor_en),.motor_dir(motor_dir),.motor_step(motor_step),
 .camera_applied_seq(ca),.roi_applied_seq(ra),.settings_busy(busy),
 .cam_sequence(cs),.exposure(ae),.sensor_gain(ag),.roi_left(rl),.roi_top(rt),.roi_right(rr),.roi_bottom(rb));
assign errors=monitor.rx_errors;
always @(posedge clk)if(motor_step!==0 || motor_dir!==0)$fatal(1,"camera settings caused physical motion");
string last_line="";
initial begin : serial_capture
 reg [7:0] b;integer i;string line;line="";
 forever begin
 @(negedge uart_tx);repeat(12)@(posedge clk);#1;
 for(i=0;i<8;i=i+1)begin b[i]=uart_tx;repeat(8)@(posedge clk);#1;end
 check(uart_tx==1,"camera telemetry UART stop");line={line,b};
 if(b==10)begin last_line=line;line="";end
 end
end
vision_camera_control control(clk,rst,camclk,rst,roiclk,rst,sof,cam_done,ae_done,
 cs,ae,ag,rl,rt,rr,rb,req,sensor_ae,sensor_ag,left,top,right,bottom,change,ca,ra,busy);
// Actual reference configuration sequencer and AE register table, faster SCL only in simulation.
uicfgcs500 #(.CLK_DIV(7)) sensor(camclk,rst,req,sensor_ae,sensor_ag,scl,sda,cam_done,ae_done);
// Camera ACK model: release SDA except for the reference controller's byte-ACK state.
assign sda=(sensor.uii2c_inst.IIC_S==3) ? 1'b0 : 1'bz;
task tick(input integer n);begin repeat(n)@(negedge clk);end endtask
task check(input integer ok,input string text);begin if(!ok)$fatal(1,"FAIL %s",text);end endtask
task sendbyte(input [7:0] b);integer i;begin rx=0;tick(8);for(i=0;i<8;i=i+1)begin rx=b[i];tick(8);end rx=1;tick(8);end endtask
task packet(input [15:0] seq,e,g,l,t,r,b,input integer badsum,lf);
reg [111:0] data;reg [7:0] checksum;integer i;string s;begin
 data={seq,e,g,l,t,r,b};checksum=0;
 for(i=0;i<14;i=i+1)checksum=checksum^(data>>(i*8));if(badsum)checksum=checksum^1;
 s=$sformatf("V%028h%02h",data,checksum);for(i=0;i<s.len();i=i+1)sendbyte(s[i]);
 if(lf)sendbyte(10);tick(16);
end endtask
task frame_start;begin @(negedge roiclk);sof=1;@(negedge roiclk);sof=0;end endtask
integer writes=0,requests=0;reg saw_low=0;reg [15:0] locked_ae,locked_ag;
always @(posedge camclk)begin
 if(req)begin requests=requests+1;locked_ae=sensor_ae;locked_ag=sensor_ag;saw_low=0;end
 if(!ae_done)saw_low=1;
 if(control.state!=0)begin
  check(sensor_ae==locked_ae && sensor_ag==locked_ag,"AE/AG held while reference I2C writes run");
 end
 if(sensor.iic_req && sensor.O_cfg_done)begin
  writes=writes+1;
  case(sensor.reg_index)
   0:check(sensor.wr_data[31:8]=={4'b0,sensor_ae[15:12],16'h003e},"AE register 3e00 packing");
   1:check(sensor.wr_data[31:8]=={sensor_ae[11:4],16'h013e},"AE register 3e01 packing");
   2:check(sensor.wr_data[31:8]=={sensor_ae[3:0],4'b0,16'h023e},"AE register 3e02 packing");
   3:check(sensor.wr_data[31:8]=={sensor.w_gain_val[15:8],16'h083e},"gain register 3e08 packing");
   4:check(sensor.wr_data[31:8]=={sensor.w_gain_val[7:0],16'h093e},"gain register 3e09 packing");
  endcase
 end
end
initial begin
 tick(5);rst=1;tick(50);check(ae==3600 && ag==143,"maximum startup request");
 check(busy,"startup completion not falsely acknowledged");frame_start;
 wait(cam_done && ae_done);wait(requests==1);wait(!ae_done);wait(ae_done);tick(50);
 check(!busy && ca==0 && ra==0 && writes>0,"startup maximum AE/AG actually traverses reference I2C flow");
 keys[0]=0;tick(24);keys[0]=1;tick(80);check(motor_en,"K1 still latches enable alongside camera settings");
 packet(1,3000,120,100,150,1000,650,0,0);check(cs==0 && ae==3600,"no commit before LF");
 sendbyte(10);tick(50);check(cs==1 && ae==3000 && ag==120,"atomic camera command accepted");
 check(busy && ra==0 && left==0,"ROI waits for frame boundary, cannot acknowledge early");
 frame_start;tick(50);check(ra==1 && left==100 && top==150 && right==1000 && bottom==650,"ROI coherent commit at new frame");
 wait(!busy);check(ca==1 && sensor_ae==3000 && sensor_ag==120,"sensor completion acknowledged");
 packet(2,3601,143,0,0,1279,719,0,1);check(cs==1,"exposure upper bound");
 packet(2,1,143,0,0,1279,719,0,1);check(cs==1,"exposure lower bound");
 packet(2,3600,144,0,0,1279,719,0,1);check(cs==1,"gain upper bound");
 packet(2,3600,15,0,0,1279,719,0,1);check(cs==1,"gain lower bound");
 packet(2,3600,143,1000,0,1000,719,0,1);check(cs==1,"zero width ROI");
 packet(2,3600,143,0,0,1280,719,0,1);check(cs==1,"ROI x bound");
 packet(2,3600,143,0,0,1279,720,0,1);check(cs==1,"ROI y bound");
 packet(2,3600,143,0,700,1279,650,0,1);check(cs==1,"inverted ROI");
 packet(2,3600,143,0,0,1279,719,1,1);check(cs==1,"camera checksum");
 packet(2,3600,143,0,0,1279,719,0,0);tick(5100);sendbyte(10);tick(30);check(cs==1,"camera packet timeout");
 packet(2,2,16,0,0,1279,719,0,1);frame_start;wait(!busy);check(ca==2 && ra==2,"minimum exposure/gain and maximum ROI");
 // Send another valid setting while an earlier I2C transaction is busy; only latest pending may win.
 packet(3,2800,100,10,10,1100,700,0,1);wait(control.state!=0);
 packet(4,3600,143,20,30,1000,600,0,1);frame_start;wait(!busy);check(ca==4 && ra==4 && sensor_ae==3600 && sensor_ag==143,"latest changed settings delivered while CDC/I2C busy");
 check(errors==10,"one error per invalid camera packet");
 wait(last_line.len()==429 && last_line.substr(166,169)=="0004" && last_line.substr(238,238)=="0");
 check(last_line.substr(222,225)=="0004" && last_line.substr(230,233)=="0004","real UART confirms both applied sequence numbers");
 $display("UART_CAMERA %s",last_line);
 check(motor_en,"camera transactions cannot toggle motor enable");keys[3]=0;#1;check(!motor_en,"K4 still immediate");tick(20);keys[3]=1;tick(50);check(!motor_en,"no restart after K4");
 rst=0;tick(10);check(busy && left==0 && right==1279 && ae==3600,"reset clears acknowledgements, restores bright defaults");
 $display("CAMERA_CONTROL_PASS: real UART bounds/checksum/atomic/timeout, 3-clock coherent mailboxes, frame ROI commit, actual reference I2C startup and updates");$finish;
end
initial begin #50000000;$fatal(1,"camera simulation timeout");end
endmodule

`timescale 1ns/1ps
// An assumed rolling-ball plant, NOT a measured mechanical calibration.
module plant_case #(parameter ID=0,parameter real INITIAL=140.0,COUPLING=0.06)(output reg done=0);
 reg clk=0;always #5 clk=~clk;reg rst=0,frame=0,simulate=0;
 reg [3:0] keys=15;reg [10:0] bx=960;reg [1:0] enc1=0,phase=0;
 wire en,dir,step,tx;wire [3:0] led;wire [15:0] cs,ae,ag,rl,rt,rr,rb;
 // All active production phase cycles /1000; outer frame33ms and100ms control.
 vision_monitor #(.DEBOUNCE_CYCLES(1000),.ENABLE_CYCLES(5000),.POR_CYCLES(8),.UART_DIV(8),
 .MS_CYCLES(50),.CONTROL_MS(100),.SETUP_CYCLES(1000),.HALF_CYCLES(125),.SETTLE_CYCLES(2500),.TIMEOUT_CYCLES(250000)) dut(
 clk,rst,keys,enc1,2'b0,frame,bx,11'd500,1'b1,1'b0,4'd15,1'b1,tx,en,dir,step,led,
 16'd0,16'd0,1'b0,cs,ae,ag,rl,rt,rr,rb);
 integer fc=0,frames=0,commands=0,encoded=0,goal=0,difference=0,i=0,trace;
 real error=0.0,velocity=0.0,peak=0.0,tail_square=0.0;integer tail=0;
 task tick(input integer n);begin repeat(n)@(negedge clk);end endtask
 task click(input integer key);begin keys[key]=0;tick(1500);keys[key]=1;tick(1500);end endtask
 always @(posedge step)begin
  commands=commands+(dir ? 1 : -1);goal=commands*32/25;difference=goal-encoded;
  for(i=0;i<(difference<0 ? -difference : difference);i=i+1)begin
   repeat(6)@(negedge clk);phase=phase+(difference>0 ? 2'd3 : 2'd1);
   case(phase)0:enc1=0;1:enc1=1;2:enc1=3;3:enc1=2;endcase
  end
  encoded=goal;
 end
 always @(negedge clk)begin
  frame<=0;
  if(rst)begin
   if(fc==1649)begin
    fc=0;frame<=1;
    if(simulate)begin
     frames=frames+1;
     // Negative measured trend assumption: positive P accelerates ball left.
     velocity=0.975*velocity-COUPLING*$itor($signed(dut.position));error=error+velocity;
     if(frames==350)begin error=error+(ID==1 ? -80.0 : 80.0);velocity=velocity+(ID==1 ? -2.0 : 2.0);end
     if((error<0 ? -error : error)>peak)peak=error<0 ? -error : error;
     if(error>800 || error< -800)$fatal(1,"plant %0d diverged, error=%f",ID,error);
     if(dut.fault!=0)$fatal(1,"plant %0d false actuator fault=%0d",ID,dut.fault);
     bx<=960+$rtoi(error);
     if(frames%10==0)$fdisplay(trace,"%0d,%f,%f,%0d,%0d,%0d,%0d",frames,error,velocity,$signed(dut.position),$signed(dut.drive_target),$signed(dut.target),dut.fault);
     if(frames>750)begin tail_square=tail_square+error*error;tail=tail+1;end
     if(frames==850)begin
      if(!dut.auto_run || error>80 || error< -80 || velocity>1.0 || velocity< -1.0)$fatal(1,"plant %0d did not settle: error=%f velocity=%f AR=%b",ID,error,velocity,dut.auto_run);
      $display("PLANT_MODEL_PASS id=%0d coupling=%f peak=%f final_error=%f final_velocity=%f rms_tail=%f P=%0d",ID,COUPLING,peak,error,velocity,$sqrt(tail_square/tail),$signed(dut.position));
      $fclose(trace);done=1;simulate=0;
     end
    end
   end else fc=fc+1;
  end
 end
 initial begin
  trace=$fopen($sformatf("sim/results/plant_trace_%0d.csv",ID),"w");$fdisplay(trace,"frame,error_pixels,velocity_pixels_per_frame,P,AT,T,fault");
  tick(5);rst=1;tick(5000);click(2);click(0);wait(dut.request_ready);click(1);wait(dut.auto_run);
  error=INITIAL;bx=960+$rtoi(error);simulate=1;
 end
endmodule
module tb_plant;
 wire a,b,c;plant_case #(.ID(0),.INITIAL(140.0),.COUPLING(0.06)) p0(a);
 plant_case #(.ID(1),.INITIAL(-180.0),.COUPLING(0.04)) p1(b);
 plant_case #(.ID(2),.INITIAL(90.0),.COUPLING(0.08)) p2(c);
 initial begin wait(a && b && c);$display("PLANT_ALL_PASS: three assumed rolling-ball models, production timing ratios,1.28 encoder and disturbance rejection; physical tuning still required");$finish;end
 initial begin #25000000;$fatal(1,"plant timeout");end
endmodule

// Source: vision_asym_limits.v
// One mechanical reference per load. Later K3 changes the control center, never
// moves this envelope. Counts use4096/rev; MC uses3200pulses/rev.
module vision_asym_limits(
 input wire clk,rst_n,capture,ready,motor_en,input wire signed [31:0] position,command_position,
 output reg qualified=0,output reg signed [31:0] anchor=0,command_anchor=0,
 output wire signed [31:0] work_min,work_max,feedback_min,feedback_max,command_min,command_max
);
 assign work_min=qualified ? anchor-182 : -64;
 assign work_max=qualified ? anchor+364 : 64;
 assign feedback_min=qualified ? anchor-204 : -64;
 assign feedback_max=qualified ? anchor+398 : 64;
 assign command_min=qualified ? command_anchor-159 : -64;
 assign command_max=qualified ? command_anchor+310 : 64;
 always @(posedge clk)begin
  if(!rst_n)begin qualified<=0;anchor<=0;command_anchor<=0;end
  else if(capture && ready && motor_en && !qualified && position>=-64 && position<=64)begin
   anchor<=position;command_anchor<=command_position;qualified<=1;
  end
 end
endmodule

// Source: vision_region_hold.v
// Built-in regional return. Custom C profiles keep their original controller.
// Region entry is position AND velocity; hysteresis prevents pixel hunting.
module vision_region_hold #(parameter WIDE_RANGE=0,NEUTRAL_BITS=8)(
 input wire clk,rst_n,run,updated,valid,stale,clear,neutral_capture,polarity,
 input wire [10:0] x,input wire [15:0] center,input wire [7:0] limit,
 input wire signed [31:0] velocity_q4,work_min,work_max,input wire signed [NEUTRAL_BITS-1:0] neutral,
 output reg signed [31:0] target_q8=0,output reg changed=0,
 output reg holding=0,output reg [1:0] phase=2,
 output wire [15:0] max_hz,slew_rate,output wire signed [31:0] filtered_velocity_q4
);
 reg signed [17:0] vh1=0,vh2=0,vh3=0;
 wire signed [19:0] pair_a=$signed(velocity_q4[17:0])+$signed(vh1);
 wire signed [19:0] pair_b=$signed(vh2)+$signed(vh3);
 assign filtered_velocity_q4=($signed(pair_a)+$signed(pair_b))>>>2;
 wire signed [31:0] control_velocity=filtered_velocity_q4;
 wire signed [12:0] error=$signed({1'b0,x})-$signed({1'b0,center});
 wire [12:0] distance=error<0 ? -error : error;
 wire [31:0] speed=control_velocity<0 ? -control_velocity : control_velocity;
 reg [3:0] pipe=0;
 reg signed [31:0] pd_stage=0,raw_stage=0,relative_stage=0,absolute_stage=0;
 reg signed [12:0] trim_stage=0;reg signed [NEUTRAL_BITS-1:0] neutral_stage=0;reg signed [17:0] cap_stage=0;reg hold_stage=0;
 reg [2:0] entry_frames=0;reg [1:0] exit_frames=0;
 reg signed [12:0] trim_q8=0;reg [12:0] range_distance=0;
 wire enter_now=distance<=15 && speed<=240;
 wire exit_now=distance>25 || speed>960;
 wire leaving=holding && (distance>45 || (exit_now && exit_frames>=1));
 wire entering=!holding && enter_now && entry_frames>=2;
 // Pixel position gains:1/4 far,1/8 mid,1/16 near. Velocity damping is
 //3/8,1/4,3/16 count/(pixel/s). No identified physical model is claimed.
 wire signed [23:0] proportional=distance>96 ? $signed(error)*24'sd64 : (distance>32 ? $signed(error)*24'sd32 : $signed(error)*24'sd16);
 wire signed [31:0] derivative=distance>96 ? $signed(control_velocity)*32'sd6 : (distance>32 ? $signed(control_velocity)*32'sd4 : $signed(control_velocity)*32'sd3);
 wire signed [31:0] pd=polarity ? -(proportional+derivative) : proportional+derivative;
 wire signed [31:0] raw_relative=(holding && !leaving) || entering ? trim_q8 : pd+$signed(trim_q8);
 wire signed [17:0] cap=$signed({1'b0,limit,8'b0});
 wire signed [31:0] relative=raw_relative>cap ? cap : (raw_relative< -cap ? -cap : raw_relative);
 wire signed [31:0] absolute=$signed(neutral)*256+relative;
 wire signed [31:0] bounded=absolute>15360 ? 15360 : (absolute< -15360 ? -15360 : absolute);
 wire signed [12:0] trim_next=trim_q8+((error>0)^polarity ? 13'sd8 : -13'sd8);
 assign max_hz=holding || range_distance<=64 ? 16'd50 : (range_distance<=128 ? 16'd100 : 16'd200);
 assign slew_rate=holding || range_distance<=64 ? 16'd80 : (range_distance<=128 ? 16'd100 : 16'd160);
 always @(posedge clk)begin
  if(!rst_n)begin target_q8<=0;changed<=0;holding<=0;phase<=2;entry_frames<=0;exit_frames<=0;trim_q8<=0;vh1<=0;vh2<=0;vh3<=0;range_distance<=0;pipe<=0;pd_stage<=0;raw_stage<=0;relative_stage<=0;absolute_stage<=0;trim_stage<=0;neutral_stage<=0;cap_stage<=0;hold_stage<=0;end
  else begin
   changed<=0;pipe<={pipe[2:0],updated && run && valid && !stale};
   if(pipe[0])raw_stage<=hold_stage ? $signed(trim_stage) : pd_stage+$signed(trim_stage);
   if(pipe[1])relative_stage<=WIDE_RANGE ? raw_stage : (raw_stage>cap_stage ? cap_stage : (raw_stage< -cap_stage ? -cap_stage : raw_stage));
   if(pipe[2])absolute_stage<=$signed(neutral_stage)*256+relative_stage;
   if(pipe[3] && run && valid && !stale)begin target_q8<=absolute_stage>(WIDE_RANGE ? work_max*256 : 15360) ? (WIDE_RANGE ? work_max*256 : 15360) : (absolute_stage<(WIDE_RANGE ? work_min*256 : -15360) ? (WIDE_RANGE ? work_min*256 : -15360) : absolute_stage);changed<=1;end
   if(updated && run && valid && !stale && !clear && !neutral_capture)begin
    vh1<=velocity_q4[17:0];vh2<=vh1;vh3<=vh2;
    pd_stage<=pd;trim_stage<=trim_q8;neutral_stage<=neutral;cap_stage<=cap;hold_stage<=(holding && !leaving) || entering;range_distance<=distance;
    if(!holding)begin
     exit_frames<=0;
     if(enter_now)begin if(entry_frames<3)entry_frames<=entry_frames+1'b1;end else entry_frames<=0;
     if(entering)begin holding<=1;entry_frames<=0;phase<=3;end
     else if((error>0 && control_velocity<0) || (error<0 && control_velocity>0))phase<=1;
     else phase<=distance<=64 ? 2 : 0;
     // K3 is the trusted level. Without measured beam dynamics, position
     // error alone must not teach a tilted "neutral" that rolls the ball away.
     trim_q8<=0;
    end else begin
     entry_frames<=0;phase<=3;
     if(exit_now)begin if(exit_frames<2)exit_frames<=exit_frames+1'b1;end else exit_frames<=0;
     if(leaving)begin holding<=0;exit_frames<=0;phase<=2;end
    end
   end
   if(!valid || stale)begin vh1<=0;vh2<=0;vh3<=0;pipe<=0;entry_frames<=0;exit_frames<=0;end
   if(!run)begin vh1<=0;vh2<=0;vh3<=0;pipe<=0;holding<=0;phase<=2;entry_frames<=0;exit_frames<=0;end
   if(clear || neutral_capture)begin vh1<=0;vh2<=0;vh3<=0;pipe<=0;target_q8<=$signed(neutral)*256;holding<=0;phase<=2;entry_frames<=0;exit_frames<=0;trim_q8<=0;changed<=0;end
  end
 end
endmodule

// Source: vision_region_probe.v
// Dedicated finite measurement image. C selection 2=static,3=neutral,1=response.
// Configuration arms a profile only; every start needs a physical K2 edge.
module vision_region_probe #(parameter MS_CYCLES=50000)(
 input wire clk,rst_n,key_event,k4_n,enable_request,settings_ready,valid,stale,frame,changed,
 input wire center_confirmed,neutral_ready,core_ready,core_busy,event_overflow,
 input wire [7:0] selection,input wire signed [7:0] neutral,
 input wire signed [31:0] position,input wire [3:0] fault,
 input wire [31:0] time_ms,input wire [10:0] x,y,
 input wire [15:0] center,roi_left,roi_right,roi_top,roi_bottom,
 output reg run=0,request=0,output wire cancel,
 output reg [3:0] status=0,phase=0,reason=0,reject=0,
 output reg signed [7:0] target=0
);
 reg [31:0] mc=0;reg [13:0] age_ms=0;reg [9:0] band_ms=0,hold_ms=0;
 reg issued=0,holding=0,previous_run=0,velocity_ready=0;
 reg [31:0] previous_frame=0;reg [10:0] previous_x=0;
 wire tick=mc==MS_CYCLES-1;
 wire static_profile=selection==2;
 wire response_profile=selection==1;
 wire signed [16:0] pe=$signed(position)-$signed(target);
 wire in_band=pe<=2 && pe>=-2;
 wire signed [16:0] xe=$signed({1'b0,x})-$signed({1'b0,center});
 wire signed [12:0] dx=$signed({1'b0,x})-$signed({1'b0,previous_x});
 wire [12:0] abs_dx=dx<0 ? -dx : dx;
 wire [31:0] dt=time_ms-previous_frame;
 wire excessive_speed=frame && velocity_ready && dt>0 && abs_dx*32'd1000>dt*32'd150;
 // Ball telemetry is1920x1080; ROI commands are1280x720 sensor pixels.
 wire [16:0] x2={6'd0,x}<<1,y2={6'd0,y}<<1;
 wire [16:0] rl3=roi_left*17'd3,rr3=roi_right*17'd3,rt3=roi_top*17'd3,rb3=roi_bottom*17'd3;
 wire near_edge=x2<=rl3+48 || x2+48>=rr3 || y2<=rt3+48 || y2+48>=rb3;
 wire ball_guard=!static_profile && (!valid || stale || near_edge || xe>120 || xe< -120 || excessive_speed);
 wire abort_now=run && (!k4_n || (!static_profile && !enable_request) || fault!=0 || changed || event_overflow || ball_guard || !settings_ready || key_event);
 assign cancel=(previous_run && !run) || abort_now;
 // Twelve legs: +/0/-/0, -/0/+/0, +/0/-/0. Each new leg has its own finite request.
 function signed [7:0] leg_target;
  input [3:0] leg;input signed [7:0] n;
  begin case(leg)
   0,6,8:leg_target=n+8;
   2,4,10:leg_target=n-8;
   default:leg_target=n;
  endcase end
 endfunction
 always @(posedge clk)begin
  if(!rst_n)begin run<=0;request<=0;status<=0;phase<=0;reason<=0;reject<=0;target<=0;mc<=0;age_ms<=0;band_ms<=0;hold_ms<=0;issued<=0;holding<=0;previous_run<=0;velocity_ready<=0;previous_frame<=0;previous_x<=0;end
  else begin
   request<=0;previous_run<=run;
   if(frame)begin previous_frame<=time_ms;previous_x<=x;velocity_ready<=valid && !stale;end
   if(tick)begin
    mc<=0;
    if(run)begin
     if(age_ms<16383)age_ms<=age_ms+1'b1;
     if(!response_profile)begin
      if(age_ms>=9999)begin run<=0;status<=2;reason<=10;end
     end else if(!holding)begin
      if(issued && core_ready && !core_busy && in_band)begin if(band_ms<100)band_ms<=band_ms+1'b1;end else band_ms<=0;
      if(band_ms>=99 && issued && core_ready && in_band)begin holding<=1;hold_ms<=0;end
      else if(age_ms>=999)begin run<=0;status<=4;reason<=11;end
     end else begin
      if(!in_band || !core_ready)begin run<=0;status<=4;reason<=12;end
      else if(hold_ms>=(phase[0] ? 599 : 399))begin
       if(phase==11)begin run<=0;status<=2;reason<=10;end
       else begin phase<=phase+1'b1;target<=leg_target(phase+1'b1,neutral);issued<=0;holding<=0;age_ms<=0;band_ms<=0;hold_ms<=0;end
      end else hold_ms<=hold_ms+1'b1;
     end
    end
   end else mc<=mc+1'b1;
   if(run && response_profile && !issued && core_ready && !abort_now)begin request<=1;issued<=1;end
   if(key_event && !run && !changed)begin
    reject<=0;
    if(selection==0 || selection>3)reject<=12;
    else if(fault!=0)reject<=9;
    else if(event_overflow)reject<=14;
    else if(!static_profile && (!enable_request || !core_ready))reject<=1;
    else if(!settings_ready)reject<=5;
    else if(!static_profile && (!center_confirmed || !neutral_ready))reject<=10;
    else if(!static_profile && ball_guard)reject<=3;
    else if(response_profile && (neutral>52 || neutral< -52 || position>60 || position< -60))reject<=13;
    else begin run<=1;status<=1;phase<=0;reason<=0;age_ms<=0;band_ms<=0;hold_ms<=0;issued<=0;holding<=0;target<=response_profile ? leg_target(0,neutral) : position[7:0];end
   end
   if(abort_now)begin
    run<=0;request<=0;status<=4;
    if(fault!=0)reason<=6;else if(!k4_n)reason<=2;else if(!enable_request)reason<=1;
    else if(changed || !settings_ready)reason<=5;else if(event_overflow)reason<=14;
    else if(key_event)reason<=7;else if(stale)reason<=4;else if(!valid)reason<=3;
    else if(near_edge)reason<=13;else if(excessive_speed)reason<=15;else reason<=9;
   end
   // A fresh configuration/reference never inherits a previous DONE.
   if(changed)begin run<=0;request<=0;status<=0;phase<=0;reason<=5;reject<=0;issued<=0;holding<=0;age_ms<=0;band_ms<=0;hold_ms<=0;velocity_ready<=0;end
  end
 end
endmodule

// Source: vision_probe_events.v
// E1:26 payload bytes + XOR8, hex, CRLF =58 wire bytes. FIFO overflow is sticky.
// Sequence increments on every capture attempt, including dropped events.
module vision_probe_events #(parameter UART_DIV=434,MS_CYCLES=50000)(
 input wire clk,rst_n,enabled,grant,input wire [31:0] time_ms,
 input wire step,dir,input wire signed [31:0] position,command_position,
 input wire [3:0] state,input wire [7:0] pending,input wire signed [15:0] commands,delta,
 input wire [7:0] diagnostic_flags,input wire [3:0] phase,status,
 input wire [7:0] selection,input wire [3:0] reason,input wire changed,
 output wire tx,output reg busy=0,output wire wants,output reg overflow=0
);
 reg [207:0] memory[0:127];
 reg [7:0] wr=0,rd=0;reg [15:0] event_seq=0;
 reg old_step=0,old_dir=0;reg [31:0] old_position=0;
 reg [3:0] old_state=0,old_phase=0,old_status=0,old_reason=0;
 reg [7:0] old_pending=0,old_diag=0,old_selection=0;
 reg [31:0] heartbeat=0;
 wire [7:0] events={dir!=old_dir,changed,time_ms-heartbeat>=100,
     pending!=old_pending || diagnostic_flags!=old_diag,
     phase!=old_phase || status!=old_status || reason!=old_reason || selection!=old_selection,
     state!=old_state,position!=old_position,step && !old_step};
 wire capture=enabled && events!=0;
 wire full=wr[7]!=rd[7] && wr[6:0]==rd[6:0];
 assign wants=enabled && wr!=rd;
 wire [207:0] payload={event_seq,time_ms,events,position,command_position,{4'd0,state},pending,commands,delta,diagnostic_flags,{4'd0,phase},{4'd0,status},selection,{4'd0,reason}};
 reg [207:0] snapshot=0;reg [7:0] checksum=0;
 reg loading=0,active=0;reg [5:0] index=0;reg [3:0] bit_index=0;
 reg [15:0] baud=0;reg [9:0] shift=1023;
 assign tx=shift[0];
 integer j;reg [7:0] folded;
 always @*begin folded=0;for(j=0;j<26;j=j+1)folded=folded^snapshot[j*8+:8];end
 function [7:0] hex_char;input [3:0] n;begin hex_char=n<10 ? "0"+n : "A"+n-10;end endfunction
 function [7:0] byte_at;input [5:0] at;begin
  if(at==0)byte_at="E";else if(at==1)byte_at="1";
  else if(at<54)byte_at=hex_char(snapshot[207-(at-2)*4 -:4]);
  else if(at==54)byte_at=hex_char(checksum[7:4]);else if(at==55)byte_at=hex_char(checksum[3:0]);
  else if(at==56)byte_at=13;else byte_at=10;
 end endfunction
 always @(posedge clk)begin
  if(!rst_n)begin wr<=0;rd<=0;event_seq<=0;overflow<=0;old_step<=0;old_dir<=0;old_position<=0;old_state<=0;old_phase<=0;old_status<=0;old_reason<=0;old_pending<=0;old_diag<=0;old_selection<=0;heartbeat<=0;busy<=0;loading<=0;active<=0;index<=0;bit_index<=0;baud<=0;shift<=1023;snapshot<=0;checksum<=0;end
  else begin
   old_step<=step;old_dir<=dir;old_position<=position;old_state<=state;old_phase<=phase;old_status<=status;old_reason<=reason;old_pending<=pending;old_diag<=diagnostic_flags;old_selection<=selection;
   if(capture)begin
    event_seq<=event_seq+1'b1;heartbeat<=time_ms;
    if(full)overflow<=1;
    else begin memory[wr[6:0]]<=payload;wr<=wr+1'b1;end
   end
   // Registered read, separate write/read addresses: block RAM implementation.
   if(!busy && wants && grant)begin snapshot<=memory[rd[6:0]];rd<=rd+1'b1;busy<=1;loading<=1;index<=0;end
   else if(loading)begin checksum<=folded;loading<=0;end
   else if(busy)begin
    if(!active)begin shift<={1'b1,byte_at(index),1'b0};baud<=0;bit_index<=0;active<=1;end
    else if(baud==UART_DIV-1)begin
     baud<=0;shift<={1'b1,shift[9:1]};
     if(bit_index==9)begin active<=0;if(index==57)busy<=0;else index<=index+1'b1;end
     else bit_index<=bit_index+1'b1;
    end else baud<=baud+1'b1;
   end
  end
 end
endmodule


// Source: vision_cascade_config.v
// Additive C01 + fixed25-byte payload + XOR8 as hex + LF (54 bytes).
// P/V/S remain owned by the unchanged legacy receiver. Only line-start C accepted.
module vision_cascade_config #(parameter UART_DIV=434,TIMEOUT_CYCLES=5000000,DEFAULT_SELECTION=0,DEFAULT_PRESET=0)(
 input wire clk,rst_n,rx,
 output reg applied=0,output wire commit_now,output reg [15:0] config_seq=0,errors=0,
 output reg [7:0] selection=DEFAULT_SELECTION,output reg diagnostics=DEFAULT_PRESET,polarity=0,
 output reg [15:0] kp=(DEFAULT_PRESET ? 819 : (DEFAULT_SELECTION==0 ? 512 : 0)),kv=(DEFAULT_PRESET ? 64 : (DEFAULT_SELECTION==0 ? 128 : 0)),ki=(DEFAULT_PRESET ? 131 : 0),inner_kp=8192,feedforward=200,max_hz=(DEFAULT_PRESET ? 200 : 50),slew_rate=(DEFAULT_PRESET ? 200 : 80),
 output reg signed [15:0] bias=0,
 output reg [7:0] delay_ms=(DEFAULT_PRESET ? 20 : 0),amplitude=4
);
 reg s1=1,s2=1;reg [1:0] state=0;reg [15:0] baud=0;reg [2:0] bitno=0;
 reg [7:0] shift=0,value=0;reg received=0;
 always @(posedge clk)begin
  if(!rst_n)begin s1<=1;s2<=1;state<=0;baud<=0;bitno<=0;shift<=0;value<=0;received<=0;end
  else begin
   s1<=rx;s2<=s1;received<=0;
   case(state)
    0:if(!s2)begin baud<=UART_DIV/2-1;state<=1;end
    1:if(baud==0)begin if(!s2)begin state<=2;baud<=UART_DIV-1;end else state<=0;end else baud<=baud-1'b1;
    2:if(baud==0)begin shift[bitno]<=s2;baud<=UART_DIV-1;if(bitno==7)begin bitno<=0;state<=3;end else bitno<=bitno+1'b1;end else baud<=baud-1'b1;
    3:if(baud==0)begin state<=0;if(s2)begin value<=shift;received<=1;end end else baud<=baud-1'b1;
   endcase
  end
 end
 function [4:0] hex;
  input [7:0] c;
  begin if(c>="0" && c<="9")hex={1'b1,c[3:0]};else if(c>="A" && c<="F")hex={1'b1,c[3:0]+4'd9};else if(c>="a" && c<="f")hex={1'b1,c[3:0]+4'd9};else hex=0;end
 endfunction
 reg line_start=1,collecting=0,discard=0;reg [5:0] digits=0;reg [207:0] packet=0;reg [31:0] age=0;
 wire [4:0] digit=hex(value);integer j;reg [7:0] checksum;
 always @*begin checksum=0;for(j=1;j<26;j=j+1)checksum=checksum^packet[j*8+:8];end
 wire signed [15:0] packet_bias=packet[119:104];
 wire good=digits==52 && packet[207:200]==1 && packet[183:176]<=3 && packet[175:168]<=1 &&
 packet[167:152]<=16384 && packet[151:136]<=2048 && packet[135:120]<=3277 &&
 packet_bias>=-2048 && packet_bias<=2048 && packet[103:88]>=256 && packet[103:88]<=32768 &&
 packet[87:72]<=256 && (packet[71:56]==50 || packet[71:56]==100 || packet[71:56]==200) &&
 packet[55:48]<=120 && (packet[47:40]==4 || packet[47:40]==8 || (packet[47:40]==16 && packet[183:176]!=3)) &&
 packet[39:24]>=8 && packet[39:24]<=256 && packet[23:16]<=1 && packet[15:8]==0 && checksum==packet[7:0];
 assign commit_now=received && value==10 && collecting && age<TIMEOUT_CYCLES-1 && good;
 always @(posedge clk)begin
  if(!rst_n)begin applied<=0;config_seq<=0;errors<=0;selection<=DEFAULT_SELECTION;diagnostics<=DEFAULT_PRESET;polarity<=0;
   kp<=DEFAULT_PRESET ? 819 : (DEFAULT_SELECTION==0 ? 512 : 0);kv<=DEFAULT_PRESET ? 64 : (DEFAULT_SELECTION==0 ? 128 : 0);ki<=DEFAULT_PRESET ? 131 : 0;bias<=0;inner_kp<=8192;feedforward<=200;max_hz<=DEFAULT_PRESET ? 200 : 50;slew_rate<=DEFAULT_PRESET ? 200 : 80;delay_ms<=DEFAULT_PRESET ? 20 : 0;amplitude<=4;
   line_start<=1;collecting<=0;discard<=0;digits<=0;packet<=0;age<=0;
  end else begin
   applied<=0;
   if(collecting)begin
    if(age>=TIMEOUT_CYCLES-1)begin collecting<=0;discard<=1;age<=0;if(errors!=65535)errors<=errors+1'b1;end
    else age<=age+1'b1;
   end
   if(received)begin
    if(value==10)begin
     line_start<=1;discard<=0;
     if(collecting && age<TIMEOUT_CYCLES-1)begin
      collecting<=0;age<=0;
      if(good)begin applied<=1;config_seq<=packet[199:184];selection<=packet[183:176];diagnostics<=packet[168];
       kp<=packet[167:152];kv<=packet[151:136];ki<=packet[135:120];bias<=packet_bias;inner_kp<=packet[103:88];
       feedforward<=packet[87:72];max_hz<=packet[71:56];delay_ms<=packet[55:48];amplitude<=packet[47:40];slew_rate<=packet[39:24];polarity<=packet[16];
      end else if(errors!=65535)errors<=errors+1'b1;
     end
    end else if(line_start)begin
     line_start<=0;
     if(value=="C")begin collecting<=1;discard<=0;digits<=0;packet<=0;age<=0;end
    end else if(collecting && !discard && age<TIMEOUT_CYCLES-1)begin
     if(digit[4] && digits<52)begin packet<={packet[203:0],digit[3:0]};digits<=digits+1'b1;end
     else begin collecting<=0;discard<=1;if(errors!=65535)errors<=errors+1'b1;end
    end
   end
  end
 end
endmodule

// Source: vision_cascade_outer.v
// Frame-time velocity and pipelined outer loop. Units: pixels,seconds,encoder counts.
module vision_cascade_outer #(parameter MS_CYCLES=50000,REGION_VELOCITY=0,WIDE_RANGE=0,NEUTRAL_BITS=8)(
 input wire clk,rst_n,run,frame,valid,stale,clear,neutral_capture,polarity,
 input wire [10:0] x,input wire [15:0] center,deadzone,kp,kv,ki,
 input wire [7:0] limit,delay_ms,input wire signed [15:0] bias,
 input wire signed [31:0] position,work_min,work_max,
 output reg signed [31:0] target_q8=0,velocity_q4=0,
 output reg signed [15:0] legacy_velocity_q4=0,
 output reg signed [NEUTRAL_BITS-1:0] neutral=0,output reg neutral_ready=1,updated=0,stable=0,
 output reg [31:0] time_ms=0,output wire [31:0] frame_time_ms,
 input wire predictive,output reg [1:0] control_phase=2
);
 function [15:0] reciprocal;
  input [7:0] d;
  begin case(d)
   8'd20:reciprocal=16'd12800;
   8'd21:reciprocal=16'd12190;
   8'd22:reciprocal=16'd11636;
   8'd23:reciprocal=16'd11130;
   8'd24:reciprocal=16'd10667;
   8'd25:reciprocal=16'd10240;
   8'd26:reciprocal=16'd9846;
   8'd27:reciprocal=16'd9481;
   8'd28:reciprocal=16'd9143;
   8'd29:reciprocal=16'd8828;
   8'd30:reciprocal=16'd8533;
   8'd31:reciprocal=16'd8258;
   8'd32:reciprocal=16'd8000;
   8'd33:reciprocal=16'd7758;
   8'd34:reciprocal=16'd7529;
   8'd35:reciprocal=16'd7314;
   8'd36:reciprocal=16'd7111;
   8'd37:reciprocal=16'd6919;
   8'd38:reciprocal=16'd6737;
   8'd39:reciprocal=16'd6564;
   8'd40:reciprocal=16'd6400;
   8'd41:reciprocal=16'd6244;
   8'd42:reciprocal=16'd6095;
   8'd43:reciprocal=16'd5953;
   8'd44:reciprocal=16'd5818;
   8'd45:reciprocal=16'd5689;
   8'd46:reciprocal=16'd5565;
   8'd47:reciprocal=16'd5447;
   8'd48:reciprocal=16'd5333;
   8'd49:reciprocal=16'd5224;
   8'd50:reciprocal=16'd5120;
   8'd51:reciprocal=16'd5020;
   8'd52:reciprocal=16'd4923;
   8'd53:reciprocal=16'd4830;
   8'd54:reciprocal=16'd4741;
   8'd55:reciprocal=16'd4655;
   8'd56:reciprocal=16'd4571;
   8'd57:reciprocal=16'd4491;
   8'd58:reciprocal=16'd4414;
   8'd59:reciprocal=16'd4339;
   8'd60:reciprocal=16'd4267;
   8'd61:reciprocal=16'd4197;
   8'd62:reciprocal=16'd4129;
   8'd63:reciprocal=16'd4063;
   8'd64:reciprocal=16'd4000;
   8'd65:reciprocal=16'd3938;
   8'd66:reciprocal=16'd3879;
   8'd67:reciprocal=16'd3821;
   8'd68:reciprocal=16'd3765;
   8'd69:reciprocal=16'd3710;
   8'd70:reciprocal=16'd3657;
   8'd71:reciprocal=16'd3606;
   8'd72:reciprocal=16'd3556;
   8'd73:reciprocal=16'd3507;
   8'd74:reciprocal=16'd3459;
   8'd75:reciprocal=16'd3413;
   8'd76:reciprocal=16'd3368;
   8'd77:reciprocal=16'd3325;
   8'd78:reciprocal=16'd3282;
   8'd79:reciprocal=16'd3241;
   8'd80:reciprocal=16'd3200;
   8'd81:reciprocal=16'd3160;
   8'd82:reciprocal=16'd3122;
   8'd83:reciprocal=16'd3084;
   8'd84:reciprocal=16'd3048;
   8'd85:reciprocal=16'd3012;
   8'd86:reciprocal=16'd2977;
   8'd87:reciprocal=16'd2943;
   8'd88:reciprocal=16'd2909;
   8'd89:reciprocal=16'd2876;
   8'd90:reciprocal=16'd2844;
   8'd91:reciprocal=16'd2813;
   8'd92:reciprocal=16'd2783;
   8'd93:reciprocal=16'd2753;
   8'd94:reciprocal=16'd2723;
   8'd95:reciprocal=16'd2695;
   8'd96:reciprocal=16'd2667;
   8'd97:reciprocal=16'd2639;
   8'd98:reciprocal=16'd2612;
   8'd99:reciprocal=16'd2586;
   8'd100:reciprocal=16'd2560;
   default:reciprocal=16'd7758;
  endcase end
 endfunction
 function signed [31:0] clamp_speed;
  input signed [31:0] v;begin clamp_speed=v>65536 ? 65536 : (v< -65536 ? -65536 : v);end
 endfunction
 reg [31:0] mc=0,last_ms=0;
 assign frame_time_ms=last_ms;
 reg [10:0] previous_x=0;reg history=0,first=1,innovation_reset=1;reg [12:0] pipe=0;
 reg signed [12:0] err=0;reg signed [10:0] dx=0;
 reg signed [17:0] measured_q4=0,position_estimate=0,advance_q4=0,predicted_position=0,residual_q4=0,predicted_error=0;
 reg [15:0] reciprocal_reg=7758;reg [10:0] dt_fraction=135;
 reg signed [34:0] velocity_product=0;reg signed [29:0] delay_stage=0;
 reg signed [33:0] p_stage=0;reg signed [30:0] d_stage=0;
 reg signed [31:0] pterm=0,dterm=0;reg signed [26:0] pd_sum=0;reg signed [15:0] relative_stage=0;
 reg signed [26:0] integral_product=0;reg signed [31:0] integral_increment=0,integral_q16=0;
 reg [6:0] integral_ms=0;reg [11:0] stable_ms=0;
 wire signed [12:0] raw_dx=$signed({1'b0,x})-$signed({1'b0,previous_x});
 wire signed [12:0] error_now=$signed({1'b0,x})-$signed({1'b0,center});
 wire signed [13:0] abs_error=error_now<0 ? -error_now : error_now;
 wire [31:0] interval=time_ms-last_ms;
 wire signed [29:0] advance_product=$signed(velocity_q4[17:0])*$signed({1'b0,dt_fraction});
 wire signed [17:0] residual_now=measured_q4-predicted_position;
 wire signed [34:0] beta_product=$signed(residual_q4)*$signed({1'b0,reciprocal_reg});
 wire signed [31:0] next_velocity=$signed(velocity_q4)+$signed(velocity_product>>>12);
 wire [10:0] delay_fraction={delay_ms,2'b0}+((delay_ms*3)>>5);

 // Bounded reversal lookahead:80ms plus5ms/count to the available braking angle.
 // No mechanical mm/angle model is inferred from manually disturbed logs.
 wire brake_positive=polarity ? velocity_q4<0 : velocity_q4>=0;
 wire signed [9:0] brake_target_raw=$signed(neutral)+(brake_positive ? $signed({1'b0,limit}) : -$signed({1'b0,limit}));
 wire signed [9:0] brake_target=brake_target_raw>60 ? 10'sd60 : (brake_target_raw< -60 ? -10'sd60 : brake_target_raw);
 wire signed [9:0] brake_span_raw=brake_positive ? brake_target-$signed(position[8:0]) : $signed(position[8:0])-brake_target;
 wire [6:0] brake_span=brake_span_raw<0 ? 0 : (brake_span_raw>120 ? 120 : brake_span_raw[6:0]);
 wire [11:0] look_fraction=12'd328+({5'd0,brake_span}<<4)+({5'd0,brake_span}<<2)+({5'd0,brake_span}>>1);
 wire [11:0] effective_fraction=predictive ? look_fraction : {1'b0,delay_fraction};
 wire signed [29:0] delay_product=$signed(velocity_q4[17:0])*$signed({1'b0,effective_fraction});
 reg signed [28:0] measured_velocity_product=0;
 wire signed [28:0] fast_velocity_product=$signed(dx)*$signed({1'b0,reciprocal_reg});
 wire signed [31:0] fast_velocity_raw=$signed(measured_velocity_product>>>4);
 wire signed [31:0] fast_velocity_next=($signed(velocity_q4)+clamp_speed(fast_velocity_raw))>>>1;
 reg signed [17:0] goal_velocity_q4=0;reg [1:0] phase_candidate=2;
 // Recognize sub-pixel/centroid noise only inside the real center deadband.
 // Four actual frames must fit a4pixel window; fast crossings never qualify.
 reg [10:0] history_x2=0,history_x3=0,window_hi=0,window_lo=0;reg capture_quiet=0;
 wire [10:0] hi_a=x>previous_x ? x : previous_x;
 wire [10:0] hi_b=history_x2>history_x3 ? history_x2 : history_x3;
 wire [10:0] lo_a=x<previous_x ? x : previous_x;
 wire [10:0] lo_b=history_x2<history_x3 ? history_x2 : history_x3;
 wire [10:0] frame_hi=hi_a>hi_b ? hi_a : hi_b;
 wire [10:0] frame_lo=lo_a<lo_b ? lo_a : lo_b;

 wire approaching=(err>0 && velocity_q4<0) || (err<0 && velocity_q4>0);
 wire predicted_crossing=(err>0 && predicted_error<=0) || (err<0 && predicted_error>=0);
 wire [17:0] predicted_distance=predicted_error<0 ? -predicted_error : predicted_error;
 wire [13:0] effective_distance=approaching ? (predicted_crossing ? 14'd0 : {1'b0,predicted_distance[16:4]}) : {1'b0,abs_error[12:0]};
 function [9:0] velocity_envelope;
  input [13:0] d;reg [14:0] value;
  begin
   if(d<=64)value=d<<1;
   else if(d<=256)value=128+((d-64)>>1);
   else value=224+((d-256)>>2);
   velocity_envelope=value>400 ? 10'd400 : value[9:0];
  end
 endfunction
 wire [9:0] speed_goal=velocity_envelope(effective_distance);
 wire signed [17:0] velocity_error_q4=predictive ? $signed(velocity_q4[17:0])-goal_velocity_q4 : $signed(velocity_q4[17:0]);

 wire signed [33:0] p_product=$signed(predicted_error)*$signed({1'b0,kp[14:0]});
 wire signed [30:0] d_product=$signed(velocity_error_q4)*$signed({1'b0,kv[11:0]});
 wire signed [26:0] i_product=$signed(error_now)*$signed({1'b0,ki[11:0]});
 wire signed [40:0] i_scaled=$signed(integral_product)*41'sd6554;
 wire signed [31:0] i_next=integral_q16+(polarity ? -integral_increment : integral_increment);
 wire signed [12:0] i_bias=($signed(integral_q16)>>>8)+$signed(bias);
 wire signed [12:0] limited_bias=i_bias>2048 ? 13'sd2048 : (i_bias< -2048 ? -13'sd2048 : i_bias);
 wire signed [26:0] relative_unclipped=pd_sum+limited_bias;
 wire signed [16:0] relative_limit=$signed({1'b0,limit,8'b0});
 wire signed [15:0] relative_clipped=relative_unclipped>relative_limit ? relative_limit[15:0] : (relative_unclipped< -relative_limit ? -relative_limit : relative_unclipped[15:0]);
 wire signed [16:0] absolute_unclipped=$signed({neutral,8'b0})+$signed(relative_stage);
 wire signed [16:0] absolute_clipped=absolute_unclipped>15360 ? 17'sd15360 : (absolute_unclipped< -15360 ? -17'sd15360 : absolute_unclipped);
 wire signed [31:0] abs_speed=velocity_q4<0 ? -velocity_q4 : velocity_q4;
 wire [15:0] stable_band=deadzone>10 ? deadzone : 16'd10;
 always @(posedge clk)begin
  if(!rst_n)begin
   mc<=0;time_ms<=0;last_ms<=0;previous_x<=0;history<=0;first<=1;innovation_reset<=1;pipe<=0;err<=0;dx<=0;
   measured_q4<=0;position_estimate<=0;advance_q4<=0;predicted_position<=0;residual_q4<=0;predicted_error<=0;
   reciprocal_reg<=7758;dt_fraction<=135;velocity_product<=0;delay_stage<=0;p_stage<=0;d_stage<=0;
   velocity_q4<=0;legacy_velocity_q4<=0;pterm<=0;dterm<=0;pd_sum<=0;relative_stage<=0;target_q8<=0;neutral<=0;neutral_ready<=1;
   integral_product<=0;integral_increment<=0;integral_q16<=0;integral_ms<=0;stable_ms<=0;stable<=0;updated<=0;measured_velocity_product<=0;goal_velocity_q4<=0;phase_candidate<=2;control_phase<=2;history_x2<=0;history_x3<=0;window_hi<=0;window_lo<=0;capture_quiet<=0;
  end else begin
   updated<=0;
   if(mc==MS_CYCLES-1)begin mc<=0;time_ms<=time_ms+1'b1;
    if(run && valid && !stale)begin
     if(abs_error<=stable_band && abs_speed<=240)begin
      if(stable_ms<3000)stable_ms<=stable_ms+1'b1;
      if(stable_ms>=2999)stable<=1;
     end else if(!stable || abs_error>stable_band*2 || abs_speed>480)begin stable_ms<=0;stable<=0;end
     if(integral_ms==99)begin
      integral_ms<=0;
      if(abs_error<240 && abs_speed<480 && relative_unclipped==relative_clipped && absolute_unclipped==absolute_clipped)
       integral_q16<=i_next>262144 ? 262144 : (i_next< -262144 ? -262144 : i_next);
     end else integral_ms<=integral_ms+1'b1;
    end else begin stable_ms<=0;stable<=0;integral_ms<=0;end
   end else mc<=mc+1'b1;
   pipe<={pipe[11:0],frame && valid && !stale};
   if(frame && valid && !stale)begin
    previous_x<=x;history_x2<=previous_x;history_x3<=history_x2;window_hi<=frame_hi;window_lo<=frame_lo;last_ms<=time_ms;err<=error_now;measured_q4<=$signed({1'b0,x,4'b0});
    dx<=raw_dx>256 ? 256 : (raw_dx< -256 ? -256 : raw_dx);
    first<=!history || !run || interval<20 || interval>100;
    reciprocal_reg<=reciprocal(interval[7:0]);dt_fraction<=interval[7:0]*4+((interval[7:0]*3)>>5);history<=run;
   end
   // Alpha-beta observer: alpha=1/2,beta=1/16; avoids differentiating pixel quantization.
   if(pipe[0])begin capture_quiet<=predictive && !REGION_VELOCITY && err<=$signed({1'b0,deadzone}) && err>=-$signed({1'b0,deadzone}) && window_hi-window_lo<=4;measured_velocity_product<=fast_velocity_product;advance_q4<=$signed(advance_product>>>12);integral_product<=i_product;
    legacy_velocity_q4<=first ? 0 : ($signed(legacy_velocity_q4)*3+dx*16)/4;
   end
   if(pipe[1])predicted_position<=position_estimate+advance_q4;
   if(pipe[2])begin residual_q4<=residual_now;innovation_reset<=first || residual_now>2048 || residual_now< -2048;position_estimate<=(first || residual_now>2048 || residual_now< -2048) ? measured_q4 : predicted_position+($signed(residual_now)>>>1);end
   if(pipe[3])velocity_product<=beta_product;
   if(pipe[4])velocity_q4<=(innovation_reset || (predictive && capture_quiet)) ? 0 : (predictive ? clamp_speed(fast_velocity_next) : clamp_speed(next_velocity));
   if(pipe[5])delay_stage<=delay_product;
   if(pipe[6])predicted_error<=err*16+$signed(delay_stage>>>12);
   if(pipe[7])begin
    if(abs_error<=deadzone && abs_speed<=128)goal_velocity_q4<=0;
    else goal_velocity_q4<=err>0 ? -$signed({4'd0,speed_goal,4'b0}) : (err<0 ? $signed({4'd0,speed_goal,4'b0}) : 18'sd0);
    if(abs_error<=24 && abs_speed<=640)phase_candidate<=2;
    else if((approaching && (predicted_crossing || abs_speed>((speed_goal+16)*16))) || (!approaching && abs_speed>640))phase_candidate<=1;
    else phase_candidate<=0;
   end
   if(pipe[8])begin p_stage<=p_product;d_stage<=d_product;end
   if(pipe[9])begin
    pterm<=predictive || (err<=$signed({1'b0,deadzone}) && err>=-$signed({1'b0,deadzone})) ? 0 : $signed(p_stage>>>8);
    dterm<=predictive ? $signed(d_stage>>>4) : (velocity_q4<48 && velocity_q4> -48 ? 0 : $signed(d_stage>>>4));
    integral_increment<=$signed(i_scaled>>>16);
   end
   if(pipe[10])pd_sum<=polarity ? -(pterm+dterm) : (pterm+dterm);
   if(pipe[11])relative_stage<=relative_clipped;
   if(pipe[12] && valid && !stale && !clear && !neutral_capture)begin target_q8<=absolute_clipped;updated<=1;control_phase<=predictive ? phase_candidate : 2'd0;end
   if(!run)begin legacy_velocity_q4<=0;history<=0;integral_q16<=0;integral_ms<=0;stable_ms<=0;stable<=0;end
   if(!valid || stale)begin legacy_velocity_q4<=0;history<=0;velocity_q4<=0;pipe<=0;stable_ms<=0;stable<=0;updated<=0;end
   if(clear || neutral_capture)begin capture_quiet<=0;history_x2<=x;history_x3<=x;window_hi<=x;window_lo<=x;control_phase<=2;goal_velocity_q4<=0;legacy_velocity_q4<=0;history<=0;velocity_q4<=0;pipe<=0;integral_q16<=0;stable_ms<=0;stable<=0;updated<=0;end
   if(neutral_capture)begin
    if(position>=(WIDE_RANGE ? work_min : -60) && position<=(WIDE_RANGE ? work_max : 60))begin neutral<=position[NEUTRAL_BITS-1:0];neutral_ready<=1;target_q8<=position*256;end
    else neutral_ready<=0;
   end
  end
 end
endmodule

// Source: vision_cascade_control.v
// Physical K2 owns both continuous run and each finite calibration profile.
module vision_cascade_control #(parameter MS_CYCLES=50000,WIDE_RANGE=0,TARGET_BITS=8)(
 input wire clk,rst_n,key_event,k4_n,enable_request,settings_ready,center_confirmed,
 input wire valid,hold_valid,stale,frame,config_changed,neutral_capture,profile_ready,region_holding,
 input wire [7:0] selection,amplitude,limit,input wire [15:0] max_hz,inner_kp,feedforward,slew_rate,runtime_seconds,kp,kv,
 input wire [15:0] roi_left,roi_right,center,input wire [10:0] x,
 input wire signed [31:0] position,raw_target_q8,command_position,input wire outer_updated,neutral_ready,
 input wire signed [TARGET_BITS-1:0] neutral,input wire signed [31:0] work_min,work_max,input wire core_ready,core_busy,input wire [7:0] core_spent,input wire [3:0] fault,input wire step,
 output wire diagnostic_quiescing,
 output reg run=0,request=0,output wire following,external_cancel,checkpoint,
 output reg [3:0] stop_reason=0,reject_reason=0,cal_status=0,cal_phase=0,
 output wire [3:0] external_cancel_reason,
 output reg signed [31:0] reference_q8=0,
 output reg signed [15:0] frequency_hz=0,
 output wire signed [TARGET_BITS-1:0] target,drive_target,
 output reg [15:0] elapsed_seconds=0,output reg [31:0] total_pulses=0
);
 reg [31:0] mc=0;reg [2:0] ctrl_ms=0;reg [9:0] sec_ms=0;
 reg [15:0] phase_ms=0,in_band_ms=0;reg [1:0] repetitions=0;
 reg pending=0,request_wait=0,step_previous=0,previous_run=0;
 reg signed [31:0] last_reference_q8=0;reg signed [15:0] feedback_term=0,ff_term=0;
 reg signed [16:0] error_stage=0;reg signed [9:0] delta_stage=0;reg signed [33:0] feedback_stage=0;reg signed [19:0] ff_stage=0;
 reg [2:0] rate_pending=0;reg inner_moving=0;
 reg signed [18:0] prediction_offset=0;reg [5:0] quiet_ms=20;
 reg quiescing=0;reg [5:0] checkpoint_ms=0;reg [11:0] segment_ms=0;
 assign diagnostic_quiescing=quiescing;
 wire checkpoint_due=run && selection==1 && core_busy && (core_spent>=48 || (segment_ms>=3500 && core_spent!=0));
 assign checkpoint=run && selection==1 && quiescing && checkpoint_ms>=40;

 localparam QW=WIDE_RANGE ? 24 : 16;
 wire signed [31:0] lo=WIDE_RANGE ? work_min : -60,hi=WIDE_RANGE ? work_max : 60;
 wire signed [QW:0] command_q8=WIDE_RANGE ? $signed(command_position[15:0])*25'sd328 : $signed(command_position[7:0])*17'sd328;
 wire signed [18:0] expected_q8=prediction_offset+command_q8;
 wire signed [31:0] feedback_q8=quiet_ms<20 ? expected_q8 : position*256;
 wire motor_cal=selection==2;
 wire permit=settings_ready && !config_changed && (motor_cal || (!stale && (run ? hold_valid : valid)));
 wire expired=run && runtime_seconds!=0 && elapsed_seconds>=runtime_seconds;
 assign following=run;
 assign external_cancel=(previous_run && !run) || (run && (key_event || expired || config_changed));
 assign external_cancel_reason=(previous_run && !run) ? stop_reason : (expired ? 4'd8 : (config_changed ? 4'd5 : 4'd7));
 wire tick=mc==MS_CYCLES-1;
 wire signed [31:0] zero_q8=$signed(neutral)*256;
 wire signed [31:0] excursion=$signed({1'b0,amplitude})*256;
 wire signed [31:0] calibration_target=cal_phase==1 ? zero_q8+excursion : (cal_phase==3 ? zero_q8-excursion : zero_q8);
 wire signed [31:0] desired=selection==1 ? raw_target_q8 : calibration_target;
 wire signed [QW:0] ramp_delta=$signed(desired[QW-1:0])-$signed(reference_q8[QW-1:0]);
 wire [14:0] ramp_scaled=slew_rate[8:0]*41;
 wire signed [16:0] ramp_step=$signed({1'b0,ramp_scaled})>>>5;
 wire signed [31:0] error_q8=reference_q8-feedback_q8;
 wire signed [31:0] abs_error=error_q8<0 ? -error_q8 : error_q8;
 wire signed [QW:0] reference_delta=$signed(reference_q8[QW-1:0])-$signed(last_reference_q8[QW-1:0]);
 wire signed [33:0] feedback_product=error_stage*$signed({1'b0,inner_kp});
 wire signed [19:0] ff_product=delta_stage*$signed({1'b0,feedforward[8:0]});
 wire signed [24:0] ff_scaled=($signed(ff_stage)<<<4)+($signed(ff_stage)<<<3)+$signed(ff_stage);
 wire signed [17:0] raw_frequency=$signed(feedback_term)+$signed(ff_term);
 wire signed [17:0] abs_frequency=raw_frequency<0 ? -raw_frequency : raw_frequency;
 wire signed [15:0] bounded_frequency=abs_frequency>$signed({1'b0,max_hz}) ? max_hz : (abs_frequency<20 ? 16'sd20 : abs_frequency[15:0]);
 wire signed [31:0] cal_error=position*256-calibration_target;
 wire [31:0] abs_cal_error=cal_error<0 ? -cal_error : cal_error;
 wire signed [31:0] ball_error=$signed({1'b0,x})-$signed({1'b0,center});
 wire [31:0] abs_ball_error=ball_error<0 ? -ball_error : ball_error;
 wire [10:0] roi_width=roi_right[10:0]-roi_left[10:0];
 wire [22:0] ball_margin_scaled=roi_width*2150;
 wire [31:0] ball_margin=ball_margin_scaled>>13; // conservative <=1pixel quantization
 function signed [TARGET_BITS-1:0] rounded;
  input signed [31:0] q,lower,upper;
  reg signed [QW:0] v,small_q;
  begin small_q=$signed(q[QW-1:0]);v=small_q>=0 ? (small_q+128)>>>8 : -(((-small_q)+128)>>>8);rounded=v>upper ? upper : (v<lower ? lower : v[TARGET_BITS-1:0]);end
 endfunction
 assign target=rounded(desired,lo,hi);assign drive_target=rounded(reference_q8,lo,hi);
 always @(posedge clk)begin
  if(!rst_n)begin run<=0;request<=0;stop_reason<=0;reject_reason<=0;cal_status<=0;cal_phase<=0;reference_q8<=0;frequency_hz<=0;
   elapsed_seconds<=0;total_pulses<=0;mc<=0;ctrl_ms<=0;sec_ms<=0;phase_ms<=0;in_band_ms<=0;repetitions<=0;
   pending<=0;request_wait<=0;step_previous<=0;previous_run<=0;last_reference_q8<=0;feedback_term<=0;ff_term<=0;rate_pending<=0;inner_moving<=0;error_stage<=0;delta_stage<=0;feedback_stage<=0;ff_stage<=0;prediction_offset<=0;quiet_ms<=20;quiescing<=0;checkpoint_ms<=0;segment_ms<=0;
  end else begin
   request<=0;rate_pending<={rate_pending[1:0],1'b0};step_previous<=step;previous_run<=run;
   if(step && !step_previous)begin total_pulses<=total_pulses+1'b1;quiet_ms<=0;end
   if(core_busy || !core_ready)request_wait<=0;
   if(outer_updated && valid)pending<=1;
   if(!valid && !motor_cal)pending<=0;
   if(tick)begin
    mc<=0;
    if(run && core_busy)begin if(segment_ms<4095)segment_ms<=segment_ms+1'b1;end else segment_ms<=0;
    if(quiescing)begin if(checkpoint_ms<63)checkpoint_ms<=checkpoint_ms+1'b1;end else checkpoint_ms<=0;
    if(quiet_ms<20)quiet_ms<=quiet_ms+1'b1;
    else prediction_offset<=position*256-command_q8;
    if(run)begin
     if(sec_ms==999)begin sec_ms<=0;if(elapsed_seconds!=65535)elapsed_seconds<=elapsed_seconds+1'b1;end else sec_ms<=sec_ms+1'b1;
     if(ctrl_ms==4)begin
      ctrl_ms<=0;
      if(!quiescing && !checkpoint_due && ((selection==1 && valid && pending) || selection==2 || (selection==3 && valid)))begin
       if(ramp_delta>ramp_step)reference_q8<=$signed(reference_q8[QW-1:0])+ramp_step;
       else if(ramp_delta< -ramp_step)reference_q8<=$signed(reference_q8[QW-1:0])-ramp_step;
       else reference_q8<=desired;
      end
      last_reference_q8<=reference_q8;
      error_stage<=error_q8>32767 ? 17'sd32767 : (error_q8< -32768 ? -17'sd32768 : error_q8[16:0]);
      delta_stage<=reference_delta>511 ? 10'sd511 : (reference_delta< -512 ? -10'sd512 : reference_delta[9:0]);rate_pending[0]<=1;
     end else ctrl_ms<=ctrl_ms+1'b1;
     if(selection>=2)begin
      if(phase_ms!=65535)phase_ms<=phase_ms+1'b1;
      if(abs_cal_error<=512)begin if(in_band_ms<1000)in_band_ms<=in_band_ms+1'b1;end else in_band_ms<=0;
      if(selection==2 && phase_ms>=250 && in_band_ms<40)begin run<=0;cal_status<=4;stop_reason<=9;frequency_hz<=0;end
      else if((selection==2 && in_band_ms>=40) || (selection==3 && phase_ms>=(cal_phase==0 ? 200 : (cal_phase==1 || cal_phase==3 ? 250 : 300))))begin
       phase_ms<=0;in_band_ms<=0;
       if(cal_phase==4)begin
        if(selection==2 || repetitions==2)begin run<=0;cal_status<=2;stop_reason<=10;frequency_hz<=0;end
        else begin repetitions<=repetitions+1'b1;cal_phase<=0;end
       end else cal_phase<=cal_phase+1'b1;
      end
      if(selection==3 && (!valid || abs_ball_error>ball_margin))begin run<=0;cal_status<=4;stop_reason<=9;frequency_hz<=0;end
     end
    end
   end else mc<=mc+1'b1;
   if(rate_pending[0])begin feedback_stage<=feedback_product;ff_stage<=ff_product;end
   if(rate_pending[1])begin feedback_term<=$signed(feedback_stage>>>16);ff_term<=$signed(ff_scaled>>>13);end
   if(rate_pending[2] && run)begin
    if(abs_frequency<5 || abs_error<=(WIDE_RANGE && !region_holding ? 256 : 128) || (raw_frequency>0 && error_q8<0) || (raw_frequency<0 && error_q8>0))begin frequency_hz<=0;inner_moving<=0;end
    else if(abs_error>(WIDE_RANGE && !region_holding ? 320 : 192) || inner_moving)begin
     frequency_hz<=raw_frequency<0 ? -bounded_frequency : bounded_frequency;inner_moving<=1;
    end else frequency_hz<=0;
   end
   if(key_event && !run)begin
    if(fault!=0)reject_reason<=9;
    else if(!enable_request)reject_reason<=1;
    else if(!core_ready)reject_reason<=2;
    else if(config_changed || !settings_ready || neutral_capture)reject_reason<=5;
    else if(!center_confirmed)reject_reason<=10;
    else if(!neutral_ready)reject_reason<=11;
    else if(!motor_cal && stale)reject_reason<=4;
    else if(!motor_cal && !valid)reject_reason<=3;
    else if(selection==1 && (!profile_ready || (kp==0 && kv==0)))reject_reason<=12;
    else if(position>hi || position<lo || (selection>=2 && ($signed(neutral)+$signed({1'b0,amplitude})>hi || $signed(neutral)-$signed({1'b0,amplitude})<lo)))reject_reason<=13;
    else begin
     run<=1;request<=1;request_wait<=1;reference_q8<=position*256;last_reference_q8<=position*256;frequency_hz<=0;
     stop_reason<=0;reject_reason<=0;elapsed_seconds<=0;total_pulses<=0;sec_ms<=0;ctrl_ms<=0;pending<=0;
     phase_ms<=0;in_band_ms<=0;cal_phase<=0;repetitions<=0;cal_status<=selection>=2 ? 1 : 0;inner_moving<=0;
     prediction_offset<=position*256-command_q8;quiet_ms<=20;
    end
   end else if(run && permit && core_ready && !request_wait && !quiescing && (abs_error>256 || frequency_hz!=0))begin request<=1;request_wait<=1;end
   if(checkpoint_due)quiescing<=1;
   if(quiescing || checkpoint_due)begin frequency_hz<=0;inner_moving<=0;rate_pending<=0;end
   if(quiescing && run && permit && core_ready)begin
    quiescing<=0;checkpoint_ms<=0;segment_ms<=0;request_wait<=0;
    reference_q8<=position*256;last_reference_q8<=position*256;
    frequency_hz<=0;feedback_term<=0;ff_term<=0;rate_pending<=0;inner_moving<=0;
    prediction_offset<=position*256-command_q8;quiet_ms<=20;
   end
   if(run && (fault!=0 || !enable_request || !k4_n || !permit || expired || key_event))begin
    run<=0;request<=0;request_wait<=0;frequency_hz<=0;pending<=0;
    if(cal_status==1)cal_status<=4;
    if(fault!=0)stop_reason<=6;else if(!k4_n)stop_reason<=2;else if(!enable_request)stop_reason<=1;
    else if(config_changed || !settings_ready)stop_reason<=5;else if(stale && !motor_cal)stop_reason<=4;
    else if(!hold_valid && !motor_cal)stop_reason<=3;else stop_reason<=expired ? 8 : 7;
   end
   if(!run)begin frequency_hz<=0;pending<=0;inner_moving<=0;quiescing<=0;checkpoint_ms<=0;segment_ms<=0;end
   if(neutral_capture && !run)begin reference_q8<=position*256;last_reference_q8<=position*256;pending<=0;prediction_offset<=position*256-command_q8;quiet_ms<=20;end
  end
 end
endmodule

// Source: vision_cascade_diag.v
// D1 compact diagnostic, 94 bytes,20ms sample opportunity; no serial motion commands.
module vision_cascade_diag #(parameter UART_DIV=434,MS_CYCLES=50000,PERIOD_MS=20,WIDE_RANGE=0)(
 input wire clk,rst_n,enabled,grant,
 input wire [31:0] time_ms,
 input wire [31:0] frame_ms,
 input wire [15:0] config_seq,
 input wire [7:0] selection,
 input wire [15:0] flags,
 input wire [7:0] cal_phase,
 input wire [7:0] cal_status,
 input wire [15:0] x,
 input wire [15:0] center,
 input wire [31:0] position,
 input wire [31:0] reference_q8,
 input wire [31:0] velocity_q4,
 input wire [15:0] frequency_hz,
 input wire [7:0] fault,
 input wire [7:0] spent,
 input wire [31:0] command_position,
 input wire [15:0] frame_number,
 input wire [(WIDE_RANGE ? 31 : 7):0] neutral,
 input wire [31:0] limit_anchor,command_anchor,work_min,work_max,feedback_min,feedback_max,
 input wire [15:0] config_errors,
 output wire tx,output reg busy=0
);
 localparam BYTES=WIDE_RANGE ? 71 : 44;
 wire [BYTES*8-1:0] payload=WIDE_RANGE ? {time_ms,frame_ms,config_seq,selection,flags,cal_phase,cal_status,x,center,position,reference_q8,velocity_q4,frequency_hz,fault,spent,command_position,frame_number,neutral,config_errors,limit_anchor,command_anchor,work_min,work_max,feedback_min,feedback_max} : {time_ms,frame_ms,config_seq,selection,flags,cal_phase,cal_status,x,center,position,reference_q8,velocity_q4,frequency_hz,fault,spent,command_position,frame_number,neutral,config_errors};
 reg [BYTES*8-1:0] snapshot=0;reg [7:0] checksum_snapshot=0;reg [7:0] checksum;integer j;
 always @*begin checksum=0;for(j=0;j<BYTES;j=j+1)checksum=checksum^payload[j*8+:8];end
 reg [31:0] timer=0;reg owed=0,bit_active=0;reg [(WIDE_RANGE ? 7 : 6):0] index=0;reg [3:0] bit_index=0;
 reg [15:0] baud=0;reg [9:0] shift=1023;
 assign tx=shift[0];
 function [7:0] hex_char;input [3:0] v;begin hex_char=v<10 ? "0"+v : "A"+v-10;end endfunction
 function [7:0] wide_message_byte;input [7:0] at;begin case(at)
0:wide_message_byte="D";
1:wide_message_byte="2";
2:wide_message_byte=hex_char(snapshot[567-:4]);
3:wide_message_byte=hex_char(snapshot[563-:4]);
4:wide_message_byte=hex_char(snapshot[559-:4]);
5:wide_message_byte=hex_char(snapshot[555-:4]);
6:wide_message_byte=hex_char(snapshot[551-:4]);
7:wide_message_byte=hex_char(snapshot[547-:4]);
8:wide_message_byte=hex_char(snapshot[543-:4]);
9:wide_message_byte=hex_char(snapshot[539-:4]);
10:wide_message_byte=hex_char(snapshot[535-:4]);
11:wide_message_byte=hex_char(snapshot[531-:4]);
12:wide_message_byte=hex_char(snapshot[527-:4]);
13:wide_message_byte=hex_char(snapshot[523-:4]);
14:wide_message_byte=hex_char(snapshot[519-:4]);
15:wide_message_byte=hex_char(snapshot[515-:4]);
16:wide_message_byte=hex_char(snapshot[511-:4]);
17:wide_message_byte=hex_char(snapshot[507-:4]);
18:wide_message_byte=hex_char(snapshot[503-:4]);
19:wide_message_byte=hex_char(snapshot[499-:4]);
20:wide_message_byte=hex_char(snapshot[495-:4]);
21:wide_message_byte=hex_char(snapshot[491-:4]);
22:wide_message_byte=hex_char(snapshot[487-:4]);
23:wide_message_byte=hex_char(snapshot[483-:4]);
24:wide_message_byte=hex_char(snapshot[479-:4]);
25:wide_message_byte=hex_char(snapshot[475-:4]);
26:wide_message_byte=hex_char(snapshot[471-:4]);
27:wide_message_byte=hex_char(snapshot[467-:4]);
28:wide_message_byte=hex_char(snapshot[463-:4]);
29:wide_message_byte=hex_char(snapshot[459-:4]);
30:wide_message_byte=hex_char(snapshot[455-:4]);
31:wide_message_byte=hex_char(snapshot[451-:4]);
32:wide_message_byte=hex_char(snapshot[447-:4]);
33:wide_message_byte=hex_char(snapshot[443-:4]);
34:wide_message_byte=hex_char(snapshot[439-:4]);
35:wide_message_byte=hex_char(snapshot[435-:4]);
36:wide_message_byte=hex_char(snapshot[431-:4]);
37:wide_message_byte=hex_char(snapshot[427-:4]);
38:wide_message_byte=hex_char(snapshot[423-:4]);
39:wide_message_byte=hex_char(snapshot[419-:4]);
40:wide_message_byte=hex_char(snapshot[415-:4]);
41:wide_message_byte=hex_char(snapshot[411-:4]);
42:wide_message_byte=hex_char(snapshot[407-:4]);
43:wide_message_byte=hex_char(snapshot[403-:4]);
44:wide_message_byte=hex_char(snapshot[399-:4]);
45:wide_message_byte=hex_char(snapshot[395-:4]);
46:wide_message_byte=hex_char(snapshot[391-:4]);
47:wide_message_byte=hex_char(snapshot[387-:4]);
48:wide_message_byte=hex_char(snapshot[383-:4]);
49:wide_message_byte=hex_char(snapshot[379-:4]);
50:wide_message_byte=hex_char(snapshot[375-:4]);
51:wide_message_byte=hex_char(snapshot[371-:4]);
52:wide_message_byte=hex_char(snapshot[367-:4]);
53:wide_message_byte=hex_char(snapshot[363-:4]);
54:wide_message_byte=hex_char(snapshot[359-:4]);
55:wide_message_byte=hex_char(snapshot[355-:4]);
56:wide_message_byte=hex_char(snapshot[351-:4]);
57:wide_message_byte=hex_char(snapshot[347-:4]);
58:wide_message_byte=hex_char(snapshot[343-:4]);
59:wide_message_byte=hex_char(snapshot[339-:4]);
60:wide_message_byte=hex_char(snapshot[335-:4]);
61:wide_message_byte=hex_char(snapshot[331-:4]);
62:wide_message_byte=hex_char(snapshot[327-:4]);
63:wide_message_byte=hex_char(snapshot[323-:4]);
64:wide_message_byte=hex_char(snapshot[319-:4]);
65:wide_message_byte=hex_char(snapshot[315-:4]);
66:wide_message_byte=hex_char(snapshot[311-:4]);
67:wide_message_byte=hex_char(snapshot[307-:4]);
68:wide_message_byte=hex_char(snapshot[303-:4]);
69:wide_message_byte=hex_char(snapshot[299-:4]);
70:wide_message_byte=hex_char(snapshot[295-:4]);
71:wide_message_byte=hex_char(snapshot[291-:4]);
72:wide_message_byte=hex_char(snapshot[287-:4]);
73:wide_message_byte=hex_char(snapshot[283-:4]);
74:wide_message_byte=hex_char(snapshot[279-:4]);
75:wide_message_byte=hex_char(snapshot[275-:4]);
76:wide_message_byte=hex_char(snapshot[271-:4]);
77:wide_message_byte=hex_char(snapshot[267-:4]);
78:wide_message_byte=hex_char(snapshot[263-:4]);
79:wide_message_byte=hex_char(snapshot[259-:4]);
80:wide_message_byte=hex_char(snapshot[255-:4]);
81:wide_message_byte=hex_char(snapshot[251-:4]);
82:wide_message_byte=hex_char(snapshot[247-:4]);
83:wide_message_byte=hex_char(snapshot[243-:4]);
84:wide_message_byte=hex_char(snapshot[239-:4]);
85:wide_message_byte=hex_char(snapshot[235-:4]);
86:wide_message_byte=hex_char(snapshot[231-:4]);
87:wide_message_byte=hex_char(snapshot[227-:4]);
88:wide_message_byte=hex_char(snapshot[223-:4]);
89:wide_message_byte=hex_char(snapshot[219-:4]);
90:wide_message_byte=hex_char(snapshot[215-:4]);
91:wide_message_byte=hex_char(snapshot[211-:4]);
92:wide_message_byte=hex_char(snapshot[207-:4]);
93:wide_message_byte=hex_char(snapshot[203-:4]);
94:wide_message_byte=hex_char(snapshot[199-:4]);
95:wide_message_byte=hex_char(snapshot[195-:4]);
96:wide_message_byte=hex_char(snapshot[191-:4]);
97:wide_message_byte=hex_char(snapshot[187-:4]);
98:wide_message_byte=hex_char(snapshot[183-:4]);
99:wide_message_byte=hex_char(snapshot[179-:4]);
100:wide_message_byte=hex_char(snapshot[175-:4]);
101:wide_message_byte=hex_char(snapshot[171-:4]);
102:wide_message_byte=hex_char(snapshot[167-:4]);
103:wide_message_byte=hex_char(snapshot[163-:4]);
104:wide_message_byte=hex_char(snapshot[159-:4]);
105:wide_message_byte=hex_char(snapshot[155-:4]);
106:wide_message_byte=hex_char(snapshot[151-:4]);
107:wide_message_byte=hex_char(snapshot[147-:4]);
108:wide_message_byte=hex_char(snapshot[143-:4]);
109:wide_message_byte=hex_char(snapshot[139-:4]);
110:wide_message_byte=hex_char(snapshot[135-:4]);
111:wide_message_byte=hex_char(snapshot[131-:4]);
112:wide_message_byte=hex_char(snapshot[127-:4]);
113:wide_message_byte=hex_char(snapshot[123-:4]);
114:wide_message_byte=hex_char(snapshot[119-:4]);
115:wide_message_byte=hex_char(snapshot[115-:4]);
116:wide_message_byte=hex_char(snapshot[111-:4]);
117:wide_message_byte=hex_char(snapshot[107-:4]);
118:wide_message_byte=hex_char(snapshot[103-:4]);
119:wide_message_byte=hex_char(snapshot[99-:4]);
120:wide_message_byte=hex_char(snapshot[95-:4]);
121:wide_message_byte=hex_char(snapshot[91-:4]);
122:wide_message_byte=hex_char(snapshot[87-:4]);
123:wide_message_byte=hex_char(snapshot[83-:4]);
124:wide_message_byte=hex_char(snapshot[79-:4]);
125:wide_message_byte=hex_char(snapshot[75-:4]);
126:wide_message_byte=hex_char(snapshot[71-:4]);
127:wide_message_byte=hex_char(snapshot[67-:4]);
128:wide_message_byte=hex_char(snapshot[63-:4]);
129:wide_message_byte=hex_char(snapshot[59-:4]);
130:wide_message_byte=hex_char(snapshot[55-:4]);
131:wide_message_byte=hex_char(snapshot[51-:4]);
132:wide_message_byte=hex_char(snapshot[47-:4]);
133:wide_message_byte=hex_char(snapshot[43-:4]);
134:wide_message_byte=hex_char(snapshot[39-:4]);
135:wide_message_byte=hex_char(snapshot[35-:4]);
136:wide_message_byte=hex_char(snapshot[31-:4]);
137:wide_message_byte=hex_char(snapshot[27-:4]);
138:wide_message_byte=hex_char(snapshot[23-:4]);
139:wide_message_byte=hex_char(snapshot[19-:4]);
140:wide_message_byte=hex_char(snapshot[15-:4]);
141:wide_message_byte=hex_char(snapshot[11-:4]);
142:wide_message_byte=hex_char(snapshot[7-:4]);
143:wide_message_byte=hex_char(snapshot[3-:4]);
144:wide_message_byte=hex_char(checksum_snapshot[7:4]);
145:wide_message_byte=hex_char(checksum_snapshot[3:0]);
146:wide_message_byte=13;
147:wide_message_byte=10;
default:wide_message_byte="?";endcase end endfunction
 function [7:0] message_byte;input [(WIDE_RANGE ? 7 : 6):0] at;begin if(WIDE_RANGE)message_byte=wide_message_byte(at);else case(at)
 0:message_byte="D";
1:message_byte="1";
2:message_byte=hex_char(snapshot[351-:4]);
3:message_byte=hex_char(snapshot[347-:4]);
4:message_byte=hex_char(snapshot[343-:4]);
5:message_byte=hex_char(snapshot[339-:4]);
6:message_byte=hex_char(snapshot[335-:4]);
7:message_byte=hex_char(snapshot[331-:4]);
8:message_byte=hex_char(snapshot[327-:4]);
9:message_byte=hex_char(snapshot[323-:4]);
10:message_byte=hex_char(snapshot[319-:4]);
11:message_byte=hex_char(snapshot[315-:4]);
12:message_byte=hex_char(snapshot[311-:4]);
13:message_byte=hex_char(snapshot[307-:4]);
14:message_byte=hex_char(snapshot[303-:4]);
15:message_byte=hex_char(snapshot[299-:4]);
16:message_byte=hex_char(snapshot[295-:4]);
17:message_byte=hex_char(snapshot[291-:4]);
18:message_byte=hex_char(snapshot[287-:4]);
19:message_byte=hex_char(snapshot[283-:4]);
20:message_byte=hex_char(snapshot[279-:4]);
21:message_byte=hex_char(snapshot[275-:4]);
22:message_byte=hex_char(snapshot[271-:4]);
23:message_byte=hex_char(snapshot[267-:4]);
24:message_byte=hex_char(snapshot[263-:4]);
25:message_byte=hex_char(snapshot[259-:4]);
26:message_byte=hex_char(snapshot[255-:4]);
27:message_byte=hex_char(snapshot[251-:4]);
28:message_byte=hex_char(snapshot[247-:4]);
29:message_byte=hex_char(snapshot[243-:4]);
30:message_byte=hex_char(snapshot[239-:4]);
31:message_byte=hex_char(snapshot[235-:4]);
32:message_byte=hex_char(snapshot[231-:4]);
33:message_byte=hex_char(snapshot[227-:4]);
34:message_byte=hex_char(snapshot[223-:4]);
35:message_byte=hex_char(snapshot[219-:4]);
36:message_byte=hex_char(snapshot[215-:4]);
37:message_byte=hex_char(snapshot[211-:4]);
38:message_byte=hex_char(snapshot[207-:4]);
39:message_byte=hex_char(snapshot[203-:4]);
40:message_byte=hex_char(snapshot[199-:4]);
41:message_byte=hex_char(snapshot[195-:4]);
42:message_byte=hex_char(snapshot[191-:4]);
43:message_byte=hex_char(snapshot[187-:4]);
44:message_byte=hex_char(snapshot[183-:4]);
45:message_byte=hex_char(snapshot[179-:4]);
46:message_byte=hex_char(snapshot[175-:4]);
47:message_byte=hex_char(snapshot[171-:4]);
48:message_byte=hex_char(snapshot[167-:4]);
49:message_byte=hex_char(snapshot[163-:4]);
50:message_byte=hex_char(snapshot[159-:4]);
51:message_byte=hex_char(snapshot[155-:4]);
52:message_byte=hex_char(snapshot[151-:4]);
53:message_byte=hex_char(snapshot[147-:4]);
54:message_byte=hex_char(snapshot[143-:4]);
55:message_byte=hex_char(snapshot[139-:4]);
56:message_byte=hex_char(snapshot[135-:4]);
57:message_byte=hex_char(snapshot[131-:4]);
58:message_byte=hex_char(snapshot[127-:4]);
59:message_byte=hex_char(snapshot[123-:4]);
60:message_byte=hex_char(snapshot[119-:4]);
61:message_byte=hex_char(snapshot[115-:4]);
62:message_byte=hex_char(snapshot[111-:4]);
63:message_byte=hex_char(snapshot[107-:4]);
64:message_byte=hex_char(snapshot[103-:4]);
65:message_byte=hex_char(snapshot[99-:4]);
66:message_byte=hex_char(snapshot[95-:4]);
67:message_byte=hex_char(snapshot[91-:4]);
68:message_byte=hex_char(snapshot[87-:4]);
69:message_byte=hex_char(snapshot[83-:4]);
70:message_byte=hex_char(snapshot[79-:4]);
71:message_byte=hex_char(snapshot[75-:4]);
72:message_byte=hex_char(snapshot[71-:4]);
73:message_byte=hex_char(snapshot[67-:4]);
74:message_byte=hex_char(snapshot[63-:4]);
75:message_byte=hex_char(snapshot[59-:4]);
76:message_byte=hex_char(snapshot[55-:4]);
77:message_byte=hex_char(snapshot[51-:4]);
78:message_byte=hex_char(snapshot[47-:4]);
79:message_byte=hex_char(snapshot[43-:4]);
80:message_byte=hex_char(snapshot[39-:4]);
81:message_byte=hex_char(snapshot[35-:4]);
82:message_byte=hex_char(snapshot[31-:4]);
83:message_byte=hex_char(snapshot[27-:4]);
84:message_byte=hex_char(snapshot[23-:4]);
85:message_byte=hex_char(snapshot[19-:4]);
86:message_byte=hex_char(snapshot[15-:4]);
87:message_byte=hex_char(snapshot[11-:4]);
88:message_byte=hex_char(snapshot[7-:4]);
89:message_byte=hex_char(snapshot[3-:4]);
90:message_byte=hex_char(checksum_snapshot[7:4]);
91:message_byte=hex_char(checksum_snapshot[3:0]);
92:message_byte=13;
93:message_byte=10;
 default:message_byte="?";endcase end endfunction
 always @(posedge clk)begin
  if(!rst_n)begin snapshot<=0;checksum_snapshot<=0;timer<=0;owed<=0;busy<=0;bit_active<=0;index<=0;bit_index<=0;baud<=0;shift<=1023;end
  else begin
   if(timer==MS_CYCLES*PERIOD_MS-1)begin timer<=0;if(enabled)owed<=1;end else timer<=timer+1'b1;
   if(!enabled)owed<=0;
   if(!busy && owed && enabled && grant)begin snapshot<=payload;checksum_snapshot<=checksum;owed<=0;index<=0;busy<=1;end
   else if(busy)begin
    if(!bit_active)begin shift<={1'b1,message_byte(index),1'b0};baud<=0;bit_index<=0;bit_active<=1;end
    else if(baud==UART_DIV-1)begin
     baud<=0;shift<={1'b1,shift[9:1]};
     if(bit_index==9)begin bit_active<=0;if(index==(WIDE_RANGE ? 147 : 93))busy<=0;else index<=index+1'b1;end
     else bit_index<=bit_index+1'b1;
    end else baud<=baud+1'b1;
   end
  end
 end
endmodule

// Source: enable_latch.v
// Physical K1 is a debounced toggle. K4 overrides it immediately and clears it.
// A qualified release must precede every accepted K1 press, including at boot,
// after K4 and after a fault. No automatic enable from a held key.
module enable_latch #(parameter integer DEBOUNCE_CYCLES=1000000,HOLD_ON_FAULT=0)(
    input wire clk,rst_n,k1_n,k4_n,inhibit,
    output wire enable_request
);
    reg sync1=0,sync2=0,candidate=0,stable_pressed=0;
    reg stable_valid=0,armed=0,latched=0;
    reg [31:0] debounce_count=0;
    assign enable_request=rst_n & k4_n & (HOLD_ON_FAULT || !inhibit) & latched;
    always @(posedge clk) begin
        if(!rst_n || !k4_n || (inhibit && !HOLD_ON_FAULT)) begin
            sync1<=0; sync2<=0; candidate<=0; stable_pressed<=0;
            stable_valid<=0; armed<=0; latched<=0; debounce_count<=0;
        end else begin
            sync1<=~k1_n; sync2<=sync1;
            if(sync2!=candidate) begin
                candidate<=sync2; debounce_count<=0; stable_valid<=0;
            end else if(debounce_count<DEBOUNCE_CYCLES-1) begin
                debounce_count<=debounce_count+1'b1;
            end else begin stable_pressed<=candidate; stable_valid<=1; end
            if(stable_valid) begin
                if(!stable_pressed) armed<=1;
                else if(armed) begin
                    // A fault permits releasing an existing hold, never re-enabling.
                    if(!inhibit || latched)latched<=~latched;
                    armed<=0;
                end
            end
        end
    end
endmodule

// Source: encoder_ab_counter.v
module encoder_ab_counter(
    input wire clk,
    input wire rst_n,
    input wire [1:0] ab_async,
    output reg signed [31:0] count,
    output reg [15:0] errors,
    output wire [1:0] ab_state
);
    reg [1:0] sync1=0,sync2=0,previous=0;
    reg [3:0] settle=0;
    reg ready=0;
    assign ab_state=sync2;
    always @(posedge clk) begin
        if(!rst_n) begin
            sync1<=0; sync2<=0; previous<=0; settle<=0; ready<=0;
            count<=0; errors<=0;
        end else begin
            sync1<=ab_async; sync2<=sync1;
            previous<=sync2;
            if(!ready) begin
                if(settle==15) ready<=1;
                else settle<=settle+1'b1;
            end else begin
                case({previous,sync2})
                    4'b0001,4'b0111,4'b1110,4'b1000: count<=count+32'sd1;
                    4'b0010,4'b1011,4'b1101,4'b0100: count<=count-32'sd1;
                    4'b0011,4'b0110,4'b1001,4'b1100:
                        if(errors!=16'hffff) errors<=errors+1'b1;
                    default: begin end
                endcase
            end
        end
    end
endmodule


// Source: vision_move_core.v
// Adapted from the physically passed position_core: same pulse phases and guards.
// New commands are K2 edges sampling a visual target, never a continuous setpoint.
module vision_move_core #(
 parameter integer DEBOUNCE_CYCLES=1000000,ENABLE_CYCLES=5000000,SETUP_CYCLES=1000000,
 HALF_CYCLES=125000,SETTLE_CYCLES=2500000,TIMEOUT_CYCLES=250000000,
 POSITION_LIMIT=64,FEEDBACK_LIMIT=64,MOVE_BUDGET=64,NO_RESPONSE_LIMIT=3,TARGET_LIMIT=60,DEADBAND=2
)(
 input wire clk,rst_n,enable_request,k4_n,k2_n,
 input wire signed [31:0] encoder_count,input wire [15:0] encoder_errors,
 input wire visual_valid,visual_stale,settings_ready,config_changed,
 input wire signed [7:0] visual_target,
 output wire motor_en,motor_dir,motor_step,
 output reg signed [31:0] command_position=0,locked_target=0,
 output wire signed [31:0] position,error,
 output reg [3:0] state=0,fault=0,cancel_reason=0,reject_reason=0,
 output reg [7:0] spent=0,
 output reg origin_valid=0,output wire ready,busy,reached,
 output reg [15:0] accepted_count=0,completed_count=0
);
 localparam [3:0] OFF=0,ARM=1,READY=2,SETUP=3,HIGH_PHASE=4,LOW_PHASE=5,SETTLE=6,CHECK=7,FAULT=8,ALIGN=9;
 wire enable_raw=enable_request & k4_n;
 wire permit=visual_valid && !visual_stale && settings_ready && !config_changed;
 reg sync1=0,sync2=0,candidate=0,stable_pressed=0,stable_valid=0;
 reg [31:0] debounce_count=0;
 reg previous_pressed=0,press_event=0;
 always @(posedge clk)begin
  if(!rst_n)begin sync1<=0;sync2<=0;candidate<=0;stable_pressed<=0;stable_valid<=0;
   debounce_count<=0;previous_pressed<=0;press_event<=0;end
  else begin
   sync1<=~k2_n;sync2<=sync1;press_event<=0;
   if(sync2!=candidate)begin candidate<=sync2;debounce_count<=0;stable_valid<=0;end
   else if(debounce_count<DEBOUNCE_CYCLES-1)debounce_count<=debounce_count+1'b1;
   else begin stable_pressed<=candidate;stable_valid<=1;end
   if(stable_valid)begin previous_pressed<=stable_pressed;
    if(stable_pressed && !previous_pressed)press_event<=1;
   end
  end
 end
 reg enabled=0,key_armed=0,active=0,done=0,dir_r=0,step_r=0;
 reg signed [31:0] origin=0;
 reg [31:0] enable_count=0,phase_count=0,action_timer=0;
 reg signed [32:0] before_position=0;
 reg [7:0] no_response=0,wrong_direction=0;
 wire signed [32:0] wide_position=origin_valid ? $signed({origin[31],origin})-$signed({encoder_count[31],encoder_count}) : 33'sd0;
 wire signed [32:0] wide_error=$signed({locked_target[31],locked_target})-wide_position;
 wire signed [32:0] displacement=wide_position-before_position;
 wire in_band=wide_error<=DEADBAND && wide_error>=-DEADBAND;
 assign position=wide_position[31:0];assign error=wide_error[31:0];
 assign motor_en=rst_n & enable_raw & enabled & (fault==0) & (encoder_errors==0);
 assign motor_dir=motor_en & dir_r;
 assign motor_step=motor_en & step_r & permit;
 assign ready=motor_en && origin_valid && state==READY && !active;
 assign busy=motor_en && active;
 assign reached=ready && done && in_band;
 always @(posedge clk)begin
  if(!rst_n)key_armed<=0;
  else if(!ready || !permit || press_event)key_armed<=0;
  else if(stable_valid && !stable_pressed)key_armed<=1;
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   enabled<=0;origin_valid<=0;origin<=0;active<=0;done<=0;dir_r<=0;step_r<=0;
   state<=OFF;fault<=0;command_position<=0;locked_target<=0;spent<=0;
   cancel_reason<=0;reject_reason<=0;accepted_count<=0;completed_count<=0;
   enable_count<=0;phase_count<=0;action_timer<=0;before_position<=0;no_response<=0;wrong_direction<=0;
  end else begin
   // Record rejected edges even while disabled/busy. No queue or held-key replay.
   if(press_event)begin
    if(fault!=0 || encoder_errors!=0)reject_reason<=9;
    else if(!enable_raw)reject_reason<=1;
    else if(busy)reject_reason<=6;
    else if(!ready)reject_reason<=2;
    else if(!settings_ready || config_changed)reject_reason<=5;
    else if(visual_stale)reject_reason<=4;
    else if(!visual_valid)reject_reason<=3;
    else if(!key_armed)reject_reason<=8;
    else if(visual_target>TARGET_LIMIT || visual_target< -TARGET_LIMIT)reject_reason<=7;
   end
   if(fault!=0)begin enabled<=0;active<=0;done<=0;step_r<=0;state<=FAULT;cancel_reason<=6;end
   else if(encoder_errors!=0)begin fault<=3;enabled<=0;active<=0;done<=0;step_r<=0;state<=FAULT;cancel_reason<=6;end
   else if(!enable_raw)begin
    enabled<=0;active<=0;done<=0;step_r<=0;state<=OFF;enable_count<=0;phase_count<=0;action_timer<=0;
    if(active)cancel_reason<=(!k4_n ? 2 : 1);
   end else if(!enabled)begin
    state<=ARM;step_r<=0;active<=0;done<=0;
    if(enable_count==ENABLE_CYCLES-1)begin enabled<=1;state<=(origin_valid ? READY : ALIGN);enable_count<=0;phase_count<=0;end
    else enable_count<=enable_count+1'b1;
   end else if(!origin_valid)begin
    state<=ALIGN;step_r<=0;
    if(phase_count==ENABLE_CYCLES-1)begin origin<=encoder_count;origin_valid<=1;phase_count<=0;state<=READY;end
    else phase_count<=phase_count+1'b1;
   end else if(wide_position>FEEDBACK_LIMIT || wide_position< -FEEDBACK_LIMIT)begin
    fault<=7;step_r<=0;active<=0;done<=0;state<=FAULT;cancel_reason<=6;
   end else if(active && action_timer>=TIMEOUT_CYCLES-1)begin
    fault<=4;step_r<=0;active<=0;done<=0;state<=FAULT;cancel_reason<=6;
   end else if(!permit)begin
    step_r<=0;active<=0;state<=READY;phase_count<=0;action_timer<=0;
    if(active)begin done<=0;cancel_reason<=config_changed || !settings_ready ? 5 : (visual_stale ? 4 : 3);end
   end else begin
    if(active)action_timer<=action_timer+1'b1;
    case(state)
     READY:begin
      step_r<=0;phase_count<=0;
      if(press_event && key_armed && visual_target<=TARGET_LIMIT && visual_target>=-TARGET_LIMIT)begin
       locked_target<={{24{visual_target[7]}},visual_target};spent<=0;no_response<=0;wrong_direction<=0;
       action_timer<=0;active<=1;done<=0;state<=CHECK;cancel_reason<=0;reject_reason<=0;
       accepted_count<=accepted_count+1'b1;
      end
     end
     CHECK:begin
      if(in_band)begin active<=0;done<=1;state<=READY;completed_count<=completed_count+1'b1;end
      else if(spent>=MOVE_BUDGET)begin fault<=5;step_r<=0;active<=0;state<=FAULT;cancel_reason<=6;end
      else if((wide_error>0 && command_position>=POSITION_LIMIT) || (wide_error<0 && command_position<=-POSITION_LIMIT))begin
       fault<=6;step_r<=0;active<=0;state<=FAULT;cancel_reason<=6;
      end else begin dir_r<=wide_error>0;before_position<=wide_position;phase_count<=0;state<=SETUP;end
     end
     SETUP:begin
      if(phase_count==SETUP_CYCLES-1)begin
       phase_count<=0;step_r<=1;command_position<=command_position+(dir_r ? 32'sd1 : -32'sd1);
       spent<=spent+1'b1;state<=HIGH_PHASE;
      end else phase_count<=phase_count+1'b1;
     end
     HIGH_PHASE:begin
      if(phase_count==HALF_CYCLES-1)begin phase_count<=0;step_r<=0;state<=LOW_PHASE;end
      else phase_count<=phase_count+1'b1;
     end
     LOW_PHASE:begin
      if(phase_count==HALF_CYCLES-1)begin phase_count<=0;state<=SETTLE;end
      else phase_count<=phase_count+1'b1;
     end
     SETTLE:begin
      if(phase_count==SETTLE_CYCLES-1)begin
       phase_count<=0;
       if(displacement==0)begin
        no_response<=no_response+1'b1;
        if(no_response>=NO_RESPONSE_LIMIT-1)begin fault<=1;step_r<=0;active<=0;state<=FAULT;cancel_reason<=6;end
        else state<=CHECK;
       end else if((dir_r && displacement<0) || (!dir_r && displacement>0))begin
        wrong_direction<=wrong_direction+1'b1;no_response<=0;
        if(wrong_direction>=1)begin fault<=2;step_r<=0;active<=0;state<=FAULT;cancel_reason<=6;end
        else state<=CHECK;
       end else begin no_response<=0;wrong_direction<=0;state<=CHECK;end
      end else phase_count<=phase_count+1'b1;
     end
     default:begin fault<=8;step_r<=0;active<=0;done<=0;state<=FAULT;cancel_reason<=6;end
    endcase
   end
  end
 end
endmodule

// Source: vision_servo_core.v
// Adapted from the physically passed position_core: same pulse phases and guards.
// External requests share the finite guard sequence; follow retargets only at CHECK.
module vision_servo_core #(
 parameter integer TARGET_BITS=8,ASYM_LIMITS=0,
 DEBOUNCE_CYCLES=1000000,ENABLE_CYCLES=5000000,SETUP_CYCLES=1000000,
 HALF_CYCLES=125000,SETTLE_CYCLES=2500000,TIMEOUT_CYCLES=250000000,
 POSITION_LIMIT=64,FEEDBACK_LIMIT=64,MOVE_BUDGET=64,NO_RESPONSE_LIMIT=3,TARGET_LIMIT=60,DEADBAND=2,EXTERNAL_CONTROL=0,FOLLOW_DEADBAND=1,HOLD_ON_FAULT=0,FAST_CAPABLE=0,FAST_MS_CYCLES=50000,CHECKPOINT_CAPABLE=0,TAIL_CHECKPOINT=0,
 FOLLOW_SETUP_CYCLES=SETUP_CYCLES,FOLLOW_SETTLE_CYCLES=SETTLE_CYCLES
)(
 input wire clk,rst_n,enable_request,k4_n,k2_n,
 input wire request,following,external_cancel,input wire [3:0] external_cancel_reason,
 input wire fast_mode,input wire signed [15:0] fast_rate_hz,input wire checkpoint,
 input wire signed [31:0] encoder_count,input wire [15:0] encoder_errors,
 input wire visual_valid,visual_stale,settings_ready,config_changed,
 input wire signed [TARGET_BITS-1:0] visual_target,
 input wire signed [31:0] target_min,target_max,feedback_min,feedback_max,command_min,command_max,
 output wire motor_en,motor_dir,motor_step,
 output reg signed [31:0] command_position=0,locked_target=0,
 output wire signed [31:0] position,error,
 output reg [3:0] state=0,fault=0,cancel_reason=0,reject_reason=0,
 output reg [7:0] spent=0,
 output reg origin_valid=0,output wire ready,busy,reached,
 output reg [15:0] accepted_count=0,completed_count=0,
 output wire [7:0] diag_pending,output wire signed [15:0] diag_commands,diag_delta,
 output wire diag_consistent,diag_wrong
);
 wire signed [31:0] tmin=ASYM_LIMITS ? target_min : -TARGET_LIMIT,tmax=ASYM_LIMITS ? target_max : TARGET_LIMIT;
 wire signed [31:0] pmin=ASYM_LIMITS ? feedback_min : -FEEDBACK_LIMIT,pmax=ASYM_LIMITS ? feedback_max : FEEDBACK_LIMIT;
 wire signed [31:0] mmin=ASYM_LIMITS ? command_min : -POSITION_LIMIT,mmax=ASYM_LIMITS ? command_max : POSITION_LIMIT;
 localparam CB=TARGET_BITS+1;
 localparam [3:0] OFF=0,ARM=1,READY=2,SETUP=3,HIGH_PHASE=4,LOW_PHASE=5,SETTLE=6,CHECK=7,FAULT=8,ALIGN=9;
 wire enable_raw=enable_request & k4_n;
 wire extended_cancel=EXTERNAL_CONTROL && external_cancel;
 wire permit=visual_valid && !visual_stale && settings_ready && !config_changed && !extended_cancel;
 reg sync1=0,sync2=0,candidate=0,stable_pressed=0,stable_valid=0;
 reg [31:0] debounce_count=0;
 reg key_armed=0;
 reg previous_pressed=0,press_event=0;
 wire command_event=EXTERNAL_CONTROL ? request : press_event;
 wire command_armed=EXTERNAL_CONTROL ? 1'b1 : key_armed;
 always @(posedge clk)begin
  if(!rst_n)begin sync1<=0;sync2<=0;candidate<=0;stable_pressed<=0;stable_valid<=0;
   debounce_count<=0;previous_pressed<=0;press_event<=0;end
  else begin
   sync1<=~k2_n;sync2<=sync1;press_event<=0;
   if(sync2!=candidate)begin candidate<=sync2;debounce_count<=0;stable_valid<=0;end
   else if(debounce_count<DEBOUNCE_CYCLES-1)debounce_count<=debounce_count+1'b1;
   else begin stable_pressed<=candidate;stable_valid<=1;end
   if(stable_valid)begin previous_pressed<=stable_pressed;
    if(stable_pressed && !previous_pressed)press_event<=1;
   end
  end
 end
 reg enabled=0,active=0,done=0,dir_r=0,step_r=0;
 reg signed [31:0] origin=0;
 reg [31:0] enable_count=0,phase_count=0,action_timer=0;
 reg signed [32:0] before_position=0;
 reg [7:0] no_response=0,wrong_direction=0;
 wire signed [32:0] wide_position=origin_valid ? $signed({origin[31],origin})-$signed({encoder_count[31],encoder_count}) : 33'sd0;
 wire signed [32:0] wide_error=$signed({locked_target[31],locked_target})-wide_position;
 wire signed [32:0] displacement=wide_position-before_position;
 wire [31:0] allowed_error=EXTERNAL_CONTROL && following ? FOLLOW_DEADBAND : DEADBAND;
 wire [31:0] setup_limit=EXTERNAL_CONTROL && following ? FOLLOW_SETUP_CYCLES : SETUP_CYCLES;
 wire [31:0] settle_limit=EXTERNAL_CONTROL && following ? FOLLOW_SETTLE_CYCLES : SETTLE_CYCLES;
 wire in_band=wide_error<=$signed(allowed_error) && wide_error>=-$signed(allowed_error);
 // Fast path shares origin,MC,budget,timeouts and latched fault logic with manual.
 reg [31:0] fast_ms=0,fast_age=0;reg [15:0] fast_credit=0;
 reg [7:0] fast_count=0;reg signed [15:0] fast_commands=0;
 reg signed [32:0] fast_anchor=0;reg fast_dir_wait=0;
 wire fast_active=FAST_CAPABLE && fast_mode;
 wire [15:0] fast_magnitude=fast_rate_hz<0 ? -fast_rate_hz : fast_rate_hz;
 wire fast_positive=fast_rate_hz>0;
 wire fast_tick=fast_ms==FAST_MS_CYCLES-1;
 wire fast_due=fast_count!=0 && fast_age>=FAST_MS_CYCLES*20-1;
 wire signed [32:0] fast_displacement=wide_position-fast_anchor;
 wire fast_wrong=(fast_commands>0 && fast_displacement<0) || (fast_commands<0 && fast_displacement>0);
 // Checkpoints never change origin,MC,limits or clear a fault. Each segment must
 // first drain real feedback; compare its net motion to the measured1.28 ratio.
 reg signed [31:0] checkpoint_p=0,checkpoint_m=0;
 // Drain the validated20ms feedback latency before a DIR reversal. The quiet
 // clock runs across segment boundaries; no pending feedback is forgiven.
 reg [31:0] fast_quiet_age=FAST_MS_CYCLES*20;reg fast_step_previous=0;
 wire signed [CB:0] checkpoint_dp=$signed(position[CB-1:0])-$signed(checkpoint_p[CB-1:0]);
 wire signed [CB:0] checkpoint_dm=$signed(command_position[CB-1:0])-$signed(checkpoint_m[CB-1:0]);
 wire signed [CB+9:0] checkpoint_pw={{9{checkpoint_dp[CB]}},checkpoint_dp};
 wire signed [CB+9:0] checkpoint_mw={{9{checkpoint_dm[CB]}},checkpoint_dm};
 wire signed [CB+9:0] checkpoint_difference=(checkpoint_pw<<<8)-((checkpoint_mw<<<8)+(checkpoint_mw<<<6)+(checkpoint_mw<<<3));
 // Optional regional-mode tail settlement: only after40ms real quiet,
 // at most2 pending pulses, at least4 net pulses with measured feedback,
 // and a tighter whole-segment residual <=1.5 encoder counts. Never applies
 // to3 consecutive no-response pulses, no movement, wrong direction or arrival.
 wire verified_tail=TAIL_CHECKPOINT && checkpoint && fast_rate_hz==0 && fast_quiet_age>=FAST_MS_CYCLES*40 &&
  fast_count>0 && fast_count<NO_RESPONSE_LIMIT && fast_commands!=0 && wrong_direction==0 &&
  (checkpoint_dm>=4 || checkpoint_dm<=-4) && (checkpoint_dp>=2 || checkpoint_dp<=-2) &&
  checkpoint_difference<=19'sd384 && checkpoint_difference>=-19'sd384;
 wire checkpoint_consistent=checkpoint_difference<=19'sd1024 && checkpoint_difference>=-19'sd1024 && wrong_direction==0;


 wire manual_pending=active && (state==HIGH_PHASE || state==LOW_PHASE || state==SETTLE);
 assign diag_pending=fast_active ? fast_count : {7'd0,manual_pending};
 assign diag_commands=fast_active ? fast_commands : (manual_pending ? (dir_r ? 16'sd1 : -16'sd1) : 16'sd0);
 assign diag_delta=fast_active ? fast_displacement[15:0] : displacement[15:0];
 assign diag_consistent=checkpoint_consistent;assign diag_wrong=wrong_direction!=0;
 assign position=wide_position[31:0];assign error=wide_error[31:0];
 assign motor_en=rst_n & enable_raw & enabled & (HOLD_ON_FAULT || ((fault==0) && (encoder_errors==0)));
 assign motor_dir=motor_en & dir_r;
 assign motor_step=motor_en & step_r & permit & (fault==0) & (encoder_errors==0);
 assign ready=motor_en && origin_valid && state==READY && !active;
 assign busy=motor_en && active;
 assign reached=ready && done && in_band;
 always @(posedge clk)begin
  if(!rst_n)key_armed<=0;
  else if(!ready || !permit || press_event)key_armed<=0;
  else if(stable_valid && !stable_pressed)key_armed<=1;
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   enabled<=0;origin_valid<=0;origin<=0;active<=0;done<=0;dir_r<=0;step_r<=0;
   state<=OFF;fault<=0;command_position<=0;locked_target<=0;spent<=0;
   cancel_reason<=0;reject_reason<=0;accepted_count<=0;completed_count<=0;
   enable_count<=0;phase_count<=0;action_timer<=0;before_position<=0;no_response<=0;wrong_direction<=0;
   fast_ms<=0;fast_age<=0;fast_credit<=0;fast_count<=0;fast_commands<=0;fast_anchor<=0;fast_dir_wait<=0;checkpoint_p<=0;checkpoint_m<=0;fast_quiet_age<=FAST_MS_CYCLES*20;fast_step_previous<=0;
  end else begin
   fast_step_previous<=motor_step;
   if(motor_step && !fast_step_previous)fast_quiet_age<=0;
   else if(fast_quiet_age<FAST_MS_CYCLES*(TAIL_CHECKPOINT ? 40 : 20))fast_quiet_age<=fast_quiet_age+1'b1;
   if(!fast_active || !active || !enable_raw || !permit || fault!=0)begin
    fast_ms<=0;fast_age<=0;fast_credit<=0;fast_count<=0;fast_commands<=0;
   end else begin
    if(fast_tick)begin
     fast_ms<=0;
     if(fast_rate_hz==0)fast_credit<=0;
     else if(fast_credit+fast_magnitude>=1000)fast_credit<=1000;
     else fast_credit<=fast_credit+fast_magnitude;
    end else fast_ms<=fast_ms+1'b1;
    if(fast_count!=0)fast_age<=fast_age+1'b1;
   end
   // Record rejected edges even while disabled/busy. No queue or held-key replay.
   if(command_event)begin
    if(fault!=0 || encoder_errors!=0)reject_reason<=9;
    else if(!enable_raw)reject_reason<=1;
    else if(busy)reject_reason<=6;
    else if(!ready)reject_reason<=2;
    else if(!settings_ready || config_changed)reject_reason<=5;
    else if(visual_stale)reject_reason<=4;
    else if(!visual_valid)reject_reason<=3;
    else if(!command_armed)reject_reason<=8;
    else if(visual_target>tmax || visual_target<tmin)reject_reason<=7;
   end
   if(fault!=0)begin if(!HOLD_ON_FAULT)enabled<=0;active<=0;done<=0;step_r<=0;state<=FAULT;cancel_reason<=6;end
   else if(encoder_errors!=0)begin fault<=3;if(!HOLD_ON_FAULT)enabled<=0;active<=0;done<=0;step_r<=0;state<=FAULT;cancel_reason<=6;end
   else if(!enable_raw)begin
    enabled<=0;active<=0;done<=0;step_r<=0;state<=OFF;enable_count<=0;phase_count<=0;action_timer<=0;
    if(active)cancel_reason<=(!k4_n ? 2 : 1);
   end else if(!enabled)begin
    state<=ARM;step_r<=0;active<=0;done<=0;
    if(enable_count==ENABLE_CYCLES-1)begin enabled<=1;state<=(origin_valid ? READY : ALIGN);enable_count<=0;phase_count<=0;end
    else enable_count<=enable_count+1'b1;
   end else if(!origin_valid)begin
    state<=ALIGN;step_r<=0;
    if(phase_count==ENABLE_CYCLES-1)begin origin<=encoder_count;origin_valid<=1;phase_count<=0;state<=READY;end
    else phase_count<=phase_count+1'b1;
   end else if(wide_position>pmax || wide_position<pmin)begin
    fault<=7;step_r<=0;active<=0;done<=0;state<=FAULT;cancel_reason<=6;
   end else if(active && action_timer>=TIMEOUT_CYCLES-1)begin
    fault<=4;step_r<=0;active<=0;done<=0;state<=FAULT;cancel_reason<=6;
   end else if(!permit)begin
    step_r<=0;active<=0;state<=READY;phase_count<=0;action_timer<=0;
    if(active)begin done<=0;cancel_reason<=extended_cancel ? external_cancel_reason : (config_changed || !settings_ready ? 5 : (visual_stale ? 4 : 3));end
   end else if(fast_active)begin
    if(active)action_timer<=action_timer+1'b1;
    // Check accumulated physical response after20ms, independent of pulse timing.
    if(fast_due)begin
     fast_age<=0;
     if(fast_displacement==0)begin
      if(fast_count>=NO_RESPONSE_LIMIT)begin fault<=1;step_r<=0;active<=0;state<=FAULT;cancel_reason<=6;end
     end else begin
      fast_count<=0;fast_commands<=0;fast_anchor<=wide_position;
      if(fast_wrong)begin
       wrong_direction<=wrong_direction+1'b1;
       if(wrong_direction>=1)begin fault<=2;step_r<=0;active<=0;state<=FAULT;cancel_reason<=6;end
      end else begin wrong_direction<=0;no_response<=0;end
     end
    end else case(state)
     READY:begin
      step_r<=0;phase_count<=0;
      if(command_event && command_armed && visual_target<=tmax && visual_target>=tmin)begin
       locked_target<={{(32-TARGET_BITS){visual_target[TARGET_BITS-1]}},visual_target};spent<=0;no_response<=0;wrong_direction<=0;
       action_timer<=0;active<=1;done<=0;state<=CHECK;cancel_reason<=0;reject_reason<=0;accepted_count<=accepted_count+1'b1;checkpoint_p<=position;checkpoint_m<=command_position;
      end
     end
     CHECK:begin
      step_r<=0;
      if(CHECKPOINT_CAPABLE && checkpoint && following && visual_target<=tmax && visual_target>=tmin && fast_rate_hz==0 && (fast_count==0 || verified_tail) && checkpoint_consistent)begin
       // A verified quiet segment boundary is not a position-arrived event.
       active<=0;done<=0;state<=READY;phase_count<=0;completed_count<=completed_count+1'b1;
       if(verified_tail)begin fast_count<=0;fast_commands<=0;fast_anchor<=wide_position;fast_age<=0;end
      end else if(visual_target>tmax || visual_target<tmin || fast_magnitude>200)begin fault<=8;active<=0;state<=FAULT;cancel_reason<=6;end
      else if(locked_target!={{(32-TARGET_BITS){visual_target[TARGET_BITS-1]}},visual_target})begin
       locked_target<={{(32-TARGET_BITS){visual_target[TARGET_BITS-1]}},visual_target};
       // Target noise inside the arrival band cannot erase a real stationary dwell.
       if($signed(visual_target)-wide_position>$signed(allowed_error) || $signed(visual_target)-wide_position<-$signed(allowed_error) || fast_rate_hz!=0 || fast_count!=0)phase_count<=0;
      end
      else if(in_band && fast_rate_hz==0 && fast_count==0)begin
       // A real100ms stationary endpoint closes an activity segment; never a frame tick.
       if(phase_count>=FAST_MS_CYCLES*100-1)begin active<=0;done<=1;state<=READY;phase_count<=0;completed_count<=completed_count+1'b1;end
       else phase_count<=phase_count+1'b1;
      end else if(spent>=MOVE_BUDGET && fast_rate_hz!=0)begin fault<=5;active<=0;state<=FAULT;cancel_reason<=6;end
      else if(fast_rate_hz!=0 && fast_credit>=1000)begin
       phase_count<=0;
       if((fast_positive && command_position>=mmax) || (!fast_positive && command_position<=mmin))begin fault<=6;active<=0;state<=FAULT;cancel_reason<=6;end
       else if(fast_positive==dir_r || (fast_count==0 && fast_quiet_age>=FAST_MS_CYCLES*20))begin fast_dir_wait<=fast_positive!=dir_r;dir_r<=fast_positive;state<=SETUP;end
      end else phase_count<=0;
     end
     SETUP:begin
      if(fast_rate_hz==0 || fast_positive!=dir_r)begin state<=CHECK;phase_count<=0;end
      else if(!fast_dir_wait || phase_count>=FOLLOW_SETUP_CYCLES-1)begin
       phase_count<=0;step_r<=1;command_position<=command_position+(dir_r ? 32'sd1 : -32'sd1);spent<=spent+1'b1;
       fast_credit<=0;
       if(fast_count==0)begin fast_anchor<=wide_position;fast_age<=0;end
       fast_count<=fast_count+1'b1;fast_commands<=fast_commands+(dir_r ? 16'sd1 : -16'sd1);state<=HIGH_PHASE;
      end else phase_count<=phase_count+1'b1;
     end
     HIGH_PHASE:begin if(phase_count==HALF_CYCLES-1)begin phase_count<=0;step_r<=0;state<=LOW_PHASE;end else phase_count<=phase_count+1'b1;end
     LOW_PHASE:begin if(phase_count==HALF_CYCLES-1)begin phase_count<=0;state<=CHECK;end else phase_count<=phase_count+1'b1;end
     default:begin fault<=8;step_r<=0;active<=0;state<=FAULT;cancel_reason<=6;end
    endcase
   end else begin
    if(active)action_timer<=action_timer+1'b1;
    case(state)
     READY:begin
      step_r<=0;phase_count<=0;
      if(command_event && command_armed && visual_target<=tmax && visual_target>=tmin)begin
       locked_target<={{(32-TARGET_BITS){visual_target[TARGET_BITS-1]}},visual_target};spent<=0;no_response<=0;wrong_direction<=0;
       action_timer<=0;active<=1;done<=0;state<=CHECK;cancel_reason<=0;reject_reason<=0;
       accepted_count<=accepted_count+1'b1;
      end
     end
     CHECK:begin
      if(EXTERNAL_CONTROL && following && (visual_target>tmax || visual_target<tmin))begin
       fault<=8;step_r<=0;active<=0;done<=0;state<=FAULT;cancel_reason<=6;
      end else if(EXTERNAL_CONTROL && following && locked_target!={{(32-TARGET_BITS){visual_target[TARGET_BITS-1]}},visual_target})begin
       // Retarget only after the previous pulse/feedback phase. Never reset budget/time.
       locked_target<={{(32-TARGET_BITS){visual_target[TARGET_BITS-1]}},visual_target};
      end else if(in_band)begin active<=0;done<=1;state<=READY;completed_count<=completed_count+1'b1;end
      else if(spent>=MOVE_BUDGET)begin fault<=5;step_r<=0;active<=0;state<=FAULT;cancel_reason<=6;end
      else if((wide_error>0 && command_position>=mmax) || (wide_error<0 && command_position<=mmin))begin
       fault<=6;step_r<=0;active<=0;state<=FAULT;cancel_reason<=6;
      end else begin dir_r<=wide_error>0;before_position<=wide_position;phase_count<=0;state<=SETUP;end
     end
     SETUP:begin
      if(phase_count==setup_limit-1)begin
       phase_count<=0;step_r<=1;command_position<=command_position+(dir_r ? 32'sd1 : -32'sd1);
       spent<=spent+1'b1;state<=HIGH_PHASE;
      end else phase_count<=phase_count+1'b1;
     end
     HIGH_PHASE:begin
      if(phase_count==HALF_CYCLES-1)begin phase_count<=0;step_r<=0;state<=LOW_PHASE;end
      else phase_count<=phase_count+1'b1;
     end
     LOW_PHASE:begin
      if(phase_count==HALF_CYCLES-1)begin phase_count<=0;state<=SETTLE;end
      else phase_count<=phase_count+1'b1;
     end
     SETTLE:begin
      if(phase_count==settle_limit-1)begin
       phase_count<=0;
       if(displacement==0)begin
        no_response<=no_response+1'b1;
        if(no_response>=NO_RESPONSE_LIMIT-1)begin fault<=1;step_r<=0;active<=0;state<=FAULT;cancel_reason<=6;end
        else state<=CHECK;
       end else if((dir_r && displacement<0) || (!dir_r && displacement>0))begin
        wrong_direction<=wrong_direction+1'b1;no_response<=0;
        if(wrong_direction>=1)begin fault<=2;step_r<=0;active<=0;state<=FAULT;cancel_reason<=6;end
        else state<=CHECK;
       end else begin no_response<=0;wrong_direction<=0;state<=CHECK;end
      end else phase_count<=phase_count+1'b1;
     end
     default:begin fault<=8;step_r<=0;active<=0;done<=0;state<=FAULT;cancel_reason<=6;end
    endcase
   end
  end
 end
endmodule

// Source: vision_result_guard.v
// Separate from raw VIS validity. Only real qualified frames refresh this timer.
module vision_result_guard #(parameter integer MS_CYCLES=50000,LOSS_GRACE_MS=500)(
 input wire clk,rst_n,clear,frame,valid,
 output wire hold_valid
);
 localparam integer LIMIT=MS_CYCLES*LOSS_GRACE_MS;
 reg seen=0;reg [31:0] elapsed=0;
 assign hold_valid=rst_n && !clear && seen && elapsed<LIMIT;
 always @(posedge clk)begin
  if(!rst_n || clear)begin seen<=0;elapsed<=0;end
  else if(frame && valid)begin seen<=1;elapsed<=0;end
  else if(seen && elapsed<LIMIT)elapsed<=elapsed+1'b1;
 end
endmodule

// Source: vision_follow_control.v
// Frame-based P + filtered velocity damping; no integral or unbounded pulse queue.
module vision_follow_control #(parameter integer MS_CYCLES=50000,CONTROL_MS=20,FOLLOW_DEADBAND=1)(
 input wire clk,rst_n,key_event,k4_n,enable_request,config_changed,
 input wire visual_valid,follow_valid,visual_stale,settings_ready,center_confirmed,frame,
 input wire neutral_capture,
 input wire [10:0] x,input wire [15:0] center,deadzone,runtime_seconds,
 input wire [7:0] gain,limit,mode,damping,slew,
 input wire signed [7:0] manual_target,
 input wire core_ready,core_busy,input wire [3:0] fault,input wire step,
 input wire signed [31:0] position,
 output reg request=0,output wire following,external_cancel,neutral_ready,
 output wire [3:0] external_cancel_reason,
 output reg run=0,output reg [3:0] stop_reason=0,reject_reason=0,
 output reg signed [7:0] drive_target=0,
 output wire signed [7:0] candidate,
 output reg signed [15:0] velocity=0,
 output reg [15:0] elapsed_seconds=0,output reg [31:0] total_pulses=0
);
 reg history=0,pending_frame=0,request_wait=0,step_previous=0;
 reg [10:0] previous_x=0;
 reg signed [7:0] held_candidate=0;
 // Separate control neutral; the executor's encoder origin/MC never changes here.
 reg signed [7:0] neutral_reference=0;
 reg neutral_in_range=1;
 assign neutral_ready=neutral_in_range;
 reg signed [7:0] target_for_drive=0;
 reg target_pipe_valid=0;
 reg [31:0] ms_count=0;reg [15:0] second_ms=0,control_ms=0;
 reg [6:0] slew_remainder=0;
 wire signed [31:0] error_pixels=$signed({21'd0,x})-$signed({16'd0,center});
 wire signed [31:0] raw_delta=$signed({21'd0,x})-$signed({21'd0,previous_x});
 wire signed [31:0] delta=raw_delta>256 ? 32'sd256 : (raw_delta< -256 ? -32'sd256 : raw_delta);
 // VE is signed Q4 pixels/frame, preserving slow motion for damping.
 wire signed [31:0] derivative=($signed(velocity)*$signed({1'b0,damping}))/16;
 wire signed [31:0] predicted=error_pixels+derivative;
 wire [31:0] magnitude=predicted<0 ? -predicted : predicted;
 wire [31:0] outside=magnitude>deadzone ? magnitude-deadzone : 32'd0;
 // Round away from zero and clear the inner +/-1 arrival band outside DZ.
 // No integral, growing offset or minimum drive inside the configured deadzone.
 wire [31:0] rounded=outside==0 ? 32'd0 : (((outside-1'b1) >> gain[6:0])+1'b1);
 wire [31:0] scaled=outside!=0 && rounded<2 ? 32'd2 : rounded;
 wire [7:0] bounded=scaled>limit ? limit : scaled[7:0];
 wire negative=(predicted<0)^gain[7];
 wire signed [7:0] pd_target=negative ? -$signed(bounded) : $signed(bounded);
 wire signed [8:0] neutral_sum=$signed({neutral_reference[7],neutral_reference})+$signed({pd_target[7],pd_target});
 wire signed [7:0] absolute_target=neutral_sum>60 ? 8'sd60 : (neutral_sum< -60 ? -8'sd60 : neutral_sum[7:0]);
 wire coast=run && mode==1 && !visual_valid;
 wire permit=(run && mode==1 ? follow_valid : visual_valid) && !visual_stale && settings_ready && !config_changed && !neutral_capture;
 assign candidate=!permit ? 8'sd0 : (mode==1 ? (coast ? held_candidate : absolute_target) : (mode==2 ? 8'sd0 : manual_target));
 wire expired=run && runtime_seconds!=0 && elapsed_seconds>=runtime_seconds;
 wire stop_button=run && key_event;
 assign following=run && mode==1;
 assign external_cancel=stop_button || expired;
 assign external_cancel_reason=expired ? 4'd8 : 4'd7;
 wire signed [31:0] drive_error=$signed(drive_target)-position;
 wire signed [31:0] target_delta=$signed(target_for_drive)-$signed(drive_target);
 // S.SL remains counts/100ms on the wire, independent of the control tick.
 // Supported CONTROL_MS 1..100 and SL 1..8 imply at most eight counts/tick.
 wire [15:0] rate_units=slew_remainder+slew*CONTROL_MS;
 reg [3:0] rate_step;
 integer rate_index;
 always @* begin
  rate_step=0;
  for(rate_index=1;rate_index<=8;rate_index=rate_index+1)
   if(rate_units>=rate_index*100)rate_step=rate_index;
 end
 wire signed [31:0] slew_wide=$signed({1'b0,rate_step});
 wire tick_ms=ms_count==MS_CYCLES-1;
 always @(posedge clk)begin
  if(!rst_n)begin
   request<=0;run<=0;drive_target<=0;velocity<=0;history<=0;pending_frame<=0;
   previous_x<=0;held_candidate<=0;neutral_reference<=0;neutral_in_range<=1;target_for_drive<=0;target_pipe_valid<=0;request_wait<=0;stop_reason<=0;reject_reason<=0;elapsed_seconds<=0;
   ms_count<=0;second_ms<=0;control_ms<=0;total_pulses<=0;step_previous<=0;slew_remainder<=0;
  end else begin
   request<=0;step_previous<=step;
   // One50MHz stage separates PD/neutral arithmetic from slew arithmetic.
   // Only real, configured vision may fill it; loss never releases stale work.
   if(!visual_valid || visual_stale || !settings_ready || config_changed || neutral_capture)target_pipe_valid<=0;
   else begin target_for_drive<=absolute_target;target_pipe_valid<=1;end
   if(visual_valid && !visual_stale && settings_ready && !config_changed && !neutral_capture)held_candidate<=absolute_target;
   if(step && !step_previous)total_pulses<=total_pulses+1'b1;
   if(core_busy || !core_ready)request_wait<=0;
   if(!run || !permit)begin history<=0;velocity<=0;pending_frame<=0;ms_count<=0;control_ms<=0;second_ms<=0;slew_remainder<=0;end
   else begin
    // Loss freezes the reference, discards queued updates and restarts velocity
    // history. It does not pause the runtime or the executor burst guards.
    if(!visual_valid)begin history<=0;velocity<=0;pending_frame<=0;slew_remainder<=0;end
    else if(frame)begin
     previous_x<=x;history<=1;pending_frame<=1;
     if(history)velocity<=($signed(velocity)*3+delta*16)/4;
     else velocity<=0;
    end
    if(tick_ms)begin
     ms_count<=0;
     if(second_ms==999)begin second_ms<=0;if(elapsed_seconds!=65535)elapsed_seconds<=elapsed_seconds+1'b1;end
     else second_ms<=second_ms+1'b1;
     if(control_ms==CONTROL_MS-1)begin
      control_ms<=0;
      if(pending_frame && visual_valid && target_pipe_valid)begin
       // Continue the ramp toward the last real result between camera frames.
       // Consume rate credit even at target; no stored burst after a quiet spell.
       slew_remainder<=rate_units-rate_step*100;
       if(target_delta>slew_wide)drive_target<=$signed(drive_target)+$signed({1'b0,rate_step});
       else if(target_delta< -slew_wide)drive_target<=$signed(drive_target)-$signed({1'b0,rate_step});
       else drive_target<=target_for_drive;
      end
     end else control_ms<=control_ms+1'b1;
    end else ms_count<=ms_count+1'b1;
   end
   if(run && (fault!=0 || !enable_request || !k4_n || !permit || expired || stop_button))begin
    run<=0;request<=0;request_wait<=0;
    if(fault!=0)stop_reason<=6;
    else if(!k4_n)stop_reason<=2;
    else if(!enable_request)stop_reason<=1;
    else if(config_changed || !settings_ready)stop_reason<=5;
    else if(visual_stale)stop_reason<=4;
    else if(!follow_valid)stop_reason<=3;
    else stop_reason<=external_cancel_reason;
   end else if(key_event && !run)begin
    if(fault!=0)reject_reason<=9;
    else if(!enable_request)reject_reason<=1;
    else if(!core_ready)reject_reason<=core_busy ? 6 : 2;
    else if(config_changed || !settings_ready || neutral_capture)reject_reason<=5;
    else if(visual_stale)reject_reason<=4;
    else if(!visual_valid)reject_reason<=3;
    else if(mode==1 && !center_confirmed)reject_reason<=10;
    else if(mode==1 && !neutral_in_range)reject_reason<=11;
    else if(candidate>60 || candidate< -60)reject_reason<=7;
    else begin
     reject_reason<=0;stop_reason<=0;
     if(mode==1)begin
      run<=1;elapsed_seconds<=0;total_pulses<=0;history<=0;pending_frame<=0;velocity<=0;slew_remainder<=0;
      drive_target<=position>60 ? 8'sd60 : (position< -60 ? -8'sd60 : position[7:0]);
      // Synchronize the idle executor target with current P; an in-band request
      // completes without STEP and makes MT/ME reflect this new session.
      request<=1;request_wait<=1;
     end else begin request<=1;request_wait<=1;drive_target<=candidate;end
    end
   end else if(run && permit && core_ready && !request_wait && (drive_error>FOLLOW_DEADBAND || drive_error< -FOLLOW_DEADBAND))begin
    request<=1;request_wait<=1;
   end
   // K3 has already passed the monitor's true-frame, idle and collision checks.
   // Capture never requests motion or rebases the actuator's safety coordinates.
   if(neutral_capture && !run)begin
    history<=0;velocity<=0;pending_frame<=0;slew_remainder<=0;request<=0;request_wait<=0;target_pipe_valid<=0;
    if(position<=60 && position>=-60)begin
     neutral_reference<=position[7:0];neutral_in_range<=1;drive_target<=position[7:0];
    end else neutral_in_range<=0;
   end
  end
 end
endmodule

// Source: vision_cdc_bridge.v
// Coherent held-data request/ack crossing. Both reset inputs share assertion.
module vision_cdc_bridge #(parameter WIDTH=241)(
    input wire src_clk,src_rst_n,src_frame,
    input wire [WIDTH-1:0] src_data,
    input wire dst_clk,dst_rst_n,
    output reg dst_frame=0,
    output reg [WIDTH-1:0] dst_data=0
);
    reg [WIDTH-1:0] held=0;
    reg request=0,ack=0,seen=0;
    reg [1:0] ack_sync=0,request_sync=0;
    always @(posedge src_clk or negedge src_rst_n) begin
        if(!src_rst_n) begin held<=0;request<=0;ack_sync<=0;end
        else begin
            ack_sync<={ack_sync[0],ack};
            if(src_frame && request==ack_sync[1]) begin held<=src_data;request<=~request;end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if(!dst_rst_n) begin request_sync<=0;seen<=0;ack<=0;dst_frame<=0;dst_data<=0;end
        else begin
            request_sync<={request_sync[0],request};dst_frame<=0;
            if(request_sync[1]!=seen) begin
                dst_data<=held;seen<=request_sync[1];ack<=request_sync[1];dst_frame<=1;
            end
        end
    end
endmodule

// Source: vision_config_rx.v
// 115200 8N1. P + seq16/center16/deadzone16/gain8/limit8/checksum8 as HEX + LF.
// XOR checksum covers the eight payload bytes; commit only on the terminating LF.
module vision_config_rx #(parameter integer UART_DIV=434,PACKET_TIMEOUT_CYCLES=5000000,DEFAULT_GAIN=3,DEFAULT_LIMIT=32,DEFAULT_MODE=1,DEFAULT_DEADZONE=5,DEFAULT_SLEW=8,DEFAULT_RUNTIME=60)(
    input wire clk,rst_n,rx,
    output reg [15:0] cfg_sequence=0,center=960,deadzone=DEFAULT_DEADZONE,
    output reg [7:0] gain=DEFAULT_GAIN,limit=DEFAULT_LIMIT,
    output reg [15:0] errors=0,
    output reg applied=0,
    output reg [15:0] cam_sequence=0,exposure=3600,sensor_gain=143,
    output reg [15:0] roi_left=0,roi_top=0,roi_right=1279,roi_bottom=719,
    input wire center_capture,input wire [15:0] captured_center,
    output reg center_applied=0,center_conflict=0,parameter_applied=0,
    output reg [15:0] follow_sequence=0,runtime_seconds=DEFAULT_RUNTIME,
    output reg [7:0] operating_mode=DEFAULT_MODE,damping=12,slew=DEFAULT_SLEW
);
    reg sync1=1,sync2=1;
    reg [1:0] rx_state=0;
    reg [15:0] baud_count=0;
    reg [2:0] bit_index=0;
    reg [7:0] shift=0,byte_value=0;
    reg byte_valid=0;
    always @(posedge clk) begin
        if(!rst_n) begin
            sync1<=1;sync2<=1;rx_state<=0;baud_count<=0;bit_index<=0;shift<=0;byte_value<=0;byte_valid<=0;
        end else begin
            sync1<=rx;sync2<=sync1;byte_valid<=0;
            case(rx_state)
                0:if(!sync2) begin baud_count<=UART_DIV/2-1;rx_state<=1;end
                1:if(baud_count==0) begin
                    if(!sync2) begin rx_state<=2;baud_count<=UART_DIV-1;bit_index<=0;end
                    else rx_state<=0;
                end else baud_count<=baud_count-1'b1;
                2:if(baud_count==0) begin
                    shift[bit_index]<=sync2;baud_count<=UART_DIV-1;
                    if(bit_index==7) rx_state<=3;else bit_index<=bit_index+1'b1;
                end else baud_count<=baud_count-1'b1;
                3:if(baud_count==0) begin
                    rx_state<=0;
                    if(sync2) begin byte_value<=shift;byte_valid<=1;end
                end else baud_count<=baud_count-1'b1;
            endcase
        end
    end
    function [4:0] hex_digit;
        input [7:0] ch;
        begin
            if(ch>="0" && ch<="9")hex_digit={1'b1,ch[3:0]};
            else if(ch>="A" && ch<="F")hex_digit={1'b1,ch[3:0]+4'd9};
            else if(ch>="a" && ch<="f")hex_digit={1'b1,ch[3:0]+4'd9};
            else hex_digit=0;
        end
    endfunction
    reg collecting=0,discard=0;
    reg [4:0] digits=0;
    reg [119:0] packet=0;
    reg camera_packet=0,control_packet=0;
    reg [31:0] packet_age=0;
    wire [4:0] digit=hex_digit(byte_value);
    wire [7:0] checksum=packet[71:64]^packet[63:56]^packet[55:48]^packet[47:40]^
        packet[39:32]^packet[31:24]^packet[23:16]^packet[15:8];
    wire bounds_ok=packet[55:40]<=1919 && packet[39:24]<=200 &&
        packet[22:16]<=12 && packet[15:8]<=60;
    wire [7:0] camera_checksum=packet[119:112]^packet[111:104]^packet[103:96]^packet[95:88]^
        packet[87:80]^packet[79:72]^packet[71:64]^packet[63:56]^packet[55:48]^packet[47:40]^
        packet[39:32]^packet[31:24]^packet[23:16]^packet[15:8];
    wire camera_bounds=packet[103:88]>=2 && packet[103:88]<=3600 &&
        packet[87:72]>=16 && packet[87:72]<=143 &&
        packet[71:56]<packet[39:24] && packet[39:24]<=1279 &&
        packet[55:40]<packet[23:8] && packet[23:8]<=719;
    wire [7:0] follow_checksum=packet[63:56]^packet[55:48]^packet[47:40]^packet[39:32]^packet[31:24]^packet[23:16]^packet[15:8];
    wire follow_bounds=packet[47:40]<=2 && packet[39:32]<=16 && packet[31:24]>=1 && packet[31:24]<=8 && packet[23:8]<=300;
    wire expired=collecting && packet_age>=PACKET_TIMEOUT_CYCLES-1;
    // A valid serial commit wins a same-cycle K3 collision. One center owner.
    wire uart_commit=byte_valid && !expired && collecting && !discard && byte_value==8'h0a &&
        ((!camera_packet && !control_packet && digits==18 && checksum==packet[7:0] && bounds_ok) ||
         (camera_packet && digits==30 && camera_checksum==packet[7:0] && camera_bounds) ||
         (control_packet && digits==16 && follow_checksum==packet[7:0] && follow_bounds));
    always @(posedge clk) begin
        if(!rst_n) begin
            cfg_sequence<=0;center<=960;deadzone<=DEFAULT_DEADZONE;gain<=DEFAULT_GAIN;limit<=DEFAULT_LIMIT;errors<=0;applied<=0;
            cam_sequence<=0;exposure<=3600;sensor_gain<=143;
            follow_sequence<=0;runtime_seconds<=DEFAULT_RUNTIME;operating_mode<=DEFAULT_MODE;damping<=12;slew<=DEFAULT_SLEW;control_packet<=0;
            roi_left<=0;roi_top<=0;roi_right<=1279;roi_bottom<=719;camera_packet<=0;
            collecting<=0;discard<=0;digits<=0;packet<=0;packet_age<=0;center_applied<=0;center_conflict<=0;parameter_applied<=0;
        end else begin
            applied<=0;center_applied<=0;center_conflict<=0;parameter_applied<=0;
            if(center_capture && captured_center<=1919)begin
                if(uart_commit)center_conflict<=1;
                else begin center<=captured_center;applied<=1;center_applied<=1;end
            end
            if(collecting) begin
                if(packet_age>=PACKET_TIMEOUT_CYCLES-1) begin
                    collecting<=0;discard<=!(byte_valid && byte_value==8'h0a);packet_age<=0;
                    if(errors!=16'hffff)errors<=errors+1'b1;
                end else packet_age<=packet_age+1'b1;
            end
            if(byte_valid && !expired) begin
                if(discard) begin
                    if(byte_value==8'h0a)discard<=0;
                end else if(!collecting) begin
                    if(byte_value=="P" || byte_value=="V" || byte_value=="S") begin
                        collecting<=1;digits<=0;packet<=0;packet_age<=0;camera_packet<=(byte_value=="V");control_packet<=(byte_value=="S");
                    end
                end else if(byte_value==8'h0a) begin
                    collecting<=0;packet_age<=0;
                    if(!camera_packet && !control_packet && digits==18 && checksum==packet[7:0] && bounds_ok) begin
                        cfg_sequence<=packet[71:56];center<=packet[55:40];deadzone<=packet[39:24];
                        gain<=packet[23:16];limit<=packet[15:8];applied<=1;parameter_applied<=1;
                    end else if(camera_packet && digits==30 && camera_checksum==packet[7:0] && camera_bounds)begin
                        applied<=1;cam_sequence<=packet[119:104];exposure<=packet[103:88];sensor_gain<=packet[87:72];
                        roi_left<=packet[71:56];roi_top<=packet[55:40];roi_right<=packet[39:24];roi_bottom<=packet[23:8];
                    end else if(control_packet && digits==16 && follow_checksum==packet[7:0] && follow_bounds)begin
                        follow_sequence<=packet[63:48];operating_mode<=packet[47:40];damping<=packet[39:32];
                        slew<=packet[31:24];runtime_seconds<=packet[23:8];applied<=1;
                    end else if(errors!=16'hffff)errors<=errors+1'b1;
                end else if(digit[4] && digits<(control_packet ? 16 : (camera_packet ? 30 : 18))) begin
                    packet<={packet[115:0],digit[3:0]};digits<=digits+1'b1;
                end else begin
                    collecting<=0;discard<=1;packet_age<=0;
                    if(errors!=16'hffff)errors<=errors+1'b1;
                end
            end
        end
    end
endmodule

// Source: vision_center_key.v
// Debounced K3 edge, release-qualified after reset. A held key never replays.
module vision_center_key #(parameter integer DEBOUNCE_CYCLES=1000000)(
 input wire clk,rst_n,key_n,output reg press_event=0
);
 reg sync1=0,sync2=0,candidate=0,stable_pressed=0,stable_valid=0;
 reg previous_pressed=0,armed=0;reg [31:0] debounce_count=0;
 always @(posedge clk)begin
  if(!rst_n)begin
   sync1<=0;sync2<=0;candidate<=0;stable_pressed<=0;stable_valid<=0;
   previous_pressed<=0;armed<=0;debounce_count<=0;press_event<=0;
  end else begin
   sync1<=~key_n;sync2<=sync1;press_event<=0;
   if(sync2!=candidate)begin candidate<=sync2;debounce_count<=0;stable_valid<=0;end
   else if(debounce_count<DEBOUNCE_CYCLES-1)debounce_count<=debounce_count+1'b1;
   else begin stable_pressed<=candidate;stable_valid<=1;end
   if(stable_valid)begin
    previous_pressed<=stable_pressed;
    if(!stable_pressed)armed<=1;
    else if(!previous_pressed)begin
     if(armed)press_event<=1;
     armed<=0;
    end
   end
  end
 end
endmodule

// Source: vision_shadow_target.v
module vision_shadow_target #(parameter integer MS_CYCLES=50000,STALE_MS=200)(
    input wire clk,rst_n,frame,
    input wire [10:0] ball_x,ball_y,
    input wire confirmed,bad_frame,
    input wire [3:0] confidence,
    input wire [15:0] center,deadzone,
    input wire [7:0] gain,limit,
    output reg [10:0] x=0,y=0,
    output wire valid,
    output reg [15:0] age_ms=16'hffff,frames=0,
    output wire signed [7:0] target,
    output reg [3:0] frame_confidence=0,
    output reg frame_bad=0
);
    reg [31:0] ms_count=0;
    reg accepted=0;
    always @(posedge clk) begin
        if(!rst_n)begin x<=0;y<=0;age_ms<=16'hffff;frames<=0;ms_count<=0;accepted<=0;frame_confidence<=0;frame_bad<=0;end
        else begin
            if(ms_count==MS_CYCLES-1) begin
                ms_count<=0;if(age_ms!=16'hffff)age_ms<=age_ms+1'b1;
            end else ms_count<=ms_count+1'b1;
            if(frame)begin
                x<=ball_x;y<=ball_y;age_ms<=0;ms_count<=0;frames<=frames+1'b1;
                frame_confidence<=confidence;frame_bad<=bad_frame;
                accepted<=confirmed && !bad_frame && confidence>=8 && ball_x<=1919 && ball_y<=1079;
            end
        end
    end
    assign valid=accepted && age_ms<STALE_MS;
    wire signed [16:0] difference=$signed({6'b0,x})-$signed({1'b0,center});
    wire [16:0] magnitude=difference[16] ? -difference : difference;
    wire [16:0] outside=(magnitude>deadzone ? magnitude-deadzone : 17'd0);
    wire [16:0] scaled=outside>>gain[6:0];
    wire [7:0] bounded=(scaled>limit ? limit : scaled[7:0]);
    wire negative=difference[16]^gain[7];
    assign target=!valid ? 8'sd0 : (negative ? -$signed(bounded) : $signed(bounded));
endmodule

// Source: vision_report_tx.v
module vision_report_tx #(parameter integer UART_DIV=434,REPORT_CYCLES=12500000,SHARED_TX=0,WIDE_RANGE=0)(
    input wire clk,rst_n,
    input wire [10:0] x,y,
    input wire valid,enabled,origin_valid,
    input wire [3:0] fault,
    input wire [3:0] quality,
    input wire bad,
    input wire [15:0] age,frames,errors1,errors2,center,deadzone,cfg_sequence,rx_errors,
    input wire [15:0] cam_sequence,exposure,sensor_gain,roi_left,roi_top,roi_right,roi_bottom,camera_applied_seq,roi_applied_seq,
    input wire settings_busy,
    input wire signed [31:0] locked_target,move_error,command_position,
    input wire [3:0] move_state,cancel_reason,reject_reason,
    input wire move_busy,reached,request_ready,
    input wire [7:0] spent,
    input wire [15:0] accepted_count,completed_count,center_count,
    input wire [3:0] center_status,
    input wire [7:0] operating_mode,damping,slew,
    input wire auto_run,center_confirmed,
    input wire [15:0] runtime_seconds,follow_sequence,elapsed_seconds,
    input wire signed [15:0] velocity,
    input wire signed [(WIDE_RANGE ? 31 : 7):0] drive_target,
    input wire [3:0] stop_reason,auto_reject,
    input wire [31:0] total_pulses,
    input wire [7:0] gain,limit,
    input wire signed [(WIDE_RANGE ? 31 : 7):0] target,
    input wire signed [31:0] count1,count2,position,
    input wire grant,output wire busy,wants,
    output wire tx
);
    reg [15:0] x_snapshot=0;
    reg [15:0] y_snapshot=0;
    reg [3:0] valid_snapshot=0;
    reg [15:0] age_snapshot=0;
    reg [15:0] frames_snapshot=0;
    reg [3:0] enabled_snapshot=0;
    reg [3:0] origin_valid_snapshot=0;
    reg [3:0] fault_snapshot=0;
    reg [31:0] count1_snapshot=0;
    reg [31:0] count2_snapshot=0;
    reg [15:0] errors1_snapshot=0;
    reg [15:0] errors2_snapshot=0;
    reg [31:0] position_snapshot=0;
    reg [(WIDE_RANGE ? 31 : 7):0] target_snapshot=0;
    reg [15:0] center_snapshot=0;
    reg [15:0] deadzone_snapshot=0;
    reg [7:0] gain_snapshot=0;
    reg [7:0] limit_snapshot=0;
    reg [15:0] cfg_sequence_snapshot=0;
    reg [15:0] rx_errors_snapshot=0;
    reg [3:0] quality_snapshot=0;
    reg [3:0] bad_snapshot=0;
    reg [15:0] cam_sequence_snapshot=0;
    reg [15:0] exposure_snapshot=0;
    reg [15:0] sensor_gain_snapshot=0;
    reg [15:0] roi_left_snapshot=0;
    reg [15:0] roi_top_snapshot=0;
    reg [15:0] roi_right_snapshot=0;
    reg [15:0] roi_bottom_snapshot=0;
    reg [15:0] camera_applied_seq_snapshot=0;
    reg [15:0] roi_applied_seq_snapshot=0;
    reg [3:0] settings_busy_snapshot=0;
    reg [31:0] locked_target_snapshot=0;
    reg [31:0] move_error_snapshot=0;
    reg [3:0] move_state_snapshot=0;
    reg [3:0] move_busy_snapshot=0;
    reg [3:0] reached_snapshot=0;
    reg [3:0] request_ready_snapshot=0;
    reg [7:0] spent_snapshot=0;
    reg [31:0] command_position_snapshot=0;
    reg [3:0] cancel_reason_snapshot=0;
    reg [3:0] reject_reason_snapshot=0;
    reg [15:0] accepted_count_snapshot=0;
    reg [15:0] completed_count_snapshot=0;
    reg [15:0] center_count_snapshot=0;
    reg [3:0] center_status_snapshot=0;
    reg [3:0] operating_mode_snapshot=0;
    reg [3:0] auto_run_snapshot=0;
    reg [7:0] damping_snapshot=0;
    reg [7:0] slew_snapshot=0;
    reg [15:0] runtime_seconds_snapshot=0;
    reg [15:0] follow_sequence_snapshot=0;
    reg [3:0] center_confirmed_snapshot=0;
    reg [15:0] velocity_snapshot=0;
    reg [(WIDE_RANGE ? 31 : 7):0] drive_target_snapshot=0;
    reg [3:0] stop_reason_snapshot=0;
    reg [3:0] auto_reject_snapshot=0;
    reg [15:0] elapsed_seconds_snapshot=0;
    reg [31:0] total_pulses_snapshot=0;
    reg [31:0] timer=0;
    reg frame_active=0,bit_active=0;
    reg report_pending=0;
    assign busy=frame_active;
    assign wants=!frame_active && (report_pending || timer==REPORT_CYCLES-1);
    reg [8:0] char_index=0;
    reg [3:0] bit_index=0;
    reg [15:0] baud_count=0;
    reg [9:0] serial_shift=10'h3ff;
    assign tx=serial_shift[0];
    function [7:0] hex_char;
        input [3:0] value;
        begin if(value<10)hex_char=8'h30+value;else hex_char=8'h37+value;end
    endfunction
    function [7:0] wide_message_byte;input [8:0] index;begin case(index)
0: wide_message_byte=8'h56;
1: wide_message_byte=8'h49;
2: wide_message_byte=8'h53;
3: wide_message_byte=8'h3d;
4: wide_message_byte=8'h36;
5: wide_message_byte=32;
6: wide_message_byte=8'h42;
7: wide_message_byte=8'h58;
8: wide_message_byte=8'h3d;
9: wide_message_byte=hex_char(x_snapshot >> 12);
10: wide_message_byte=hex_char(x_snapshot >> 8);
11: wide_message_byte=hex_char(x_snapshot >> 4);
12: wide_message_byte=hex_char(x_snapshot >> 0);
13: wide_message_byte=32;
14: wide_message_byte=8'h42;
15: wide_message_byte=8'h59;
16: wide_message_byte=8'h3d;
17: wide_message_byte=hex_char(y_snapshot >> 12);
18: wide_message_byte=hex_char(y_snapshot >> 8);
19: wide_message_byte=hex_char(y_snapshot >> 4);
20: wide_message_byte=hex_char(y_snapshot >> 0);
21: wide_message_byte=32;
22: wide_message_byte=8'h56;
23: wide_message_byte=8'h3d;
24: wide_message_byte=hex_char(valid_snapshot >> 0);
25: wide_message_byte=32;
26: wide_message_byte=8'h41;
27: wide_message_byte=8'h47;
28: wide_message_byte=8'h45;
29: wide_message_byte=8'h3d;
30: wide_message_byte=hex_char(age_snapshot >> 12);
31: wide_message_byte=hex_char(age_snapshot >> 8);
32: wide_message_byte=hex_char(age_snapshot >> 4);
33: wide_message_byte=hex_char(age_snapshot >> 0);
34: wide_message_byte=32;
35: wide_message_byte=8'h46;
36: wide_message_byte=8'h52;
37: wide_message_byte=8'h3d;
38: wide_message_byte=hex_char(frames_snapshot >> 12);
39: wide_message_byte=hex_char(frames_snapshot >> 8);
40: wide_message_byte=hex_char(frames_snapshot >> 4);
41: wide_message_byte=hex_char(frames_snapshot >> 0);
42: wide_message_byte=32;
43: wide_message_byte=8'h45;
44: wide_message_byte=8'h4e;
45: wide_message_byte=8'h3d;
46: wide_message_byte=hex_char(enabled_snapshot >> 0);
47: wide_message_byte=32;
48: wide_message_byte=8'h5a;
49: wide_message_byte=8'h3d;
50: wide_message_byte=hex_char(origin_valid_snapshot >> 0);
51: wide_message_byte=32;
52: wide_message_byte=8'h46;
53: wide_message_byte=8'h3d;
54: wide_message_byte=hex_char(fault_snapshot >> 0);
55: wide_message_byte=32;
56: wide_message_byte=8'h45;
57: wide_message_byte=8'h31;
58: wide_message_byte=8'h3d;
59: wide_message_byte=hex_char(count1_snapshot >> 28);
60: wide_message_byte=hex_char(count1_snapshot >> 24);
61: wide_message_byte=hex_char(count1_snapshot >> 20);
62: wide_message_byte=hex_char(count1_snapshot >> 16);
63: wide_message_byte=hex_char(count1_snapshot >> 12);
64: wide_message_byte=hex_char(count1_snapshot >> 8);
65: wide_message_byte=hex_char(count1_snapshot >> 4);
66: wide_message_byte=hex_char(count1_snapshot >> 0);
67: wide_message_byte=32;
68: wide_message_byte=8'h45;
69: wide_message_byte=8'h32;
70: wide_message_byte=8'h3d;
71: wide_message_byte=hex_char(count2_snapshot >> 28);
72: wide_message_byte=hex_char(count2_snapshot >> 24);
73: wide_message_byte=hex_char(count2_snapshot >> 20);
74: wide_message_byte=hex_char(count2_snapshot >> 16);
75: wide_message_byte=hex_char(count2_snapshot >> 12);
76: wide_message_byte=hex_char(count2_snapshot >> 8);
77: wide_message_byte=hex_char(count2_snapshot >> 4);
78: wide_message_byte=hex_char(count2_snapshot >> 0);
79: wide_message_byte=32;
80: wide_message_byte=8'h58;
81: wide_message_byte=8'h31;
82: wide_message_byte=8'h3d;
83: wide_message_byte=hex_char(errors1_snapshot >> 12);
84: wide_message_byte=hex_char(errors1_snapshot >> 8);
85: wide_message_byte=hex_char(errors1_snapshot >> 4);
86: wide_message_byte=hex_char(errors1_snapshot >> 0);
87: wide_message_byte=32;
88: wide_message_byte=8'h58;
89: wide_message_byte=8'h32;
90: wide_message_byte=8'h3d;
91: wide_message_byte=hex_char(errors2_snapshot >> 12);
92: wide_message_byte=hex_char(errors2_snapshot >> 8);
93: wide_message_byte=hex_char(errors2_snapshot >> 4);
94: wide_message_byte=hex_char(errors2_snapshot >> 0);
95: wide_message_byte=32;
96: wide_message_byte=8'h50;
97: wide_message_byte=8'h3d;
98: wide_message_byte=hex_char(position_snapshot >> 28);
99: wide_message_byte=hex_char(position_snapshot >> 24);
100: wide_message_byte=hex_char(position_snapshot >> 20);
101: wide_message_byte=hex_char(position_snapshot >> 16);
102: wide_message_byte=hex_char(position_snapshot >> 12);
103: wide_message_byte=hex_char(position_snapshot >> 8);
104: wide_message_byte=hex_char(position_snapshot >> 4);
105: wide_message_byte=hex_char(position_snapshot >> 0);
106: wide_message_byte=32;
107: wide_message_byte=8'h54;
108: wide_message_byte=8'h3d;
109: wide_message_byte=hex_char(target_snapshot >> 28);
110: wide_message_byte=hex_char(target_snapshot >> 24);
111: wide_message_byte=hex_char(target_snapshot >> 20);
112: wide_message_byte=hex_char(target_snapshot >> 16);
113: wide_message_byte=hex_char(target_snapshot >> 12);
114: wide_message_byte=hex_char(target_snapshot >> 8);
115: wide_message_byte=hex_char(target_snapshot >> 4);
116: wide_message_byte=hex_char(target_snapshot >> 0);
117: wide_message_byte=32;
118: wide_message_byte=8'h43;
119: wide_message_byte=8'h3d;
120: wide_message_byte=hex_char(center_snapshot >> 12);
121: wide_message_byte=hex_char(center_snapshot >> 8);
122: wide_message_byte=hex_char(center_snapshot >> 4);
123: wide_message_byte=hex_char(center_snapshot >> 0);
124: wide_message_byte=32;
125: wide_message_byte=8'h44;
126: wide_message_byte=8'h5a;
127: wide_message_byte=8'h3d;
128: wide_message_byte=hex_char(deadzone_snapshot >> 12);
129: wide_message_byte=hex_char(deadzone_snapshot >> 8);
130: wide_message_byte=hex_char(deadzone_snapshot >> 4);
131: wide_message_byte=hex_char(deadzone_snapshot >> 0);
132: wide_message_byte=32;
133: wide_message_byte=8'h47;
134: wide_message_byte=8'h3d;
135: wide_message_byte=hex_char(gain_snapshot >> 4);
136: wide_message_byte=hex_char(gain_snapshot >> 0);
137: wide_message_byte=32;
138: wide_message_byte=8'h4c;
139: wide_message_byte=8'h3d;
140: wide_message_byte=hex_char(limit_snapshot >> 4);
141: wide_message_byte=hex_char(limit_snapshot >> 0);
142: wide_message_byte=32;
143: wide_message_byte=8'h53;
144: wide_message_byte=8'h45;
145: wide_message_byte=8'h51;
146: wide_message_byte=8'h3d;
147: wide_message_byte=hex_char(cfg_sequence_snapshot >> 12);
148: wide_message_byte=hex_char(cfg_sequence_snapshot >> 8);
149: wide_message_byte=hex_char(cfg_sequence_snapshot >> 4);
150: wide_message_byte=hex_char(cfg_sequence_snapshot >> 0);
151: wide_message_byte=32;
152: wide_message_byte=8'h52;
153: wide_message_byte=8'h58;
154: wide_message_byte=8'h45;
155: wide_message_byte=8'h3d;
156: wide_message_byte=hex_char(rx_errors_snapshot >> 12);
157: wide_message_byte=hex_char(rx_errors_snapshot >> 8);
158: wide_message_byte=hex_char(rx_errors_snapshot >> 4);
159: wide_message_byte=hex_char(rx_errors_snapshot >> 0);
160: wide_message_byte=32;
161: wide_message_byte=8'h51;
162: wide_message_byte=8'h3d;
163: wide_message_byte=hex_char(quality_snapshot >> 0);
164: wide_message_byte=32;
165: wide_message_byte=8'h42;
166: wide_message_byte=8'h3d;
167: wide_message_byte=hex_char(bad_snapshot >> 0);
168: wide_message_byte=32;
169: wide_message_byte=8'h43;
170: wide_message_byte=8'h53;
171: wide_message_byte=8'h3d;
172: wide_message_byte=hex_char(cam_sequence_snapshot >> 12);
173: wide_message_byte=hex_char(cam_sequence_snapshot >> 8);
174: wide_message_byte=hex_char(cam_sequence_snapshot >> 4);
175: wide_message_byte=hex_char(cam_sequence_snapshot >> 0);
176: wide_message_byte=32;
177: wide_message_byte=8'h41;
178: wide_message_byte=8'h45;
179: wide_message_byte=8'h3d;
180: wide_message_byte=hex_char(exposure_snapshot >> 12);
181: wide_message_byte=hex_char(exposure_snapshot >> 8);
182: wide_message_byte=hex_char(exposure_snapshot >> 4);
183: wide_message_byte=hex_char(exposure_snapshot >> 0);
184: wide_message_byte=32;
185: wide_message_byte=8'h41;
186: wide_message_byte=8'h47;
187: wide_message_byte=8'h3d;
188: wide_message_byte=hex_char(sensor_gain_snapshot >> 12);
189: wide_message_byte=hex_char(sensor_gain_snapshot >> 8);
190: wide_message_byte=hex_char(sensor_gain_snapshot >> 4);
191: wide_message_byte=hex_char(sensor_gain_snapshot >> 0);
192: wide_message_byte=32;
193: wide_message_byte=8'h52;
194: wide_message_byte=8'h4c;
195: wide_message_byte=8'h3d;
196: wide_message_byte=hex_char(roi_left_snapshot >> 12);
197: wide_message_byte=hex_char(roi_left_snapshot >> 8);
198: wide_message_byte=hex_char(roi_left_snapshot >> 4);
199: wide_message_byte=hex_char(roi_left_snapshot >> 0);
200: wide_message_byte=32;
201: wide_message_byte=8'h52;
202: wide_message_byte=8'h54;
203: wide_message_byte=8'h3d;
204: wide_message_byte=hex_char(roi_top_snapshot >> 12);
205: wide_message_byte=hex_char(roi_top_snapshot >> 8);
206: wide_message_byte=hex_char(roi_top_snapshot >> 4);
207: wide_message_byte=hex_char(roi_top_snapshot >> 0);
208: wide_message_byte=32;
209: wide_message_byte=8'h52;
210: wide_message_byte=8'h52;
211: wide_message_byte=8'h3d;
212: wide_message_byte=hex_char(roi_right_snapshot >> 12);
213: wide_message_byte=hex_char(roi_right_snapshot >> 8);
214: wide_message_byte=hex_char(roi_right_snapshot >> 4);
215: wide_message_byte=hex_char(roi_right_snapshot >> 0);
216: wide_message_byte=32;
217: wide_message_byte=8'h52;
218: wide_message_byte=8'h42;
219: wide_message_byte=8'h3d;
220: wide_message_byte=hex_char(roi_bottom_snapshot >> 12);
221: wide_message_byte=hex_char(roi_bottom_snapshot >> 8);
222: wide_message_byte=hex_char(roi_bottom_snapshot >> 4);
223: wide_message_byte=hex_char(roi_bottom_snapshot >> 0);
224: wide_message_byte=32;
225: wide_message_byte=8'h43;
226: wide_message_byte=8'h41;
227: wide_message_byte=8'h3d;
228: wide_message_byte=hex_char(camera_applied_seq_snapshot >> 12);
229: wide_message_byte=hex_char(camera_applied_seq_snapshot >> 8);
230: wide_message_byte=hex_char(camera_applied_seq_snapshot >> 4);
231: wide_message_byte=hex_char(camera_applied_seq_snapshot >> 0);
232: wide_message_byte=32;
233: wide_message_byte=8'h52;
234: wide_message_byte=8'h41;
235: wide_message_byte=8'h3d;
236: wide_message_byte=hex_char(roi_applied_seq_snapshot >> 12);
237: wide_message_byte=hex_char(roi_applied_seq_snapshot >> 8);
238: wide_message_byte=hex_char(roi_applied_seq_snapshot >> 4);
239: wide_message_byte=hex_char(roi_applied_seq_snapshot >> 0);
240: wide_message_byte=32;
241: wide_message_byte=8'h43;
242: wide_message_byte=8'h42;
243: wide_message_byte=8'h3d;
244: wide_message_byte=hex_char(settings_busy_snapshot >> 0);
245: wide_message_byte=32;
246: wide_message_byte=8'h4d;
247: wide_message_byte=8'h54;
248: wide_message_byte=8'h3d;
249: wide_message_byte=hex_char(locked_target_snapshot >> 28);
250: wide_message_byte=hex_char(locked_target_snapshot >> 24);
251: wide_message_byte=hex_char(locked_target_snapshot >> 20);
252: wide_message_byte=hex_char(locked_target_snapshot >> 16);
253: wide_message_byte=hex_char(locked_target_snapshot >> 12);
254: wide_message_byte=hex_char(locked_target_snapshot >> 8);
255: wide_message_byte=hex_char(locked_target_snapshot >> 4);
256: wide_message_byte=hex_char(locked_target_snapshot >> 0);
257: wide_message_byte=32;
258: wide_message_byte=8'h4d;
259: wide_message_byte=8'h45;
260: wide_message_byte=8'h3d;
261: wide_message_byte=hex_char(move_error_snapshot >> 28);
262: wide_message_byte=hex_char(move_error_snapshot >> 24);
263: wide_message_byte=hex_char(move_error_snapshot >> 20);
264: wide_message_byte=hex_char(move_error_snapshot >> 16);
265: wide_message_byte=hex_char(move_error_snapshot >> 12);
266: wide_message_byte=hex_char(move_error_snapshot >> 8);
267: wide_message_byte=hex_char(move_error_snapshot >> 4);
268: wide_message_byte=hex_char(move_error_snapshot >> 0);
269: wide_message_byte=32;
270: wide_message_byte=8'h4d;
271: wide_message_byte=8'h53;
272: wide_message_byte=8'h3d;
273: wide_message_byte=hex_char(move_state_snapshot >> 0);
274: wide_message_byte=32;
275: wide_message_byte=8'h4d;
276: wide_message_byte=8'h42;
277: wide_message_byte=8'h3d;
278: wide_message_byte=hex_char(move_busy_snapshot >> 0);
279: wide_message_byte=32;
280: wide_message_byte=8'h4f;
281: wide_message_byte=8'h4b;
282: wide_message_byte=8'h3d;
283: wide_message_byte=hex_char(reached_snapshot >> 0);
284: wide_message_byte=32;
285: wide_message_byte=8'h52;
286: wide_message_byte=8'h44;
287: wide_message_byte=8'h3d;
288: wide_message_byte=hex_char(request_ready_snapshot >> 0);
289: wide_message_byte=32;
290: wide_message_byte=8'h53;
291: wide_message_byte=8'h50;
292: wide_message_byte=8'h3d;
293: wide_message_byte=hex_char(spent_snapshot >> 4);
294: wide_message_byte=hex_char(spent_snapshot >> 0);
295: wide_message_byte=32;
296: wide_message_byte=8'h4d;
297: wide_message_byte=8'h43;
298: wide_message_byte=8'h3d;
299: wide_message_byte=hex_char(command_position_snapshot >> 28);
300: wide_message_byte=hex_char(command_position_snapshot >> 24);
301: wide_message_byte=hex_char(command_position_snapshot >> 20);
302: wide_message_byte=hex_char(command_position_snapshot >> 16);
303: wide_message_byte=hex_char(command_position_snapshot >> 12);
304: wide_message_byte=hex_char(command_position_snapshot >> 8);
305: wide_message_byte=hex_char(command_position_snapshot >> 4);
306: wide_message_byte=hex_char(command_position_snapshot >> 0);
307: wide_message_byte=32;
308: wide_message_byte=8'h43;
309: wide_message_byte=8'h52;
310: wide_message_byte=8'h3d;
311: wide_message_byte=hex_char(cancel_reason_snapshot >> 0);
312: wide_message_byte=32;
313: wide_message_byte=8'h52;
314: wide_message_byte=8'h4a;
315: wide_message_byte=8'h3d;
316: wide_message_byte=hex_char(reject_reason_snapshot >> 0);
317: wide_message_byte=32;
318: wide_message_byte=8'h41;
319: wide_message_byte=8'h43;
320: wide_message_byte=8'h3d;
321: wide_message_byte=hex_char(accepted_count_snapshot >> 12);
322: wide_message_byte=hex_char(accepted_count_snapshot >> 8);
323: wide_message_byte=hex_char(accepted_count_snapshot >> 4);
324: wide_message_byte=hex_char(accepted_count_snapshot >> 0);
325: wide_message_byte=32;
326: wide_message_byte=8'h44;
327: wide_message_byte=8'h43;
328: wide_message_byte=8'h3d;
329: wide_message_byte=hex_char(completed_count_snapshot >> 12);
330: wide_message_byte=hex_char(completed_count_snapshot >> 8);
331: wide_message_byte=hex_char(completed_count_snapshot >> 4);
332: wide_message_byte=hex_char(completed_count_snapshot >> 0);
333: wide_message_byte=32;
334: wide_message_byte=8'h43;
335: wide_message_byte=8'h43;
336: wide_message_byte=8'h3d;
337: wide_message_byte=hex_char(center_count_snapshot >> 12);
338: wide_message_byte=hex_char(center_count_snapshot >> 8);
339: wide_message_byte=hex_char(center_count_snapshot >> 4);
340: wide_message_byte=hex_char(center_count_snapshot >> 0);
341: wide_message_byte=32;
342: wide_message_byte=8'h4b;
343: wide_message_byte=8'h43;
344: wide_message_byte=8'h3d;
345: wide_message_byte=hex_char(center_status_snapshot >> 0);
346: wide_message_byte=32;
347: wide_message_byte=8'h4d;
348: wide_message_byte=8'h44;
349: wide_message_byte=8'h3d;
350: wide_message_byte=hex_char(operating_mode_snapshot >> 0);
351: wide_message_byte=32;
352: wide_message_byte=8'h41;
353: wide_message_byte=8'h52;
354: wide_message_byte=8'h3d;
355: wide_message_byte=hex_char(auto_run_snapshot >> 0);
356: wide_message_byte=32;
357: wide_message_byte=8'h4b;
358: wide_message_byte=8'h44;
359: wide_message_byte=8'h3d;
360: wide_message_byte=hex_char(damping_snapshot >> 4);
361: wide_message_byte=hex_char(damping_snapshot >> 0);
362: wide_message_byte=32;
363: wide_message_byte=8'h53;
364: wide_message_byte=8'h4c;
365: wide_message_byte=8'h3d;
366: wide_message_byte=hex_char(slew_snapshot >> 4);
367: wide_message_byte=hex_char(slew_snapshot >> 0);
368: wide_message_byte=32;
369: wide_message_byte=8'h52;
370: wide_message_byte=8'h55;
371: wide_message_byte=8'h3d;
372: wide_message_byte=hex_char(runtime_seconds_snapshot >> 12);
373: wide_message_byte=hex_char(runtime_seconds_snapshot >> 8);
374: wide_message_byte=hex_char(runtime_seconds_snapshot >> 4);
375: wide_message_byte=hex_char(runtime_seconds_snapshot >> 0);
376: wide_message_byte=32;
377: wide_message_byte=8'h44;
378: wide_message_byte=8'h53;
379: wide_message_byte=8'h3d;
380: wide_message_byte=hex_char(follow_sequence_snapshot >> 12);
381: wide_message_byte=hex_char(follow_sequence_snapshot >> 8);
382: wide_message_byte=hex_char(follow_sequence_snapshot >> 4);
383: wide_message_byte=hex_char(follow_sequence_snapshot >> 0);
384: wide_message_byte=32;
385: wide_message_byte=8'h43;
386: wide_message_byte=8'h5a;
387: wide_message_byte=8'h3d;
388: wide_message_byte=hex_char(center_confirmed_snapshot >> 0);
389: wide_message_byte=32;
390: wide_message_byte=8'h56;
391: wide_message_byte=8'h45;
392: wide_message_byte=8'h3d;
393: wide_message_byte=hex_char(velocity_snapshot >> 12);
394: wide_message_byte=hex_char(velocity_snapshot >> 8);
395: wide_message_byte=hex_char(velocity_snapshot >> 4);
396: wide_message_byte=hex_char(velocity_snapshot >> 0);
397: wide_message_byte=32;
398: wide_message_byte=8'h41;
399: wide_message_byte=8'h54;
400: wide_message_byte=8'h3d;
401: wide_message_byte=hex_char(drive_target_snapshot >> 28);
402: wide_message_byte=hex_char(drive_target_snapshot >> 24);
403: wide_message_byte=hex_char(drive_target_snapshot >> 20);
404: wide_message_byte=hex_char(drive_target_snapshot >> 16);
405: wide_message_byte=hex_char(drive_target_snapshot >> 12);
406: wide_message_byte=hex_char(drive_target_snapshot >> 8);
407: wide_message_byte=hex_char(drive_target_snapshot >> 4);
408: wide_message_byte=hex_char(drive_target_snapshot >> 0);
409: wide_message_byte=32;
410: wide_message_byte=8'h53;
411: wide_message_byte=8'h52;
412: wide_message_byte=8'h3d;
413: wide_message_byte=hex_char(stop_reason_snapshot >> 0);
414: wide_message_byte=32;
415: wide_message_byte=8'h41;
416: wide_message_byte=8'h4a;
417: wide_message_byte=8'h3d;
418: wide_message_byte=hex_char(auto_reject_snapshot >> 0);
419: wide_message_byte=32;
420: wide_message_byte=8'h45;
421: wide_message_byte=8'h4c;
422: wide_message_byte=8'h3d;
423: wide_message_byte=hex_char(elapsed_seconds_snapshot >> 12);
424: wide_message_byte=hex_char(elapsed_seconds_snapshot >> 8);
425: wide_message_byte=hex_char(elapsed_seconds_snapshot >> 4);
426: wide_message_byte=hex_char(elapsed_seconds_snapshot >> 0);
427: wide_message_byte=32;
428: wide_message_byte=8'h54;
429: wide_message_byte=8'h53;
430: wide_message_byte=8'h3d;
431: wide_message_byte=hex_char(total_pulses_snapshot >> 28);
432: wide_message_byte=hex_char(total_pulses_snapshot >> 24);
433: wide_message_byte=hex_char(total_pulses_snapshot >> 20);
434: wide_message_byte=hex_char(total_pulses_snapshot >> 16);
435: wide_message_byte=hex_char(total_pulses_snapshot >> 12);
436: wide_message_byte=hex_char(total_pulses_snapshot >> 8);
437: wide_message_byte=hex_char(total_pulses_snapshot >> 4);
438: wide_message_byte=hex_char(total_pulses_snapshot >> 0);
439: wide_message_byte=13;
440: wide_message_byte=10;
 default:wide_message_byte="?";endcase end endfunction
    function [7:0] message_byte;
        input [8:0] index;
        begin if(WIDE_RANGE)message_byte=wide_message_byte(index);else case(index)
                0: message_byte=8'h56;
                1: message_byte=8'h49;
                2: message_byte=8'h53;
                3: message_byte=8'h3d;
                4: message_byte=8'h35;
                5: message_byte=8'h20;
                6: message_byte=8'h42;
                7: message_byte=8'h58;
                8: message_byte=8'h3d;
                9: message_byte=hex_char(x_snapshot >> 12);
                10: message_byte=hex_char(x_snapshot >> 8);
                11: message_byte=hex_char(x_snapshot >> 4);
                12: message_byte=hex_char(x_snapshot >> 0);
                13: message_byte=8'h20;
                14: message_byte=8'h42;
                15: message_byte=8'h59;
                16: message_byte=8'h3d;
                17: message_byte=hex_char(y_snapshot >> 12);
                18: message_byte=hex_char(y_snapshot >> 8);
                19: message_byte=hex_char(y_snapshot >> 4);
                20: message_byte=hex_char(y_snapshot >> 0);
                21: message_byte=8'h20;
                22: message_byte=8'h56;
                23: message_byte=8'h3d;
                24: message_byte=hex_char(valid_snapshot >> 0);
                25: message_byte=8'h20;
                26: message_byte=8'h41;
                27: message_byte=8'h47;
                28: message_byte=8'h45;
                29: message_byte=8'h3d;
                30: message_byte=hex_char(age_snapshot >> 12);
                31: message_byte=hex_char(age_snapshot >> 8);
                32: message_byte=hex_char(age_snapshot >> 4);
                33: message_byte=hex_char(age_snapshot >> 0);
                34: message_byte=8'h20;
                35: message_byte=8'h46;
                36: message_byte=8'h52;
                37: message_byte=8'h3d;
                38: message_byte=hex_char(frames_snapshot >> 12);
                39: message_byte=hex_char(frames_snapshot >> 8);
                40: message_byte=hex_char(frames_snapshot >> 4);
                41: message_byte=hex_char(frames_snapshot >> 0);
                42: message_byte=8'h20;
                43: message_byte=8'h45;
                44: message_byte=8'h4e;
                45: message_byte=8'h3d;
                46: message_byte=hex_char(enabled_snapshot >> 0);
                47: message_byte=8'h20;
                48: message_byte=8'h5a;
                49: message_byte=8'h3d;
                50: message_byte=hex_char(origin_valid_snapshot >> 0);
                51: message_byte=8'h20;
                52: message_byte=8'h46;
                53: message_byte=8'h3d;
                54: message_byte=hex_char(fault_snapshot >> 0);
                55: message_byte=8'h20;
                56: message_byte=8'h45;
                57: message_byte=8'h31;
                58: message_byte=8'h3d;
                59: message_byte=hex_char(count1_snapshot >> 28);
                60: message_byte=hex_char(count1_snapshot >> 24);
                61: message_byte=hex_char(count1_snapshot >> 20);
                62: message_byte=hex_char(count1_snapshot >> 16);
                63: message_byte=hex_char(count1_snapshot >> 12);
                64: message_byte=hex_char(count1_snapshot >> 8);
                65: message_byte=hex_char(count1_snapshot >> 4);
                66: message_byte=hex_char(count1_snapshot >> 0);
                67: message_byte=8'h20;
                68: message_byte=8'h45;
                69: message_byte=8'h32;
                70: message_byte=8'h3d;
                71: message_byte=hex_char(count2_snapshot >> 28);
                72: message_byte=hex_char(count2_snapshot >> 24);
                73: message_byte=hex_char(count2_snapshot >> 20);
                74: message_byte=hex_char(count2_snapshot >> 16);
                75: message_byte=hex_char(count2_snapshot >> 12);
                76: message_byte=hex_char(count2_snapshot >> 8);
                77: message_byte=hex_char(count2_snapshot >> 4);
                78: message_byte=hex_char(count2_snapshot >> 0);
                79: message_byte=8'h20;
                80: message_byte=8'h58;
                81: message_byte=8'h31;
                82: message_byte=8'h3d;
                83: message_byte=hex_char(errors1_snapshot >> 12);
                84: message_byte=hex_char(errors1_snapshot >> 8);
                85: message_byte=hex_char(errors1_snapshot >> 4);
                86: message_byte=hex_char(errors1_snapshot >> 0);
                87: message_byte=8'h20;
                88: message_byte=8'h58;
                89: message_byte=8'h32;
                90: message_byte=8'h3d;
                91: message_byte=hex_char(errors2_snapshot >> 12);
                92: message_byte=hex_char(errors2_snapshot >> 8);
                93: message_byte=hex_char(errors2_snapshot >> 4);
                94: message_byte=hex_char(errors2_snapshot >> 0);
                95: message_byte=8'h20;
                96: message_byte=8'h50;
                97: message_byte=8'h3d;
                98: message_byte=hex_char(position_snapshot >> 28);
                99: message_byte=hex_char(position_snapshot >> 24);
                100: message_byte=hex_char(position_snapshot >> 20);
                101: message_byte=hex_char(position_snapshot >> 16);
                102: message_byte=hex_char(position_snapshot >> 12);
                103: message_byte=hex_char(position_snapshot >> 8);
                104: message_byte=hex_char(position_snapshot >> 4);
                105: message_byte=hex_char(position_snapshot >> 0);
                106: message_byte=8'h20;
                107: message_byte=8'h54;
                108: message_byte=8'h3d;
                109: message_byte=hex_char(target_snapshot >> 4);
                110: message_byte=hex_char(target_snapshot >> 0);
                111: message_byte=8'h20;
                112: message_byte=8'h43;
                113: message_byte=8'h3d;
                114: message_byte=hex_char(center_snapshot >> 12);
                115: message_byte=hex_char(center_snapshot >> 8);
                116: message_byte=hex_char(center_snapshot >> 4);
                117: message_byte=hex_char(center_snapshot >> 0);
                118: message_byte=8'h20;
                119: message_byte=8'h44;
                120: message_byte=8'h5a;
                121: message_byte=8'h3d;
                122: message_byte=hex_char(deadzone_snapshot >> 12);
                123: message_byte=hex_char(deadzone_snapshot >> 8);
                124: message_byte=hex_char(deadzone_snapshot >> 4);
                125: message_byte=hex_char(deadzone_snapshot >> 0);
                126: message_byte=8'h20;
                127: message_byte=8'h47;
                128: message_byte=8'h3d;
                129: message_byte=hex_char(gain_snapshot >> 4);
                130: message_byte=hex_char(gain_snapshot >> 0);
                131: message_byte=8'h20;
                132: message_byte=8'h4c;
                133: message_byte=8'h3d;
                134: message_byte=hex_char(limit_snapshot >> 4);
                135: message_byte=hex_char(limit_snapshot >> 0);
                136: message_byte=8'h20;
                137: message_byte=8'h53;
                138: message_byte=8'h45;
                139: message_byte=8'h51;
                140: message_byte=8'h3d;
                141: message_byte=hex_char(cfg_sequence_snapshot >> 12);
                142: message_byte=hex_char(cfg_sequence_snapshot >> 8);
                143: message_byte=hex_char(cfg_sequence_snapshot >> 4);
                144: message_byte=hex_char(cfg_sequence_snapshot >> 0);
                145: message_byte=8'h20;
                146: message_byte=8'h52;
                147: message_byte=8'h58;
                148: message_byte=8'h45;
                149: message_byte=8'h3d;
                150: message_byte=hex_char(rx_errors_snapshot >> 12);
                151: message_byte=hex_char(rx_errors_snapshot >> 8);
                152: message_byte=hex_char(rx_errors_snapshot >> 4);
                153: message_byte=hex_char(rx_errors_snapshot >> 0);
                154: message_byte=8'h20;
                155: message_byte=8'h51;
                156: message_byte=8'h3d;
                157: message_byte=hex_char(quality_snapshot >> 0);
                158: message_byte=8'h20;
                159: message_byte=8'h42;
                160: message_byte=8'h3d;
                161: message_byte=hex_char(bad_snapshot >> 0);
                162: message_byte=8'h20;
                163: message_byte=8'h43;
                164: message_byte=8'h53;
                165: message_byte=8'h3d;
                166: message_byte=hex_char(cam_sequence_snapshot >> 12);
                167: message_byte=hex_char(cam_sequence_snapshot >> 8);
                168: message_byte=hex_char(cam_sequence_snapshot >> 4);
                169: message_byte=hex_char(cam_sequence_snapshot >> 0);
                170: message_byte=8'h20;
                171: message_byte=8'h41;
                172: message_byte=8'h45;
                173: message_byte=8'h3d;
                174: message_byte=hex_char(exposure_snapshot >> 12);
                175: message_byte=hex_char(exposure_snapshot >> 8);
                176: message_byte=hex_char(exposure_snapshot >> 4);
                177: message_byte=hex_char(exposure_snapshot >> 0);
                178: message_byte=8'h20;
                179: message_byte=8'h41;
                180: message_byte=8'h47;
                181: message_byte=8'h3d;
                182: message_byte=hex_char(sensor_gain_snapshot >> 12);
                183: message_byte=hex_char(sensor_gain_snapshot >> 8);
                184: message_byte=hex_char(sensor_gain_snapshot >> 4);
                185: message_byte=hex_char(sensor_gain_snapshot >> 0);
                186: message_byte=8'h20;
                187: message_byte=8'h52;
                188: message_byte=8'h4c;
                189: message_byte=8'h3d;
                190: message_byte=hex_char(roi_left_snapshot >> 12);
                191: message_byte=hex_char(roi_left_snapshot >> 8);
                192: message_byte=hex_char(roi_left_snapshot >> 4);
                193: message_byte=hex_char(roi_left_snapshot >> 0);
                194: message_byte=8'h20;
                195: message_byte=8'h52;
                196: message_byte=8'h54;
                197: message_byte=8'h3d;
                198: message_byte=hex_char(roi_top_snapshot >> 12);
                199: message_byte=hex_char(roi_top_snapshot >> 8);
                200: message_byte=hex_char(roi_top_snapshot >> 4);
                201: message_byte=hex_char(roi_top_snapshot >> 0);
                202: message_byte=8'h20;
                203: message_byte=8'h52;
                204: message_byte=8'h52;
                205: message_byte=8'h3d;
                206: message_byte=hex_char(roi_right_snapshot >> 12);
                207: message_byte=hex_char(roi_right_snapshot >> 8);
                208: message_byte=hex_char(roi_right_snapshot >> 4);
                209: message_byte=hex_char(roi_right_snapshot >> 0);
                210: message_byte=8'h20;
                211: message_byte=8'h52;
                212: message_byte=8'h42;
                213: message_byte=8'h3d;
                214: message_byte=hex_char(roi_bottom_snapshot >> 12);
                215: message_byte=hex_char(roi_bottom_snapshot >> 8);
                216: message_byte=hex_char(roi_bottom_snapshot >> 4);
                217: message_byte=hex_char(roi_bottom_snapshot >> 0);
                218: message_byte=8'h20;
                219: message_byte=8'h43;
                220: message_byte=8'h41;
                221: message_byte=8'h3d;
                222: message_byte=hex_char(camera_applied_seq_snapshot >> 12);
                223: message_byte=hex_char(camera_applied_seq_snapshot >> 8);
                224: message_byte=hex_char(camera_applied_seq_snapshot >> 4);
                225: message_byte=hex_char(camera_applied_seq_snapshot >> 0);
                226: message_byte=8'h20;
                227: message_byte=8'h52;
                228: message_byte=8'h41;
                229: message_byte=8'h3d;
                230: message_byte=hex_char(roi_applied_seq_snapshot >> 12);
                231: message_byte=hex_char(roi_applied_seq_snapshot >> 8);
                232: message_byte=hex_char(roi_applied_seq_snapshot >> 4);
                233: message_byte=hex_char(roi_applied_seq_snapshot >> 0);
                234: message_byte=8'h20;
                235: message_byte=8'h43;
                236: message_byte=8'h42;
                237: message_byte=8'h3d;
                238: message_byte=hex_char(settings_busy_snapshot >> 0);
                239: message_byte=8'h20;
                240: message_byte=8'h4d;
                241: message_byte=8'h54;
                242: message_byte=8'h3d;
                243: message_byte=hex_char(locked_target_snapshot >> 28);
                244: message_byte=hex_char(locked_target_snapshot >> 24);
                245: message_byte=hex_char(locked_target_snapshot >> 20);
                246: message_byte=hex_char(locked_target_snapshot >> 16);
                247: message_byte=hex_char(locked_target_snapshot >> 12);
                248: message_byte=hex_char(locked_target_snapshot >> 8);
                249: message_byte=hex_char(locked_target_snapshot >> 4);
                250: message_byte=hex_char(locked_target_snapshot >> 0);
                251: message_byte=8'h20;
                252: message_byte=8'h4d;
                253: message_byte=8'h45;
                254: message_byte=8'h3d;
                255: message_byte=hex_char(move_error_snapshot >> 28);
                256: message_byte=hex_char(move_error_snapshot >> 24);
                257: message_byte=hex_char(move_error_snapshot >> 20);
                258: message_byte=hex_char(move_error_snapshot >> 16);
                259: message_byte=hex_char(move_error_snapshot >> 12);
                260: message_byte=hex_char(move_error_snapshot >> 8);
                261: message_byte=hex_char(move_error_snapshot >> 4);
                262: message_byte=hex_char(move_error_snapshot >> 0);
                263: message_byte=8'h20;
                264: message_byte=8'h4d;
                265: message_byte=8'h53;
                266: message_byte=8'h3d;
                267: message_byte=hex_char(move_state_snapshot >> 0);
                268: message_byte=8'h20;
                269: message_byte=8'h4d;
                270: message_byte=8'h42;
                271: message_byte=8'h3d;
                272: message_byte=hex_char(move_busy_snapshot >> 0);
                273: message_byte=8'h20;
                274: message_byte=8'h4f;
                275: message_byte=8'h4b;
                276: message_byte=8'h3d;
                277: message_byte=hex_char(reached_snapshot >> 0);
                278: message_byte=8'h20;
                279: message_byte=8'h52;
                280: message_byte=8'h44;
                281: message_byte=8'h3d;
                282: message_byte=hex_char(request_ready_snapshot >> 0);
                283: message_byte=8'h20;
                284: message_byte=8'h53;
                285: message_byte=8'h50;
                286: message_byte=8'h3d;
                287: message_byte=hex_char(spent_snapshot >> 4);
                288: message_byte=hex_char(spent_snapshot >> 0);
                289: message_byte=8'h20;
                290: message_byte=8'h4d;
                291: message_byte=8'h43;
                292: message_byte=8'h3d;
                293: message_byte=hex_char(command_position_snapshot >> 28);
                294: message_byte=hex_char(command_position_snapshot >> 24);
                295: message_byte=hex_char(command_position_snapshot >> 20);
                296: message_byte=hex_char(command_position_snapshot >> 16);
                297: message_byte=hex_char(command_position_snapshot >> 12);
                298: message_byte=hex_char(command_position_snapshot >> 8);
                299: message_byte=hex_char(command_position_snapshot >> 4);
                300: message_byte=hex_char(command_position_snapshot >> 0);
                301: message_byte=8'h20;
                302: message_byte=8'h43;
                303: message_byte=8'h52;
                304: message_byte=8'h3d;
                305: message_byte=hex_char(cancel_reason_snapshot >> 0);
                306: message_byte=8'h20;
                307: message_byte=8'h52;
                308: message_byte=8'h4a;
                309: message_byte=8'h3d;
                310: message_byte=hex_char(reject_reason_snapshot >> 0);
                311: message_byte=8'h20;
                312: message_byte=8'h41;
                313: message_byte=8'h43;
                314: message_byte=8'h3d;
                315: message_byte=hex_char(accepted_count_snapshot >> 12);
                316: message_byte=hex_char(accepted_count_snapshot >> 8);
                317: message_byte=hex_char(accepted_count_snapshot >> 4);
                318: message_byte=hex_char(accepted_count_snapshot >> 0);
                319: message_byte=8'h20;
                320: message_byte=8'h44;
                321: message_byte=8'h43;
                322: message_byte=8'h3d;
                323: message_byte=hex_char(completed_count_snapshot >> 12);
                324: message_byte=hex_char(completed_count_snapshot >> 8);
                325: message_byte=hex_char(completed_count_snapshot >> 4);
                326: message_byte=hex_char(completed_count_snapshot >> 0);
                327: message_byte=8'h20;
                328: message_byte=8'h43;
                329: message_byte=8'h43;
                330: message_byte=8'h3d;
                331: message_byte=hex_char(center_count_snapshot >> 12);
                332: message_byte=hex_char(center_count_snapshot >> 8);
                333: message_byte=hex_char(center_count_snapshot >> 4);
                334: message_byte=hex_char(center_count_snapshot >> 0);
                335: message_byte=8'h20;
                336: message_byte=8'h4b;
                337: message_byte=8'h43;
                338: message_byte=8'h3d;
                339: message_byte=hex_char(center_status_snapshot >> 0);
                340: message_byte=8'h20;
                341: message_byte=8'h4d;
                342: message_byte=8'h44;
                343: message_byte=8'h3d;
                344: message_byte=hex_char(operating_mode_snapshot >> 0);
                345: message_byte=8'h20;
                346: message_byte=8'h41;
                347: message_byte=8'h52;
                348: message_byte=8'h3d;
                349: message_byte=hex_char(auto_run_snapshot >> 0);
                350: message_byte=8'h20;
                351: message_byte=8'h4b;
                352: message_byte=8'h44;
                353: message_byte=8'h3d;
                354: message_byte=hex_char(damping_snapshot >> 4);
                355: message_byte=hex_char(damping_snapshot >> 0);
                356: message_byte=8'h20;
                357: message_byte=8'h53;
                358: message_byte=8'h4c;
                359: message_byte=8'h3d;
                360: message_byte=hex_char(slew_snapshot >> 4);
                361: message_byte=hex_char(slew_snapshot >> 0);
                362: message_byte=8'h20;
                363: message_byte=8'h52;
                364: message_byte=8'h55;
                365: message_byte=8'h3d;
                366: message_byte=hex_char(runtime_seconds_snapshot >> 12);
                367: message_byte=hex_char(runtime_seconds_snapshot >> 8);
                368: message_byte=hex_char(runtime_seconds_snapshot >> 4);
                369: message_byte=hex_char(runtime_seconds_snapshot >> 0);
                370: message_byte=8'h20;
                371: message_byte=8'h44;
                372: message_byte=8'h53;
                373: message_byte=8'h3d;
                374: message_byte=hex_char(follow_sequence_snapshot >> 12);
                375: message_byte=hex_char(follow_sequence_snapshot >> 8);
                376: message_byte=hex_char(follow_sequence_snapshot >> 4);
                377: message_byte=hex_char(follow_sequence_snapshot >> 0);
                378: message_byte=8'h20;
                379: message_byte=8'h43;
                380: message_byte=8'h5a;
                381: message_byte=8'h3d;
                382: message_byte=hex_char(center_confirmed_snapshot >> 0);
                383: message_byte=8'h20;
                384: message_byte=8'h56;
                385: message_byte=8'h45;
                386: message_byte=8'h3d;
                387: message_byte=hex_char(velocity_snapshot >> 12);
                388: message_byte=hex_char(velocity_snapshot >> 8);
                389: message_byte=hex_char(velocity_snapshot >> 4);
                390: message_byte=hex_char(velocity_snapshot >> 0);
                391: message_byte=8'h20;
                392: message_byte=8'h41;
                393: message_byte=8'h54;
                394: message_byte=8'h3d;
                395: message_byte=hex_char(drive_target_snapshot >> 4);
                396: message_byte=hex_char(drive_target_snapshot >> 0);
                397: message_byte=8'h20;
                398: message_byte=8'h53;
                399: message_byte=8'h52;
                400: message_byte=8'h3d;
                401: message_byte=hex_char(stop_reason_snapshot >> 0);
                402: message_byte=8'h20;
                403: message_byte=8'h41;
                404: message_byte=8'h4a;
                405: message_byte=8'h3d;
                406: message_byte=hex_char(auto_reject_snapshot >> 0);
                407: message_byte=8'h20;
                408: message_byte=8'h45;
                409: message_byte=8'h4c;
                410: message_byte=8'h3d;
                411: message_byte=hex_char(elapsed_seconds_snapshot >> 12);
                412: message_byte=hex_char(elapsed_seconds_snapshot >> 8);
                413: message_byte=hex_char(elapsed_seconds_snapshot >> 4);
                414: message_byte=hex_char(elapsed_seconds_snapshot >> 0);
                415: message_byte=8'h20;
                416: message_byte=8'h54;
                417: message_byte=8'h53;
                418: message_byte=8'h3d;
                419: message_byte=hex_char(total_pulses_snapshot >> 28);
                420: message_byte=hex_char(total_pulses_snapshot >> 24);
                421: message_byte=hex_char(total_pulses_snapshot >> 20);
                422: message_byte=hex_char(total_pulses_snapshot >> 16);
                423: message_byte=hex_char(total_pulses_snapshot >> 12);
                424: message_byte=hex_char(total_pulses_snapshot >> 8);
                425: message_byte=hex_char(total_pulses_snapshot >> 4);
                426: message_byte=hex_char(total_pulses_snapshot >> 0);
                427: message_byte=8'h0d;
                428: message_byte=8'h0a;
            default:message_byte="?";
        endcase end
    endfunction
    always @(posedge clk)begin
        if(!rst_n)begin
            x_snapshot<=0; y_snapshot<=0; valid_snapshot<=0; age_snapshot<=0; frames_snapshot<=0; enabled_snapshot<=0; origin_valid_snapshot<=0; fault_snapshot<=0; count1_snapshot<=0; count2_snapshot<=0; errors1_snapshot<=0; errors2_snapshot<=0; position_snapshot<=0; target_snapshot<=0; center_snapshot<=0; deadzone_snapshot<=0; gain_snapshot<=0; limit_snapshot<=0; cfg_sequence_snapshot<=0; rx_errors_snapshot<=0; quality_snapshot<=0; bad_snapshot<=0; cam_sequence_snapshot<=0; exposure_snapshot<=0; sensor_gain_snapshot<=0; roi_left_snapshot<=0; roi_top_snapshot<=0; roi_right_snapshot<=0; roi_bottom_snapshot<=0; camera_applied_seq_snapshot<=0; roi_applied_seq_snapshot<=0; settings_busy_snapshot<=0; locked_target_snapshot<=0; move_error_snapshot<=0; move_state_snapshot<=0; move_busy_snapshot<=0; reached_snapshot<=0; request_ready_snapshot<=0; spent_snapshot<=0; command_position_snapshot<=0; cancel_reason_snapshot<=0; reject_reason_snapshot<=0; accepted_count_snapshot<=0; completed_count_snapshot<=0; center_count_snapshot<=0; center_status_snapshot<=0; operating_mode_snapshot<=0; auto_run_snapshot<=0; damping_snapshot<=0; slew_snapshot<=0; runtime_seconds_snapshot<=0; follow_sequence_snapshot<=0; center_confirmed_snapshot<=0; velocity_snapshot<=0; drive_target_snapshot<=0; stop_reason_snapshot<=0; auto_reject_snapshot<=0; elapsed_seconds_snapshot<=0; total_pulses_snapshot<=0;
            timer<=0;report_pending<=0;frame_active<=0;bit_active<=0;char_index<=0;bit_index<=0;
            baud_count<=0;serial_shift<=10'h3ff;
        end else begin
            if(timer==REPORT_CYCLES-1)begin timer<=0;report_pending<=1;end else timer<=timer+1'b1;
            if(!frame_active && (timer==REPORT_CYCLES-1 || report_pending) && (!SHARED_TX || grant))begin
                report_pending<=0;
                x_snapshot<=x; y_snapshot<=y; valid_snapshot<=valid; age_snapshot<=age; frames_snapshot<=frames; enabled_snapshot<=enabled; origin_valid_snapshot<=origin_valid; fault_snapshot<=fault; count1_snapshot<=count1; count2_snapshot<=count2; errors1_snapshot<=errors1; errors2_snapshot<=errors2; position_snapshot<=position; target_snapshot<=target; center_snapshot<=center; deadzone_snapshot<=deadzone; gain_snapshot<=gain; limit_snapshot<=limit; cfg_sequence_snapshot<=cfg_sequence; rx_errors_snapshot<=rx_errors; quality_snapshot<=quality; bad_snapshot<=bad; cam_sequence_snapshot<=cam_sequence; exposure_snapshot<=exposure; sensor_gain_snapshot<=sensor_gain; roi_left_snapshot<=roi_left; roi_top_snapshot<=roi_top; roi_right_snapshot<=roi_right; roi_bottom_snapshot<=roi_bottom; camera_applied_seq_snapshot<=camera_applied_seq; roi_applied_seq_snapshot<=roi_applied_seq; settings_busy_snapshot<=settings_busy; locked_target_snapshot<=locked_target; move_error_snapshot<=move_error; move_state_snapshot<=move_state; move_busy_snapshot<=move_busy; reached_snapshot<=reached; request_ready_snapshot<=request_ready; spent_snapshot<=spent; command_position_snapshot<=command_position; cancel_reason_snapshot<=cancel_reason; reject_reason_snapshot<=reject_reason; accepted_count_snapshot<=accepted_count; completed_count_snapshot<=completed_count; center_count_snapshot<=center_count; center_status_snapshot<=center_status; operating_mode_snapshot<=operating_mode; auto_run_snapshot<=auto_run; damping_snapshot<=damping; slew_snapshot<=slew; runtime_seconds_snapshot<=runtime_seconds; follow_sequence_snapshot<=follow_sequence; center_confirmed_snapshot<=center_confirmed; velocity_snapshot<=velocity; drive_target_snapshot<=drive_target; stop_reason_snapshot<=stop_reason; auto_reject_snapshot<=auto_reject; elapsed_seconds_snapshot<=elapsed_seconds; total_pulses_snapshot<=total_pulses;
                char_index<=0;frame_active<=1;
            end else if(frame_active)begin
                if(!bit_active)begin
                    serial_shift<={1'b1,message_byte(char_index),1'b0};
                    baud_count<=0;bit_index<=0;bit_active<=1;
                end else if(baud_count==UART_DIV-1)begin
                    baud_count<=0;serial_shift<={1'b1,serial_shift[9:1]};
                    if(bit_index==9)begin
                        bit_active<=0;
                        if(char_index==(WIDE_RANGE ? 440 : 428))frame_active<=0;else char_index<=char_index+1'b1;
                    end else bit_index<=bit_index+1'b1;
                end else baud_count<=baud_count+1'b1;
            end
        end
    end
endmodule

// Source: vision_monitor.v
// Guarded manual / continuous PD / explicit software-reference return modes.
module vision_monitor #(
    parameter integer DEBOUNCE_CYCLES=1000000,ENABLE_CYCLES=5000000,POR_CYCLES=512,
    UART_DIV=434,REPORT_CYCLES=12500000,MS_CYCLES=50000,STALE_MS=200,PACKET_TIMEOUT_CYCLES=5000000,
    SETUP_CYCLES=1000000,HALF_CYCLES=125000,SETTLE_CYCLES=2500000,TIMEOUT_CYCLES=250000000,DEFAULT_MODE=1,DEFAULT_GAIN=3,DEFAULT_LIMIT=32,CONTROL_MS=20,LOSS_GRACE_MS=500,
    DEFAULT_DEADZONE=5,DEFAULT_SLEW=8,DEFAULT_RUNTIME=60,CASCADE_DEFAULT=0,CASCADE_PRESET=0,REGION_PROBE=0,REGION_RETURN=0,ASYM_RANGE=0,
    FOLLOW_SETUP_CYCLES=(SETUP_CYCLES>=4 ? SETUP_CYCLES/4 : 1),FOLLOW_SETTLE_CYCLES=(SETTLE_CYCLES>=5 ? SETTLE_CYCLES/5 : 1)
)(
    input wire clk,external_rst_n,
    input wire [3:0] key_n,
    input wire [1:0] encoder_j1,encoder_j2,
    input wire frame,
    input wire [10:0] ball_x,ball_y,
    input wire confirmed,bad_frame,
    input wire [3:0] confidence,
    input wire rx,
    output wire tx,motor_en,motor_dir,motor_step,
    output wire [3:0] led,
    input wire [15:0] camera_applied_seq,roi_applied_seq,
    input wire settings_busy,
    output wire [15:0] cam_sequence,exposure,sensor_gain,roi_left,roi_top,roi_right,roi_bottom
);
    localparam TW=ASYM_RANGE ? 16 : 8;
    wire limits_ready;wire signed [31:0] limit_anchor,command_anchor,work_min,work_max,feedback_min,feedback_max,command_min,command_max;
    reg [31:0] por_count=0;
    always @(posedge clk)begin
        if(!external_rst_n)por_count<=0;
        else if(por_count<POR_CYCLES)por_count<=por_count+1'b1;
    end
    wire rst_n=external_rst_n && por_count>=POR_CYCLES;
    wire signed [31:0] count1,count2;
    wire [15:0] errors1,errors2;
    wire [1:0] ab1,ab2;
    encoder_ab_counter counter1(clk,rst_n,encoder_j1,count1,errors1,ab1);
    encoder_ab_counter counter2(clk,rst_n,encoder_j2,count2,errors2,ab2);
    wire [15:0] cfg_sequence,center,deadzone,rx_errors;
    wire [7:0] gain,limit;
    wire [10:0] x,y;
    wire center_applied,center_conflict;
    wire applied,parameter_applied;
    wire [15:0] follow_sequence,runtime_seconds;
    wire [7:0] operating_mode,damping,slew;
    reg center_confirmed=0;
    reg result_frame=0;
    always @(posedge clk)begin
        if(!rst_n)begin center_confirmed<=0;result_frame<=0;end
        else begin result_frame<=frame;if(center_applied)center_confirmed<=1;end
    end
    wire center_event,center_request;
    reg [15:0] center_count=0;
    reg [3:0] center_status=0;
    vision_config_rx #(.UART_DIV(UART_DIV),.PACKET_TIMEOUT_CYCLES(PACKET_TIMEOUT_CYCLES),.DEFAULT_MODE(DEFAULT_MODE),.DEFAULT_GAIN(DEFAULT_GAIN),.DEFAULT_LIMIT(DEFAULT_LIMIT),
        .DEFAULT_DEADZONE(DEFAULT_DEADZONE),.DEFAULT_SLEW(DEFAULT_SLEW),.DEFAULT_RUNTIME(DEFAULT_RUNTIME)) config_rx(
        clk,rst_n,rx,cfg_sequence,center,deadzone,gain,limit,rx_errors,applied,
        cam_sequence,exposure,sensor_gain,roi_left,roi_top,roi_right,roi_bottom,
        center_request,{5'b0,x},center_applied,center_conflict,parameter_applied,follow_sequence,runtime_seconds,operating_mode,damping,slew);
    wire valid;
    wire [15:0] age,frames;
    wire signed [7:0] manual_target;wire signed [TW-1:0] target,drive_target;
    wire [3:0] frame_confidence;
    wire frame_bad;
    vision_shadow_target #(.MS_CYCLES(MS_CYCLES),.STALE_MS(STALE_MS)) shadow(
        clk,rst_n,frame,ball_x,ball_y,confirmed,bad_frame,confidence,center,deadzone,gain,limit,
        x,y,valid,age,frames,manual_target,frame_confidence,frame_bad);
    reg fresh_after_settings=0;
    always @(posedge clk)begin
        if(!rst_n || applied || center_request || settings_busy)fresh_after_settings<=0;
        else if(frame && confirmed && !bad_frame && confidence>=8 && ball_x<=1919 && ball_y<=1079)fresh_after_settings<=1;
    end
    wire settings_ready=!settings_busy && fresh_after_settings;
    wire [3:0] state,fault,cancel_reason,reject_reason,core_reject,supervisor_reject,stop_reason;
    wire auto_run,request,following,external_cancel,key2_event,neutral_ready;
    wire [3:0] external_cancel_reason;
    wire signed [15:0] velocity;
    wire [15:0] elapsed_seconds;
    wire [31:0] total_pulses;
    assign reject_reason=supervisor_reject!=0 ? supervisor_reject : core_reject;
    wire enable_request,origin_valid,ready,busy,reached;
    wire [7:0] spent;
    wire [15:0] accepted_count,completed_count;
    wire c_applied,c_commit,c_diagnostics,c_polarity;
    wire [15:0] c_sequence,c_errors,c_kp,c_kv,c_ki,c_inner_kp,c_ff,c_max_hz,c_slew;
    wire signed [15:0] c_bias;
    wire [7:0] c_selection,c_delay,c_amplitude;
    wire c_selected=c_selection!=0 && operating_mode==1;
    wire effective_settings_ready=c_selected && c_selection==2 ? !settings_busy : settings_ready;
    wire l_run,l_request,l_following,l_external_cancel,l_neutral_ready;
    wire [3:0] l_external_reason,l_stop,l_reject;
    wire signed [7:0] l_target,l_drive;
    wire signed [15:0] l_velocity;
    wire [15:0] l_elapsed;wire [31:0] l_pulses;
    wire n_run,n_request,n_following,n_external_cancel,n_neutral_ready,n_updated,n_stable,n_checkpoint,n_quiescing;
    wire [1:0] n_control_phase,n_outer_phase,n_region_phase;
    wire signed [31:0] n_outer_raw,n_region_raw,n_region_velocity;
    wire n_region_changed,n_region_holding,n_region_active;
    wire [15:0] n_region_hz,n_region_slew;
    wire [3:0] n_external_reason,n_stop,n_reject,n_cal_status,n_cal_phase;
    wire signed [TW-1:0] n_target,n_drive,n_neutral;
    wire signed [15:0] n_velocity_legacy,n_frequency;
    wire signed [31:0] n_raw_q8,n_reference_q8,n_velocity;
    wire [15:0] n_elapsed;wire [31:0] n_pulses,n_time,n_frame_time;
    wire legacy_tx,legacy_busy,legacy_wants,diag_tx,diag_busy,event_tx,event_busy,event_wants,event_overflow;
    wire p_run,p_request,p_cancel;wire [3:0] p_status,p_phase,p_reason,p_reject;wire signed [7:0] p_target;
    wire [7:0] d_pending;wire signed [15:0] d_commands,d_delta;wire d_consistent,d_wrong;
    reg c_fresh=0,c_profile_ready=CASCADE_PRESET,c_builtin=CASCADE_PRESET;
    assign n_region_active=REGION_RETURN && c_selected && c_builtin;
    assign n_raw_q8=ASYM_RANGE ? n_region_raw : n_region_active ? n_region_raw : n_outer_raw;
    assign n_control_phase=ASYM_RANGE ? n_region_phase : n_region_active ? n_region_phase : n_outer_phase;
    always @(posedge clk)begin
      if(!rst_n)begin c_profile_ready<=CASCADE_PRESET;c_builtin<=CASCADE_PRESET;end
      else begin
        if(c_applied)begin c_builtin<=0;c_profile_ready<=c_selection==1 && (c_kp!=0 || c_kv!=0);end
        else if(applied || center_request || settings_busy)c_profile_ready<=c_builtin;
      end
      if(!rst_n || c_applied || applied || settings_busy)c_fresh<=0;
      else if(result_frame && valid)c_fresh<=1;
    end
    assign auto_run=REGION_PROBE ? p_run : c_selected ? n_run : l_run;
    assign request=REGION_PROBE ? p_request : c_selected ? n_request : l_request;
    assign following=REGION_PROBE ? 1'b0 : c_selected ? n_following : l_following;
    assign external_cancel=REGION_PROBE ? p_cancel : c_selected ? n_external_cancel : l_external_cancel;
    assign external_cancel_reason=REGION_PROBE ? p_reason : c_selected ? n_external_reason : l_external_reason;
    assign neutral_ready=c_selected ? n_neutral_ready : l_neutral_ready;
    assign stop_reason=REGION_PROBE ? p_reason : c_selected ? n_stop : l_stop;
    assign supervisor_reject=REGION_PROBE ? p_reject : c_selected ? n_reject : l_reject;
    assign drive_target=REGION_PROBE ? p_target : c_selected ? n_drive : l_drive;
    assign target=REGION_PROBE ? p_target : c_selected ? ((valid || n_run || c_selection==2) ? n_target : 8'sd0) : l_target;
    assign velocity=c_selected ? n_velocity_legacy : l_velocity;
    assign elapsed_seconds=c_selected ? n_elapsed : l_elapsed;
    assign total_pulses=c_selected ? n_pulses : l_pulses;
    wire signed [(ASYM_RANGE ? 31 : 7):0] report_target=target,report_drive=drive_target,report_neutral=n_neutral;
    assign tx=legacy_busy ? legacy_tx : (event_busy ? event_tx : diag_tx);

    wire signed [31:0] command_position,locked_target,position,move_error;
    vision_center_key #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) center_key(clk,rst_n,key_n[2],center_event);
    wire hold_valid;
    vision_result_guard #(.MS_CYCLES(MS_CYCLES),.LOSS_GRACE_MS(LOSS_GRACE_MS)) loss_guard(
        .clk(clk),.rst_n(rst_n),.clear(applied || center_request || settings_busy),
        .frame(result_frame),.valid(valid),.hold_valid(hold_valid));
    wire incoming_bad=frame && (!confirmed || bad_frame || confidence<8 || ball_x>1919 || ball_y>1079);
    wire executor_valid=REGION_PROBE ? (p_run && c_selection==2 ? 1'b1 : (valid && !incoming_bad)) : (c_selected && c_selection==2 && n_run) ? 1'b1 : ((auto_run && operating_mode==1) ? hold_valid : (valid && !incoming_bad));
    assign center_request=center_event && !c_commit && key_n[3] && fault==0 && !busy && !auto_run && settings_ready &&
        !applied && valid && age<STALE_MS && !incoming_bad && (!ASYM_RANGE || ready);
    always @(posedge clk)begin
        if(!rst_n)begin center_count<=0;center_status<=0;end
        else if(center_applied)begin center_count<=center_count+1'b1;center_status<=(position>(ASYM_RANGE ? work_max : 60) || position<(ASYM_RANGE ? work_min : -60)) ? 8 : 1;end
        else if(center_conflict || (center_event && c_commit))center_status<=7;
        else if(center_event)begin
            if(!key_n[3])center_status<=6;
            else if(fault!=0)center_status<=5;
            else if(busy || auto_run)center_status<=4;
            else if(!settings_ready || applied)center_status<=3;
            else if(!valid || age>=STALE_MS || incoming_bad)center_status<=2;
        end
    end
    enable_latch #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES),.HOLD_ON_FAULT(1)) enable_control(
        clk,rst_n,key_n[0],key_n[3],fault!=0,enable_request);
    vision_center_key #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) key2(clk,rst_n,key_n[1],key2_event);
    vision_follow_control #(.MS_CYCLES(MS_CYCLES),.CONTROL_MS(CONTROL_MS)) supervisor(
        .clk(clk),.rst_n(rst_n),.key_event(key2_event && !c_selected && !REGION_PROBE && !ASYM_RANGE),.k4_n(key_n[3]),.enable_request(enable_request),
        .config_changed(applied || center_request || c_applied),.visual_valid(valid && !incoming_bad),.follow_valid(hold_valid),.visual_stale(age>=STALE_MS),
        .settings_ready(settings_ready),.center_confirmed(center_confirmed),.frame(result_frame),.neutral_capture(center_applied),
        .x(x),.center(center),.deadzone(deadzone),.runtime_seconds(runtime_seconds),.gain(gain),.limit(limit),
        .mode(operating_mode),.damping(damping),.slew(slew),.manual_target(manual_target),
        .core_ready(ready),.core_busy(busy),.fault(fault),.step(motor_step),.position(position),
        .request(l_request),.following(l_following),.neutral_ready(l_neutral_ready),.external_cancel(l_external_cancel),.external_cancel_reason(l_external_reason),
        .run(l_run),.stop_reason(l_stop),.reject_reason(l_reject),.drive_target(l_drive),
        .candidate(l_target),.velocity(l_velocity),.elapsed_seconds(l_elapsed),.total_pulses(l_pulses));
    vision_servo_core #(.TARGET_BITS(TW),.ASYM_LIMITS(ASYM_RANGE),.EXTERNAL_CONTROL(1),.HOLD_ON_FAULT(1),.FAST_CAPABLE(1),.CHECKPOINT_CAPABLE(1),.FAST_MS_CYCLES(MS_CYCLES),.TAIL_CHECKPOINT(REGION_RETURN),.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES),.ENABLE_CYCLES(ENABLE_CYCLES),
        .SETUP_CYCLES(SETUP_CYCLES),.HALF_CYCLES(HALF_CYCLES),.SETTLE_CYCLES(SETTLE_CYCLES),
        .FOLLOW_SETUP_CYCLES(FOLLOW_SETUP_CYCLES),.FOLLOW_SETTLE_CYCLES(FOLLOW_SETTLE_CYCLES),
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)) motor_core(
        .clk(clk),.rst_n(rst_n),.enable_request(enable_request),.k4_n(key_n[3]),.k2_n(1'b1),
        .request(request),.following(following),.external_cancel(external_cancel),.external_cancel_reason(external_cancel_reason),
        .fast_mode(!REGION_PROBE && c_selected && (n_run || busy)),.fast_rate_hz(n_frequency),.checkpoint(!REGION_PROBE && c_selected && n_checkpoint),
        .encoder_count(count1),.encoder_errors(errors1),.visual_valid(executor_valid),.visual_stale(c_selected && c_selection==2 ? 1'b0 : age>=STALE_MS),
        .target_min(work_min),.target_max(work_max),.feedback_min(feedback_min),.feedback_max(feedback_max),.command_min(command_min),.command_max(command_max),
        .settings_ready(effective_settings_ready),.config_changed(applied || center_request || c_applied),.visual_target(drive_target),
        .motor_en(motor_en),.motor_dir(motor_dir),.motor_step(motor_step),
        .command_position(command_position),.locked_target(locked_target),.position(position),.error(move_error),
        .state(state),.fault(fault),.cancel_reason(cancel_reason),.reject_reason(core_reject),.spent(spent),
        .origin_valid(origin_valid),.ready(ready),.busy(busy),.reached(reached),
        .accepted_count(accepted_count),.completed_count(completed_count),.diag_pending(d_pending),.diag_commands(d_commands),.diag_delta(d_delta),.diag_consistent(d_consistent),.diag_wrong(d_wrong));
    wire request_ready=(!ASYM_RANGE || (n_region_active && limits_ready)) && (c_selected ? (ready && (c_selection==2 || (valid && !incoming_bad && age<STALE_MS && c_fresh))) : (auto_run || (ready && valid && !incoming_bad && age<STALE_MS))) && effective_settings_ready && !applied && !c_applied && !center_request && fault==0 && (operating_mode!=1 || (center_confirmed && neutral_ready && (!c_selected || c_selection!=1 || c_profile_ready)));
    assign led={fault!=0,request_ready,(busy || auto_run),motor_en};
    vision_report_tx #(.WIDE_RANGE(ASYM_RANGE),.UART_DIV(UART_DIV),.REPORT_CYCLES(REPORT_CYCLES),.SHARED_TX(1)) report_tx(
        .clk(clk),.rst_n(rst_n),.x(x),.y(y),.valid(valid),.age(age),.frames(frames),
        .enabled(motor_en),.origin_valid(origin_valid),.fault(fault),.count1(count1),.count2(count2),
        .errors1(errors1),.errors2(errors2),.position(position),.target(report_target),
        .center(center),.deadzone(deadzone),.gain(gain),.limit(limit),.cfg_sequence(cfg_sequence),.rx_errors(rx_errors),
        .quality(frame_confidence),.bad(frame_bad),.cam_sequence(cam_sequence),.exposure(exposure),.sensor_gain(sensor_gain),
        .roi_left(roi_left),.roi_top(roi_top),.roi_right(roi_right),.roi_bottom(roi_bottom),
        .camera_applied_seq(camera_applied_seq),.roi_applied_seq(roi_applied_seq),.settings_busy(settings_busy),.locked_target(locked_target),.move_error(move_error),.move_state(state),
        .move_busy(busy),.reached(reached),.request_ready(request_ready),.spent(spent),.command_position(command_position),
        .cancel_reason(cancel_reason),.reject_reason(reject_reason),.accepted_count(accepted_count),.completed_count(completed_count),
        .center_count(center_count),.center_status(center_status),
        .operating_mode(operating_mode),.auto_run(auto_run),.damping(damping),.slew(slew),.runtime_seconds(runtime_seconds),
        .follow_sequence(follow_sequence),.center_confirmed(center_confirmed),.velocity(velocity),.drive_target(report_drive),
        .stop_reason(stop_reason),.auto_reject(supervisor_reject),.elapsed_seconds(elapsed_seconds),.total_pulses(total_pulses),.grant(!diag_busy && !event_busy),.busy(legacy_busy),.wants(legacy_wants),.tx(legacy_tx));

    vision_cascade_config #(.UART_DIV(UART_DIV),.TIMEOUT_CYCLES(PACKET_TIMEOUT_CYCLES),.DEFAULT_SELECTION(CASCADE_DEFAULT),.DEFAULT_PRESET(CASCADE_PRESET)) cascade_config(
     .clk(clk),.rst_n(rst_n),.rx(rx),.applied(c_applied),.commit_now(c_commit),.config_seq(c_sequence),.errors(c_errors),.selection(c_selection),
     .diagnostics(c_diagnostics),.polarity(c_polarity),.kp(c_kp),.kv(c_kv),.ki(c_ki),.inner_kp(c_inner_kp),.feedforward(c_ff),
     .max_hz(c_max_hz),.slew_rate(c_slew),.bias(c_bias),.delay_ms(c_delay),.amplitude(c_amplitude));
    vision_cascade_outer #(.WIDE_RANGE(ASYM_RANGE),.NEUTRAL_BITS(TW),.MS_CYCLES(MS_CYCLES),.REGION_VELOCITY(REGION_RETURN)) cascade_outer(
     .clk(clk),.rst_n(rst_n),.run(n_run && c_selected),.frame(result_frame),.valid(valid && !incoming_bad),.stale(age>=STALE_MS),
     .clear(applied || c_applied || settings_busy),.neutral_capture(center_applied),.polarity(c_polarity),.x(x),.center(center),.deadzone(deadzone),
     .kp(c_kp),.kv(c_kv),.ki(c_ki),.limit(limit),.delay_ms(c_delay),.bias(c_bias),.position(position),.work_min(work_min),.work_max(work_max),.target_q8(n_outer_raw),
     .velocity_q4(n_velocity),.legacy_velocity_q4(n_velocity_legacy),.neutral(n_neutral),.neutral_ready(n_neutral_ready),
     .updated(n_updated),.stable(n_stable),.time_ms(n_time),.frame_time_ms(n_frame_time),
     .predictive(ASYM_RANGE ? 1'b1 : (c_selected && c_builtin)),.control_phase(n_outer_phase));
    vision_cascade_control #(.WIDE_RANGE(ASYM_RANGE),.TARGET_BITS(TW),.MS_CYCLES(MS_CYCLES)) cascade_control(
     .clk(clk),.rst_n(rst_n),.key_event(key2_event && c_selected && !REGION_PROBE),.k4_n(key_n[3]),.enable_request(enable_request),
     .settings_ready(effective_settings_ready && (c_selection==2 || c_fresh)),.center_confirmed(center_confirmed),
     .valid(valid && !incoming_bad),.hold_valid(hold_valid),.stale(age>=STALE_MS),.frame(result_frame),
     .config_changed(applied || c_applied || center_request || !c_selected),.neutral_capture(center_applied),
     .region_holding(n_region_holding),.profile_ready(c_profile_ready && (!ASYM_RANGE || (limits_ready && c_builtin))),.selection(c_selection),.amplitude(c_amplitude),.limit(limit),.max_hz(n_region_active && n_region_hz<c_max_hz ? n_region_hz : c_max_hz),.inner_kp(c_inner_kp),.feedforward(c_ff),
     .slew_rate(n_region_active && n_region_slew<c_slew ? n_region_slew : c_slew),.runtime_seconds(runtime_seconds),.kp(c_kp),.kv(c_kv),.roi_left(roi_left),.roi_right(roi_right),.center(center),.x(x),
     .position(position),.command_position(command_position),.raw_target_q8(n_raw_q8),.outer_updated(n_region_active ? n_region_changed : n_updated),.neutral_ready(n_neutral_ready),.neutral(n_neutral),.work_min(work_min),.work_max(work_max),
     .core_ready(ready),.core_busy(busy),.core_spent(spent),.fault(fault),.step(motor_step),.checkpoint(n_checkpoint),.diagnostic_quiescing(n_quiescing),.run(n_run),.request(n_request),.following(n_following),
     .external_cancel(n_external_cancel),.external_cancel_reason(n_external_reason),.stop_reason(n_stop),.reject_reason(n_reject),
     .cal_status(n_cal_status),.cal_phase(n_cal_phase),.reference_q8(n_reference_q8),.frequency_hz(n_frequency),.target(n_target),.drive_target(n_drive),
     .elapsed_seconds(n_elapsed),.total_pulses(n_pulses));
    vision_region_hold #(.WIDE_RANGE(ASYM_RANGE),.NEUTRAL_BITS(TW)) region_hold(
     .clk(clk),.rst_n(rst_n),.run(n_run && n_region_active),.updated(n_updated),.valid(valid && !incoming_bad),.stale(age>=STALE_MS),
     .clear(applied || c_applied || settings_busy),.neutral_capture(center_applied),.polarity(c_polarity),.x(x),.center(center),.limit(limit),
     .velocity_q4(n_velocity),.work_min(work_min),.work_max(work_max),.neutral(n_neutral),.target_q8(n_region_raw),.changed(n_region_changed),.holding(n_region_holding),.phase(n_region_phase),
     .max_hz(n_region_hz),.slew_rate(n_region_slew),.filtered_velocity_q4(n_region_velocity));
    vision_asym_limits asym_limits(.clk(clk),.rst_n(rst_n),.capture(center_applied && ASYM_RANGE),.ready(ready),.motor_en(motor_en),.position(position),.command_position(command_position),
     .qualified(limits_ready),.anchor(limit_anchor),.command_anchor(command_anchor),.work_min(work_min),.work_max(work_max),.feedback_min(feedback_min),.feedback_max(feedback_max),.command_min(command_min),.command_max(command_max));
    vision_region_probe #(.MS_CYCLES(MS_CYCLES)) region_probe(
     .clk(clk),.rst_n(rst_n),.key_event(key2_event && REGION_PROBE),.k4_n(key_n[3]),.enable_request(enable_request),
     .settings_ready(settings_ready),.valid(valid && !incoming_bad),.stale(age>=STALE_MS),.frame(result_frame),
     .changed(applied || c_applied || center_request),.center_confirmed(center_confirmed),.neutral_ready(n_neutral_ready),
     .core_ready(ready),.core_busy(busy),.event_overflow(event_overflow),.selection(c_selection),.neutral(n_neutral[7:0]),.position(position),.fault(fault),
     .time_ms(n_time),.x(x),.y(y),.center(center),.roi_left(roi_left),.roi_right(roi_right),.roi_top(roi_top),.roi_bottom(roi_bottom),
     .run(p_run),.request(p_request),.cancel(p_cancel),.status(p_status),.phase(p_phase),.reason(p_reason),.reject(p_reject),.target(p_target));
    vision_probe_events #(.UART_DIV(UART_DIV),.MS_CYCLES(MS_CYCLES)) probe_events(
     .clk(clk),.rst_n(rst_n),.enabled(REGION_PROBE!=0),.grant(!legacy_busy && !legacy_wants && !diag_busy),
     .time_ms(n_time),.step(motor_step),.dir(motor_dir),.position(position),.command_position(command_position),.state(state),
     .pending(d_pending),.commands(d_commands),.delta(d_delta),.diagnostic_flags({event_overflow,n_quiescing,d_consistent,d_wrong,fault}),
     .phase(p_phase),.status(p_status),.selection(c_selection),.reason(p_reason),.changed(c_applied || applied || center_request),
     .tx(event_tx),.busy(event_busy),.wants(event_wants),.overflow(event_overflow));
    // [trim] cascade UART diagnostics removed to recover slice area.
    // The legacy report_tx / probe event stream are kept unchanged.
    assign diag_tx   = 1'b1;
    assign diag_busy = 1'b0;
endmodule

// Source: vision_value_cdc.v
// Coalescing configuration mailbox: transfer once per changed value, keep bus held until ack.
module vision_value_cdc #(parameter WIDTH=48)(
 input wire src_clk,src_rst_n,input wire [WIDTH-1:0] src_value,
 input wire dst_clk,dst_rst_n,output reg [WIDTH-1:0] dst_value=0,output reg dst_valid=0
);
 reg [WIDTH-1:0] held=0,last_sent=0;
 reg initialized=0,request=0,ack=0,seen=0;
 reg [1:0] ack_sync=0,req_sync=0;
 always @(posedge src_clk or negedge src_rst_n)begin
  if(!src_rst_n)begin held<=0;last_sent<=0;initialized<=0;request<=0;ack_sync<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(request==ack_sync[1] && (!initialized || last_sent!=src_value))begin
    held<=src_value;last_sent<=src_value;initialized<=1;request<=~request;
   end
  end
 end
 always @(posedge dst_clk or negedge dst_rst_n)begin
  if(!dst_rst_n)begin req_sync<=0;seen<=0;ack<=0;dst_value<=0;dst_valid<=0;end
  else begin
   req_sync<={req_sync[0],request};
   if(req_sync[1]!=seen)begin dst_value<=held;dst_valid<=1;seen<=req_sync[1];ack<=req_sync[1];end
  end
 end
endmodule

// Source: vision_camera_control.v
// Separate camera/ROI path. No buttons or motor outputs in this module.
module vision_camera_control(
 input wire sys_clk,sys_rst_n,cam_clk,cam_rst_n,roi_clk,roi_rst_n,frame_start,
 input wire cam_cfg_done,ae_cfg_done,
 input wire [15:0] requested_seq,exposure,sensor_gain,roi_left,roi_top,roi_right,roi_bottom,
 output reg ae_req=0,output reg [15:0] ae=3600,ag=143,
 output reg [10:0] left=0,top=0,right=1279,bottom=719,output reg roi_change=0,
 output wire [15:0] camera_applied_seq,roi_applied_seq,output wire settings_busy
);
 wire [47:0] desired_cam;wire cam_valid;
 vision_value_cdc #(.WIDTH(48)) cam_mailbox(sys_clk,sys_rst_n,{requested_seq,exposure,sensor_gain},
  cam_clk,cam_rst_n,desired_cam,cam_valid);
 reg [1:0] state=0;
 reg [47:0] active=0,cam_applied=0;
 reg cam_applied_valid=0;
 always @(posedge cam_clk or negedge cam_rst_n)begin
  if(!cam_rst_n)begin state<=0;ae_req<=0;ae<=3600;ag<=143;active<=0;cam_applied<=0;cam_applied_valid<=0;end
  else begin
   ae_req<=0;
   case(state)
    0:if(cam_valid && cam_cfg_done && ae_cfg_done && (!cam_applied_valid || desired_cam!=cam_applied))begin
      active<=desired_cam;ae<=desired_cam[31:16];ag<=desired_cam[15:0];ae_req<=1;state<=1;
     end
    1:if(!ae_cfg_done)state<=2;
    2:if(ae_cfg_done)begin cam_applied<=active;cam_applied_valid<=1;state<=0;end
    default:state<=0;
   endcase
  end
 end
 wire [79:0] desired_roi;wire roi_valid;
 vision_value_cdc #(.WIDTH(80)) roi_mailbox(sys_clk,sys_rst_n,{requested_seq,roi_left,roi_top,roi_right,roi_bottom},
  roi_clk,roi_rst_n,desired_roi,roi_valid);
 reg [79:0] roi_applied=0;reg roi_applied_valid=0;
 always @(posedge roi_clk or negedge roi_rst_n)begin
  if(!roi_rst_n)begin left<=0;top<=0;right<=1279;bottom<=719;roi_change<=0;roi_applied<=0;roi_applied_valid<=0;end
  else begin
   roi_change<=0;
   if(frame_start && roi_valid && (!roi_applied_valid || desired_roi!=roi_applied))begin
    left<=desired_roi[58:48];top<=desired_roi[42:32];right<=desired_roi[26:16];bottom<=desired_roi[10:0];
    roi_applied<=desired_roi;roi_applied_valid<=1;roi_change<=1;
   end
  end
 end
 wire [48:0] cam_return;wire cam_return_valid;
 wire [16:0] roi_return;wire roi_return_valid;
 vision_value_cdc #(.WIDTH(49)) cam_ack(cam_clk,cam_rst_n,{cam_applied_valid,cam_applied},
  sys_clk,sys_rst_n,cam_return,cam_return_valid);
 vision_value_cdc #(.WIDTH(17)) roi_ack(roi_clk,roi_rst_n,{roi_applied_valid,roi_applied[79:64]},
  sys_clk,sys_rst_n,roi_return,roi_return_valid);
 assign camera_applied_seq=cam_return[47:32];
 assign roi_applied_seq=roi_return[15:0];
 assign settings_busy=!(cam_return_valid && cam_return[48] &&
  cam_return[47:0]=={requested_seq,exposure,sensor_gain} && roi_return_valid && roi_return[16] &&
  roi_return[15:0]==requested_seq);
endmodule

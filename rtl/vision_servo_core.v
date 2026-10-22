// Adapted from the physically passed position_core: same pulse phases and guards.
// External requests share the finite guard sequence; follow retargets only at CHECK.
module vision_servo_core #(
 parameter integer DEBOUNCE_CYCLES=1000000,ENABLE_CYCLES=5000000,SETUP_CYCLES=1000000,
 HALF_CYCLES=125000,SETTLE_CYCLES=2500000,TIMEOUT_CYCLES=250000000,
 POSITION_LIMIT=64,FEEDBACK_LIMIT=64,MOVE_BUDGET=64,NO_RESPONSE_LIMIT=3,TARGET_LIMIT=60,DEADBAND=2,EXTERNAL_CONTROL=0,FOLLOW_DEADBAND=1
)(
 input wire clk,rst_n,enable_request,k4_n,k2_n,
 input wire request,following,external_cancel,input wire [3:0] external_cancel_reason,
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
 wire in_band=wide_error<=$signed(allowed_error) && wide_error>=-$signed(allowed_error);
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
   if(command_event)begin
    if(fault!=0 || encoder_errors!=0)reject_reason<=9;
    else if(!enable_raw)reject_reason<=1;
    else if(busy)reject_reason<=6;
    else if(!ready)reject_reason<=2;
    else if(!settings_ready || config_changed)reject_reason<=5;
    else if(visual_stale)reject_reason<=4;
    else if(!visual_valid)reject_reason<=3;
    else if(!command_armed)reject_reason<=8;
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
    if(active)begin done<=0;cancel_reason<=extended_cancel ? external_cancel_reason : (config_changed || !settings_ready ? 5 : (visual_stale ? 4 : 3));end
   end else begin
    if(active)action_timer<=action_timer+1'b1;
    case(state)
     READY:begin
      step_r<=0;phase_count<=0;
      if(command_event && command_armed && visual_target<=TARGET_LIMIT && visual_target>=-TARGET_LIMIT)begin
       locked_target<={{24{visual_target[7]}},visual_target};spent<=0;no_response<=0;wrong_direction<=0;
       action_timer<=0;active<=1;done<=0;state<=CHECK;cancel_reason<=0;reject_reason<=0;
       accepted_count<=accepted_count+1'b1;
      end
     end
     CHECK:begin
      if(EXTERNAL_CONTROL && following && (visual_target>TARGET_LIMIT || visual_target< -TARGET_LIMIT))begin
       fault<=8;step_r<=0;active<=0;done<=0;state<=FAULT;cancel_reason<=6;
      end else if(EXTERNAL_CONTROL && following && locked_target!={{24{visual_target[7]}},visual_target})begin
       // Retarget only after the previous pulse/feedback phase. Never reset budget/time.
       locked_target<={{24{visual_target[7]}},visual_target};
      end else if(in_band)begin active<=0;done<=1;state<=READY;completed_count<=completed_count+1'b1;end
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

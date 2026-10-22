// Source: enable_latch.v
// Physical K1 is a debounced toggle. K4 overrides it immediately and clears it.
// A qualified release must precede every accepted K1 press, including at boot,
// after K4 and after a fault. No automatic enable from a held key.
module enable_latch #(parameter integer DEBOUNCE_CYCLES=1000000)(
    input wire clk,rst_n,k1_n,k4_n,inhibit,
    output wire enable_request
);
    reg sync1=0,sync2=0,candidate=0,stable_pressed=0;
    reg stable_valid=0,armed=0,latched=0;
    reg [31:0] debounce_count=0;
    assign enable_request=rst_n & k4_n & !inhibit & latched;
    always @(posedge clk) begin
        if(!rst_n || !k4_n || inhibit) begin
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
                else if(armed) begin latched<=~latched; armed<=0; end
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
module vision_follow_control #(parameter integer MS_CYCLES=50000,CONTROL_MS=100,FOLLOW_DEADBAND=1)(
 input wire clk,rst_n,key_event,k4_n,enable_request,config_changed,
 input wire visual_valid,follow_valid,visual_stale,settings_ready,center_confirmed,frame,
 input wire [10:0] x,input wire [15:0] center,deadzone,runtime_seconds,
 input wire [7:0] gain,limit,mode,damping,slew,
 input wire signed [7:0] manual_target,
 input wire core_ready,core_busy,input wire [3:0] fault,input wire step,
 input wire signed [31:0] position,
 output reg request=0,output wire following,external_cancel,
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
 reg [31:0] ms_count=0;reg [15:0] second_ms=0,control_ms=0;
 wire signed [31:0] error_pixels=$signed({21'd0,x})-$signed({16'd0,center});
 wire signed [31:0] raw_delta=$signed({21'd0,x})-$signed({21'd0,previous_x});
 wire signed [31:0] delta=raw_delta>256 ? 32'sd256 : (raw_delta< -256 ? -32'sd256 : raw_delta);
 // VE is signed Q4 pixels/frame, preserving slow motion for damping.
 wire signed [31:0] derivative=($signed(velocity)*$signed({1'b0,damping}))/16;
 wire signed [31:0] predicted=error_pixels+derivative;
 wire [31:0] magnitude=predicted<0 ? -predicted : predicted;
 wire [31:0] outside=magnitude>deadzone ? magnitude-deadzone : 32'd0;
 wire [31:0] scaled=outside >> gain[6:0];
 wire [7:0] bounded=scaled>limit ? limit : scaled[7:0];
 wire negative=(predicted<0)^gain[7];
 wire signed [7:0] pd_target=negative ? -$signed(bounded) : $signed(bounded);
 wire coast=run && mode==1 && !visual_valid;
 wire permit=(run && mode==1 ? follow_valid : visual_valid) && !visual_stale && settings_ready && !config_changed;
 assign candidate=!permit ? 8'sd0 : (mode==1 ? (coast ? held_candidate : pd_target) : (mode==2 ? 8'sd0 : manual_target));
 wire expired=run && runtime_seconds!=0 && elapsed_seconds>=runtime_seconds;
 wire stop_button=run && key_event;
 assign following=run && mode==1;
 assign external_cancel=stop_button || expired;
 assign external_cancel_reason=expired ? 4'd8 : 4'd7;
 wire signed [31:0] drive_error=$signed(drive_target)-position;
 wire signed [31:0] target_delta=$signed(pd_target)-$signed(drive_target);
 wire signed [31:0] slew_wide=$signed({1'b0,slew});
 wire tick_ms=ms_count==MS_CYCLES-1;
 always @(posedge clk)begin
  if(!rst_n)begin
   request<=0;run<=0;drive_target<=0;velocity<=0;history<=0;pending_frame<=0;
   previous_x<=0;held_candidate<=0;request_wait<=0;stop_reason<=0;reject_reason<=0;elapsed_seconds<=0;
   ms_count<=0;second_ms<=0;control_ms<=0;total_pulses<=0;step_previous<=0;
  end else begin
   request<=0;step_previous<=step;
   if(visual_valid && !visual_stale && settings_ready && !config_changed)held_candidate<=pd_target;
   if(step && !step_previous)total_pulses<=total_pulses+1'b1;
   if(core_busy || !core_ready)request_wait<=0;
   if(!run || !permit)begin history<=0;velocity<=0;pending_frame<=0;ms_count<=0;control_ms<=0;second_ms<=0;end
   else begin
    // Loss freezes the reference, discards queued updates and restarts velocity
    // history. It does not pause the runtime or the executor burst guards.
    if(!visual_valid)begin history<=0;velocity<=0;pending_frame<=0;end
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
      if(pending_frame && visual_valid)begin
       pending_frame<=0;
       if(target_delta>slew_wide)drive_target<=$signed(drive_target)+slew;
       else if(target_delta< -slew_wide)drive_target<=$signed(drive_target)-slew;
       else drive_target<=pd_target;
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
    else if(config_changed || !settings_ready)reject_reason<=5;
    else if(visual_stale)reject_reason<=4;
    else if(!visual_valid)reject_reason<=3;
    else if(mode==1 && !center_confirmed)reject_reason<=10;
    else if(candidate>60 || candidate< -60)reject_reason<=7;
    else begin
     reject_reason<=0;stop_reason<=0;
     if(mode==1)begin
      run<=1;elapsed_seconds<=0;total_pulses<=0;history<=0;pending_frame<=0;velocity<=0;
      drive_target<=position>60 ? 8'sd60 : (position< -60 ? -8'sd60 : position[7:0]);
     end else begin request<=1;request_wait<=1;drive_target<=candidate;end
    end
   end else if(run && permit && core_ready && !request_wait && (drive_error>FOLLOW_DEADBAND || drive_error< -FOLLOW_DEADBAND))begin
    request<=1;request_wait<=1;
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
module vision_config_rx #(parameter integer UART_DIV=434,PACKET_TIMEOUT_CYCLES=5000000,DEFAULT_GAIN=4,DEFAULT_LIMIT=12,DEFAULT_MODE=1)(
    input wire clk,rst_n,rx,
    output reg [15:0] cfg_sequence=0,center=960,deadzone=20,
    output reg [7:0] gain=DEFAULT_GAIN,limit=DEFAULT_LIMIT,
    output reg [15:0] errors=0,
    output reg applied=0,
    output reg [15:0] cam_sequence=0,exposure=3600,sensor_gain=143,
    output reg [15:0] roi_left=0,roi_top=0,roi_right=1279,roi_bottom=719,
    input wire center_capture,input wire [15:0] captured_center,
    output reg center_applied=0,center_conflict=0,parameter_applied=0,
    output reg [15:0] follow_sequence=0,runtime_seconds=30,
    output reg [7:0] operating_mode=DEFAULT_MODE,damping=12,slew=2
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
            cfg_sequence<=0;center<=960;deadzone<=20;gain<=DEFAULT_GAIN;limit<=DEFAULT_LIMIT;errors<=0;applied<=0;
            cam_sequence<=0;exposure<=3600;sensor_gain<=143;
            follow_sequence<=0;runtime_seconds<=30;operating_mode<=DEFAULT_MODE;damping<=12;slew<=2;control_packet<=0;
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
module vision_report_tx #(parameter integer UART_DIV=434,REPORT_CYCLES=12500000)(
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
    input wire signed [7:0] drive_target,
    input wire [3:0] stop_reason,auto_reject,
    input wire [31:0] total_pulses,
    input wire [7:0] gain,limit,
    input wire signed [7:0] target,
    input wire signed [31:0] count1,count2,position,
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
    reg [7:0] target_snapshot=0;
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
    reg [7:0] drive_target_snapshot=0;
    reg [3:0] stop_reason_snapshot=0;
    reg [3:0] auto_reject_snapshot=0;
    reg [15:0] elapsed_seconds_snapshot=0;
    reg [31:0] total_pulses_snapshot=0;
    reg [31:0] timer=0;
    reg frame_active=0,bit_active=0;
    reg [8:0] char_index=0;
    reg [3:0] bit_index=0;
    reg [15:0] baud_count=0;
    reg [9:0] serial_shift=10'h3ff;
    assign tx=serial_shift[0];
    function [7:0] hex_char;
        input [3:0] value;
        begin if(value<10)hex_char=8'h30+value;else hex_char=8'h37+value;end
    endfunction
    function [7:0] message_byte;
        input [8:0] index;
        begin case(index)
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
            timer<=0;frame_active<=0;bit_active<=0;char_index<=0;bit_index<=0;
            baud_count<=0;serial_shift<=10'h3ff;
        end else begin
            if(timer==REPORT_CYCLES-1)timer<=0;else timer<=timer+1'b1;
            if(!frame_active && timer==REPORT_CYCLES-1)begin
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
                        if(char_index==428)frame_active<=0;else char_index<=char_index+1'b1;
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
    SETUP_CYCLES=1000000,HALF_CYCLES=125000,SETTLE_CYCLES=2500000,TIMEOUT_CYCLES=250000000,DEFAULT_MODE=1,DEFAULT_GAIN=4,DEFAULT_LIMIT=12,CONTROL_MS=100,LOSS_GRACE_MS=500
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
    vision_config_rx #(.UART_DIV(UART_DIV),.PACKET_TIMEOUT_CYCLES(PACKET_TIMEOUT_CYCLES),.DEFAULT_MODE(DEFAULT_MODE),.DEFAULT_GAIN(DEFAULT_GAIN),.DEFAULT_LIMIT(DEFAULT_LIMIT)) config_rx(
        clk,rst_n,rx,cfg_sequence,center,deadzone,gain,limit,rx_errors,applied,
        cam_sequence,exposure,sensor_gain,roi_left,roi_top,roi_right,roi_bottom,
        center_request,{5'b0,x},center_applied,center_conflict,parameter_applied,follow_sequence,runtime_seconds,operating_mode,damping,slew);
    wire valid;
    wire [15:0] age,frames;
    wire signed [7:0] manual_target,target,drive_target;
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
    wire auto_run,request,following,external_cancel,key2_event;
    wire [3:0] external_cancel_reason;
    wire signed [15:0] velocity;
    wire [15:0] elapsed_seconds;
    wire [31:0] total_pulses;
    assign reject_reason=supervisor_reject!=0 ? supervisor_reject : core_reject;
    wire enable_request,origin_valid,ready,busy,reached;
    wire [7:0] spent;
    wire [15:0] accepted_count,completed_count;
    wire signed [31:0] command_position,locked_target,position,move_error;
    vision_center_key #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) center_key(clk,rst_n,key_n[2],center_event);
    wire hold_valid;
    vision_result_guard #(.MS_CYCLES(MS_CYCLES),.LOSS_GRACE_MS(LOSS_GRACE_MS)) loss_guard(
        .clk(clk),.rst_n(rst_n),.clear(applied || center_request || settings_busy),
        .frame(result_frame),.valid(valid),.hold_valid(hold_valid));
    wire incoming_bad=frame && (!confirmed || bad_frame || confidence<8 || ball_x>1919 || ball_y>1079);
    wire executor_valid=(auto_run && operating_mode==1) ? hold_valid : (valid && !incoming_bad);
    assign center_request=center_event && key_n[3] && fault==0 && !busy && !auto_run && settings_ready &&
        !applied && valid && age<STALE_MS && !incoming_bad;
    always @(posedge clk)begin
        if(!rst_n)begin center_count<=0;center_status<=0;end
        else if(center_applied)begin center_count<=center_count+1'b1;center_status<=1;end
        else if(center_conflict)center_status<=7;
        else if(center_event)begin
            if(!key_n[3])center_status<=6;
            else if(fault!=0)center_status<=5;
            else if(busy || auto_run)center_status<=4;
            else if(!settings_ready || applied)center_status<=3;
            else if(!valid || age>=STALE_MS || incoming_bad)center_status<=2;
        end
    end
    enable_latch #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) enable_control(
        clk,rst_n,key_n[0],key_n[3],fault!=0,enable_request);
    vision_center_key #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) key2(clk,rst_n,key_n[1],key2_event);
    vision_follow_control #(.MS_CYCLES(MS_CYCLES),.CONTROL_MS(CONTROL_MS)) supervisor(
        .clk(clk),.rst_n(rst_n),.key_event(key2_event),.k4_n(key_n[3]),.enable_request(enable_request),
        .config_changed(applied || center_request),.visual_valid(valid && !incoming_bad),.follow_valid(hold_valid),.visual_stale(age>=STALE_MS),
        .settings_ready(settings_ready),.center_confirmed(center_confirmed),.frame(result_frame),
        .x(x),.center(center),.deadzone(deadzone),.runtime_seconds(runtime_seconds),.gain(gain),.limit(limit),
        .mode(operating_mode),.damping(damping),.slew(slew),.manual_target(manual_target),
        .core_ready(ready),.core_busy(busy),.fault(fault),.step(motor_step),.position(position),
        .request(request),.following(following),.external_cancel(external_cancel),.external_cancel_reason(external_cancel_reason),
        .run(auto_run),.stop_reason(stop_reason),.reject_reason(supervisor_reject),.drive_target(drive_target),
        .candidate(target),.velocity(velocity),.elapsed_seconds(elapsed_seconds),.total_pulses(total_pulses));
    vision_servo_core #(.EXTERNAL_CONTROL(1),.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES),.ENABLE_CYCLES(ENABLE_CYCLES),
        .SETUP_CYCLES(SETUP_CYCLES),.HALF_CYCLES(HALF_CYCLES),.SETTLE_CYCLES(SETTLE_CYCLES),
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)) motor_core(
        .clk(clk),.rst_n(rst_n),.enable_request(enable_request),.k4_n(key_n[3]),.k2_n(1'b1),
        .request(request),.following(following),.external_cancel(external_cancel),.external_cancel_reason(external_cancel_reason),
        .encoder_count(count1),.encoder_errors(errors1),.visual_valid(executor_valid),.visual_stale(age>=STALE_MS),
        .settings_ready(settings_ready),.config_changed(applied || center_request),.visual_target(drive_target),
        .motor_en(motor_en),.motor_dir(motor_dir),.motor_step(motor_step),
        .command_position(command_position),.locked_target(locked_target),.position(position),.error(move_error),
        .state(state),.fault(fault),.cancel_reason(cancel_reason),.reject_reason(core_reject),.spent(spent),
        .origin_valid(origin_valid),.ready(ready),.busy(busy),.reached(reached),
        .accepted_count(accepted_count),.completed_count(completed_count));
    wire request_ready=(auto_run || (ready && valid && !incoming_bad && age<STALE_MS)) && settings_ready && !applied && !center_request && fault==0 && (operating_mode!=1 || center_confirmed);
    assign led={fault!=0,request_ready,(busy || auto_run),motor_en};
    vision_report_tx #(.UART_DIV(UART_DIV),.REPORT_CYCLES(REPORT_CYCLES)) report_tx(
        .clk(clk),.rst_n(rst_n),.x(x),.y(y),.valid(valid),.age(age),.frames(frames),
        .enabled(motor_en),.origin_valid(origin_valid),.fault(fault),.count1(count1),.count2(count2),
        .errors1(errors1),.errors2(errors2),.position(position),.target(target),
        .center(center),.deadzone(deadzone),.gain(gain),.limit(limit),.cfg_sequence(cfg_sequence),.rx_errors(rx_errors),
        .quality(frame_confidence),.bad(frame_bad),.cam_sequence(cam_sequence),.exposure(exposure),.sensor_gain(sensor_gain),
        .roi_left(roi_left),.roi_top(roi_top),.roi_right(roi_right),.roi_bottom(roi_bottom),
        .camera_applied_seq(camera_applied_seq),.roi_applied_seq(roi_applied_seq),.settings_busy(settings_busy),.locked_target(locked_target),.move_error(move_error),.move_state(state),
        .move_busy(busy),.reached(reached),.request_ready(request_ready),.spent(spent),.command_position(command_position),
        .cancel_reason(cancel_reason),.reject_reason(reject_reason),.accepted_count(accepted_count),.completed_count(completed_count),
        .center_count(center_count),.center_status(center_status),
        .operating_mode(operating_mode),.auto_run(auto_run),.damping(damping),.slew(slew),.runtime_seconds(runtime_seconds),
        .follow_sequence(follow_sequence),.center_confirmed(center_confirmed),.velocity(velocity),.drive_target(drive_target),
        .stop_reason(stop_reason),.auto_reject(supervisor_reject),.elapsed_seconds(elapsed_seconds),.total_pulses(total_pulses),.tx(tx));
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

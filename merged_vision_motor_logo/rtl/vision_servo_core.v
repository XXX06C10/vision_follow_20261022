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

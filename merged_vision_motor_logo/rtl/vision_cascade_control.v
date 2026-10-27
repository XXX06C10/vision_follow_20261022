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

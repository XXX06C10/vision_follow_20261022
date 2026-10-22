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

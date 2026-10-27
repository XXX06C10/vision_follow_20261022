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

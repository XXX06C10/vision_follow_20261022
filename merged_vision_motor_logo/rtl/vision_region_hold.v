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

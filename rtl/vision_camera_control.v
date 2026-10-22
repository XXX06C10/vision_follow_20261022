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

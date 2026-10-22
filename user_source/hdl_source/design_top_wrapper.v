

module design_top_wrapper (
    input wire        I_sys_clk,
    input wire        I_rst_n,
    output wire [3:0] LED,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    input wire [3:0]  I_button,

    output wire       O_screen_pwm,
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,
    output wire       O_uart_tx,
    input wire        I_uart_rx,
    output wire       O_motor_step,
    output wire       O_motor_dir,
    output wire       O_motor_en,
    output wire       O_motor_step_v2,O_motor_dir_v1,O_motor_en_u2,
    input wire [1:0] I_encoder_j1,I_encoder_j2,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n   
);
    wire S_visual_confirmed,S_visual_heartbeat,S_visual_bright,S_visual_error;



    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
	wire        	S_pll_hdmi_lock;
    wire        	S_rst_n;
    wire S_detect_rst_n = I_rst_n & S_rst_n;
    wire S_roi_change;
    wire [10:0] S_roi_left,S_roi_top,S_roi_right,S_roi_bottom;
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    wire         S_video_rd_valid;
    wire         S_scaler_rd_en;
    wire [23:0]  S_scaled_pixel;
    reg          S_hdmi_vsync_d, S_hdmi_hsync_d, S_hdmi_de_d;
    reg          S_hdmi_user_d, S_hdmi_last_d;

    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;


    assign O_screen_pwm = 1'b1;
    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_hdmi_lock;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    assign S_hdmi_debug_status = {
        (|S_ddr_user_rd_data),
        S_ddr_user_rd_valid,
        S_vo_ddr_rd_en,
        S_vi_ddr_wr_en
    };
	
	assign O_cam_rst 	= 1'b1;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    (                 ),
        .clk5_out    (                 ),

        .lock        ( S_pll_lock        )

    );

    pll_hdmi u_pll_hdmi_1080p(
        .refclk      ( I_sys_clk          ),
        .clk0_out    ( S_hdmi_pixel_clk   ),
        .clk1_out    ( S_hdmi_serial_clk  ),
        .clk2_out    (                    ),
        .clk3_out    (                    ),
        .lock        ( S_pll_hdmi_lock    )
    );


	

  
    wire [15:0] S_cam_sequence,S_exposure,S_sensor_gain;
    wire [15:0] S_cfg_left,S_cfg_top,S_cfg_right,S_cfg_bottom;
    wire [15:0] S_cam_applied_seq,S_roi_applied_seq;
    wire S_settings_busy;
    vision_camera_control u_camera_control(
        .sys_clk(I_sys_clk),.sys_rst_n(S_detect_rst_n),
        .cam_clk(S_24m_clk),.cam_rst_n(S_detect_rst_n),
        .roi_clk(S_csi_rx_clk),.roi_rst_n(S_detect_rst_n),.frame_start(S_ISP_O_tuser),
        .cam_cfg_done(S_cam_cfg_done),.ae_cfg_done(S_ae_cfg_done),
        .requested_seq(S_cam_sequence),.exposure(S_exposure),.sensor_gain(S_sensor_gain),
        .roi_left(S_cfg_left),.roi_top(S_cfg_top),.roi_right(S_cfg_right),.roi_bottom(S_cfg_bottom),
        .ae_req(S_ae_req),.ae(S_ae),.ag(S_ag),
        .left(S_roi_left),.top(S_roi_top),.right(S_roi_right),.bottom(S_roi_bottom),.roi_change(S_roi_change),
        .camera_applied_seq(S_cam_applied_seq),.roi_applied_seq(S_roi_applied_seq),.settings_busy(S_settings_busy));

    uicfgcs500 #(
      .CLK_DIV(24000000 / 100000 - 1)
  ) u_uicfgcs500 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_rst_n),  //系统复位输入
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完�?
      .O_ae_cfg_done(S_ae_cfg_done)  //AE配置完成
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "0dB"   ),//"0dB","1.5dB","3dB","4.5dB","6dB","7.5dB","9dB","10.5dB"
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream�?
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

image_correction #(
    .DATA_WIDTH(40)
)(
    .I_clk   		(S_csi_rx_clk),
    .I_rst_n 		(S_rst_n),
					
    .I_raw_data 	(S_axis_tdata),
    .I_raw_frame_end 	(S_axis_tlast),
    .I_raw_valid	(S_axis_tvalid),
    .I_raw_frame_start 	(S_axis_tuser),
					
    .O_raw_tdata 	(S_raw_tdata ),
    .O_raw_tlast 	(S_raw_tlast ),
    .O_raw_tvalid	(S_raw_tvalid),
    .O_raw_tuser 	(S_raw_tuser )
);


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


//ISP算法顶层模块
isp_top u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.O_tready        (S_ISP_O_tready),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.I_tready        ()
  );


    // Detector only mirrors the ISP bus; zero-latency wires feed video_in.
    wire [127:0] S_detect_tdata;
    wire S_detect_tvalid, S_detect_tuser, S_detect_tlast;
    wire S_detect_done, S_detect_hit, S_detect_fault;
    wire [21:0] S_detect_red_pixels, S_detect_largest_area;
    wire [23:0] S_detect_largest_perimeter;
    wire [10:0] S_detect_xmin, S_detect_xmax, S_detect_ymin, S_detect_ymax;
    wire [3:0] S_detect_geometry;
    wire [16:0] S_detect_fifo_peak;
    wire [11:0] S_detect_lines;
    wire [8:0] S_detect_labels;
    wire S_detect_bad_frame, S_detect_label_exhausted, S_detect_box_valid;
    wire [1:0] S_detect_error;
    wire [5:0] S_detect_mask;
    wire [4:0] S_detect_meta;
    wire S_motor_cmd_valid, S_motor_busy, S_motor_zero_valid;
    wire [2:0] S_motor_cmd;
    wire signed [6:0] S_motor_position;
    wire [7:0] S_motor_command_count;
    wire [10:0] S_fast_box_left, S_fast_box_right, S_fast_box_top, S_fast_box_bottom;
    wire [10:0] S_ball_x, S_ball_y;
    wire S_track_valid;
    wire [10:0] S_track_x,S_track_y;
    wire [10:0] S_diag_xmin = (S_detect_xmin * 3) >> 1;
    wire [10:0] S_diag_xmax = (S_detect_xmax * 3) >> 1;
    wire [10:0] S_diag_ymin = (S_detect_ymin * 3) >> 1;
    wire [10:0] S_diag_ymax = (S_detect_ymax * 3) >> 1;
    wire [10:0] S_diag_ball_x = (S_ball_x * 3) >> 1;
    wire [10:0] S_diag_ball_y = (S_ball_y * 3) >> 1;
    wire [3:0] S_ball_confidence;
    wire [2:0] S_ball_clusters;
    assign S_detect_fault = 0;
    assign S_detect_fifo_peak = 0;
    assign S_detect_label_exhausted = 0;
    assign S_detect_labels = {6'd0, S_ball_clusters};
    reg [1:0] S_detect_reset_sync;
    always @(posedge S_csi_rx_clk or negedge S_detect_rst_n) begin
        if(!S_detect_rst_n) S_detect_reset_sync <= 2'b00;
        else S_detect_reset_sync <= {S_detect_reset_sync[0],1'b1};
    end
    assign S_motor_busy=0,S_motor_zero_valid=0,S_motor_position=0,S_motor_command_count=0;
    steel_ball_detect #(
        .IMG_WIDTH(1280), .IMG_HEIGHT(720),
        .MAX_AREA(200000), .LABEL_BITS(8), .FIFO_BITS(16), .FILL_MIN_PERCENT(50),
        .ASPECT_PERCENT(65), .CIRCULARITY_SCALE(48),
        .TRACK_EN(1), .TRACK_RADIUS(48), .TRACK_COLOR_MIN(35),
        .COLOR_MODE(1), .COLOR_MIN(45), .COLOR_RATIO_LOG2(1), .COLOR_HIGHLIGHT_FILL(0),
        .OVERLAY_EN(1), .OVERLAY_BOX(1), .EXTERNAL_BOX(1), .BOX_COLOR(24'h00FF00),
        .BOX_MARGIN(3), .BOX_THICKNESS(2), .ROI_OVERLAY(1)
    ) u_steel_ball_detect (
        .I_clk(S_csi_rx_clk), .I_rst_n(S_detect_reset_sync[1]),
        .S_AXIS_tdata(S_ISP_O_tdata), .S_AXIS_tvalid(S_ISP_O_tvalid),
        .S_AXIS_tuser(S_ISP_O_tuser), .S_AXIS_tlast(S_ISP_O_tlast),
        .I_track_valid(S_track_valid),.I_track_x(S_track_x),.I_track_y(S_track_y),
        .I_box_valid(S_detect_box_valid),
        .I_box_left(S_fast_box_left), .I_box_right(S_fast_box_right),
        .I_box_top(S_fast_box_top), .I_box_bottom(S_fast_box_bottom),
        .I_box_confirmed(S_visual_confirmed),
        .I_roi_left(S_roi_left), .I_roi_top(S_roi_top),
        .I_roi_right(S_roi_right), .I_roi_bottom(S_roi_bottom),
        .O_color_mask(S_detect_mask), .O_color_meta(S_detect_meta),
        .M_AXIS_tdata(S_detect_tdata), .M_AXIS_tvalid(S_detect_tvalid),
        .M_AXIS_tuser(S_detect_tuser), .M_AXIS_tlast(S_detect_tlast),
        .O_ball_detected(), .O_frame_done(),
        .O_frame_hit(), .O_fault(),
        .O_bright(), .O_error_code(),
        .O_diag_red_pixels(),
        .O_diag_largest_area(),
        .O_diag_largest_perimeter(),
        .O_diag_xmin(), .O_diag_xmax(),
        .O_diag_ymin(), .O_diag_ymax(),
        .O_diag_geometry(),
        .O_diag_fifo_peak(),
        .O_diag_lines(), .O_diag_labels(),
        .O_diag_bad_frame(),
        .O_diag_label_exhausted(),
        .O_diag_box_valid()
    );
    steel_ball_stream_bbox #(
        .IMG_WIDTH(1280), .IMG_HEIGHT(720),
        .MIN_AREA(8), .BRIGHT_MIN(20), .MAX_AREA(90000),
        .CONFIRM_FRAMES(2),
        .FILL_MIN_PERCENT(15), .FILL_MAX_PERCENT(100),
        .DOMINANCE_PERCENT(40), .ASPECT_PERCENT(35),
        .CLUSTER_RADIUS(8), .MAX_JUMP(147), .EDGE_MARGIN(2), .MAX_DIM(267),
        .MARK_HALF(9), .MARK_AUTO(1), .MARK_MIN_HALF(43),
        .MARK_MAX_HALF(128), .MARK_MARGIN(5), .ROI_EN(1)
    ) u_steel_ball_stream_bbox (
        .I_clk(S_csi_rx_clk), .I_rst_n(S_detect_reset_sync[1] & ~S_roi_change),
        .I_mask(S_detect_mask), .I_meta(S_detect_meta),
        .I_roi_left(S_roi_left), .I_roi_top(S_roi_top),
        .I_roi_right(S_roi_right), .I_roi_bottom(S_roi_bottom),
        .O_frame_done(S_detect_done), .O_frame_hit(S_detect_hit),
        .O_ball_detected(S_visual_confirmed), .O_bright(S_visual_bright),
        .O_box_valid(S_detect_box_valid),
        .O_box_left(S_fast_box_left), .O_box_right(S_fast_box_right),
        .O_box_top(S_fast_box_top), .O_box_bottom(S_fast_box_bottom),
        .O_red_pixels(S_detect_red_pixels),
        .O_area(S_detect_largest_area),
        .O_perimeter(S_detect_largest_perimeter),
        .O_xmin(S_detect_xmin), .O_xmax(S_detect_xmax),
        .O_ymin(S_detect_ymin), .O_ymax(S_detect_ymax),
        .O_geometry(S_detect_geometry),
        .O_lines(S_detect_lines),
        .O_bad_frame(S_detect_bad_frame),
        .O_error_code(S_detect_error),
        .O_ball_x(S_ball_x), .O_ball_y(S_ball_y),
        .O_confidence(S_ball_confidence),
        .O_track_valid(S_track_valid),.O_track_x(S_track_x),.O_track_y(S_track_y),
        .O_cluster_count(S_ball_clusters)
    );
    // Legacy diagnostic counters are not consumed by this milestone.
    wire [21:0] S_uart_word_count=0;
    wire [11:0] S_uart_line_count=0;
    wire [15:0] S_uart_done_count=0;
    wire [240:0] S_uart_diag_live = {
        S_detect_red_pixels, S_detect_largest_area, S_detect_largest_perimeter,
        S_diag_xmin, S_diag_xmax, S_diag_ymin, S_diag_ymax,
        S_detect_geometry, S_detect_labels, S_detect_fifo_peak,
        S_visual_bright, S_detect_hit, S_detect_box_valid, S_visual_confirmed,
        S_detect_bad_frame, S_detect_label_exhausted, S_detect_fault, S_detect_error,
        S_uart_word_count, S_uart_line_count, S_uart_done_count, S_detect_lines,
        S_diag_ball_x, S_diag_ball_y, S_detect_box_valid, S_visual_confirmed, S_ball_confidence
    };
    reg [1:0] S_uart_reset_sync;
    wire S_uart_frame_pulse;
    reg S_uart_motion_pulse;
    wire [240:0] S_uart_diag_received;
    wire [21:0] S_uart_red, S_uart_area;
    wire [23:0] S_uart_perimeter;
    wire [10:0] S_uart_xmin, S_uart_xmax, S_uart_ymin, S_uart_ymax;
    wire [3:0] S_uart_geometry;
    wire [8:0] S_uart_labels;
    wire [16:0] S_uart_fifo_peak;
    wire S_uart_bright, S_uart_hit, S_uart_box, S_uart_ball;
    wire S_uart_bad, S_uart_labels_full, S_uart_fault;
    wire [1:0] S_uart_error;
    wire [21:0] S_uart_input_words;
    wire [11:0] S_uart_input_lines, S_uart_detect_lines;
    wire [15:0] S_uart_detect_frames;
    wire [10:0] S_uart_ball_x, S_uart_ball_y;
    wire S_uart_candidate, S_uart_confirmed;
    wire [3:0] S_uart_confidence;
    wire [1:0] S_motion_valid;
    wire [10:0] S_position_tenth_mm;
    wire signed [15:0] S_speed_mm_s;
    assign {
        S_uart_red, S_uart_area, S_uart_perimeter,
        S_uart_xmin, S_uart_xmax, S_uart_ymin, S_uart_ymax,
        S_uart_geometry, S_uart_labels, S_uart_fifo_peak,
        S_uart_bright, S_uart_hit, S_uart_box, S_uart_ball,
        S_uart_bad, S_uart_labels_full, S_uart_fault, S_uart_error,
        S_uart_input_words, S_uart_input_lines, S_uart_detect_frames, S_uart_detect_lines,
        S_uart_ball_x, S_uart_ball_y, S_uart_candidate, S_uart_confirmed, S_uart_confidence
    } = S_uart_diag_received;
    vision_cdc_bridge #(.WIDTH(241)) u_vision_cdc(
        .src_clk(S_csi_rx_clk),.src_rst_n(S_detect_reset_sync[1]),.src_frame(S_detect_done),
        .src_data(S_uart_diag_live),.dst_clk(I_sys_clk),.dst_rst_n(S_uart_reset_sync[1]),
        .dst_frame(S_uart_frame_pulse),.dst_data(S_uart_diag_received));
    always @(posedge I_sys_clk or negedge S_detect_rst_n) begin
        if(!S_detect_rst_n) S_uart_reset_sync <= 0;
        else S_uart_reset_sync <= {S_uart_reset_sync[0], 1'b1};
    end
    always @(posedge I_sys_clk or negedge S_uart_reset_sync[1]) begin
        if(!S_uart_reset_sync[1]) S_uart_motion_pulse<=0;
        else S_uart_motion_pulse<=S_uart_frame_pulse;
    end
    assign S_motion_valid=0,S_position_tenth_mm=0,S_speed_mm_s=0;
    vision_monitor u_vision_monitor(
        .clk(I_sys_clk),.external_rst_n(S_uart_reset_sync[1]),.key_n(I_button),
        .encoder_j1(I_encoder_j1),.encoder_j2(I_encoder_j2),.frame(S_uart_motion_pulse),
        .ball_x(S_uart_ball_x),.ball_y(S_uart_ball_y),.confirmed(S_uart_confirmed),
        .bad_frame(S_uart_bad | S_uart_labels_full | S_uart_fault | (S_uart_error!=0)),
        .confidence(S_uart_confidence),.rx(I_uart_rx),.tx(O_uart_tx),
        .motor_en(O_motor_en),.motor_dir(O_motor_dir),.motor_step(O_motor_step),.led(LED),
        .cam_sequence(S_cam_sequence),.exposure(S_exposure),.sensor_gain(S_sensor_gain),
        .roi_left(S_cfg_left),.roi_top(S_cfg_top),.roi_right(S_cfg_right),.roi_bottom(S_cfg_bottom),
        .camera_applied_seq(S_cam_applied_seq),.roi_applied_seq(S_roi_applied_seq),.settings_busy(S_settings_busy));
    assign O_motor_step_v2=O_motor_step;
    assign O_motor_dir_v1=O_motor_dir;
    assign O_motor_en_u2=O_motor_en;
    steel_ball_status_led u_detect_status (
        .I_clk(S_csi_rx_clk), .I_rst_n(S_detect_reset_sync[1]),
        .I_sof(S_ISP_O_tuser), .I_error_code(S_detect_error),
        .O_heartbeat(S_visual_heartbeat), .O_error_led(S_visual_error)
    );

    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_detect_tuser  ),
        .I_camera_valid       ( S_detect_tvalid ),
        .I_camera_data        ( S_detect_tdata  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_scaler_rd_en      ),
        .O_vdieo_data        ( S_video_rd_data      ),
        .O_vdieo_valid       ( S_video_rd_valid     )
    );



    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1920 ),
        .H_FrameSize  ( 2200 ),
        .H_SyncStart  ( 2008 ),
        .H_SyncEnd    ( 2052 ),
        .V_ActiveSize ( 1080  ),
        .V_FrameSize  ( 1125  ),
        .V_SyncStart  ( 1084  ),
        .V_SyncEnd    ( 1089  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    wide_view_scaler u_wide_view_scaler (
        .I_clk(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n),
        .I_frame_sync(S_video_out_vsync), .I_video_de(S_hdmi_de),
        .I_source_data(S_video_rd_data), .I_source_valid(S_video_rd_valid),
        .O_source_rd_en(S_scaler_rd_en), .O_pixel(S_scaled_pixel)
    );
    always @(posedge S_hdmi_pixel_clk) begin
        S_hdmi_vsync_d <= S_hdmi_vsync;
        S_hdmi_hsync_d <= S_hdmi_hsync;
        S_hdmi_de_d <= S_hdmi_de;
        S_hdmi_user_d <= S_hdmi_user;
        S_hdmi_last_d <= S_hdmi_last;
    end

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1920 ),
        .IMG_HEIGHT ( 1080  ),
        .DEBUG_MODE ( 0    )
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync_d     ),
        .I_video_hsync   ( S_hdmi_hsync_d     ),
        .I_video_de      ( S_hdmi_de_d        ),
        .I_video_user    ( S_hdmi_user_d      ),
        .I_video_last    ( S_hdmi_last_d      ),
        .I_debug_status  ( S_hdmi_debug_status),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .I_video_rd_data ( S_scaled_pixel     ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    )
    );

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_out_vsync  ),
        .I_video_in_de      ( S_hdmi_out_de     ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_hdmi_out_data   ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );


    
endmodule

// ===== 1080P HDMI ʱ�� PLL���� pll_hdmi.v ������=====
/************************************************************\
**	Copyright (c) 2012-2025 Anlogic Inc.
**	All Right Reserved.
\************************************************************/
/************************************************************\
**	Build time: Jun 14 2025 22:20:07
**	TD version	:	6.1.155545
************************************************************/
///////////////////////////////////////////////////////////////////////////////
//	Input frequency:                50.000000000 MHz
//	Clock multiplication factor: 3
//	Clock division factor:       2
//	Clock information:
//		Clock name	| Frequency 	| Phase shift 
//		C0        	| 75.000000   MHz	    |  0.0000 DEG 
//		C1        	| 375.000000   MHz	    |  0.0000 DEG 
//		C2        	| 70.312500   MHz		|  0.0000 DEG 
//		C3        	| 23.936170   MHz		|  0.0000 DEG 
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ns / 100 fs 

module pll_hdmi
(
  input                         refclk,
  output                        clk0_out,
  output                        clk1_out,
  output                        clk2_out,
  output                        clk3_out,
  output                        lock
);
  wire							  clk0_buf;
PH1P_LOGIC_BUFG bufg_feedback (
 .i(clk0_buf), 
 .o(clk0_out) 
 ); 

 
  wire							  clk2_buf;
PH1P_LOGIC_BUFG bufg_c2 (
 .i(clk2_buf), 
 .o(clk2_out) 
 ); 

  wire							  clk3_buf;
PH1P_PHY_MLCLK mlclk_c3 (
 .ce(1'b1), 
 .clkin(clk3_buf), 
 .clkout(clk3_out) 
 ); 





  ph1p_phy_pll_wrapper_27c53fedd344
  #(
      .FBKCLK("CLKC0_EXT"),
      .FBKCLK_INT("CLKC0_EXT"),
      .FIN("50.000000000"),
      .REFCLK_DIV(2),
      .FBCLK_DIV(3),
      .CLKC0_ENABLE("ENABLE"),
      .CLKC0_DIV(15),
      .CLKC0_CPHASE(14),
      .CLKC0_FPHASE(0),
      .CLKC0_FPHASE_RSTSEL(0),
      .CLKC0_DUTY50("ENABLE"),
      .CLKC0_DUTY_INT(8),
      .CLKC1_ENABLE("ENABLE"),
      .CLKC1_DIV(3),
      .CLKC1_CPHASE(2),
      .CLKC1_FPHASE(0),
      .CLKC1_FPHASE_RSTSEL(0),
      .CLKC1_DUTY50("ENABLE"),
      .CLKC1_DUTY_INT(2),
      .CLKC2_ENABLE("ENABLE"),
      .CLKC2_DIV(16),
      .CLKC2_CPHASE(15),
      .CLKC2_FPHASE(0),
      .CLKC2_FPHASE_RSTSEL(0),
      .CLKC2_DUTY50("ENABLE"),
      .CLKC2_DUTY_INT(8),
      .CLKC3_ENABLE("ENABLE"),
      .CLKC3_DIV(47),
      .CLKC3_CPHASE(46),
      .CLKC3_FPHASE(0),
      .CLKC3_FPHASE_RSTSEL(0),
      .CLKC3_DUTY50("ENABLE"),
      .CLKC3_DUTY_INT(24),
      .PLL_USR_RST("DISABLE"),
      .PLL_FEED_TYPE("EXTERNAL"),
      .PLL_FASTLOOP("ENABLE"),
      .LPF_RES(0),
      .LPF_CAP(25),
      .ICP_CUR(29),
      .PHASE_PATH_SEL(0),
      .DYN_PHASE_PATH_SEL("DISABLE"),
      .DYN_FPHASE_EN("DISABLE"),
      .PI_OUT_SEL("NORMAL"),
      .PI_FRAC_EN("DISABLE"),
      .CLKC0_PI_SHIFT_EN("DISABLE"),
      .FEEDBK_MODE("NORMAL"),
      .DYN_CPHASE_EN("DISABLE")
  )ph1p_phy_pll_wrapper_27c53fedd344_Inst
  (
      .clk4_en(1'b0),
      .clk4_out(),
      .clkb4_out(),
      .clk5_en(1'b0),
      .clk5_out(),
      .clkb5_out(),
      .clk6_en(1'b0),
      .clk6_out(),
      .clkb6_out(),
      .refclk(refclk),
      .drp_clk(1'b0),
      .drp_rstn(1'b1),
      .drp_sel(1'b0),
      .drp_rd(1'b0),
      .drp_wr(1'b0),
      .drp_addr(8'b00000000),
      .drp_wdata(8'b00000000),
      .drp_err(),
      .drp_rdy(),
      .drp_rdata(),
      .psclk(1'b0),
      .psclksel(3'b000),
      .psstep(1'b0),
      .psdown(1'b0),
      .cps_step(1'b0),
      .psdone(),
      .ssc_reset(1'b0),
	  .clkc_rst(2'b00),
      .pllpd(1'b0),
      .reset(1'b0),
      .fbclk(clk0_out),
      .wakeup(1'b1),
      .refclk_rst(1'b0),
      .clk0_en(1'b1),
      .clkb0_out(),
      .clk0_out(clk0_buf),
      .clk1_en(1'b1),
      .clkb1_out(),
      .clk1_out(clk1_out),
      .clk2_en(1'b1),
      .clkb2_out(),
      .clk2_out(clk2_buf),
      .clk3_en(1'b1),
      .clkb3_out(),
      .clk3_out(clk3_buf),
      .lock(lock)
  );
endmodule

// ===== 1080P PLL Ӳ�� wrapper������ 32 �Ź��̣�������� unknown module��=====

module ph1p_phy_pll_wrapper_27c53fedd344 #(
    parameter FBKCLK                 = "CLKC0_EXT",     
    parameter FBKCLK_INT             = "VCO_PHASE0",     
    parameter FIN                    = "100.0000",     	
    parameter REFCLK_DIV             = 1,                                   
    parameter FBCLK_DIV              = 1,               

    parameter CLKC0_ENABLE           = "DISABLE", 
    parameter PREDIV_MUXC0           = "VCO",                   
    parameter DIVOUT_MUXC0           = "DIV",	
    parameter CLKC0_DIV              = 1,  
    parameter CLKC0_CPHASE           = 0,  
    parameter CLKC0_FPHASE           = 0,
    parameter CLKC0_FPHASE_RSTSEL    = 0, 
    parameter CLKC0_DUTY50           = "ENABLE",              
    parameter CLKC0_DUTY_INT         = 1,    

    parameter CLKC1_ENABLE           = "DISABLE", 
    parameter PREDIV_MUXC1           = "VCO",                   
    parameter DIVOUT_MUXC1           = "DIV",	
    parameter CLKC1_DIV              = 1,  
    parameter CLKC1_CPHASE           = 0,  
    parameter CLKC1_FPHASE           = 0,
    parameter CLKC1_FPHASE_RSTSEL    = 0, 
    parameter CLKC1_DUTY50           = "ENABLE",              
    parameter CLKC1_DUTY_INT         = 1,    

    parameter CLKC2_ENABLE           = "DISABLE", 
    parameter PREDIV_MUXC2           = "VCO",                   
    parameter DIVOUT_MUXC2           = "DIV",	
    parameter CLKC2_DIV              = 1,  
    parameter CLKC2_CPHASE           = 0,  
    parameter CLKC2_FPHASE           = 0,
    parameter CLKC2_FPHASE_RSTSEL    = 0, 
    parameter CLKC2_DUTY50           = "ENABLE",              
    parameter CLKC2_DUTY_INT         = 1,    

    parameter CLKC3_ENABLE           = "DISABLE", 
    parameter PREDIV_MUXC3           = "VCO",                   
    parameter DIVOUT_MUXC3           = "DIV",	
    parameter CLKC3_DIV              = 1,  
    parameter CLKC3_CPHASE           = 0,  
    parameter CLKC3_FPHASE           = 0,
    parameter CLKC3_FPHASE_RSTSEL    = 0, 
    parameter CLKC3_DUTY50           = "ENABLE",              
    parameter CLKC3_DUTY_INT         = 1,    
	
    parameter CLKC4_ENABLE           = "DISABLE", 
    parameter PREDIV_MUXC4           = "VCO",                   
    parameter DIVOUT_MUXC4           = "DIV",	
    parameter CLKC4_DIV              = 1,  
    parameter CLKC4_CPHASE           = 0,  
    parameter CLKC4_FPHASE           = 0,
    parameter CLKC4_FPHASE_RSTSEL    = 0, 
    parameter CLKC4_DUTY50           = "ENABLE",              
    parameter CLKC4_DUTY_INT         = 1,    

    parameter CLKC5_ENABLE           = "DISABLE", 
    parameter PREDIV_MUXC5           = "VCO",                   
    parameter DIVOUT_MUXC5           = "DIV",	
    parameter CLKC5_DIV              = 1,  
    parameter CLKC5_CPHASE           = 0,  
    parameter CLKC5_FPHASE           = 0,
    parameter CLKC5_FPHASE_RSTSEL    = 0, 
    parameter CLKC5_DUTY50           = "ENABLE",              
    parameter CLKC5_DUTY_INT         = 1,    
	
    parameter CLKC6_ENABLE           = "DISABLE", 
    parameter PREDIV_MUXC6           = "VCO",                   
    parameter DIVOUT_MUXC6           = "DIV",	
    parameter CLKC6_DIV              = 1,  
    parameter CLKC6_CPHASE           = 0,  
    parameter CLKC6_FPHASE           = 0,
    parameter CLKC6_FPHASE_RSTSEL    = 0, 
    parameter CLKC6_DUTY50           = "ENABLE",              
    parameter CLKC6_DUTY_INT         = 1,  

    parameter WORK_MODE              = "USER",          

    parameter PD_DIG                 = "ENABLE",        
    parameter PLL_USR_RST            = "ENABLE",        
    parameter PLL_FEED_TYPE          = "INTERNAL",      
    parameter PLL_FASTLOOP           = "DISABLE",     

// loop     parameter
    parameter LPF_RES                = 3,               
    parameter LPF_CAP                = 3,               
    parameter ICP_CUR                = 1,               
//frac-N
    parameter FRAC_ENABLE            = "DISABLE",       
    parameter SDM_FRAC               = 0,                              

//fine phase shift
    parameter PHASE_PATH_SEL         = 0,               
    parameter DYN_PHASE_PATH_SEL     = "DISABLE",       
    parameter DYN_FPHASE_EN          = "DISABLE",       

//ssc
    parameter SSC_ENABLE             = "DISABLE",             
    parameter SSC_RST                = "ENABLE",        
    parameter SSC_MODE               = "CENTER",        
    parameter SSC_FREQ_DIV           = 0,                              
    parameter SSC_RNGE               = 242,                            

// lock detector
    parameter REFCLK_DET_BYP         = "DISABLE",       
//channel 0 frac div related
    parameter CLKC0_FRAC_EN          = "DISABLE",             
    parameter CLKC0_FRAC             = 0,               
    parameter PI_OUT_SEL             = "NORMAL",        
    parameter PI_FRAC_EN             = "DISABLE",       
    parameter CLKC0_PI_SHIFT_EN      = "DISABLE",       

    parameter FEEDBK_MODE            = "NORMAL",        
    parameter DYN_CPHASE_EN          = "DISABLE"       
) (
//Global clocks
    input           refclk,                          
    input           fbclk, 
//output clocks
	input			clk0_en,
	output			clk0_out,
	output			clkb0_out,

	input			clk1_en,
	output			clk1_out,
	output			clkb1_out,

	input			clk2_en,
	output			clk2_out,
	output			clkb2_out,

	input			clk3_en,
	output			clk3_out,
	output			clkb3_out,

	input			clk4_en,
	output			clk4_out,
	output			clkb4_out,

	input			clk5_en,
	output			clk5_out,
	output			clkb5_out,

	input			clk6_en,
	output			clk6_out,
	output			clkb6_out,
//PLL  lock detector
    output          lock,           
//PLL and output path reset 
    input           reset,       
    input   [1:0]   clkc_rst,          
    input           refclk_rst,     
    input           wakeup,         
       
//PLL dynamic coarse phase shift
    input           cps_step,       
//PLL power done
    input           pllpd,          
//PLL   dynami fine phase shift
    input           psclk,          
    input   [2:0]   psclksel,       
    input           psstep,         
    input           psdown,         
    output          psdone,         
//Ports of drp
    input           drp_clk,        
    input           drp_rstn,       
    input           drp_sel,        
    input           drp_rd,         
    input           drp_wr,         
    input  [7:0]    drp_addr,                      
    input  [7:0]    drp_wdata,                     
    output          drp_err,        
    output          drp_rdy,        
    output [7:0]    drp_rdata,                       
//Ports of ssc
    input           ssc_reset                     
);

	PH1P_PHY_PLL_V2 #(
    .FBKCLK                 (FBKCLK),
    .FBKCLK_INT             (FBKCLK_INT),
    .FIN                    (FIN),
//loop     parameter
    .REFCLK_DIV             (REFCLK_DIV),
    .FBCLK_DIV              (FBCLK_DIV),
//fine phase shift
    .CLKC0_FPHASE           (CLKC0_FPHASE),
    .CLKC1_FPHASE           (CLKC1_FPHASE),
    .CLKC2_FPHASE           (CLKC2_FPHASE),
    .CLKC3_FPHASE           (CLKC3_FPHASE),
    .CLKC4_FPHASE           (CLKC4_FPHASE),
    .CLKC5_FPHASE           (CLKC5_FPHASE),
    .CLKC6_FPHASE           (CLKC6_FPHASE),
//coarse phase shift
    .PREDIV_MUXC0           (PREDIV_MUXC0),
    .PREDIV_MUXC1           (PREDIV_MUXC1),
    .PREDIV_MUXC2           (PREDIV_MUXC2),
    .PREDIV_MUXC3           (PREDIV_MUXC3),
    .PREDIV_MUXC4           (PREDIV_MUXC4),
    .PREDIV_MUXC5           (PREDIV_MUXC5),
    .PREDIV_MUXC6           (PREDIV_MUXC6),

    .CLKC0_CPHASE           (CLKC0_CPHASE),
    .CLKC1_CPHASE           (CLKC1_CPHASE),
    .CLKC2_CPHASE           (CLKC2_CPHASE),
    .CLKC3_CPHASE           (CLKC3_CPHASE),
    .CLKC4_CPHASE           (CLKC4_CPHASE),
    .CLKC5_CPHASE           (CLKC5_CPHASE),
    .CLKC6_CPHASE           (CLKC6_CPHASE),
//output divider
    .CLKC0_DIV              (CLKC0_DIV),
    .CLKC1_DIV              (CLKC1_DIV),
    .CLKC2_DIV              (CLKC2_DIV),
    .CLKC3_DIV              (CLKC3_DIV),
    .CLKC4_DIV              (CLKC4_DIV),
    .CLKC5_DIV              (CLKC5_DIV),
    .CLKC6_DIV              (CLKC6_DIV),
// duty
    .CLKC0_DUTY_INT         (CLKC0_DUTY_INT),
    .CLKC1_DUTY_INT         (CLKC1_DUTY_INT),
    .CLKC2_DUTY_INT         (CLKC2_DUTY_INT),
    .CLKC3_DUTY_INT         (CLKC3_DUTY_INT),
    .CLKC4_DUTY_INT         (CLKC4_DUTY_INT),
    .CLKC5_DUTY_INT         (CLKC5_DUTY_INT),
    .CLKC6_DUTY_INT         (CLKC6_DUTY_INT),
// clk output
    .DIVOUT_MUXC0           (DIVOUT_MUXC0),
    .DIVOUT_MUXC1           (DIVOUT_MUXC1),
    .DIVOUT_MUXC2           (DIVOUT_MUXC2),
    .DIVOUT_MUXC3           (DIVOUT_MUXC3),
    .DIVOUT_MUXC4           (DIVOUT_MUXC4),
    .DIVOUT_MUXC5           (DIVOUT_MUXC5),
    .DIVOUT_MUXC6           (DIVOUT_MUXC6),

    .CLKC0_ENABLE           (CLKC0_ENABLE),
    .CLKC1_ENABLE           (CLKC1_ENABLE),
    .CLKC2_ENABLE           (CLKC2_ENABLE),
    .CLKC3_ENABLE           (CLKC3_ENABLE),
    .CLKC4_ENABLE           (CLKC4_ENABLE),
    .CLKC5_ENABLE           (CLKC5_ENABLE),
    .CLKC6_ENABLE           (CLKC6_ENABLE),

    .WORK_MODE              (WORK_MODE),

    .PD_DIG                 (PD_DIG),
    .PLL_USR_RST            (PLL_USR_RST),
    .PLL_FEED_TYPE          (PLL_FEED_TYPE),
    .PLL_FASTLOOP           (PLL_FASTLOOP),

// loop     parameter
    .LPF_RES                (LPF_RES),
    .LPF_CAP                (LPF_CAP),
    .ICP_CUR                (ICP_CUR),
//frac-N
    .FRAC_ENABLE            (FRAC_ENABLE),
    .SDM_FRAC               (SDM_FRAC),

//fine phase shift
    .PHASE_PATH_SEL         (PHASE_PATH_SEL),
    .DYN_PHASE_PATH_SEL     (DYN_PHASE_PATH_SEL),
    .DYN_FPHASE_EN          (DYN_FPHASE_EN),

    .CLKC0_FPHASE_RSTSEL    (CLKC0_FPHASE_RSTSEL),
    .CLKC1_FPHASE_RSTSEL    (CLKC1_FPHASE_RSTSEL),
    .CLKC2_FPHASE_RSTSEL    (CLKC2_FPHASE_RSTSEL),
    .CLKC3_FPHASE_RSTSEL    (CLKC3_FPHASE_RSTSEL),
    .CLKC4_FPHASE_RSTSEL    (CLKC4_FPHASE_RSTSEL),
    .CLKC5_FPHASE_RSTSEL    (CLKC5_FPHASE_RSTSEL),
    .CLKC6_FPHASE_RSTSEL    (CLKC6_FPHASE_RSTSEL),

    .CLKC0_DUTY50           (CLKC0_DUTY50),
    .CLKC1_DUTY50           (CLKC1_DUTY50),
    .CLKC2_DUTY50           (CLKC2_DUTY50),
    .CLKC3_DUTY50           (CLKC3_DUTY50),
    .CLKC4_DUTY50           (CLKC4_DUTY50),
    .CLKC5_DUTY50           (CLKC5_DUTY50),
    .CLKC6_DUTY50           (CLKC6_DUTY50),
//ssc
    .SSC_ENABLE             (SSC_ENABLE),
    .SSC_RST                (SSC_RST),
    .SSC_MODE               (SSC_MODE),
    .SSC_FREQ_DIV           (SSC_FREQ_DIV),
    .SSC_RNGE               (SSC_RNGE),

// lock detector
    .REFCLK_DET_BYP         (REFCLK_DET_BYP),
//channel 0 frac div related
    .CLKC0_FRAC_EN          (CLKC0_FRAC_EN),
    .CLKC0_FRAC             (CLKC0_FRAC),
    .PI_OUT_SEL             (PI_OUT_SEL),
    .PI_FRAC_EN             (PI_FRAC_EN),
    .CLKC0_PI_SHIFT_EN      (CLKC0_PI_SHIFT_EN),

    .FEEDBK_MODE            (FEEDBK_MODE),
    .DYN_CPHASE_EN          (DYN_CPHASE_EN)
	) u_PH1P_PHY_PLL (
		.refclk			( refclk		),
		.refclk_rst		( refclk_rst	),
		.fbclk			( fbclk			),
		.clkc			({clk6_out,clk5_out,clk4_out,clk3_out,clk2_out,clk1_out,clk0_out}),
		.clkcb			({clkb6_out,clkb5_out,clkb4_out,clkb3_out,clkb2_out,clkb1_out,clkb0_out}),
		.lock			( lock			),
		.pllreset		( reset  		),
		.clkc_rst		( clkc_rst		),
		.clkc_en		({clk6_en,clk5_en,clk4_en,clk3_en,clk2_en,clk1_en,clk0_en}),
		.wakeup			( wakeup		),
		.cps_step		( cps_step		),
		.pllpd			( pllpd			),
		.drp_clk		( drp_clk		),
		.drp_rstn		( drp_rstn		),
		.drp_sel		( drp_sel		),
		.drp_rd			( drp_rd		),
		.drp_wr			( drp_wr		),
		.drp_addr		( drp_addr		),
		.drp_wdata		( drp_wdata		),
		.drp_err		( drp_err		),
		.drp_rdy		( drp_rdy		),
		.drp_rdata		( drp_rdata		),
		.psdone			( psdone		),
		.psclk			( psclk			),
		.psdown			( psdown		),
		.psstep			( psstep		),
		.psclksel		( psclksel		),
		.ssc_reset		( ssc_reset		),
        .scan_clk       ( 1'b1          ),
        .scan_enable    ( 1'b1          ),
        .scan_mode      ( 1'b0          ),
        .scan_resetn    ( 1'b1          )
	);

endmodule


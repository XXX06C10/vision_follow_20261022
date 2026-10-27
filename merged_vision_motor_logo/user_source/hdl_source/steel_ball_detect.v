`timescale 1ns/1ps
// Mirror for the EXISTING isp_top: MSB-first packed RGB888, 5/5/6 pixels
// per three 128-bit transfers. tuser may precede valid; legacy tlast leads
// the last packed transfer by one clock. Neither sideband is valid-gated.
// All video outputs are wires. No backpressure is introduced.
module steel_ball_detect #(
    parameter IMG_WIDTH = 1920,
    parameter IMG_HEIGHT = 1080,
    parameter Y_THRESHOLD = 200,
    parameter MIN_AREA = 80,
    parameter MAX_AREA = 12000,
    parameter CONFIRM_FRAMES = 4,
    parameter LABEL_BITS = 6,       // at most 63 provisional labels/frame
    parameter FIFO_BITS = 12,       // 4096 mask tokens, not RGB words
    parameter TLAST_DELAY = 1,
    parameter FILL_MIN_PERCENT = 65,
    parameter FILL_MAX_PERCENT = 90,
    parameter ASPECT_PERCENT = 75,
    parameter CIRCULARITY_SCALE = 24, // P^2 <= 24*A; grid perimeter
    parameter COLOR_MODE = 0,          // 0=luminance, 1=red, 2=green, 3=blue, 4=magenta
    parameter COLOR_MIN = 60,          // dominant-channel absolute floor
    parameter COLOR_RATIO_LOG2 = 1,    // dominant >= 2^this * each other channel
    parameter COLOR_HIGHLIGHT_FILL = 0, // also mask very-bright pixels (fills specular highlight)
    parameter OVERLAY_EN = 0,          // 1=color masked pixels magenta in video pass-through
    parameter OVERLAY_COLOR = 24'hFF00FF, // overlay RGB (magenta)
    parameter OVERLAY_BOX = 0,
    parameter EXTERNAL_BOX = 0,
    parameter BOX_COLOR = 24'h00FF00,
    parameter CAND_COLOR = 24'hFFC000,
    parameter BOX_MARGIN = 3,
    parameter BOX_THICKNESS = 2,
    parameter ROI_OVERLAY = 0,
    parameter TRACK_EN = 0,
    parameter TRACK_RADIUS = 48,
    parameter TRACK_COLOR_MIN = 35
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire [127:0] S_AXIS_tdata,
    input wire S_AXIS_tvalid,
    input wire S_AXIS_tuser,
    input wire S_AXIS_tlast,
    input wire I_box_valid,
    input wire [10:0] I_box_left,
    input wire [10:0] I_box_right,
    input wire [10:0] I_box_top,
    input wire [10:0] I_box_bottom,
    input wire I_box_confirmed,
    input wire [10:0] I_roi_left,
    input wire [10:0] I_roi_top,
    input wire [10:0] I_roi_right,
    input wire [10:0] I_roi_bottom,
    input wire I_track_valid,
    input wire [10:0] I_track_x,I_track_y,
    output wire [5:0] O_color_mask,
    output wire [4:0] O_color_meta,
    output wire [127:0] M_AXIS_tdata,
    output wire M_AXIS_tvalid,
    output wire M_AXIS_tuser,
    output wire M_AXIS_tlast,
    output wire O_ball_detected,
    output reg O_frame_done,
    output reg O_frame_hit,
    output reg O_fault,
    output reg O_bright,
    output reg [1:0] O_error_code,
    output reg [21:0] O_diag_red_pixels,
    output reg [21:0] O_diag_largest_area,
    output reg [23:0] O_diag_largest_perimeter,
    output reg [10:0] O_diag_xmin,
    output reg [10:0] O_diag_xmax,
    output reg [10:0] O_diag_ymin,
    output reg [10:0] O_diag_ymax,
    output reg [3:0] O_diag_geometry,
    output reg [FIFO_BITS:0] O_diag_fifo_peak,
    output wire [11:0] O_diag_lines,
    output wire [LABEL_BITS:0] O_diag_labels,
    output wire O_diag_bad_frame,
    output wire O_diag_label_exhausted,
    output wire O_diag_box_valid
);
    // Overlay (mask-recolor) outputs, declared before use in M_AXIS below.
    reg [127:0] ov_data;
    reg ov_valid, ov_user, ov_last;
    assign M_AXIS_tdata  = OVERLAY_EN ? ov_data  : S_AXIS_tdata;
    assign M_AXIS_tvalid = OVERLAY_EN ? ov_valid : S_AXIS_tvalid;
    assign M_AXIS_tuser  = OVERLAY_EN ? ov_user  : S_AXIS_tuser;
    assign M_AXIS_tlast  = OVERLAY_EN ? ov_last  : S_AXIS_tlast;

    localparam LABELS = (1 << LABEL_BITS);
    localparam DEPTH = (1 << FIFO_BITS);
    reg last_d;
    wire line_end = TLAST_DELAY ? last_d : S_AXIS_tlast;
    reg [1:0] phase;
    reg [15:0] carry;
    wire [1:0] effective_phase = S_AXIS_tuser ? 2'd0 : phase;
    reg [143:0] rgb;
    always @* begin
        case(effective_phase)
            0: rgb = {S_AXIS_tdata[127:8],24'd0};
            1: rgb = {carry[7:0],S_AXIS_tdata[127:16],24'd0};
            default: rgb = {carry,S_AXIS_tdata};
        endcase
    end
    // Six parallel luminance/color units, registered before summing and threshold.
    reg [15:0] yr[0:5], yg[0:5], yb[0:5];
    reg [15:0] lum[0:5];
    reg [7:0] rr[0:5], gg[0:5], bb[0:5];
    reg [7:0] rr1[0:5], gg1[0:5], bb1[0:5];
    reg [4:0] meta0, meta1, meta2; // {SOF,EOL,pixel_count}
    reg [5:0] mask2;
    reg [5:0] local0,local1;
    reg [5:0] local_now;
    // EOL belongs to the final packed group. Advance these coordinates AFTER
    // that group; the legacy overlay counters have different boundary timing.
    reg [10:0] local_x,local_y;
    wire [10:0] local_base_x=S_AXIS_tuser ? 11'd0 : local_x;
    wire [10:0] local_base_y=S_AXIS_tuser ? 11'd0 : local_y;
    assign O_color_mask = mask2;
    assign O_color_meta = meta2;
    integer k;
    // Per-pixel mask, inlined (parameter-folded). COLOR_MODE=0 keeps the
    // original luminance test; 1/2/3/4 use a channel-dominance ratio plus an
    // absolute floor that rejects near-black shadows/noise. COLOR_HIGHLIGHT_FILL
    // additionally masks very bright pixels (fills a specular highlight).
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            last_d <= 0; phase <= 0; carry <= 0;
            meta0 <= 0; meta1 <= 0; meta2 <= 0; mask2 <= 0;local0<=0;local1<=0;local_x<=0;local_y<=0;
            for(k=0;k<6;k=k+1) begin
                yr[k]<=0; yg[k]<=0; yb[k]<=0; lum[k]<=0; rr[k]<=0; gg[k]<=0; bb[k]<=0;
                rr1[k]<=0; gg1[k]<=0; bb1[k]<=0;
            end
        end else begin
            last_d <= S_AXIS_tlast;
            if(S_AXIS_tuser)begin
                local_x<=line_end ? 11'd0 : (S_AXIS_tvalid ? (effective_phase==2 ? 11'd6 : 11'd5) : 11'd0);
                local_y<=line_end ? 11'd1 : 11'd0;
            end else if(line_end)begin local_x<=0;local_y<=local_y+1'b1;end
            else if(S_AXIS_tvalid)local_x<=local_x+(effective_phase==2 ? 11'd6 : 11'd5);
            if(line_end) phase <= 0;
            else if(S_AXIS_tuser) phase <= S_AXIS_tvalid ? 1 : 0;
            else if(S_AXIS_tvalid) phase <= (effective_phase==2) ? 0 : effective_phase+1'b1;
            if(S_AXIS_tvalid) carry <= S_AXIS_tdata[15:0];
            meta0 <= {S_AXIS_tuser,line_end,
                S_AXIS_tvalid ? ((effective_phase==2) ? 3'd6 : 3'd5) : 3'd0};
            meta1 <= meta0; meta2 <= meta1;local0<=local_now;local1<=local0;
            for(k=0;k<6;k=k+1) begin
                yr[k] <= rgb[143-k*24 -: 8] * 16'd77;
                yg[k] <= rgb[135-k*24 -: 8] * 16'd150;
                yb[k] <= rgb[127-k*24 -: 8] * 16'd29;
                lum[k] <= yr[k]+yg[k]+yb[k];
                rr[k] <= rgb[143-k*24 -: 8];
                gg[k] <= rgb[135-k*24 -: 8];
                bb[k] <= rgb[127-k*24 -: 8];
                rr1[k] <= rr[k];
                gg1[k] <= gg[k];
                bb1[k] <= bb[k];
                mask2[k] <=
                    (COLOR_MODE!=0 && COLOR_HIGHLIGHT_FILL && lum[k][15:8] > 8'd235) ||
                    (COLOR_MODE==0 && lum[k][15:8] > Y_THRESHOLD) ||
                    (COLOR_MODE==1 && rr1[k] > ({1'b0,gg1[k]} << COLOR_RATIO_LOG2) &&
                                     rr1[k] > ({1'b0,bb1[k]} << COLOR_RATIO_LOG2) &&
                                     rr1[k] >= COLOR_MIN) ||
                    (COLOR_MODE==1 && TRACK_EN && local1[k] && rr1[k]>=TRACK_COLOR_MIN &&
                     ({2'd0,rr1[k]} << 1)>(({2'd0,gg1[k]} << 1)+{2'd0,gg1[k]}) &&
                     ({2'd0,rr1[k]} << 1)>(({2'd0,bb1[k]} << 1)+{2'd0,bb1[k]})) ||
                    (COLOR_MODE==2 && gg1[k] > ({1'b0,rr1[k]} << COLOR_RATIO_LOG2) &&
                                     gg1[k] > ({1'b0,bb1[k]} << COLOR_RATIO_LOG2) &&
                                     gg1[k] >= COLOR_MIN) ||
                    (COLOR_MODE==3 && bb1[k] > ({1'b0,rr1[k]} << COLOR_RATIO_LOG2) &&
                                     bb1[k] > ({1'b0,gg1[k]} << COLOR_RATIO_LOG2) &&
                                     bb1[k] >= COLOR_MIN) ||
                    (COLOR_MODE==4 && rr1[k] > ({1'b0,gg1[k]} << COLOR_RATIO_LOG2) &&
                                     bb1[k] > ({1'b0,gg1[k]} << COLOR_RATIO_LOG2) &&
                                     rr1[k] >= COLOR_MIN && bb1[k] >= COLOR_MIN);
            end
        end
    end

    reg [127:0] dd0, dd1, dd2, dd3;
    reg [1:0] pdp1, pdp2, pdp3, pdp4;
    reg vd0, vd1, vd2, vd3, ud0, ud1, ud2, ud3, ld0, ld1, ld2, ld3;
    reg [5:0] mask2_reg;
    reg [5:0] box_mask0, box_mask1, box_mask2, box_mask3;
    reg [5:0] roi_mask0, roi_mask1, roi_mask2, roi_mask3;
    reg conf0, conf1, conf2, conf3;
    reg [5:0] box_now;
    reg [5:0] roi_now;
    reg [10:0] video_x, video_y;
    reg [10:0] box_left, box_right, box_top, box_bottom;
    reg box_valid;
    wire active_box_valid = EXTERNAL_BOX ? I_box_valid : box_valid;
    wire [10:0] active_box_left = EXTERNAL_BOX ? I_box_left : box_left;
    wire [10:0] active_box_right = EXTERNAL_BOX ? I_box_right : box_right;
    wire [10:0] active_box_top = EXTERNAL_BOX ? I_box_top : box_top;
    wire [10:0] active_box_bottom = EXTERNAL_BOX ? I_box_bottom : box_bottom;
    wire [10:0] video_base_x = (S_AXIS_tuser || line_end) ? 11'd0 : video_x;
    wire [10:0] video_base_y = S_AXIS_tuser ? 11'd0 : line_end ? video_y+11'd1 : video_y;
    wire [2:0] video_count = effective_phase==2 ? 3'd6 : 3'd5;
    wire [10:0] track_left=I_track_x>TRACK_RADIUS ? I_track_x-TRACK_RADIUS : 11'd0;
    wire [10:0] track_top=I_track_y>TRACK_RADIUS ? I_track_y-TRACK_RADIUS : 11'd0;
    wire [11:0] track_right={1'b0,I_track_x}+TRACK_RADIUS;
    wire [11:0] track_bottom={1'b0,I_track_y}+TRACK_RADIUS;
    integer q;
    reg [11:0] box_px;
    reg [11:0] local_px;
    always @* begin
        box_now = 0;
        roi_now = 0;
        local_now = 0;
        box_px = 0;
        local_px = 0;
        for(q=0;q<6;q=q+1) begin
            box_px = {1'b0,video_base_x}+q;
            local_px = {1'b0,local_base_x}+q;
            if(TRACK_EN && I_track_valid && q<video_count &&
               local_px<IMG_WIDTH && local_base_y<IMG_HEIGHT &&
               local_px>=track_left && local_px<=track_right &&
               local_base_y>=track_top && local_base_y<=track_bottom &&
               local_px>=I_roi_left && local_px<=I_roi_right &&
               local_base_y>=I_roi_top && local_base_y<=I_roi_bottom)local_now[q]=1;
            if(q<video_count && active_box_valid && box_px>=active_box_left && box_px<=active_box_right &&
               video_base_y>=active_box_top && video_base_y<=active_box_bottom &&
               (box_px<active_box_left+BOX_THICKNESS || box_px+BOX_THICKNESS>active_box_right ||
                video_base_y<active_box_top+BOX_THICKNESS || video_base_y+BOX_THICKNESS>active_box_bottom))
                box_now[q] = 1;
            if(ROI_OVERLAY && q<video_count &&
               box_px>=I_roi_left && box_px<=I_roi_right &&
               video_base_y>=I_roi_top && video_base_y<=I_roi_bottom &&
               (box_px<I_roi_left+11'd2 || box_px+12'd2>I_roi_right ||
                video_base_y<I_roi_top+11'd2 || video_base_y+11'd2>I_roi_bottom))
                roi_now[q] = 1;
        end
    end
    function [127:0] ov_recolor;
        input [127:0] d;
        input [1:0] ph;
        input [5:0] ma;
        input mb0;
        input [23:0] col;
        reg [15:0] bm;
        reg [127:0] sel;
        reg [383:0] rep;
        integer i;
        begin
            case(ph)
                2'd0: bm = {ma[0],ma[0],ma[0],ma[1],ma[1],ma[1],ma[2],ma[2],ma[2],ma[3],ma[3],ma[3],ma[4],ma[4],ma[4],mb0};
                2'd1: bm = {ma[0],ma[0],ma[1],ma[1],ma[1],ma[2],ma[2],ma[2],ma[3],ma[3],ma[3],ma[4],ma[4],ma[4],mb0,mb0};
                default: bm = {ma[0],ma[1],ma[1],ma[1],ma[2],ma[2],ma[2],ma[3],ma[3],ma[3],ma[4],ma[4],ma[4],ma[5],ma[5],ma[5]};
            endcase
            sel = 128'd0;
            for(i=0;i<16;i=i+1) sel[8*i +: 8] = {8{bm[i]}};
            rep = {16{col}};
            case(ph)
                2'd0: ov_recolor = (d & ~sel) | (rep[383:256] & sel);
                2'd1: ov_recolor = (d & ~sel) | (rep[255:128] & sel);
                default: ov_recolor = (d & ~sel) | (rep[127:0] & sel);
            endcase
        end
    endfunction
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            dd0<=0; dd1<=0; dd2<=0; dd3<=0;
            pdp1<=0; pdp2<=0; pdp3<=0; pdp4<=0;
            vd0<=0; vd1<=0; vd2<=0; vd3<=0;
            ud0<=0; ud1<=0; ud2<=0; ud3<=0;
            ld0<=0; ld1<=0; ld2<=0; ld3<=0;
            mask2_reg<=0; box_mask0<=0; box_mask1<=0; box_mask2<=0; box_mask3<=0;
            roi_mask0<=0; roi_mask1<=0; roi_mask2<=0; roi_mask3<=0;
            conf0<=0; conf1<=0; conf2<=0; conf3<=0;
            video_x<=0; video_y<=0; ov_data<=0;
            ov_valid<=0; ov_user<=0; ov_last<=0;
        end else begin
            if(S_AXIS_tuser) video_y<=0;
            else if(line_end) video_y<=video_y+11'd1;
            if(S_AXIS_tvalid) video_x<=video_base_x+video_count;
            else if(S_AXIS_tuser || line_end) video_x<=0;
            dd0<=S_AXIS_tdata; dd1<=dd0; dd2<=dd1; dd3<=dd2;
            pdp1<=effective_phase; pdp2<=pdp1; pdp3<=pdp2; pdp4<=pdp3;
            vd0<=S_AXIS_tvalid; vd1<=vd0; vd2<=vd1; vd3<=vd2;
            ud0<=S_AXIS_tuser;  ud1<=ud0;  ud2<=ud1; ud3<=ud2;
            ld0<=S_AXIS_tlast;  ld1<=ld0;  ld2<=ld1; ld3<=ld2;
            mask2_reg<=mask2;
            box_mask0<=box_now; box_mask1<=box_mask0; box_mask2<=box_mask1; box_mask3<=box_mask2;
            roi_mask0<=roi_now; roi_mask1<=roi_mask0; roi_mask2<=roi_mask1; roi_mask3<=roi_mask2;
            conf0<=I_box_confirmed; conf1<=conf0; conf2<=conf1; conf3<=conf2;
            ov_valid<=vd3; ov_user<=ud3; ov_last<=ld3;
            ov_data<=ov_recolor(ROI_OVERLAY ?
                ov_recolor(dd3,pdp4,roi_mask3,roi_mask2[0],24'h00BFFF) : dd3, pdp4,
                OVERLAY_BOX ? box_mask3 : mask2_reg,
                OVERLAY_BOX ? box_mask2[0] : mask2[0],
                OVERLAY_BOX ? (conf3 ? BOX_COLOR : CAND_COLOR) : OVERLAY_COLOR);
        end
    end

    // Synchronous BRAM FIFO absorbs line bursts. On overflow discard the
    // damaged frame and restart at the NEXT input SOF, including its pixels.
    reg [10:0] fifo[0:DEPTH-1];
    reg [FIFO_BITS:0] wp,rp;
    wire empty = wp==rp;
    wire full = (wp[FIFO_BITS]!=rp[FIFO_BITS]) &&
                (wp[FIFO_BITS-1:0]==rp[FIFO_BITS-1:0]);
    wire push = |meta2;
    reg [10:0] fq;
    reg [5:0] bits_left;
    reg [2:0] count_left;
    reg token_eol;
    reg [4:0] state;
    localparam IDLE=0, TOKEN=1, SOF=2, PIX=3, ABOVE=4, ROOT_L=5,
        ROOT_U=6, READ_A=7, SAVE_A=8, READ_B=9, WRITE=10,
        ADVANCE=11, ENDLINE=12, EVAL_READ=13, EVAL_SAVE=14,
        EVAL_MUL=15, EVAL_TEST=16, FRAME=17;
    wire overflow_event = push && full && !O_fault;
    wire recover_sof = O_fault && meta2[4];
    wire pop = state==IDLE && !empty && !O_fault;
    always @(posedge I_clk) begin
        if(recover_sof || (push && !full && !O_fault))
            fifo[recover_sof ? {FIFO_BITS{1'b0}} : wp[FIFO_BITS-1:0]] <= {meta2,mask2};
        if(pop) fq <= fifo[rp[FIFO_BITS-1:0]];
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin wp<=0; rp<=0; O_fault<=0; end
        else if(recover_sof) begin wp<=1; rp<=0; O_fault<=0; end
        else if(O_fault) begin wp<=0; rp<=0; end
        else begin
            if(push && !full) wp<=wp+1'b1;
            if(pop) rp<=rp+1'b1;
            if(push && full) O_fault<=1;
        end
    end

    // Full-resolution CCL with fast clearing of all-dark 5/6-pixel tokens.
    // Four-connected pixel CCL. Labels are union-find nodes; union always
    // points the larger root at the smaller root, so cycles are impossible.
    // Node IDs are not recycled within a frame, avoiding stale line aliases.
    reg [LABEL_BITS-1:0] parent[0:LABELS-1];
    localparam LINE_WORD_BITS = 16*LABEL_BITS;
    reg [LINE_WORD_BITS-1:0] line_labels[0:IMG_WIDTH/16-1];
    reg [LINE_WORD_BITS-1:0] above_word, line_word_next;
    reg skip_token;
    reg [LABEL_BITS-1:0] left_label,a,b,root,other;
    reg [LABEL_BITS:0] next_label;
    reg [10:0] x,y;
    wire [LABEL_BITS-1:0] above_q = above_word[x[3:0]*LABEL_BITS +: LABEL_BITS];
    reg [11:0] line_pixels,lines;
    reg seen_frame,bad_frame,hit,ball;
    reg label_exhausted;
    assign O_diag_lines = lines;
    assign O_diag_labels = next_label;
    assign O_diag_bad_frame = bad_frame;
    assign O_diag_label_exhausted = label_exhausted;
    assign O_diag_box_valid = box_valid;
    reg [7:0] streak;
    assign O_ball_detected = ball && !O_fault;
    reg [1:0] neighbors;
    reg merging,new_label;
    // {area[21:0], perimeter[23:0], xmin,xmax,ymin,ymax (11 bits each)}
    reg [89:0] stats[0:LABELS-1];
    reg [89:0] sq,sa,combined;
    reg [LABEL_BITS-1:0] eval_id;
    wire [LABEL_BITS-1:0] read_id =
        state==EVAL_READ ? eval_id : state==READ_B ? other : root;
    always @(posedge I_clk) begin
        if(state==PIX) above_word <= line_labels[x[10:4]];
        if((state==ABOVE && !bits_left[0]) || state==WRITE)
            line_labels[x[10:4]] <= line_word_next;
        sq <= stats[read_id];
        if(state==WRITE) stats[root] <= combined;
    end
    always @* begin
        line_word_next = above_word;
        if(skip_token) begin
            case(x[3:0])
                0: line_word_next[5*LABEL_BITS-1:0] = 0;
                5: line_word_next[10*LABEL_BITS-1:5*LABEL_BITS] = 0;
                10: line_word_next[16*LABEL_BITS-1:10*LABEL_BITS] = 0;
                default: line_word_next = 0; // malformed stream; frame is rejected
            endcase
        end else line_word_next[x[3:0]*LABEL_BITS +: LABEL_BITS] = (state==WRITE) ? root : 0;
    end
    function [10:0] min11;
        input [10:0] aa,bb;
        begin min11=(aa<bb)?aa:bb; end
    endfunction
    function [10:0] max11;
        input [10:0] aa,bb;
        begin max11=(aa>bb)?aa:bb; end
    endfunction

    reg [21:0] ca;
    reg [23:0] cp;
    reg [10:0] cx0,cx1,cy0,cy1;
    always @* begin
        ca=22'd1; cp=24'd4;
        cx0=x; cx1=x; cy0=y; cy1=y;
        if(!new_label) begin
            ca=sa[89:68]+22'd1;
            cp=sa[67:44]+24'd4-({22'd0,neighbors}<<1);
            cx0=min11(sa[43:33],x); cx1=max11(sa[32:22],x);
            cy0=min11(sa[21:11],y); cy1=max11(sa[10:0],y);
            if(merging) begin
                ca=ca+sq[89:68]; cp=cp+sq[67:44];
                cx0=min11(cx0,sq[43:33]); cx1=max11(cx1,sq[32:22]);
                cy0=min11(cy0,sq[21:11]); cy1=max11(cy1,sq[10:0]);
            end
        end
        combined={ca,cp,cx0,cx1,cy0,cy1};
    end
    reg [11:0] bw,bh;
    reg [23:0] bbox;
    reg [31:0] perimeter_squared;
    reg [31:0] area100,area_circ,bbox_min,bbox_max;
    reg [17:0] width100,height100,width_limit,height_limit;
    reg [21:0] box_candidate_area;
    reg [10:0] box_candidate_xmin,box_candidate_xmax,box_candidate_ymin,box_candidate_ymax;
    wire geometry_match = eval_id<next_label && parent[eval_id]==eval_id &&
        sa[89:68]>=MIN_AREA && sa[89:68]<=MAX_AREA &&
        area100>=bbox_min && area100<=bbox_max &&
        width100>=height_limit && height100>=width_limit &&
        sa[67:60]==0 && perimeter_squared<=area_circ;
    integer j;
    // Input-side evidence remains available even when the CCL cannot keep up.
    // Saturate after 80 bright pixels; do not mistake one hot pixel for a target.
    reg [7:0] bright_count;
    reg [2:0] bright_add;
    reg [21:0] red_count;
    reg [FIFO_BITS:0] fifo_peak_current;
    wire [FIFO_BITS:0] fifo_used = wp-rp;
    integer n;
    always @* begin
        bright_add=0;
        for(n=0;n<6;n=n+1)
            if(n<meta2[2:0] && mask2[n]) bright_add=bright_add+1'b1;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin bright_count<=0; O_bright<=0; O_error_code<=0; end
        else begin
            if(meta2[4]) begin
                O_bright <= bright_count>=80;
                bright_count <= {5'd0,bright_add};
            end else if(bright_count<80) bright_count<=bright_count+bright_add;
            if(O_error_code==0) begin
                if(overflow_event) O_error_code<=1;
                else if(state==EVAL_READ && eval_id==0) begin
                    if(label_exhausted) O_error_code<=2;
                    else if(bad_frame || lines!=IMG_HEIGHT || line_pixels!=0) O_error_code<=3;
                end
            end
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            red_count<=0; O_diag_red_pixels<=0;
            fifo_peak_current<=0; O_diag_fifo_peak<=0;
        end else if(meta2[4]) begin
            O_diag_red_pixels<=red_count;
            red_count<=bright_add;
            O_diag_fifo_peak<=fifo_peak_current;
            fifo_peak_current<=fifo_used;
        end else begin
            red_count<=red_count+bright_add;
            if(fifo_used>fifo_peak_current) fifo_peak_current<=fifo_used;
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            skip_token<=0; state<=IDLE; bits_left<=0; count_left<=0; token_eol<=0;
            x<=0; y<=0; line_pixels<=0; lines<=0; left_label<=0;
            a<=0; b<=0; root<=0; other<=0; next_label<=1;
            seen_frame<=0; bad_frame<=0; hit<=0; ball<=0; streak<=0; label_exhausted<=0;
            neighbors<=0; merging<=0; new_label<=0; sa<=0; eval_id<=1;
            bw<=0; bh<=0; bbox<=0; perimeter_squared<=0;
            area100<=0; area_circ<=0; bbox_min<=0; bbox_max<=0;
            width100<=0; height100<=0; width_limit<=0; height_limit<=0;
            box_candidate_area<=0;
            box_candidate_xmin<=0; box_candidate_xmax<=0;
            box_candidate_ymin<=0; box_candidate_ymax<=0;
            O_diag_largest_area<=0; O_diag_largest_perimeter<=0;
            O_diag_xmin<=0; O_diag_xmax<=0; O_diag_ymin<=0; O_diag_ymax<=0;
            O_diag_geometry<=0;
            box_left<=0; box_right<=0; box_top<=0; box_bottom<=0; box_valid<=0;
            O_frame_done<=0; O_frame_hit<=0;
            for(j=0;j<LABELS;j=j+1) parent[j]<=j;
        end else if(O_fault || overflow_event) begin
            state<=IDLE; seen_frame<=0; ball<=0; streak<=0;
            box_valid<=0;
            O_frame_done<=0; O_frame_hit<=0;
        end else begin
            O_frame_done<=0;
            case(state)
            IDLE: if(!empty) state<=TOKEN;
            TOKEN: begin
                bits_left<=fq[5:0]; count_left<=fq[8:6]; token_eol<=fq[9];
                skip_token<=(fq[5:0]==0);
                if(fq[10]) begin
                    eval_id<=1; hit<=0; box_candidate_area<=0;
                    O_diag_largest_area<=0; O_diag_largest_perimeter<=0;
                    O_diag_xmin<=0; O_diag_xmax<=0; O_diag_ymin<=0; O_diag_ymax<=0;
                    O_diag_geometry<=0;
                    if(seen_frame) state<=EVAL_READ;
                    else state<=SOF;
                end else if(seen_frame) state <= (fq[8:6]!=0) ? PIX : ENDLINE;
                else state<=IDLE;
            end
            SOF: begin
                x<=0; y<=0; line_pixels<=0; lines<=0; left_label<=0;
                next_label<=1; bad_frame<=0; seen_frame<=1; label_exhausted<=0;
                for(j=0;j<LABELS;j=j+1) parent[j]<=j;
                state <= (count_left!=0) ? PIX : ENDLINE;
            end
            PIX: state<=ABOVE;
            ABOVE: begin
                if(bad_frame || !bits_left[0]) begin
                    left_label<=0; state<=ADVANCE;
                end else begin
                    a<=left_label;
                    b<=(y==0) ? 0 : above_q;
                    neighbors <= {1'b0,(left_label!=0)} + {1'b0,((y!=0)&&(above_q!=0))};
                    state<=ROOT_L;
                end
            end
            ROOT_L: if(a!=0 && parent[a]!=a) a<=parent[a]; else state<=ROOT_U;
            ROOT_U: begin
                if(b!=0 && parent[b]!=b) b<=parent[b];
                else begin
                    merging <= (a!=0 && b!=0 && a!=b);
                    new_label <= (a==0 && b==0);
                    if(a==0 && b==0) begin
                        if(next_label==LABELS) begin
                            bad_frame<=1; label_exhausted<=1; left_label<=0; state<=ADVANCE;
                        end else begin
                            root<=next_label[LABEL_BITS-1:0]; other<=0;
                            next_label<=next_label+1'b1; state<=READ_A;
                        end
                    end else begin
                        root <= (a==0) ? b : (b==0 || a<b) ? a : b;
                        other <= (a>b) ? a : b;
                        state<=READ_A;
                    end
                end
            end
            READ_A: state<=SAVE_A;
            SAVE_A: begin sa<=sq; state<=READ_B; end
            READ_B: state<=WRITE;
            WRITE: begin
                if(merging) parent[other]<=root;
                left_label<=root; state<=ADVANCE;
            end
            ENDLINE: begin
                if(token_eol) begin
                    if(line_pixels!=IMG_WIDTH) bad_frame<=1;
                    x<=0; line_pixels<=0; left_label<=0;
                    if(y<IMG_HEIGHT-1) y<=y+1'b1;
                    if(lines<IMG_HEIGHT+1) lines<=lines+1'b1;
                end
                state<=IDLE;
            end
            EVAL_READ: state<=EVAL_SAVE;
            EVAL_SAVE: begin
                sa<=sq;
                bw<={1'b0,sq[32:22]}-{1'b0,sq[43:33]}+12'd1;
                bh<={1'b0,sq[10:0]}-{1'b0,sq[21:11]}+12'd1;
                state<=EVAL_MUL;
            end
            EVAL_MUL: begin
                bbox<=bw*bh;
                perimeter_squared<=sa[59:44]*sa[59:44];
                area100<=sa[89:68]*32'd100;
                area_circ<=sa[89:68]*CIRCULARITY_SCALE;
                width100<=bw*18'd100; height100<=bh*18'd100;
                width_limit<=bw*ASPECT_PERCENT; height_limit<=bh*ASPECT_PERCENT;
                state<=EVAL_TEST;
            end
            EVAL_TEST: begin
                bbox_min<=bbox*FILL_MIN_PERCENT; bbox_max<=bbox*FILL_MAX_PERCENT;
                state<=FRAME;
            end
            FRAME: begin
                if(eval_id<next_label && parent[eval_id]==eval_id && sa[89:68]>O_diag_largest_area) begin
                    O_diag_largest_area<=sa[89:68];
                    O_diag_largest_perimeter<=sa[67:44];
                    O_diag_xmin<=sa[43:33]; O_diag_xmax<=sa[32:22];
                    O_diag_ymin<=sa[21:11]; O_diag_ymax<=sa[10:0];
                    O_diag_geometry<={sa[67:60]==0 && perimeter_squared<=area_circ,
                        width100>=height_limit && height100>=width_limit,
                        area100>=bbox_min && area100<=bbox_max,
                        sa[89:68]>=MIN_AREA && sa[89:68]<=MAX_AREA};
                end
                if(geometry_match) begin
                    hit<=1;
                    if(sa[89:68]>box_candidate_area) begin
                        box_candidate_area<=sa[89:68];
                        box_candidate_xmin<=sa[43:33]; box_candidate_xmax<=sa[32:22];
                        box_candidate_ymin<=sa[21:11]; box_candidate_ymax<=sa[10:0];
                    end
                end
                if(eval_id==LABELS-1) begin
                    // One extra EVAL_READ with ID zero commits hit on next cycle.
                    state<=EVAL_READ; eval_id<=0;
                end else begin eval_id<=eval_id+1'b1; state<=EVAL_READ; end
            end
            default: state<=IDLE;
            endcase
            if((state==ABOVE && (bad_frame || !bits_left[0])) || state==WRITE || state==ADVANCE) begin
                bits_left<=bits_left>>1; count_left<=count_left-1'b1;
                if(line_pixels>=IMG_WIDTH || lines>=IMG_HEIGHT) bad_frame<=1;
                if(skip_token) begin
                    if(line_pixels+count_left>IMG_WIDTH) bad_frame<=1;
                    line_pixels<=line_pixels+count_left;
                    if(x+count_left<IMG_WIDTH) x<=x+count_left;
                    count_left<=0; state<=ENDLINE;
                end else begin
                    line_pixels<=line_pixels+1'b1;
                    if(x<IMG_WIDTH-1) x<=x+1'b1;
                    state <= (count_left==1) ? ENDLINE : PIX;
                end
            end
            if(state==EVAL_READ && eval_id==0) begin
                O_frame_done<=1;
                O_frame_hit<=hit && !bad_frame && !O_fault && lines==IMG_HEIGHT && line_pixels==0;
                box_valid<=hit && !bad_frame && !O_fault && lines==IMG_HEIGHT && line_pixels==0;
                box_left<=(box_candidate_xmin>BOX_MARGIN) ? box_candidate_xmin-BOX_MARGIN : 0;
                box_right<=(box_candidate_xmax+BOX_MARGIN<IMG_WIDTH) ? box_candidate_xmax+BOX_MARGIN : IMG_WIDTH-1;
                box_top<=(box_candidate_ymin>BOX_MARGIN) ? box_candidate_ymin-BOX_MARGIN : 0;
                box_bottom<=(box_candidate_ymax+BOX_MARGIN<IMG_HEIGHT) ? box_candidate_ymax+BOX_MARGIN : IMG_HEIGHT-1;
                if(bad_frame || O_fault || lines!=IMG_HEIGHT || line_pixels!=0) begin
                    // Invalid frames break confirmation; never use them as evidence.
                    streak<=0; ball<=0;
                end else if(hit==ball) streak<=0;
                else if(streak==CONFIRM_FRAMES-1) begin ball<=hit; streak<=0; end
                else streak<=streak+1'b1;
                state<=SOF;
            end
        end
    end
endmodule

// LED1 toggles every 15 input frames. LED3 reports the first error since
// reset as 1/2/3 short flashes in a repeating 2-second group.
module steel_ball_status_led #(
    parameter TICK_CYCLES = 11251100,
    parameter HEARTBEAT_FRAMES = 15
)(
    input wire I_clk, I_rst_n, I_sof,
    input wire [1:0] I_error_code,
    output reg O_heartbeat,
    output wire O_error_led
);
    reg [23:0] tick_count;
    reg [4:0] slot;
    reg [1:0] group_code;
    reg [7:0] frame_count;
    reg sof_d;
    assign O_error_led = (group_code!=0) && !slot[0] && (slot < {group_code,1'b0});
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            tick_count<=0; slot<=19; group_code<=0;
            frame_count<=0; O_heartbeat<=0; sof_d<=0;
        end else begin
            sof_d<=I_sof;
            if(I_sof && !sof_d) begin
                if(frame_count==HEARTBEAT_FRAMES-1) begin
                    frame_count<=0; O_heartbeat<=!O_heartbeat;
                end else frame_count<=frame_count+1'b1;
            end
            if(tick_count==TICK_CYCLES-1) begin
                tick_count<=0;
                if(slot==19) begin slot<=0; group_code<=I_error_code; end
                else slot<=slot+1'b1;
            end else tick_count<=tick_count+1'b1;
        end
    end
endmodule

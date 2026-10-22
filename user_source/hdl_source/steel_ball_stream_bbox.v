module steel_ball_stream_bbox #(
    parameter IMG_WIDTH = 1920,
    parameter IMG_HEIGHT = 1080,
    parameter MIN_AREA = 40,
    parameter BRIGHT_MIN = 80,
    parameter MAX_AREA = 200000,
    parameter CONFIRM_FRAMES = 2,
    parameter FILL_MIN_PERCENT = 15,
    parameter FILL_MAX_PERCENT = 100,
    parameter DOMINANCE_PERCENT = 40,
    parameter ASPECT_PERCENT = 50,
    parameter CLUSTER_RADIUS = 16,
    parameter MAX_JUMP = 220,
    parameter EDGE_MARGIN = 4,
    parameter MAX_DIM = 256,
    parameter MARK_HALF = 14,
    parameter MARK_AUTO = 0,
    parameter MARK_MIN_HALF = 64,
    parameter MARK_MAX_HALF = 192,
    parameter MARK_MARGIN = 8,
    parameter ROI_EN = 0,
    parameter CLUSTERS = 4,
    parameter TRACK_MISSES = 2,
    parameter TRACK_RADIUS = 48
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire [5:0] I_mask,
    input wire [4:0] I_meta,
    input wire [10:0] I_roi_left,
    input wire [10:0] I_roi_top,
    input wire [10:0] I_roi_right,
    input wire [10:0] I_roi_bottom,
    output reg O_frame_done,
    output reg O_frame_hit,
    output reg O_ball_detected,
    output reg O_bright,
    output reg O_box_valid,
    output reg [10:0] O_box_left,
    output reg [10:0] O_box_right,
    output reg [10:0] O_box_top,
    output reg [10:0] O_box_bottom,
    output reg [21:0] O_red_pixels,
    output reg [21:0] O_area,
    output reg [23:0] O_perimeter,
    output reg [10:0] O_xmin,
    output reg [10:0] O_xmax,
    output reg [10:0] O_ymin,
    output reg [10:0] O_ymax,
    output reg [3:0] O_geometry,
    output wire [11:0] O_lines,
    output reg O_bad_frame,
    output reg [1:0] O_error_code,
    output reg [10:0] O_ball_x,
    output reg [10:0] O_ball_y,
    output reg [3:0] O_confidence,
    output reg [2:0] O_cluster_count,
    output reg O_track_valid,
    output reg [10:0] O_track_x,O_track_y
);
    integer k, m;
    reg [2:0] mask_count;

    reg [11:0] x, y, lines;
    reg [21:0] red_count;
    reg seen_frame, bad_frame;
    assign O_lines = lines;

    reg token_sof;
    reg [11:0] base_x;
    reg [10:0] base_y;
    reg y_edge_ok, x_edge_ok, use_m;
    reg roi_ok;
    reg [2:0] rc_n, first_n, last_n;
    reg [3:0] ksum_n;
    always @* begin
        token_sof = I_meta[4];
        base_x = token_sof ? 12'd0 : x;
        base_y = token_sof ? 11'd0 : y[10:0];
        y_edge_ok = (base_y>=EDGE_MARGIN) && (base_y < IMG_HEIGHT-EDGE_MARGIN);
        x_edge_ok = (base_x>=EDGE_MARGIN) && (base_x <= IMG_WIDTH-EDGE_MARGIN-6);
        mask_count = 0;
        rc_n = 0; first_n = 0; last_n = 0; ksum_n = 0;
        for(k=0;k<6;k=k+1) begin
            roi_ok = !ROI_EN || ((base_x+{9'd0,k[2:0]}>=I_roi_left) &&
                (base_x+{9'd0,k[2:0]}<=I_roi_right) && (base_y>=I_roi_top) &&
                (base_y<=I_roi_bottom));
            use_m = (k<I_meta[2:0]) && I_mask[k] && y_edge_ok && x_edge_ok && roi_ok;
            if((k<I_meta[2:0]) && I_mask[k] && roi_ok)
                mask_count = mask_count+1'b1;
            if(use_m) begin
                rc_n = rc_n + 1'b1;
                if(rc_n==1) first_n = k[2:0];
                last_n = k[2:0];
                ksum_n = ksum_n + k[3:0];
            end
        end
    end

    reg [11:0] g_base;
    reg [10:0] g_y;
    reg [2:0] g_rc, g_first, g_last;
    reg [3:0] g_ksum;
    reg g_en;
    reg [11:0] t_first, t_last;
    reg [10:0] t_first_m, t_last_p, t_y_m, t_y_p;
    reg [31:0] t_sumx, t_sumy;
    always @* begin
        t_first = g_base + {9'd0,g_first};
        t_last = g_base + {9'd0,g_last};
        t_first_m = (t_first[11:0]>CLUSTER_RADIUS) ? (t_first[11:0]-CLUSTER_RADIUS) : 11'd0;
        t_last_p = t_last[10:0] + CLUSTER_RADIUS;
        t_y_m = (g_y>CLUSTER_RADIUS) ? (g_y-CLUSTER_RADIUS) : 11'd0;
        t_y_p = g_y + CLUSTER_RADIUS;
        t_sumx = g_base * {29'd0,g_rc} + {28'd0,g_ksum};
        t_sumy = {21'd0,g_rc} * {21'd0,g_y};
    end

    reg active[0:CLUSTERS-1];
    reg [10:0] c_xmin[0:CLUSTERS-1], c_xmax[0:CLUSTERS-1], c_ymin[0:CLUSTERS-1], c_ymax[0:CLUSTERS-1];
    reg [10:0] c_gxmin[0:CLUSTERS-1], c_gxmax[0:CLUSTERS-1], c_gymin[0:CLUSTERS-1], c_gymax[0:CLUSTERS-1];
    reg [21:0] c_cnt[0:CLUSTERS-1];
    reg [31:0] c_sumx[0:CLUSTERS-1], c_sumy[0:CLUSTERS-1];

    reg [2:0] target, free_index;
    reg target_found, free_found;
    reg [21:0] min01, min23;
    reg [1:0] i01, i23;
    reg [CLUSTERS-1:0] matchv;
    always @* begin
        for(m=0;m<CLUSTERS;m=m+1)
            matchv[m] = active[m] &&
                ({1'b0,g_base} >= {1'b0,c_gxmin[m]}) &&
                ({1'b0,g_base} <= {1'b0,c_gxmax[m]}) &&
                (g_y >= c_gymin[m]) &&
                (g_y <= c_gymax[m]);
        target_found = |matchv;
        if(matchv[0]) target = 2'd0;
        else if(matchv[1]) target = 2'd1;
        else if(matchv[2]) target = 2'd2;
        else target = 2'd3;
        free_found = 0; free_index = 0;
        for(m=CLUSTERS-1;m>=0;m=m-1)
            if(!active[m]) begin free_found = 1; free_index = m[2:0]; end
        if(!free_found) begin
            if(c_cnt[1]<c_cnt[0]) begin min01=c_cnt[1]; i01=2'd1; end
            else begin min01=c_cnt[0]; i01=2'd0; end
            if(c_cnt[3]<c_cnt[2]) begin min23=c_cnt[3]; i23=2'd3; end
            else begin min23=c_cnt[2]; i23=2'd2; end
            free_found = 1;
            free_index = (min23<min01) ? i23 : i01;
        end
    end

    reg scanning;
    reg [3:0] scan_idx;
    reg [21:0] scan_best;
    reg [31:0] scan_total;
    reg scan_found;
    reg [2:0] active_count;
    always @* begin
        active_count = 0;
        for(m=0;m<CLUSTERS;m=m+1)
            if(active[m] && c_cnt[m]!=0) active_count = active_count + 1'b1;
    end

    reg [3:0] eval_stage;
    reg sel_valid, sel_frame_ok, sel_bad;
    reg [21:0] sel_cnt, sel_total_red;
    reg [31:0] sel_clustered_total;
    reg [2:0] sel_clusters;
    reg [10:0] sel_xmin, sel_xmax, sel_ymin, sel_ymax;
    reg [31:0] sel_sumx, sel_sumy;
    reg [11:0] sel_w, sel_h;
    reg [23:0] sel_bbox, sel_perim;
    reg [31:0] sel_fill100, sel_bbox_min, sel_bbox_max;
    reg [17:0] sel_w100, sel_h100, sel_wlimit, sel_hlimit;
    reg [3:0] geometry_r;
    reg cand_r, cand_loose_r, qualified_r;
    reg [5:0] div_idx;
    reg [31:0] div_rem, div_quo, div_shr, div_den, div_tmp, div_qnext;
    reg [10:0] center_x, center_y;
    reg [10:0] mark_half_r;
    wire [11:0] mark_span = sel_w>=sel_h ? sel_w : sel_h;
    wire [11:0] mark_auto_half = ((mark_span+12'd1)>>1)+MARK_MARGIN;
    wire [11:0] mark_right = {1'b0,center_x}+{1'b0,mark_half_r};
    wire [11:0] mark_bottom = {1'b0,center_y}+{1'b0,mark_half_r};
    reg [10:0] prev_x, prev_y;
    reg prev_valid;
    reg [7:0] streak;
    reg [11:0] jump;
    reg jump_ok;
    reg [1:0] missed;
    wire [10:0] track_dx=center_x>=O_track_x ? center_x-O_track_x : O_track_x-center_x;
    wire [10:0] track_dy=center_y>=O_track_y ? center_y-O_track_y : O_track_y-center_y;
    wire local_recovery=O_track_valid && track_dx<=TRACK_RADIUS && track_dy<=TRACK_RADIUS;
    task tracking_miss;
        begin
            if(O_track_valid && missed<TRACK_MISSES)missed<=missed+1'b1;
            else begin O_track_valid<=0;missed<=0;streak<=0;prev_valid<=0;end
        end
    endtask

    reg w_area_ok, w_asp_ok, w_compact_ok, w_dom_ok, w_loose, w_full;
    always @* begin
        div_tmp = {div_rem[30:0], div_shr[31]};
        div_qnext = {div_quo[30:0], (div_tmp>=div_den)};
        if(center_x>=prev_x) jump = center_x - prev_x; else jump = prev_x - center_x;
        if(center_y>=prev_y) jump = jump + (center_y - prev_y); else jump = jump + (prev_y - center_y);
        jump_ok = !prev_valid || (jump<=MAX_JUMP);
        w_area_ok = (sel_cnt>=MIN_AREA) && (sel_cnt<=MAX_AREA);
        w_asp_ok = (sel_w100>=sel_hlimit) && (sel_h100>=sel_wlimit);
        w_compact_ok = (sel_w<=MAX_DIM) && (sel_h<=MAX_DIM);
        w_dom_ok = (sel_clustered_total!=0) && (sel_fill100 >= (sel_clustered_total*DOMINANCE_PERCENT));
        w_loose = sel_valid && sel_frame_ok && w_area_ok && w_asp_ok && w_compact_ok;
        w_full = w_loose && w_dom_ok;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            seen_frame<=0; bad_frame<=0; x<=0; y<=0; lines<=0; red_count<=0;
            g_base<=0; g_y<=0; g_rc<=0; g_first<=0; g_last<=0; g_ksum<=0; g_en<=0;
            scanning<=0; scan_idx<=0; scan_best<=0; scan_total<=0; scan_found<=0;
            eval_stage<=0;
            sel_valid<=0; sel_frame_ok<=0; sel_bad<=0;
            sel_cnt<=0; sel_total_red<=0; sel_clustered_total<=0; sel_clusters<=0;
            sel_xmin<=0; sel_xmax<=0; sel_ymin<=0; sel_ymax<=0;
            sel_sumx<=0; sel_sumy<=0;
            sel_w<=0; sel_h<=0; sel_bbox<=0; sel_perim<=0;
            sel_fill100<=0; sel_bbox_min<=0; sel_bbox_max<=0;
            sel_w100<=0; sel_h100<=0; sel_wlimit<=0; sel_hlimit<=0;
            geometry_r<=0; cand_r<=0; cand_loose_r<=0; qualified_r<=0;
            div_idx<=0; div_rem<=0; div_quo<=0; div_shr<=0; div_den<=0;
            center_x<=0; center_y<=0; mark_half_r<=MARK_HALF;
            prev_x<=0; prev_y<=0; prev_valid<=0; streak<=0;
            O_track_valid<=0;O_track_x<=0;O_track_y<=0;missed<=0;
            O_frame_done<=0; O_frame_hit<=0; O_ball_detected<=0; O_bright<=0;
            O_box_valid<=0; O_box_left<=0; O_box_right<=0; O_box_top<=0; O_box_bottom<=0;
            O_red_pixels<=0; O_area<=0; O_perimeter<=0;
            O_xmin<=0; O_xmax<=0; O_ymin<=0; O_ymax<=0;
            O_geometry<=0; O_bad_frame<=0; O_error_code<=0;
            O_ball_x<=0; O_ball_y<=0; O_confidence<=0; O_cluster_count<=0;
            for(m=0;m<CLUSTERS;m=m+1) begin
                active[m]<=0; c_cnt[m]<=0; c_sumx[m]<=0; c_sumy[m]<=0;
                c_xmin[m]<=11'h7ff; c_xmax[m]<=0; c_ymin[m]<=11'h7ff; c_ymax[m]<=0;
                c_gxmin[m]<=11'h7ff; c_gxmax[m]<=0; c_gymin[m]<=11'h7ff; c_gymax[m]<=0;
            end
        end else begin
            O_frame_done <= 0;
            g_base <= base_x; g_y <= base_y; g_rc <= rc_n; g_first <= first_n;
            g_last <= last_n; g_ksum <= ksum_n; g_en <= seen_frame && !I_meta[4];

            if(I_meta[4]) begin
                if(seen_frame) begin
                    sel_frame_ok <= !bad_frame && (lines==IMG_HEIGHT);
                    sel_bad <= bad_frame;
                    sel_total_red <= red_count;
                    scan_idx <= 0; scan_best <= 0; scan_total <= 0; scan_found <= 0;
                    scanning <= 1;
                end
                seen_frame <= 1;
                red_count <= mask_count;
                x <= I_meta[3] ? 12'd0 : {9'd0,I_meta[2:0]};
                y <= I_meta[3] ? 12'd1 : 12'd0;
                lines <= I_meta[3] ? 12'd1 : 12'd0;
                bad_frame <= I_meta[3] && I_meta[2:0]!=IMG_WIDTH;
            end else begin
                if(seen_frame) begin
                    red_count <= red_count + mask_count;
                    if(I_meta[3]) begin
                        if(x+I_meta[2:0]!=IMG_WIDTH || lines>=IMG_HEIGHT) bad_frame<=1;
                        x<=0; y<=y+1'b1; lines<=lines+1'b1;
                    end else if(I_meta[2:0]!=0) begin
                        if(x+I_meta[2:0]>IMG_WIDTH) bad_frame<=1;
                        x<=x+I_meta[2:0];
                    end
                end
                if(scanning) begin
                    if(scan_idx < CLUSTERS) begin
                        if(active[scan_idx] && c_cnt[scan_idx]>=MIN_AREA &&
                           (!scan_found || c_cnt[scan_idx]>scan_best)) begin
                            scan_best <= c_cnt[scan_idx]; scan_found <= 1;
                            sel_xmin <= c_xmin[scan_idx]; sel_xmax <= c_xmax[scan_idx];
                            sel_ymin <= c_ymin[scan_idx]; sel_ymax <= c_ymax[scan_idx];
                            sel_sumx <= c_sumx[scan_idx]; sel_sumy <= c_sumy[scan_idx];
                        end
                        if(active[scan_idx])
                            scan_total <= scan_total + c_cnt[scan_idx];
                        scan_idx <= scan_idx + 1'b1;
                    end else begin
                        sel_valid <= scan_found;
                        sel_cnt <= scan_best;
                        sel_clustered_total <= scan_total;
                        sel_clusters <= active_count;
                        for(m=0;m<CLUSTERS;m=m+1) begin
                            active[m]<=0; c_cnt[m]<=0; c_sumx[m]<=0; c_sumy[m]<=0;
                            c_xmin[m]<=11'h7ff; c_xmax[m]<=0; c_ymin[m]<=11'h7ff; c_ymax[m]<=0;
                            c_gxmin[m]<=11'h7ff; c_gxmax[m]<=0; c_gymin[m]<=11'h7ff; c_gymax[m]<=0;
                        end
                        scanning <= 0;
                        eval_stage <= 1;
                    end
                end
            end

            if(g_en && g_rc!=0 && !I_meta[4]) begin
                for(m=0;m<CLUSTERS;m=m+1) begin
                    if(target_found && target==m[2:0]) begin
                        active[m]<=1;
                        c_cnt[m]<=c_cnt[m]+g_rc;
                        c_sumx[m]<=c_sumx[m]+t_sumx;
                        c_sumy[m]<=c_sumy[m]+t_sumy;
                        if(t_first<c_xmin[m]) c_xmin[m]<=t_first;
                        if(t_last>c_xmax[m]) c_xmax[m]<=t_last;
                        if(g_y[10:0]<c_ymin[m]) c_ymin[m]<=g_y[10:0];
                        if(g_y[10:0]>c_ymax[m]) c_ymax[m]<=g_y[10:0];
                        if(t_first_m<c_gxmin[m]) c_gxmin[m]<=t_first_m;
                        if(t_last_p>c_gxmax[m]) c_gxmax[m]<=t_last_p;
                        if(t_y_m<c_gymin[m]) c_gymin[m]<=t_y_m;
                        if(t_y_p>c_gymax[m]) c_gymax[m]<=t_y_p;
                    end else if(!target_found && free_index==m[2:0]) begin
                        active[m]<=1;
                        c_cnt[m]<=g_rc;
                        c_sumx[m]<=t_sumx;
                        c_sumy[m]<=t_sumy;
                        c_xmin[m]<=t_first; c_xmax[m]<=t_last;
                        c_ymin[m]<=g_y[10:0]; c_ymax[m]<=g_y[10:0];
                        c_gxmin[m]<=t_first_m; c_gxmax[m]<=t_last_p;
                        c_gymin[m]<=t_y_m; c_gymax[m]<=t_y_p;
                    end
                end
            end

            if(!I_meta[4] && eval_stage!=0) begin
                case(eval_stage)
                    1: begin
                        sel_w <= sel_xmax - sel_xmin + 11'd1;
                        sel_h <= sel_ymax - sel_ymin + 11'd1;
                        eval_stage <= 2;
                    end
                    2: begin
                        sel_bbox <= sel_w * sel_h;
                        sel_perim <= (({12'd0,sel_w} + {12'd0,sel_h}) << 1);
                        sel_fill100 <= sel_cnt * 32'd100;
                        sel_w100 <= sel_w * 18'd100;
                        sel_h100 <= sel_h * 18'd100;
                        sel_wlimit <= sel_w * ASPECT_PERCENT;
                        sel_hlimit <= sel_h * ASPECT_PERCENT;
                        eval_stage <= 3;
                    end
                    3: begin
                        sel_bbox_min <= sel_bbox * FILL_MIN_PERCENT;
                        sel_bbox_max <= sel_bbox * FILL_MAX_PERCENT;
                        if(MARK_AUTO) begin
                            if(mark_auto_half<MARK_MIN_HALF) mark_half_r<=MARK_MIN_HALF;
                            else if(mark_auto_half>MARK_MAX_HALF) mark_half_r<=MARK_MAX_HALF;
                            else mark_half_r<=mark_auto_half[10:0];
                        end else mark_half_r<=MARK_HALF;
                        eval_stage <= 4;
                    end
                    4: begin
                        geometry_r <= sel_valid ? {w_compact_ok,w_asp_ok,w_dom_ok,w_area_ok} : 4'd0;
                        cand_loose_r <= w_loose;
                        cand_r <= w_full;
                        qualified_r <= w_full;
                        O_red_pixels <= sel_total_red;
                        O_area <= sel_valid ? sel_cnt : 22'd0;
                        O_perimeter <= sel_valid ? sel_perim : 24'd0;
                        O_xmin <= sel_valid ? sel_xmin : 11'd0;
                        O_xmax <= sel_valid ? sel_xmax : 11'd0;
                        O_ymin <= sel_valid ? sel_ymin : 11'd0;
                        O_ymax <= sel_valid ? sel_ymax : 11'd0;
                        O_geometry <= sel_valid ? {w_compact_ok,w_asp_ok,w_dom_ok,w_area_ok} : 4'd0;
                        O_bright <= sel_total_red>=BRIGHT_MIN;
                        O_bad_frame <= sel_bad;
                        O_error_code <= sel_bad ? 2'd3 : 2'd0;
                        O_cluster_count <= sel_clusters;
                        O_frame_hit <= w_full;
                        if(sel_valid && sel_frame_ok && (sel_cnt>=MIN_AREA) && (sel_cnt<=MAX_AREA)) begin
                            div_den <= {10'd0,sel_cnt};
                            div_shr <= sel_sumx;
                            div_rem <= 0; div_quo <= 0; div_idx <= 6'd32;
                            eval_stage <= 5;
                        end else begin
                            cand_loose_r <= 0; cand_r <= 0; qualified_r <= 0;
                            O_ball_detected <= 0; O_ball_x <= 0; O_ball_y <= 0; O_confidence <= 0;
                            O_box_valid <= 0; O_box_left<=0; O_box_right<=0;
                            O_box_top<=0; O_box_bottom<=0;
                            tracking_miss;
                            O_frame_done <= 1;
                            eval_stage <= 0;
                        end
                    end
                    5: begin
                        div_rem <= (div_tmp>=div_den) ? (div_tmp-div_den) : div_tmp;
                        div_quo <= div_qnext;
                        div_shr <= {div_shr[30:0],1'b0};
                        if(div_idx<=6'd1) begin
                            center_x <= div_qnext[10:0];
                            div_shr <= sel_sumy; div_rem <= 0; div_quo <= 0;
                            div_idx <= 6'd32; eval_stage <= 6;
                        end else div_idx <= div_idx-1'b1;
                    end
                    6: begin
                        div_rem <= (div_tmp>=div_den) ? (div_tmp-div_den) : div_tmp;
                        div_quo <= div_qnext;
                        div_shr <= {div_shr[30:0],1'b0};
                        if(div_idx<=6'd1) begin
                            center_y <= div_qnext[10:0];
                            eval_stage <= 7;
                        end else div_idx <= div_idx-1'b1;
                    end
                    7: begin
                        if(cand_r && jump_ok && (!O_track_valid || missed==0 || local_recovery)) begin
                            missed<=0;
                            if(streak>=CONFIRM_FRAMES-1 || local_recovery) begin
                                O_ball_detected<=1; O_confidence<=4'hF;
                                O_track_valid<=1;O_track_x<=center_x;O_track_y<=center_y;
                            end else begin
                                O_ball_detected<=0; O_confidence<=4'h8; streak<=streak+1'b1;
                            end
                            prev_x<=center_x; prev_y<=center_y; prev_valid<=1;
                        end else begin
                            O_ball_detected<=0;tracking_miss;
                            O_confidence<= cand_loose_r ? 4'h4 : 4'h0;
                            if(!O_track_valid && cand_loose_r && jump_ok) begin
                                prev_x<=center_x; prev_y<=center_y; prev_valid<=1;
                            end
                        end
                        O_ball_x <= cand_loose_r ? center_x : 11'd0;
                        O_ball_y <= cand_loose_r ? center_y : 11'd0;
                        O_box_valid <= cand_loose_r;
                        O_box_left <= (center_x>mark_half_r) ? center_x-mark_half_r : 11'd0;
                        O_box_right <= (mark_right<IMG_WIDTH) ? mark_right[10:0] : IMG_WIDTH-1;
                        O_box_top <= (center_y>mark_half_r) ? center_y-mark_half_r : 11'd0;
                        O_box_bottom <= (mark_bottom<IMG_HEIGHT) ? mark_bottom[10:0] : IMG_HEIGHT-1;
                        O_frame_done<=1;
                        eval_stage<=0;
                    end
                    default: eval_stage<=0;
                endcase
            end
        end
    end
endmodule

`timescale 1ns/1ps
module tb_detector;
    reg clk=0, rst_n=0;
    always #5 clk=~clk;
    reg [5:0] mask=0;
    reg [4:0] meta=0;
    wire frame_done, frame_hit, ball, bright, box_valid, bad_frame;
    wire [10:0] box_left, box_right, box_top, box_bottom;
    wire [21:0] red_pixels, area;
    wire [23:0] perimeter;
    wire [10:0] xmin, xmax, ymin, ymax, ball_x, ball_y;
    wire [3:0] geometry, confidence;
    wire [11:0] lines;
    wire [2:0] clusters;
    wire [1:0] error_code;
    integer y, group_index, phase, p, px, count, fr;
    reg [5:0] token_mask;
    reg [10:0] roi_right=1279,roi_bottom=719;

    steel_ball_stream_bbox #(
        .IMG_WIDTH(1280), .IMG_HEIGHT(720),
        .MIN_AREA(8), .BRIGHT_MIN(20), .MAX_AREA(90000),
        .CONFIRM_FRAMES(2), .FILL_MIN_PERCENT(15), .FILL_MAX_PERCENT(100),
        .DOMINANCE_PERCENT(40), .ASPECT_PERCENT(35),
        .CLUSTER_RADIUS(8), .MAX_JUMP(147), .EDGE_MARGIN(2), .MAX_DIM(267),
        .MARK_HALF(9), .MARK_AUTO(1), .MARK_MIN_HALF(43),
        .MARK_MAX_HALF(128), .MARK_MARGIN(5), .ROI_EN(1)
    ) dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_mask(mask), .I_meta(meta),
        .I_roi_left(11'd0),.I_roi_top(11'd0),.I_roi_right(roi_right),.I_roi_bottom(roi_bottom),
        .O_frame_done(frame_done), .O_frame_hit(frame_hit),
        .O_ball_detected(ball), .O_bright(bright), .O_box_valid(box_valid),
        .O_box_left(box_left), .O_box_right(box_right),
        .O_box_top(box_top), .O_box_bottom(box_bottom),
        .O_red_pixels(red_pixels), .O_area(area), .O_perimeter(perimeter),
        .O_xmin(xmin), .O_xmax(xmax), .O_ymin(ymin), .O_ymax(ymax),
        .O_geometry(geometry), .O_lines(lines),
        .O_bad_frame(bad_frame), .O_error_code(error_code),
        .O_ball_x(ball_x), .O_ball_y(ball_y),
        .O_confidence(confidence), .O_cluster_count(clusters)
    );

    reg dest_clk=0; always #10 dest_clk=~dest_clk;
    wire [10:0] display_x=(ball_x*3)>>1,display_y=(ball_y*3)>>1;
    wire [240:0] received; wire received_frame;
    vision_cdc_bridge #(.WIDTH(241)) bridge(
        clk,rst_n,frame_done,{213'd0,display_x,display_y,ball,(bad_frame | (error_code!=0)),confidence},
        dest_clk,rst_n,received_frame,received);
    wire valid; wire signed [7:0] target;wire [10:0] shadow_x,shadow_y;
    vision_shadow_target #(.MS_CYCLES(20)) shadow(
        .clk(dest_clk),.rst_n(rst_n),.frame(received_frame),.ball_x(received[27:17]),.ball_y(received[16:6]),
        .confirmed(received[5]),.bad_frame(received[4]),.confidence(received[3:0]),
        .center(16'd960),.deadzone(16'd20),.gain(8'd7),.limit(8'd5),
        .x(shadow_x),.y(shadow_y),.valid(valid),.target(target));
    initial begin
        repeat(4) @(negedge clk); rst_n=1;
        @(negedge clk); meta=5'b10000;
        for(fr=0;fr<7;fr=fr+1) begin
            if(fr==5)begin roi_right=500;roi_bottom=400;end
            for(y=0;y<720;y=y+1) begin
                for(group_index=0;group_index<80;group_index=group_index+1) begin
                    for(phase=0;phase<3;phase=phase+1) begin
                        count=phase==2 ? 6 : 5;
                        token_mask=0;
                        for(p=0;p<count;p=p+1) begin
                            px=group_index*16+(phase==0 ? 0 : phase==1 ? 5 : 10)+p;
                            if(fr<2 && (px-1000)*(px-1000)+(y-500)*(y-500)<=25)
                                token_mask[p]=1;
                            if(fr>=3 && px>=999 && px<=1001 && y>=499 && y<=501)
                                token_mask[p]=1;
                        end
                        @(negedge clk);
                        mask=token_mask;
                        meta={1'b0,(group_index==79 && phase==2),count[2:0]};
                    end
                end
            end
            @(negedge clk); mask=0; meta=5'b10000;
            @(negedge clk); meta=0;
            wait(frame_done); #1;
            if(fr==0 && (!frame_hit || !box_valid || ball || red_pixels!=81 || area!=81))
                $fatal(1,"720p candidate failed area=%0d ball=%b",area,ball);
            if(fr==1 && (!frame_hit || !box_valid || !ball || ball_x!=1000 || ball_y!=500 || geometry!=4'hf))
                $fatal(1,"720p confirmation failed center=(%0d,%0d) geometry=%h",ball_x,ball_y,geometry);
            if(fr==2 && (frame_hit || box_valid || ball || area!=0 || ball_x!=0 || ball_y!=0 || confidence!=0))
                $fatal(1,"720p loss failed area=%0d center=(%0d,%0d)",area,ball_x,ball_y);
            if(fr==3 && (!frame_hit || !box_valid || !ball || area!=9 || ball_x!=1000 || ball_y!=500))
                $fatal(1,"720p weak candidate failed area=%0d center=(%0d,%0d)",area,ball_x,ball_y);
            if(fr==4 && (!frame_hit || !box_valid || !ball || area!=9 || ball_x!=1000 || ball_y!=500))
                $fatal(1,"720p weak confirmation failed area=%0d center=(%0d,%0d)",area,ball_x,ball_y);
            if(fr>=5 && (ball || area!=0 || box_valid))$fatal(1,"ROI-excluded ball still detected");
            repeat(20) @(negedge dest_clk);
            if(valid!==ball || (ball && (shadow_x!=1500 || shadow_y!=750 || target!=4)) || (!ball && target!=0))
                $fatal(1,"completed-frame CDC/scale/target failed frame=%0d x=%0d y=%0d valid=%b target=%0d",fr,shadow_x,shadow_y,valid,target);
            if(bad_frame || error_code!=0)
                $fatal(1,"720p frame shape failed error=%0d",error_code);
        end
        $display("PASS 720p small ball, loss clear, weak nine-pixel ball");
        repeat(4010) @(negedge dest_clk);
        if(valid || target!=0)$fatal(1,"detector results stopped but target remained valid");
        $display("DETECTOR_INTEGRATION_PASS: full 720p pixels, confirmation/loss/weak ball, changed ROI excludes outside ball, completed-result CDC, scale once and watchdog");
        $finish;
    end
endmodule

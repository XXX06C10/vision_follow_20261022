"""Retain the reference 720p pixel fixture, add the actual completed-result CDC and target path."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1]
reference=root.parent/'lab_hd_1_mipi_hdmi_1080p_steel_ball_motor_manual/steel_ball_verification/tb_stream_bbox_720p.v'
text=reference.read_text().replace('module tb_stream_bbox_720p;','module tb_detector;')
text=text.replace('    reg [5:0] token_mask;', '    reg [5:0] token_mask;\n    reg [10:0] roi_right=1279,roi_bottom=719;')
text=text.replace('.MARK_MAX_HALF(128), .MARK_MARGIN(5)', '.MARK_MAX_HALF(128), .MARK_MARGIN(5), .ROI_EN(1)')
text=text.replace('.I_clk(clk), .I_rst_n(rst_n), .I_mask(mask), .I_meta(meta),', '.I_clk(clk), .I_rst_n(rst_n), .I_mask(mask), .I_meta(meta),\n        .I_roi_left(11\'d0),.I_roi_top(11\'d0),.I_roi_right(roi_right),.I_roi_bottom(roi_bottom),')
text=text.replace('fr<5;','fr<7;').replace('            for(y=0;y<720;', '            if(fr==5)begin roi_right=500;roi_bottom=400;end\n            for(y=0;y<720;')
text=text.replace('            if(bad_frame || error_code!=0)', '''            if(fr>=5 && (ball || area!=0 || box_valid))$fatal(1,"ROI-excluded ball still detected");
            if(bad_frame || error_code!=0)''')
text=text.replace('    initial begin','''    reg dest_clk=0; always #10 dest_clk=~dest_clk;
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
    initial begin''',1)
text=text.replace('            if(bad_frame || error_code!=0)', '''            repeat(20) @(negedge dest_clk);
            if(valid!==ball || (ball && (shadow_x!=1500 || shadow_y!=750 || target!=4)) || (!ball && target!=0))
                $fatal(1,"completed-frame CDC/scale/target failed frame=%0d x=%0d y=%0d valid=%b target=%0d",fr,shadow_x,shadow_y,valid,target);
            if(bad_frame || error_code!=0)''')
text=text.replace('        $finish;','''        repeat(4010) @(negedge dest_clk);
        if(valid || target!=0)$fatal(1,"detector results stopped but target remained valid");
        $display("DETECTOR_INTEGRATION_PASS: full 720p pixels, confirmation/loss/weak ball, changed ROI excludes outside ball, completed-result CDC, scale once and watchdog");
        $finish;''')
(root/'sim/tb_detector.v').write_text(text)
(root/'sim/detector_fixture_lineage.json').write_text(json.dumps({'reference':str(reference),'sha256':hashlib.sha256(reference.read_bytes()).hexdigest()},indent=2))

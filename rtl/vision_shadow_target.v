module vision_shadow_target #(parameter integer MS_CYCLES=50000,STALE_MS=200)(
    input wire clk,rst_n,frame,
    input wire [10:0] ball_x,ball_y,
    input wire confirmed,bad_frame,
    input wire [3:0] confidence,
    input wire [15:0] center,deadzone,
    input wire [7:0] gain,limit,
    output reg [10:0] x=0,y=0,
    output wire valid,
    output reg [15:0] age_ms=16'hffff,frames=0,
    output wire signed [7:0] target,
    output reg [3:0] frame_confidence=0,
    output reg frame_bad=0
);
    reg [31:0] ms_count=0;
    reg accepted=0;
    always @(posedge clk) begin
        if(!rst_n)begin x<=0;y<=0;age_ms<=16'hffff;frames<=0;ms_count<=0;accepted<=0;frame_confidence<=0;frame_bad<=0;end
        else begin
            if(ms_count==MS_CYCLES-1) begin
                ms_count<=0;if(age_ms!=16'hffff)age_ms<=age_ms+1'b1;
            end else ms_count<=ms_count+1'b1;
            if(frame)begin
                x<=ball_x;y<=ball_y;age_ms<=0;ms_count<=0;frames<=frames+1'b1;
                frame_confidence<=confidence;frame_bad<=bad_frame;
                accepted<=confirmed && !bad_frame && confidence>=8 && ball_x<=1919 && ball_y<=1079;
            end
        end
    end
    assign valid=accepted && age_ms<STALE_MS;
    wire signed [16:0] difference=$signed({6'b0,x})-$signed({1'b0,center});
    wire [16:0] magnitude=difference[16] ? -difference : difference;
    wire [16:0] outside=(magnitude>deadzone ? magnitude-deadzone : 17'd0);
    wire [16:0] scaled=outside>>gain[6:0];
    wire [7:0] bounded=(scaled>limit ? limit : scaled[7:0]);
    wire negative=difference[16]^gain[7];
    assign target=!valid ? 8'sd0 : (negative ? -$signed(bounded) : $signed(bounded));
endmodule

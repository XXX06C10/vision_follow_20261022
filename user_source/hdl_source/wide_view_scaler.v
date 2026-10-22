module wide_view_scaler (
    input wire I_clk,
    input wire I_rst_n,
    input wire I_frame_sync,
    input wire I_video_de,
    input wire [23:0] I_source_data,
    input wire I_source_valid,
    output wire O_source_rd_en,
    output wire [23:0] O_pixel
);
    reg [23:0] line0 [0:1279];
    reg [23:0] line1 [0:1279];
    reg [23:0] read0, read1;
    reg read_bank, read_valid;
    reg frame_sync_d, de_d;
    reg [11:0] startup;
    reg startup_done;
    reg fetching;
    reg fetch_bank;
    reg [9:0] fetch_line;
    reg [10:0] fetch_x;
    reg req_d1, req_d2;
    reg req_bank_d1, req_bank_d2;
    reg [10:0] req_x_d1, req_x_d2;
    reg req_last_d1, req_last_d2;
    reg bank0_ready, bank1_ready;
    reg [9:0] bank0_line, bank1_line;
    reg [9:0] src_y;
    reg [1:0] y_phase;
    reg [10:0] src_x;
    reg [1:0] x_phase;
    wire frame_start = I_frame_sync && !frame_sync_d;
    wire line_start = I_video_de && !de_d;
    wire line_end = !I_video_de && de_d;
    wire [9:0] next_line = src_y + 10'd1;
    wire next_bank_ready = src_y[0] ? bank0_ready : bank1_ready;
    wire [9:0] next_bank_line = src_y[0] ? bank0_line : bank1_line;
    assign O_source_rd_en = fetching;
    assign O_pixel = read_valid ? (read_bank ? read1 : read0) : 24'd0;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            frame_sync_d <= 0;
            de_d <= 0;
            startup <= 0;
            startup_done <= 0;
            fetching <= 0;
            fetch_bank <= 0;
            fetch_line <= 0;
            fetch_x <= 0;
            req_d1 <= 0;
            req_d2 <= 0;
            req_bank_d1 <= 0;
            req_bank_d2 <= 0;
            req_x_d1 <= 0;
            req_x_d2 <= 0;
            req_last_d1 <= 0;
            req_last_d2 <= 0;
            bank0_ready <= 0;
            bank1_ready <= 0;
            bank0_line <= 0;
            bank1_line <= 0;
            src_y <= 0;
            y_phase <= 0;
            src_x <= 0;
            x_phase <= 0;
        end else begin
            frame_sync_d <= I_frame_sync;
            de_d <= I_video_de;
            req_d1 <= O_source_rd_en;
            req_d2 <= req_d1;
            req_bank_d1 <= fetch_bank;
            req_bank_d2 <= req_bank_d1;
            req_x_d1 <= fetch_x;
            req_x_d2 <= req_x_d1;
            req_last_d1 <= O_source_rd_en && fetch_x == 11'd1279;
            req_last_d2 <= req_last_d1;
            if(frame_start) begin
                startup <= 0;
                startup_done <= 0;
                fetching <= 0;
                bank0_ready <= 0;
                bank1_ready <= 0;
                src_y <= 0;
                y_phase <= 0;
                src_x <= 0;
                x_phase <= 0;
                req_d1 <= 0;
                req_d2 <= 0;
            end else begin
                if(!startup_done) begin
                    if(startup == 12'd2047) begin
                        startup_done <= 1;
                        fetching <= 1;
                        fetch_bank <= 0;
                        fetch_line <= 0;
                        fetch_x <= 0;
                    end else startup <= startup + 1'b1;
                end
                if(fetching) begin
                    if(fetch_x == 11'd1279) fetching <= 0;
                    else fetch_x <= fetch_x + 1'b1;
                end
                if(line_start && next_line < 10'd720 && !fetching &&
                   (!next_bank_ready || next_bank_line != next_line)) begin
                    fetching <= 1;
                    fetch_bank <= ~src_y[0];
                    fetch_line <= next_line;
                    fetch_x <= 0;
                    if(src_y[0]) bank0_ready <= 0;
                    else bank1_ready <= 0;
                end
                if(line_end) begin
                    src_x <= 0;
                    x_phase <= 0;
                    if(y_phase == 2'd2) y_phase <= 0;
                    else y_phase <= y_phase + 1'b1;
                    if(y_phase != 0) src_y <= src_y + 1'b1;
                end else if(I_video_de) begin
                    if(x_phase == 2'd2) x_phase <= 0;
                    else x_phase <= x_phase + 1'b1;
                    if(x_phase != 0) src_x <= src_x + 1'b1;
                end
                if(req_d2 && I_source_valid && req_last_d2) begin
                    if(req_bank_d2) begin
                        bank1_ready <= 1;
                        bank1_line <= fetch_line;
                    end else begin
                        bank0_ready <= 1;
                        bank0_line <= fetch_line;
                    end
                end
            end
        end
    end

    always @(posedge I_clk) begin
        if(req_d2 && I_source_valid) begin
            if(req_bank_d2) line1[req_x_d2] <= I_source_data;
            else line0[req_x_d2] <= I_source_data;
        end
        read0 <= line0[src_x];
        read1 <= line1[src_x];
        read_bank <= src_y[0];
        read_valid <= I_video_de && (src_y[0] ?
            bank1_ready && bank1_line == src_y :
            bank0_ready && bank0_line == src_y);
    end
endmodule

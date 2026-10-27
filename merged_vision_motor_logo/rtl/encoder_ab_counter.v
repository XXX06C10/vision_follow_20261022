module encoder_ab_counter(
    input wire clk,
    input wire rst_n,
    input wire [1:0] ab_async,
    output reg signed [31:0] count,
    output reg [15:0] errors,
    output wire [1:0] ab_state
);
    reg [1:0] sync1=0,sync2=0,previous=0;
    reg [3:0] settle=0;
    reg ready=0;
    assign ab_state=sync2;
    always @(posedge clk) begin
        if(!rst_n) begin
            sync1<=0; sync2<=0; previous<=0; settle<=0; ready<=0;
            count<=0; errors<=0;
        end else begin
            sync1<=ab_async; sync2<=sync1;
            previous<=sync2;
            if(!ready) begin
                if(settle==15) ready<=1;
                else settle<=settle+1'b1;
            end else begin
                case({previous,sync2})
                    4'b0001,4'b0111,4'b1110,4'b1000: count<=count+32'sd1;
                    4'b0010,4'b1011,4'b1101,4'b0100: count<=count-32'sd1;
                    4'b0011,4'b0110,4'b1001,4'b1100:
                        if(errors!=16'hffff) errors<=errors+1'b1;
                    default: begin end
                endcase
            end
        end
    end
endmodule


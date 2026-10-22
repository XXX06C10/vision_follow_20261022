// Coherent held-data request/ack crossing. Both reset inputs share assertion.
module vision_cdc_bridge #(parameter WIDTH=241)(
    input wire src_clk,src_rst_n,src_frame,
    input wire [WIDTH-1:0] src_data,
    input wire dst_clk,dst_rst_n,
    output reg dst_frame=0,
    output reg [WIDTH-1:0] dst_data=0
);
    reg [WIDTH-1:0] held=0;
    reg request=0,ack=0,seen=0;
    reg [1:0] ack_sync=0,request_sync=0;
    always @(posedge src_clk or negedge src_rst_n) begin
        if(!src_rst_n) begin held<=0;request<=0;ack_sync<=0;end
        else begin
            ack_sync<={ack_sync[0],ack};
            if(src_frame && request==ack_sync[1]) begin held<=src_data;request<=~request;end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if(!dst_rst_n) begin request_sync<=0;seen<=0;ack<=0;dst_frame<=0;dst_data<=0;end
        else begin
            request_sync<={request_sync[0],request};dst_frame<=0;
            if(request_sync[1]!=seen) begin
                dst_data<=held;seen<=request_sync[1];ack<=request_sync[1];dst_frame<=1;
            end
        end
    end
endmodule

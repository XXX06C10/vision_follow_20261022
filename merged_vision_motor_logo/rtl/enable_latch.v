// Physical K1 is a debounced toggle. K4 overrides it immediately and clears it.
// A qualified release must precede every accepted K1 press, including at boot,
// after K4 and after a fault. No automatic enable from a held key.
module enable_latch #(parameter integer DEBOUNCE_CYCLES=1000000,HOLD_ON_FAULT=0)(
    input wire clk,rst_n,k1_n,k4_n,inhibit,
    output wire enable_request
);
    reg sync1=0,sync2=0,candidate=0,stable_pressed=0;
    reg stable_valid=0,armed=0,latched=0;
    reg [31:0] debounce_count=0;
    assign enable_request=rst_n & k4_n & (HOLD_ON_FAULT || !inhibit) & latched;
    always @(posedge clk) begin
        if(!rst_n || !k4_n || (inhibit && !HOLD_ON_FAULT)) begin
            sync1<=0; sync2<=0; candidate<=0; stable_pressed<=0;
            stable_valid<=0; armed<=0; latched<=0; debounce_count<=0;
        end else begin
            sync1<=~k1_n; sync2<=sync1;
            if(sync2!=candidate) begin
                candidate<=sync2; debounce_count<=0; stable_valid<=0;
            end else if(debounce_count<DEBOUNCE_CYCLES-1) begin
                debounce_count<=debounce_count+1'b1;
            end else begin stable_pressed<=candidate; stable_valid<=1; end
            if(stable_valid) begin
                if(!stable_pressed) armed<=1;
                else if(armed) begin
                    // A fault permits releasing an existing hold, never re-enabling.
                    if(!inhibit || latched)latched<=~latched;
                    armed<=0;
                end
            end
        end
    end
endmodule

`timescale 1ns/1ps
//============================================================================
// logo_rom.v
//   146x94 Anlogic logo image (25-bit: {valid, R, G, B}), stored in block RAM.
//   Data is generated from the lab_hd_3_osd anlogic_logo_rom.v image.
//============================================================================
module logo_rom (
    input  wire        I_clk,
    input  wire [13:0] I_addr,
    output reg  [24:0] O_pixel
);

    (* ram_style = "block" *) reg [24:0] mem [0:13723];

    initial begin
        $readmemh("logo_rom.hex", mem);
    end

    always @(posedge I_clk) begin
        O_pixel <= mem[I_addr];
    end

endmodule

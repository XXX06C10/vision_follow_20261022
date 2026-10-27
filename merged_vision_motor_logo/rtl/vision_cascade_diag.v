// D1 compact diagnostic, 94 bytes,20ms sample opportunity; no serial motion commands.
module vision_cascade_diag #(parameter UART_DIV=434,MS_CYCLES=50000,PERIOD_MS=20,WIDE_RANGE=0)(
 input wire clk,rst_n,enabled,grant,
 input wire [31:0] time_ms,
 input wire [31:0] frame_ms,
 input wire [15:0] config_seq,
 input wire [7:0] selection,
 input wire [15:0] flags,
 input wire [7:0] cal_phase,
 input wire [7:0] cal_status,
 input wire [15:0] x,
 input wire [15:0] center,
 input wire [31:0] position,
 input wire [31:0] reference_q8,
 input wire [31:0] velocity_q4,
 input wire [15:0] frequency_hz,
 input wire [7:0] fault,
 input wire [7:0] spent,
 input wire [31:0] command_position,
 input wire [15:0] frame_number,
 input wire [(WIDE_RANGE ? 31 : 7):0] neutral,
 input wire [31:0] limit_anchor,command_anchor,work_min,work_max,feedback_min,feedback_max,
 input wire [15:0] config_errors,
 output wire tx,output reg busy=0
);
 localparam BYTES=WIDE_RANGE ? 71 : 44;
 wire [BYTES*8-1:0] payload=WIDE_RANGE ? {time_ms,frame_ms,config_seq,selection,flags,cal_phase,cal_status,x,center,position,reference_q8,velocity_q4,frequency_hz,fault,spent,command_position,frame_number,neutral,config_errors,limit_anchor,command_anchor,work_min,work_max,feedback_min,feedback_max} : {time_ms,frame_ms,config_seq,selection,flags,cal_phase,cal_status,x,center,position,reference_q8,velocity_q4,frequency_hz,fault,spent,command_position,frame_number,neutral,config_errors};
 reg [BYTES*8-1:0] snapshot=0;reg [7:0] checksum_snapshot=0;reg [7:0] checksum;integer j;
 always @*begin checksum=0;for(j=0;j<BYTES;j=j+1)checksum=checksum^payload[j*8+:8];end
 reg [31:0] timer=0;reg owed=0,bit_active=0;reg [(WIDE_RANGE ? 7 : 6):0] index=0;reg [3:0] bit_index=0;
 reg [15:0] baud=0;reg [9:0] shift=1023;
 assign tx=shift[0];
 function [7:0] hex_char;input [3:0] v;begin hex_char=v<10 ? "0"+v : "A"+v-10;end endfunction
 function [7:0] wide_message_byte;input [7:0] at;begin case(at)
0:wide_message_byte="D";
1:wide_message_byte="2";
2:wide_message_byte=hex_char(snapshot[567-:4]);
3:wide_message_byte=hex_char(snapshot[563-:4]);
4:wide_message_byte=hex_char(snapshot[559-:4]);
5:wide_message_byte=hex_char(snapshot[555-:4]);
6:wide_message_byte=hex_char(snapshot[551-:4]);
7:wide_message_byte=hex_char(snapshot[547-:4]);
8:wide_message_byte=hex_char(snapshot[543-:4]);
9:wide_message_byte=hex_char(snapshot[539-:4]);
10:wide_message_byte=hex_char(snapshot[535-:4]);
11:wide_message_byte=hex_char(snapshot[531-:4]);
12:wide_message_byte=hex_char(snapshot[527-:4]);
13:wide_message_byte=hex_char(snapshot[523-:4]);
14:wide_message_byte=hex_char(snapshot[519-:4]);
15:wide_message_byte=hex_char(snapshot[515-:4]);
16:wide_message_byte=hex_char(snapshot[511-:4]);
17:wide_message_byte=hex_char(snapshot[507-:4]);
18:wide_message_byte=hex_char(snapshot[503-:4]);
19:wide_message_byte=hex_char(snapshot[499-:4]);
20:wide_message_byte=hex_char(snapshot[495-:4]);
21:wide_message_byte=hex_char(snapshot[491-:4]);
22:wide_message_byte=hex_char(snapshot[487-:4]);
23:wide_message_byte=hex_char(snapshot[483-:4]);
24:wide_message_byte=hex_char(snapshot[479-:4]);
25:wide_message_byte=hex_char(snapshot[475-:4]);
26:wide_message_byte=hex_char(snapshot[471-:4]);
27:wide_message_byte=hex_char(snapshot[467-:4]);
28:wide_message_byte=hex_char(snapshot[463-:4]);
29:wide_message_byte=hex_char(snapshot[459-:4]);
30:wide_message_byte=hex_char(snapshot[455-:4]);
31:wide_message_byte=hex_char(snapshot[451-:4]);
32:wide_message_byte=hex_char(snapshot[447-:4]);
33:wide_message_byte=hex_char(snapshot[443-:4]);
34:wide_message_byte=hex_char(snapshot[439-:4]);
35:wide_message_byte=hex_char(snapshot[435-:4]);
36:wide_message_byte=hex_char(snapshot[431-:4]);
37:wide_message_byte=hex_char(snapshot[427-:4]);
38:wide_message_byte=hex_char(snapshot[423-:4]);
39:wide_message_byte=hex_char(snapshot[419-:4]);
40:wide_message_byte=hex_char(snapshot[415-:4]);
41:wide_message_byte=hex_char(snapshot[411-:4]);
42:wide_message_byte=hex_char(snapshot[407-:4]);
43:wide_message_byte=hex_char(snapshot[403-:4]);
44:wide_message_byte=hex_char(snapshot[399-:4]);
45:wide_message_byte=hex_char(snapshot[395-:4]);
46:wide_message_byte=hex_char(snapshot[391-:4]);
47:wide_message_byte=hex_char(snapshot[387-:4]);
48:wide_message_byte=hex_char(snapshot[383-:4]);
49:wide_message_byte=hex_char(snapshot[379-:4]);
50:wide_message_byte=hex_char(snapshot[375-:4]);
51:wide_message_byte=hex_char(snapshot[371-:4]);
52:wide_message_byte=hex_char(snapshot[367-:4]);
53:wide_message_byte=hex_char(snapshot[363-:4]);
54:wide_message_byte=hex_char(snapshot[359-:4]);
55:wide_message_byte=hex_char(snapshot[355-:4]);
56:wide_message_byte=hex_char(snapshot[351-:4]);
57:wide_message_byte=hex_char(snapshot[347-:4]);
58:wide_message_byte=hex_char(snapshot[343-:4]);
59:wide_message_byte=hex_char(snapshot[339-:4]);
60:wide_message_byte=hex_char(snapshot[335-:4]);
61:wide_message_byte=hex_char(snapshot[331-:4]);
62:wide_message_byte=hex_char(snapshot[327-:4]);
63:wide_message_byte=hex_char(snapshot[323-:4]);
64:wide_message_byte=hex_char(snapshot[319-:4]);
65:wide_message_byte=hex_char(snapshot[315-:4]);
66:wide_message_byte=hex_char(snapshot[311-:4]);
67:wide_message_byte=hex_char(snapshot[307-:4]);
68:wide_message_byte=hex_char(snapshot[303-:4]);
69:wide_message_byte=hex_char(snapshot[299-:4]);
70:wide_message_byte=hex_char(snapshot[295-:4]);
71:wide_message_byte=hex_char(snapshot[291-:4]);
72:wide_message_byte=hex_char(snapshot[287-:4]);
73:wide_message_byte=hex_char(snapshot[283-:4]);
74:wide_message_byte=hex_char(snapshot[279-:4]);
75:wide_message_byte=hex_char(snapshot[275-:4]);
76:wide_message_byte=hex_char(snapshot[271-:4]);
77:wide_message_byte=hex_char(snapshot[267-:4]);
78:wide_message_byte=hex_char(snapshot[263-:4]);
79:wide_message_byte=hex_char(snapshot[259-:4]);
80:wide_message_byte=hex_char(snapshot[255-:4]);
81:wide_message_byte=hex_char(snapshot[251-:4]);
82:wide_message_byte=hex_char(snapshot[247-:4]);
83:wide_message_byte=hex_char(snapshot[243-:4]);
84:wide_message_byte=hex_char(snapshot[239-:4]);
85:wide_message_byte=hex_char(snapshot[235-:4]);
86:wide_message_byte=hex_char(snapshot[231-:4]);
87:wide_message_byte=hex_char(snapshot[227-:4]);
88:wide_message_byte=hex_char(snapshot[223-:4]);
89:wide_message_byte=hex_char(snapshot[219-:4]);
90:wide_message_byte=hex_char(snapshot[215-:4]);
91:wide_message_byte=hex_char(snapshot[211-:4]);
92:wide_message_byte=hex_char(snapshot[207-:4]);
93:wide_message_byte=hex_char(snapshot[203-:4]);
94:wide_message_byte=hex_char(snapshot[199-:4]);
95:wide_message_byte=hex_char(snapshot[195-:4]);
96:wide_message_byte=hex_char(snapshot[191-:4]);
97:wide_message_byte=hex_char(snapshot[187-:4]);
98:wide_message_byte=hex_char(snapshot[183-:4]);
99:wide_message_byte=hex_char(snapshot[179-:4]);
100:wide_message_byte=hex_char(snapshot[175-:4]);
101:wide_message_byte=hex_char(snapshot[171-:4]);
102:wide_message_byte=hex_char(snapshot[167-:4]);
103:wide_message_byte=hex_char(snapshot[163-:4]);
104:wide_message_byte=hex_char(snapshot[159-:4]);
105:wide_message_byte=hex_char(snapshot[155-:4]);
106:wide_message_byte=hex_char(snapshot[151-:4]);
107:wide_message_byte=hex_char(snapshot[147-:4]);
108:wide_message_byte=hex_char(snapshot[143-:4]);
109:wide_message_byte=hex_char(snapshot[139-:4]);
110:wide_message_byte=hex_char(snapshot[135-:4]);
111:wide_message_byte=hex_char(snapshot[131-:4]);
112:wide_message_byte=hex_char(snapshot[127-:4]);
113:wide_message_byte=hex_char(snapshot[123-:4]);
114:wide_message_byte=hex_char(snapshot[119-:4]);
115:wide_message_byte=hex_char(snapshot[115-:4]);
116:wide_message_byte=hex_char(snapshot[111-:4]);
117:wide_message_byte=hex_char(snapshot[107-:4]);
118:wide_message_byte=hex_char(snapshot[103-:4]);
119:wide_message_byte=hex_char(snapshot[99-:4]);
120:wide_message_byte=hex_char(snapshot[95-:4]);
121:wide_message_byte=hex_char(snapshot[91-:4]);
122:wide_message_byte=hex_char(snapshot[87-:4]);
123:wide_message_byte=hex_char(snapshot[83-:4]);
124:wide_message_byte=hex_char(snapshot[79-:4]);
125:wide_message_byte=hex_char(snapshot[75-:4]);
126:wide_message_byte=hex_char(snapshot[71-:4]);
127:wide_message_byte=hex_char(snapshot[67-:4]);
128:wide_message_byte=hex_char(snapshot[63-:4]);
129:wide_message_byte=hex_char(snapshot[59-:4]);
130:wide_message_byte=hex_char(snapshot[55-:4]);
131:wide_message_byte=hex_char(snapshot[51-:4]);
132:wide_message_byte=hex_char(snapshot[47-:4]);
133:wide_message_byte=hex_char(snapshot[43-:4]);
134:wide_message_byte=hex_char(snapshot[39-:4]);
135:wide_message_byte=hex_char(snapshot[35-:4]);
136:wide_message_byte=hex_char(snapshot[31-:4]);
137:wide_message_byte=hex_char(snapshot[27-:4]);
138:wide_message_byte=hex_char(snapshot[23-:4]);
139:wide_message_byte=hex_char(snapshot[19-:4]);
140:wide_message_byte=hex_char(snapshot[15-:4]);
141:wide_message_byte=hex_char(snapshot[11-:4]);
142:wide_message_byte=hex_char(snapshot[7-:4]);
143:wide_message_byte=hex_char(snapshot[3-:4]);
144:wide_message_byte=hex_char(checksum_snapshot[7:4]);
145:wide_message_byte=hex_char(checksum_snapshot[3:0]);
146:wide_message_byte=13;
147:wide_message_byte=10;
default:wide_message_byte="?";endcase end endfunction
 function [7:0] message_byte;input [(WIDE_RANGE ? 7 : 6):0] at;begin if(WIDE_RANGE)message_byte=wide_message_byte(at);else case(at)
 0:message_byte="D";
1:message_byte="1";
2:message_byte=hex_char(snapshot[351-:4]);
3:message_byte=hex_char(snapshot[347-:4]);
4:message_byte=hex_char(snapshot[343-:4]);
5:message_byte=hex_char(snapshot[339-:4]);
6:message_byte=hex_char(snapshot[335-:4]);
7:message_byte=hex_char(snapshot[331-:4]);
8:message_byte=hex_char(snapshot[327-:4]);
9:message_byte=hex_char(snapshot[323-:4]);
10:message_byte=hex_char(snapshot[319-:4]);
11:message_byte=hex_char(snapshot[315-:4]);
12:message_byte=hex_char(snapshot[311-:4]);
13:message_byte=hex_char(snapshot[307-:4]);
14:message_byte=hex_char(snapshot[303-:4]);
15:message_byte=hex_char(snapshot[299-:4]);
16:message_byte=hex_char(snapshot[295-:4]);
17:message_byte=hex_char(snapshot[291-:4]);
18:message_byte=hex_char(snapshot[287-:4]);
19:message_byte=hex_char(snapshot[283-:4]);
20:message_byte=hex_char(snapshot[279-:4]);
21:message_byte=hex_char(snapshot[275-:4]);
22:message_byte=hex_char(snapshot[271-:4]);
23:message_byte=hex_char(snapshot[267-:4]);
24:message_byte=hex_char(snapshot[263-:4]);
25:message_byte=hex_char(snapshot[259-:4]);
26:message_byte=hex_char(snapshot[255-:4]);
27:message_byte=hex_char(snapshot[251-:4]);
28:message_byte=hex_char(snapshot[247-:4]);
29:message_byte=hex_char(snapshot[243-:4]);
30:message_byte=hex_char(snapshot[239-:4]);
31:message_byte=hex_char(snapshot[235-:4]);
32:message_byte=hex_char(snapshot[231-:4]);
33:message_byte=hex_char(snapshot[227-:4]);
34:message_byte=hex_char(snapshot[223-:4]);
35:message_byte=hex_char(snapshot[219-:4]);
36:message_byte=hex_char(snapshot[215-:4]);
37:message_byte=hex_char(snapshot[211-:4]);
38:message_byte=hex_char(snapshot[207-:4]);
39:message_byte=hex_char(snapshot[203-:4]);
40:message_byte=hex_char(snapshot[199-:4]);
41:message_byte=hex_char(snapshot[195-:4]);
42:message_byte=hex_char(snapshot[191-:4]);
43:message_byte=hex_char(snapshot[187-:4]);
44:message_byte=hex_char(snapshot[183-:4]);
45:message_byte=hex_char(snapshot[179-:4]);
46:message_byte=hex_char(snapshot[175-:4]);
47:message_byte=hex_char(snapshot[171-:4]);
48:message_byte=hex_char(snapshot[167-:4]);
49:message_byte=hex_char(snapshot[163-:4]);
50:message_byte=hex_char(snapshot[159-:4]);
51:message_byte=hex_char(snapshot[155-:4]);
52:message_byte=hex_char(snapshot[151-:4]);
53:message_byte=hex_char(snapshot[147-:4]);
54:message_byte=hex_char(snapshot[143-:4]);
55:message_byte=hex_char(snapshot[139-:4]);
56:message_byte=hex_char(snapshot[135-:4]);
57:message_byte=hex_char(snapshot[131-:4]);
58:message_byte=hex_char(snapshot[127-:4]);
59:message_byte=hex_char(snapshot[123-:4]);
60:message_byte=hex_char(snapshot[119-:4]);
61:message_byte=hex_char(snapshot[115-:4]);
62:message_byte=hex_char(snapshot[111-:4]);
63:message_byte=hex_char(snapshot[107-:4]);
64:message_byte=hex_char(snapshot[103-:4]);
65:message_byte=hex_char(snapshot[99-:4]);
66:message_byte=hex_char(snapshot[95-:4]);
67:message_byte=hex_char(snapshot[91-:4]);
68:message_byte=hex_char(snapshot[87-:4]);
69:message_byte=hex_char(snapshot[83-:4]);
70:message_byte=hex_char(snapshot[79-:4]);
71:message_byte=hex_char(snapshot[75-:4]);
72:message_byte=hex_char(snapshot[71-:4]);
73:message_byte=hex_char(snapshot[67-:4]);
74:message_byte=hex_char(snapshot[63-:4]);
75:message_byte=hex_char(snapshot[59-:4]);
76:message_byte=hex_char(snapshot[55-:4]);
77:message_byte=hex_char(snapshot[51-:4]);
78:message_byte=hex_char(snapshot[47-:4]);
79:message_byte=hex_char(snapshot[43-:4]);
80:message_byte=hex_char(snapshot[39-:4]);
81:message_byte=hex_char(snapshot[35-:4]);
82:message_byte=hex_char(snapshot[31-:4]);
83:message_byte=hex_char(snapshot[27-:4]);
84:message_byte=hex_char(snapshot[23-:4]);
85:message_byte=hex_char(snapshot[19-:4]);
86:message_byte=hex_char(snapshot[15-:4]);
87:message_byte=hex_char(snapshot[11-:4]);
88:message_byte=hex_char(snapshot[7-:4]);
89:message_byte=hex_char(snapshot[3-:4]);
90:message_byte=hex_char(checksum_snapshot[7:4]);
91:message_byte=hex_char(checksum_snapshot[3:0]);
92:message_byte=13;
93:message_byte=10;
 default:message_byte="?";endcase end endfunction
 always @(posedge clk)begin
  if(!rst_n)begin snapshot<=0;checksum_snapshot<=0;timer<=0;owed<=0;busy<=0;bit_active<=0;index<=0;bit_index<=0;baud<=0;shift<=1023;end
  else begin
   if(timer==MS_CYCLES*PERIOD_MS-1)begin timer<=0;if(enabled)owed<=1;end else timer<=timer+1'b1;
   if(!enabled)owed<=0;
   if(!busy && owed && enabled && grant)begin snapshot<=payload;checksum_snapshot<=checksum;owed<=0;index<=0;busy<=1;end
   else if(busy)begin
    if(!bit_active)begin shift<={1'b1,message_byte(index),1'b0};baud<=0;bit_index<=0;bit_active<=1;end
    else if(baud==UART_DIV-1)begin
     baud<=0;shift<={1'b1,shift[9:1]};
     if(bit_index==9)begin bit_active<=0;if(index==(WIDE_RANGE ? 147 : 93))busy<=0;else index<=index+1'b1;end
     else bit_index<=bit_index+1'b1;
    end else baud<=baud+1'b1;
   end
  end
 end
endmodule

from pathlib import Path
import json
root=Path(__file__).resolve().parent
# Reuse the already verified coherent-snapshot / 8N1 serializer structure.
layout=[('VIS',1,'version','literal'),('BX',4,'x','hex'),('BY',4,'y','hex'),('V',1,'valid','hex'),
 ('AGE',4,'age','hex'),('FR',4,'frames','hex'),('EN',1,'enabled','hex'),('Z',1,'origin_valid','hex'),
 ('F',1,'fault','hex'),('E1',8,'count1','hex'),('E2',8,'count2','hex'),('X1',4,'errors1','hex'),('X2',4,'errors2','hex'),
 ('P',8,'position','hex'),('T',2,'target','hex'),('C',4,'center','hex'),('DZ',4,'deadzone','hex'),
 ('G',2,'gain','hex'),('L',2,'limit','hex'),('SEQ',4,'cfg_sequence','hex'),('RXE',4,'rx_errors','hex'),
 ('Q',1,'quality','hex'),('B',1,'bad','hex'),
 ('CS',4,'cam_sequence','hex'),('AE',4,'exposure','hex'),('AG',4,'sensor_gain','hex'),
 ('RL',4,'roi_left','hex'),('RT',4,'roi_top','hex'),('RR',4,'roi_right','hex'),('RB',4,'roi_bottom','hex'),
 ('CA',4,'camera_applied_seq','hex'),('RA',4,'roi_applied_seq','hex'),('CB',1,'settings_busy','hex'),
 ('MT',8,'locked_target','hex'),('ME',8,'move_error','hex'),('MS',1,'move_state','hex'),('MB',1,'move_busy','hex'),
 ('OK',1,'reached','hex'),('RD',1,'request_ready','hex'),('SP',2,'spent','hex'),('MC',8,'command_position','hex'),
 ('CR',1,'cancel_reason','hex'),('RJ',1,'reject_reason','hex'),('AC',4,'accepted_count','hex'),('DC',4,'completed_count','hex'),('CC',4,'center_count','hex'),('KC',1,'center_status','hex'),
 ('MD',1,'operating_mode','hex'),('AR',1,'auto_run','hex'),('KD',2,'damping','hex'),('SL',2,'slew','hex'),
 ('RU',4,'runtime_seconds','hex'),('DS',4,'follow_sequence','hex'),('CZ',1,'center_confirmed','hex'),
 ('VE',4,'velocity','hex'),('AT',2,'drive_target','hex'),('SR',1,'stop_reason','hex'),('AJ',1,'auto_reject','hex'),
 ('EL',4,'elapsed_seconds','hex'),('TS',8,'total_pulses','hex')]
assert len({entry[0] for entry in layout}) == len(layout), 'Duplicate telemetry field'
template=' '.join(k+'='+'0'*n for k,n,_,_ in layout)+'\r\n'
regs=[];resets=[];captures=[];cases=[];index=0
for field,(key,width,signal,kind) in enumerate(layout):
    if field:cases.append(f"                {index}: message_byte=8'h20;");index+=int(bool(field))
    for ch in key+'=':cases.append(f"                {index}: message_byte=8'h{ord(ch):02x};");index+=1
    if kind!='literal':
        regs.append(f'    reg [{width*4-1}:0] {signal}_snapshot=0;')
        resets.append(f'{signal}_snapshot<=0;');captures.append(f'{signal}_snapshot<={signal};')
    for digit in range(width):
        value="8'h35" if kind=='literal' else f'hex_char({signal}_snapshot >> {(width-1-digit)*4})'
        cases.append(f'                {index}: message_byte={value};');index+=1
cases.extend([f"                {index}: message_byte=8'h0d;",f"                {index+1}: message_byte=8'h0a;"])
source='''module vision_report_tx #(parameter integer UART_DIV=434,REPORT_CYCLES=12500000)(
    input wire clk,rst_n,
    input wire [10:0] x,y,
    input wire valid,enabled,origin_valid,
    input wire [3:0] fault,
    input wire [3:0] quality,
    input wire bad,
    input wire [15:0] age,frames,errors1,errors2,center,deadzone,cfg_sequence,rx_errors,
    input wire [15:0] cam_sequence,exposure,sensor_gain,roi_left,roi_top,roi_right,roi_bottom,camera_applied_seq,roi_applied_seq,
    input wire settings_busy,
    input wire signed [31:0] locked_target,move_error,command_position,
    input wire [3:0] move_state,cancel_reason,reject_reason,
    input wire move_busy,reached,request_ready,
    input wire [7:0] spent,
    input wire [15:0] accepted_count,completed_count,center_count,
    input wire [3:0] center_status,
    input wire [7:0] operating_mode,damping,slew,
    input wire auto_run,center_confirmed,
    input wire [15:0] runtime_seconds,follow_sequence,elapsed_seconds,
    input wire signed [15:0] velocity,
    input wire signed [7:0] drive_target,
    input wire [3:0] stop_reason,auto_reject,
    input wire [31:0] total_pulses,
    input wire [7:0] gain,limit,
    input wire signed [7:0] target,
    input wire signed [31:0] count1,count2,position,
    output wire tx
);
REGS
    reg [31:0] timer=0;
    reg frame_active=0,bit_active=0;
    reg [8:0] char_index=0;
    reg [3:0] bit_index=0;
    reg [15:0] baud_count=0;
    reg [9:0] serial_shift=10'h3ff;
    assign tx=serial_shift[0];
    function [7:0] hex_char;
        input [3:0] value;
        begin if(value<10)hex_char=8'h30+value;else hex_char=8'h37+value;end
    endfunction
    function [7:0] message_byte;
        input [8:0] index;
        begin case(index)
CASES
            default:message_byte="?";
        endcase end
    endfunction
    always @(posedge clk)begin
        if(!rst_n)begin
            RESET
            timer<=0;frame_active<=0;bit_active<=0;char_index<=0;bit_index<=0;
            baud_count<=0;serial_shift<=10'h3ff;
        end else begin
            if(timer==REPORT_CYCLES-1)timer<=0;else timer<=timer+1'b1;
            if(!frame_active && timer==REPORT_CYCLES-1)begin
                CAPTURE
                char_index<=0;frame_active<=1;
            end else if(frame_active)begin
                if(!bit_active)begin
                    serial_shift<={1'b1,message_byte(char_index),1'b0};
                    baud_count<=0;bit_index<=0;bit_active<=1;
                end else if(baud_count==UART_DIV-1)begin
                    baud_count<=0;serial_shift<={1'b1,serial_shift[9:1]};
                    if(bit_index==9)begin
                        bit_active<=0;
                        if(char_index==LAST)frame_active<=0;else char_index<=char_index+1'b1;
                    end else bit_index<=bit_index+1'b1;
                end else baud_count<=baud_count+1'b1;
            end
        end
    end
endmodule
'''.replace('REGS','\n'.join(regs)).replace('CASES','\n'.join(cases)).replace('RESET',' '.join(resets)).replace('CAPTURE',' '.join(captures)).replace('LAST',str(len(template)-1))
(root/'rtl/vision_report_tx.v').write_text(source,encoding='utf-8')
(root/'frame_layout.json').write_text(json.dumps({'layout':layout,'template':template,'bytes':len(template)},indent=2),encoding='utf-8')
print('Vision UART bytes:',len(template))

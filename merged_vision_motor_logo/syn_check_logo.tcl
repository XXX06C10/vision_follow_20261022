set base [file dirname [info script]]
cd $base
cd td_project
import_device ph1_35p.db -package PH1P35MDG324 -speed 3
open_project -single_run vision_follow_20261022.al
elaborate -top design_top_wrapper
read_adc ../user_source/constraints_source/pin.adc
read_ip_adc -ip ddr2 -file ../user_source/hdl_source/ph1p35_ddr/ddr2/src/adc/ddr2.adc
optimize_rtl
read_sdc vision_follow_20261022.sdc
read_sdc -ip ddr2 ../user_source/hdl_source/ph1p35_ddr/ddr2/src/sdc/ddr2.sdc
read_sdc -ip w128_d512_fifo ../user_source/ip_source/w128_d512_fifo/w128_d512_fifo.tcl
file mkdir ../logo_Runs/syn
cd ../logo_Runs/syn
report_area -file logo_rtl.area
optimize_gate -packarea logo_gate.area
puts LOGO_SYN_COMPLETE
exit

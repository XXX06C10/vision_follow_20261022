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
file mkdir ../logo_Runs/phy_1
cd ../logo_Runs/phy_1
report_area -file logo_rtl.area
optimize_gate -packarea logo_gate.area
legalize_phy_inst
export_db logo_gate.db
place
report_area -io_info -file logo_phy.area
route
update_timing -mode final
report_timing_status -file logo_phy.ts
report_timing_summary -file logo_pr.timing
write_verilog ../../logo_routed_netlist.v
export_db logo_pr.db
bitgen -bit merged_vision_motor_logo.bit
puts MERGED_VISION_MOTOR_LOGO_BUILD_COMPLETE
exit

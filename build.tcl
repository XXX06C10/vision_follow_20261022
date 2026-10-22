set base [file dirname [info script]]
cd $base
cd td_project
import_device ph1_35p.db -package PH1P35MDG324 -speed 3
open_project -single_run vision_follow_20261022.al
elaborate -top design_top_wrapper
read_adc ../user_source/constraints_source/pin.adc
# Camera DDR IP additions to the already proven motor build sequence.
read_ip_adc -ip ddr2 -file ../user_source/hdl_source/ph1p35_ddr/ddr2/src/adc/ddr2.adc
optimize_rtl
read_sdc vision_follow_20261022.sdc
read_sdc -ip ddr2 ../user_source/hdl_source/ph1p35_ddr/ddr2/src/sdc/ddr2.sdc
read_sdc -ip w128_d512_fifo ../user_source/ip_source/w128_d512_fifo/w128_d512_fifo.tcl
file mkdir ../vision_follow_20261022_Runs/phy_1
cd ../vision_follow_20261022_Runs/phy_1
report_area -file vision_follow_20261022_rtl.area
optimize_gate -packarea vision_follow_20261022_gate.area
legalize_phy_inst
export_db vision_follow_20261022_gate.db
place
report_area -io_info -file vision_follow_20261022_phy.area
route
update_timing -mode final
report_timing_status -file vision_follow_20261022_phy.ts
report_timing_summary -file vision_follow_20261022_pr.timing
write_verilog ../../routed_netlist.v
export_db vision_follow_20261022_pr.db
bitgen -bit vision_follow_20261022.bit
puts VISION_FOLLOW_20261022_BUILD_COMPLETE
exit

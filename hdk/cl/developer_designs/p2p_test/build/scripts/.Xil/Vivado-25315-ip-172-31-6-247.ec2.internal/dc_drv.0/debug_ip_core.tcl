#
#Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
#
set_param power.enableUnconnectedCarry8PinPower 1
set_param hd.clockRoutingWireReduction 0
set_param checkpoint.writeSynthRtdsInDcp 1
set_param power.enableCarry8RouteBelPower 1
set_param power.BramSDPPropagationFix 1
set_param synth.enableIncremental 0
set_param synth.incremental.totalBlackboxInstancesCount 1
set_param power.enableLutRouteBelPower 1
set_param project.replaceDontTouchWithKeepHierarchySoft 0
debug::add_scope -scope route
debug::set_visibility -level 5
set_param chipscope.flow 0
set_param ips.addSpecialDataToCacheID 0
set script_paths /home/centos/aws-fpga/hdk/cl/developer_designs/p2p_test/build/scripts/.Xil/Vivado-25315-ip-172-31-6-247.ec2.internal/xsdbm_CV.0/out/get_cs_ip.tcl
foreach script_path $script_paths { 
    source $script_path
    lappend debug_ip_vlnv $ip_vlnv 
    lappend debug_ip_module_name $ip_module_name
    lappend debug_params $params
    lappend debug_intf_params $intf_params
    lappend debug_connectivity $connectivity
    lappend debug_output_xci $output_xci
    lappend debug_output_dcp $output_dcp
    lappend debug_output_dir $output_dir
    lappend debug_synth_opts $synth_opts
    lappend debug_xdc_files $xdc_files
}
set debug_ip_repo_paths $ip_repo_paths
set debug_ip_output_repo $ip_output_repo
set debug_ip_cache_permissions $ip_cache_permissions

set debug_oopbus_ip_repo_paths [get_param chipscope.oopbus_ip_repo_paths]
set ordering_constrs 1
set jobs 2

source {/opt/Xilinx/Vivado/2021.2/scripts/ip/ipxchipscope.tcl}

set failed [catch {ipx::chipscope::run_multi_gen_and_synth_ip_core $part $board_part $board_connections $board_part_repo_paths $tool_flow $debug_ip_vlnv $debug_ip_module_name $debug_params $debug_output_xci $debug_output_dcp $debug_output_dir $debug_ip_repo_paths $debug_ip_output_repo $debug_ip_cache_permissions $debug_oopbus_ip_repo_paths $debug_synth_opts $debug_xdc_files $ordering_constrs $jobs} errMessage]

if { $failed } {
send_msg_id {IP_Flow 19-3805} ERROR "Failed to generate and synthesize debug IPs. \n $errMessage"
  exit 1
}

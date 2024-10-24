# BSD 3-Clause License
#
# Copyright (c) 2023, Google LLC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

sta::define_cmd_args "preview_dft" {[-verbose]}

proc preview_dft { args } {
  sta::parse_key_args "preview_dft" args \
    keys {} \
    flags {-verbose}

  sta::check_argc_eq0 "preview_dft" $args

  if { [ord::get_db_block] == "NULL" } {
    utl::error DFT 1 "No design block found."
  }

  set verbose [info exists flags(-verbose)]

  dft::preview_dft $verbose
}

sta::define_cmd_args "scan_replace" {[-keep_pl]}
proc scan_replace { args } {
  sta::parse_key_args "scan_replace" args \
    keys {} flags {-keep_pl}

  if { [ord::get_db_block] == "NULL" } {
    utl::error DFT 8 "No design block found."
  }

  set enable_keep_placement [expr [info exists flags(-keep_pl)]]

  dft::scan_replace $enable_keep_placement
}

sta::define_cmd_args "write_scan_chains" {[json_file_out]}
proc write_scan_chains { args } {
  sta::parse_key_args "write_scan_chains" args \
    keys {} flags {}

  if { [ord::get_db_block] == "NULL" } {
    utl::error DFT 15 "No design block found."
  }

  dft::write_scan_chains [lindex $args 0]
}


sta::define_cmd_args "insert_boundary_scan_registers" {
  [-clock_name clock_name]
  [-clock_edge clock_edge]
  [-reset_name reset_name]
  [-reset_active reset_active]
  [-input_mux input_mux_str]
}
proc insert_boundary_scan_registers { args } {
  set key_list {
    -clock_name
    -clock_edge
    -reset_name
    -reset_active
    -input_mux
    -testing_net
  }
  sta::parse_key_args "insert_boundary_scan_registers" args \
    keys "$key_list -ignore_ports_rx" flags {}

  if { [ord::get_db_block] == "NULL" } {
    utl::error DFT 16 "No design block found."
  }
  
  foreach key $key_list {
    if { ![info exists keys($key)] } {
      utl::error DFT 19 "Required key $key not provided."
    }
  }
  
  set ignored_ports_pattern ""
  if { [info exists keys(-ignore_ports_rx)] } {
    set ignored_ports_pattern "$keys(-ignore_ports_rx)"
  }
  
  set input_mux_elements [split $keys(-input_mux) "/"]
  if { [llength $input_mux_elements] != 5 } {
    utl::error DFT 20 "Invalid multiplexer specification: [llength $input_mux_elements]/5 elements provided."
  }
  
  lassign "$input_mux_elements" master s a0 a1 y
  set input_mux [[::ord::get_db] findMaster $master]
  if { "$input_mux" == "NULL" } {
    utl::error DFT 21 "Unknown cell '$master'."
  }
  set input_mux_s [$input_mux findMTerm $s]
  if { "$input_mux_s" == "NULL" } {
    utl::error DFT 22 "Selection input '$s' not found for '$master'."
  }
  set input_mux_a0 [$input_mux findMTerm $a0]
  if { "$input_mux_a0" == "NULL" } {
    utl::error DFT 23 "Multiplexed input '$a0' not found for '$master'."
  }
  set input_mux_a1 [$input_mux findMTerm $a1]
  if { "$input_mux_a1" == "NULL" } {
    utl::error DFT 24 "Multiplexed input '$a1' not found for '$master'."
  }
  set input_mux_y [$input_mux findMTerm $y]
  if { "$input_mux_y" == "NULL" } {
    utl::error DFT 25 "Multiplexed output '$y' not found for '$master'."
  }
  
  dft::insert_boundary_scan_registers\
    $keys(-clock_name) $keys(-clock_edge)\
    $keys(-reset_name) $keys(-reset_active)\
    $input_mux $input_mux_s $input_mux_a0 $input_mux_a1 $input_mux_y\
    $keys(-testing_net)\
    $ignored_ports_pattern
}

sta::define_cmd_args "insert_dft" {
  [-per_chain_enable]
  [-scan_enable_name_pattern scan_enable_name_pattern]
  [-scan_in_name_pattern scan_in_name_pattern]
  [-scan_out_name_pattern scan_out_name_pattern]
}
proc insert_dft { args } {
  sta::parse_key_args "insert_dft" args \
    keys {-scan_enable_name_pattern -scan_in_name_pattern -scan_out_name_pattern}\
    flags {-per_chain_enable}

  if { [ord::get_db_block] == "NULL" } {
    utl::error DFT 9 "No design block found."
  }
  
  foreach {flag default} {
    -scan_enable_name_pattern "scan_enable_{}"
    -scan_in_name_pattern "scan_in_{}"
    -scan_out_name_pattern "scan_out_{}"
  } {
    if { ![info exists keys($flag)] } {
      set keys($flag) $default
    }
  }
  
  dft::insert_dft [info exists $flags(-per_chain_enable)]\
    $keys(-scan_enable_name_pattern)\
    $keys(-scan_in_name_pattern)\
    $keys(-scan_out_name_pattern)
}

sta::define_cmd_args "set_dft_config" { [-max_length max_length] \
                                        [-max_chains max_chains] \
                                        [-clock_mixing clock_mixing]}
proc set_dft_config { args } {
  sta::parse_key_args "set_dft_config" args \
    keys {-max_length -max_chains -clock_mixing} \
    flags {}

  sta::check_argc_eq0 "set_dft_config" $args

  if { [info exists keys(-max_length)] } {
    set max_length $keys(-max_length)
    sta::check_positive_integer "-max_length" $max_length
    dft::set_dft_config_max_length $max_length
  }

  if { [info exists keys(-max_chains)] } {
    set max_chains $keys(-max_chains)
    sta::check_positive_integer "-max_chains" $max_chains
    dft::set_dft_config_max_chains $max_chains
  }

  if { [info exists keys(-clock_mixing)] } {
    set clock_mixing $keys(-clock_mixing)
    puts $clock_mixing
    dft::set_dft_config_clock_mixing $clock_mixing
  }
}

sta::define_cmd_args "report_dft_config" { }
proc report_dft_config { } {
  sta::parse_key_args "report_dft_config" args keys {} flags {}
  dft::report_dft_config
}

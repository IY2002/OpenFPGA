/********************************************************************
 * This file includes functions that are used to build 3D switch block
 *  modules in the fabric
 * The functions in this file are modified versions of some of the 
 *  functions in build_routing_modules.cpp
 *******************************************************************/


#include <vector>

/* Headers from vtrutil library */
#include "vtr_assert.h"
#include "vtr_geometry.h"
#include "vtr_log.h"
#include "vtr_time.h"

/* Headers from openfpgautil library */
#include "build_memory_modules.h"
#include "build_module_graph_utils.h"
#include "build_3d_switch_block_module_utils.h"
#include "build_3d_switch_block_module.h"
#include "module_manager_utils.h"
#include "openfpga_naming.h"
#include "openfpga_reserved_words.h"
#include "openfpga_rr_graph_utils.h"
#include "openfpga_side_manager.h"
#include "rr_gsb_utils.h"


namespace openfpga {

/*********************************************************************
 * Generate a short interconneciton in switch box
 * There are two cases should be noticed.
 * 1. The actual fan-in of cur_rr_node is 0. In this case,
      the cur_rr_node need to be short connected to itself
      which is on the opposite side of this switch block
 * 2. The actual fan-in of cur_rr_node is 0. In this case,
 *    The cur_rr_node need to connected to the drive_rr_node
 ********************************************************************/
static void build_3d_switch_block_module_short_interc(
  ModuleManager& module_manager, const ModuleId& sb_module,
  const VprDeviceAnnotation& device_annotation, const DeviceGrid& grids,
  const RRGraphView& rr_graph, const RRGSB& rr_gsb, const e_side& chan_side,
  const RRNodeId& cur_rr_node, const RRNodeId& drive_rr_node,
  const std::map<ModulePinInfo, ModuleNetId>& input_port_to_module_nets) {
  /* Find the name of output port */
  ModulePinInfo output_port_info = find_3d_switch_block_module_chan_port(
    module_manager, sb_module, rr_graph, rr_gsb, chan_side, cur_rr_node,
    OUT_PORT);
  enum e_side input_pin_side = chan_side;
  int index = -1;

  /* Generate the input port object */
  switch (rr_graph.node_type(drive_rr_node)) {
    case OPIN: {
      rr_gsb.get_node_side_and_index(rr_graph, drive_rr_node, IN_PORT,
                                     input_pin_side, index);
      break;
    }
    case CHANX:
    case CHANY: {
      /* This should be an input in the data structure of RRGSB */
      if (cur_rr_node == drive_rr_node) {
        /* To be strict, the input should locate on the opposite side.
         * Use the else part if this may change in some architecture.
         */
        SideManager side_manager(chan_side);

        // Find side of the sink node (if channel is vertical, sink is not on the opposite side)
        if (chan_side == e_side::ABOVE || chan_side == e_side::UNDER){
          // Find sink_node
          RRNodeId sink_node;
          for (auto edge : rr_graph.edge_range(cur_rr_node)){
            if (rr_graph.edge_src_node((RREdgeId) edge) == cur_rr_node) sink_node = rr_graph.edge_sink_node((RREdgeId) edge);
          }
          // Find side of sink node
          rr_gsb.get_node_side_and_index(rr_graph, sink_node, IN_PORT,
                                       input_pin_side, index);

        } else {
          input_pin_side = side_manager.get_opposite();
        }
      } else {
        /* The input could be at any side of the switch block, find it */
        rr_gsb.get_node_side_and_index(rr_graph, drive_rr_node, IN_PORT,
                                       input_pin_side, index);
      }
      break;
    }
    default: /* SOURCE, IPIN, SINK are invalid*/
      VTR_LOGF_ERROR(__FILE__, __LINE__,
                     "Invalid rr_node type! Should be [OPIN|CHANX|CHANY].\n");
      exit(1);
  }
  /* Find the name of input port */
  ModulePinInfo input_port_info = find_3d_switch_block_module_input_port(
    module_manager, sb_module, grids, device_annotation, rr_graph, rr_gsb,
    input_pin_side, drive_rr_node);

  /* The input port and output port must match in size */
  BasicPort input_port =
    module_manager.module_port(sb_module, input_port_info.first);
  BasicPort output_port =
    module_manager.module_port(sb_module, output_port_info.first);

  /* Create a module net for this short-wire connection */
  ModuleNetId net = input_port_to_module_nets.at(input_port_info);
  /* Skip Configuring the net source, it is done before */
  /* Configure the net sink */
  module_manager.add_module_net_sink(sb_module, net, sb_module, 0,
                                     output_port_info.first,
                                     output_port_info.second);
}

/*********************************************************************
 * Build a instance of a routing multiplexer as well as
 * associated memory modules for a connection inside a switch block
 ********************************************************************/
static void build_3d_switch_block_mux_module(
  ModuleManager& module_manager, const ModuleId& sb_module,
  const VprDeviceAnnotation& device_annotation, const DeviceGrid& grids,
  const RRGraphView& rr_graph, const RRGSB& rr_gsb,
  const CircuitLibrary& circuit_lib, const e_side& chan_side,
  const size_t& chan_node_id, const RRNodeId& cur_rr_node,
  const std::vector<RRNodeId>& driver_rr_nodes, const RRSwitchId& switch_index,
  const std::map<ModulePinInfo, ModuleNetId>& input_port_to_module_nets,
  const bool& group_config_block, const size_t& layer) {
  /* Check current rr_node is CHANX or CHANY*/
  VTR_ASSERT((CHANX == rr_graph.node_type(cur_rr_node)) ||
             (CHANY == rr_graph.node_type(cur_rr_node)));

  /* Get the circuit model id of the routing multiplexer */
  CircuitModelId mux_model =
    device_annotation.rr_switch_circuit_model(switch_index);

  /* Find the input size of the implementation of a routing multiplexer */
  size_t datapath_mux_size = driver_rr_nodes.size();

  // Add number of vertical connections to the mux size
  
  // if (layer == 0 || layer == grids.get_num_layers() - 1){ // if layer is top or bottom then there is only 1 extra input to mux (above or below)
  //   datapath_mux_size += 1;
  // } else { // else there are 2 extra inputs to mux (above and below)
  //   datapath_mux_size += 2;
  // }

  /* Find the module name of the multiplexer and try to find it in the module
   * manager */
  std::string mux_module_name = generate_mux_subckt_name(
    circuit_lib, mux_model, datapath_mux_size, std::string(""));
  ModuleId mux_module = module_manager.find_module(mux_module_name);
  VTR_ASSERT(true == module_manager.valid_module_id(mux_module));

  /* Get the MUX instance id from the module manager */
  size_t mux_instance_id = module_manager.num_instance(sb_module, mux_module);
  /* Instanciate the MUX Module */
  module_manager.add_child_module(sb_module, mux_module);

  /* Give an instance name: this name should be consistent with the block name
   * given in SDC manager, If you want to bind the SDC generation to modules
   */
  std::string mux_instance_name = generate_sb_memory_instance_name(
    SWITCH_BLOCK_MUX_INSTANCE_PREFIX, chan_side, chan_node_id, std::string(""));
  module_manager.set_child_instance_name(sb_module, mux_module, mux_instance_id,
                                         mux_instance_name);

  /* Generate input ports that are wired to the input bus of the routing
   * multiplexer */
  std::vector<ModulePinInfo> sb_input_port_ids =
    find_3d_switch_block_module_input_ports(module_manager, sb_module, grids,
                                         device_annotation, rr_graph, rr_gsb,
                                         driver_rr_nodes, layer, chan_node_id / 2); // WEIRD MAYBE?

  /* Link input bus port to Switch Block inputs */
  std::vector<CircuitPortId> mux_model_input_ports =
    circuit_lib.model_ports_by_type(mux_model, CIRCUIT_MODEL_PORT_INPUT, true);
  VTR_ASSERT(1 == mux_model_input_ports.size());
  /* Find the module port id of the input port */
  ModulePortId mux_input_port_id = module_manager.find_module_port(
    mux_module, circuit_lib.port_prefix(mux_model_input_ports[0]));
  VTR_ASSERT(
    true == module_manager.valid_module_port_id(mux_module, mux_input_port_id));
  BasicPort mux_input_port =
    module_manager.module_port(mux_module, mux_input_port_id);

  /* Check port size should match */
  VTR_ASSERT(mux_input_port.get_width() == sb_input_port_ids.size());
  for (size_t pin_id = 0; pin_id < sb_input_port_ids.size(); ++pin_id) {
    /* Use the exising net */
    ModuleNetId net = input_port_to_module_nets.at(sb_input_port_ids[pin_id]);
    /* Configure the net source only if it is not yet in the source list */
    if (false ==
        module_manager.net_source_exist(sb_module, net, sb_module, 0,
                                        sb_input_port_ids[pin_id].first,
                                        sb_input_port_ids[pin_id].second)) {
      module_manager.add_module_net_source(sb_module, net, sb_module, 0,
                                           sb_input_port_ids[pin_id].first,
                                           sb_input_port_ids[pin_id].second);
    }
    /* Configure the net sink */
    module_manager.add_module_net_sink(sb_module, net, mux_module,
                                       mux_instance_id, mux_input_port_id,
                                       mux_input_port.pins()[pin_id]);
  }

  /* Link output port to Switch Block outputs */
  std::vector<CircuitPortId> mux_model_output_ports =
    circuit_lib.model_ports_by_type(mux_model, CIRCUIT_MODEL_PORT_OUTPUT, true);
  VTR_ASSERT(1 == mux_model_output_ports.size());
  /* Use the port name convention in the circuit library */
  ModulePortId mux_output_port_id = module_manager.find_module_port(
    mux_module, circuit_lib.port_prefix(mux_model_output_ports[0]));
  VTR_ASSERT(true == module_manager.valid_module_port_id(mux_module,
                                                         mux_output_port_id));
  BasicPort mux_output_port =
    module_manager.module_port(mux_module, mux_output_port_id);
  ModulePinInfo sb_output_port_id = find_3d_switch_block_module_chan_port(
    module_manager, sb_module, rr_graph, rr_gsb, chan_side, cur_rr_node,
    OUT_PORT);
  BasicPort sb_output_port =
    module_manager.module_port(sb_module, sb_output_port_id.first);

  /* Check port size should match */
  VTR_ASSERT(1 == mux_output_port.get_width());
  for (size_t pin_id = 0; pin_id < mux_output_port.pins().size(); ++pin_id) {
    /* Configuring the net source */
    ModuleNetId net = create_module_source_pin_net(
      module_manager, sb_module, mux_module, mux_instance_id,
      mux_output_port_id, mux_output_port.pins()[pin_id]);
    /* Configure the net sink */
    module_manager.add_module_net_sink(sb_module, net, sb_module, 0,
                                       sb_output_port_id.first,
                                       sb_output_port_id.second);
  }

  /* Instanciate memory modules */
  /* Find the name and module id of the memory module */
  std::string mem_module_name =
    generate_mux_subckt_name(circuit_lib, mux_model, datapath_mux_size,
                             std::string(MEMORY_MODULE_POSTFIX));
  if (group_config_block) {
    mem_module_name =
      generate_mux_subckt_name(circuit_lib, mux_model, datapath_mux_size,
                               std::string(MEMORY_FEEDTHROUGH_MODULE_POSTFIX));
  }
  ModuleId mem_module = module_manager.find_module(mem_module_name);
  VTR_ASSERT(true == module_manager.valid_module_id(mem_module));

  size_t mem_instance_id = module_manager.num_instance(sb_module, mem_module);
  module_manager.add_child_module(sb_module, mem_module);
  /* Give an instance name: this name should be consistent with the block name
   * given in bitstream manager, If you want to bind the bitstream generation to
   * modules
   */
  std::string mem_instance_name = generate_sb_memory_instance_name(
    SWITCH_BLOCK_MEM_INSTANCE_PREFIX, chan_side, chan_node_id, std::string(""),
    group_config_block);
  module_manager.set_child_instance_name(sb_module, mem_module, mem_instance_id,
                                         mem_instance_name);

  /* Add nets to connect regular and mode-select SRAM ports to the SRAM port of
   * memory module */
  add_module_nets_between_logic_and_memory_sram_bus(
    module_manager, sb_module, mux_module, mux_instance_id, mem_module,
    mem_instance_id, circuit_lib, mux_model);
  /* Update memory and instance list */
  size_t config_child_id = module_manager.num_configurable_children(
    sb_module, ModuleManager::e_config_child_type::LOGICAL);
  module_manager.add_configurable_child(
    sb_module, mem_module, mem_instance_id,
    group_config_block ? ModuleManager::e_config_child_type::LOGICAL
                       : ModuleManager::e_config_child_type::UNIFIED);
  /* For logical memory, define the physical memory here */
  if (group_config_block) {
    std::string physical_mem_module_name =
      generate_mux_subckt_name(circuit_lib, mux_model, datapath_mux_size,
                               std::string(MEMORY_MODULE_POSTFIX));
    ModuleId physical_mem_module =
      module_manager.find_module(physical_mem_module_name);
    VTR_ASSERT(true == module_manager.valid_module_id(physical_mem_module));
    module_manager.set_logical2physical_configurable_child(
      sb_module, config_child_id, physical_mem_module);
    std::string physical_mem_instance_name = generate_sb_memory_instance_name(
      SWITCH_BLOCK_MEM_INSTANCE_PREFIX, chan_side, chan_node_id,
      std::string(""), false);
    module_manager.set_logical2physical_configurable_child_instance_name(
      sb_module, config_child_id, physical_mem_instance_name);
  }
}

std::vector<RRSwitchId> driver_switches;

/*********************************************************************
 * Generate child modules for a interconnection inside switch block
 * The interconnection could be either a wire or a routing multiplexer,
 * which depends on the fan-in of the rr_nodes in the switch block
 ********************************************************************/
static void build_3d_switch_block_interc_modules(
  ModuleManager& module_manager, const ModuleId& sb_module,
  const VprDeviceAnnotation& device_annotation, const DeviceGrid& grids,
  const RRGraphView& rr_graph, const RRGSB& rr_gsb,
  const CircuitLibrary& circuit_lib, const e_side& chan_side,
  const size_t& chan_node_id,
  const std::map<ModulePinInfo, ModuleNetId>& input_port_to_module_nets,
  const bool& group_config_block, const size_t& layer) {

  /** ADD THE VERTICAL CONNECTIONS TO THE NECESSARY MUXES, 
   *  PROBABLY LAYER ONLY TAKES EFFECT IN DEEPER FUNCTIONS
   *  TO BE IMPLEMENTED
  */

  std::vector<RRNodeId> driver_rr_nodes;
  
  /* Get the node */
  const RRNodeId& cur_rr_node = rr_gsb.get_chan_node(chan_side, chan_node_id);

  /* Determine if the interc lies inside a channel wire, that is interc between
   * segments */
  if (false ==
      rr_gsb.is_sb_node_passing_wire(rr_graph, chan_side, chan_node_id)) {
    driver_rr_nodes = get_rr_gsb_chan_node_configurable_driver_nodes(
      rr_graph, rr_gsb, chan_side, chan_node_id);
    /* Special: if there are zero-driver nodes. We skip here */
    if (0 == driver_rr_nodes.size()) {
      return;
    }
  }

  size_t num_driver_nodes = driver_rr_nodes.size();


  if (0 == num_driver_nodes) {
    /* Print a special direct connection*/
    build_3d_switch_block_module_short_interc(
      module_manager, sb_module, device_annotation, grids, rr_graph, rr_gsb,
      chan_side, cur_rr_node, cur_rr_node, input_port_to_module_nets);
  } else if (1 == num_driver_nodes) {
    /* Print a direct connection*/
    build_3d_switch_block_module_short_interc(
      module_manager, sb_module, device_annotation, grids, rr_graph, rr_gsb,
      chan_side, cur_rr_node, driver_rr_nodes[0], input_port_to_module_nets);
  } else if (1 < num_driver_nodes) {
    /* Print the multiplexer, fan_in >= 2 */
    driver_switches =
      get_rr_graph_driver_switches(rr_graph, cur_rr_node);
    // VTR_ASSERT(1 == driver_switches.size());
    build_3d_switch_block_mux_module(
      module_manager, sb_module, device_annotation, grids, rr_graph, rr_gsb,
      circuit_lib, chan_side, chan_node_id, cur_rr_node, driver_rr_nodes,
      driver_switches[0], input_port_to_module_nets, group_config_block, layer);
  } /*Nothing should be done else*/
}

/**
 * Build the vertical Muxes for the vertical connections
 */
static void build_3d_switch_block_vertical_mux_module(
  ModuleManager& module_manager, const ModuleId& sb_module,
  const VprDeviceAnnotation& device_annotation, const DeviceGrid& grids,
  const RRGraphView& rr_graph, const RRGSB& rr_gsb,
  const CircuitLibrary& circuit_lib, const e_side& chan_side,
  const size_t& chan_node_id, 
   const RRSwitchId& switch_index,
  const std::map<ModulePinInfo, ModuleNetId>& input_port_to_module_nets,
  const bool& group_config_block, const size_t& layer, const std::string& direction) {
    
    /**
     * ASSUMPTION: the CLB outputs are not connected to the vertical connections
     * in routing the output would have to connect to a channel then a SB to get routed
     */
    /* Get the circuit model id of the routing multiplexer */
    CircuitModelId mux_model =
      device_annotation.rr_switch_circuit_model(switch_index);

    std::vector<ModulePinInfo> sb_input_ports;

    for (size_t iside = 0; iside < rr_gsb.get_num_sides(); ++iside){
      SideManager side_manager(iside);
      t_rr_type chan_type;
      if (side_manager.get_side() == e_side::TOP || side_manager.get_side() == e_side::BOTTOM){
        chan_type = CHANY;
      } else {
        chan_type = CHANX;
      }

      std::string chan_port_name = generate_sb_module_track_port_name(
            chan_type, side_manager.get_side(), PORTS::IN_PORT);

        /* Must find a valid port id in the Switch Block module */
        ModulePortId chan_port_id =
            module_manager.find_module_port(sb_module, chan_port_name);
        if (module_manager.valid_module_port_id(sb_module, chan_port_id) != true) continue;

        sb_input_ports.push_back(ModulePinInfo(chan_port_id, chan_node_id));
    }

    size_t datapath_mux_size = sb_input_ports.size();

    std::string mux_module_name = generate_mux_subckt_name(
    circuit_lib, mux_model, datapath_mux_size, std::string(""));
    ModuleId mux_module = module_manager.find_module(mux_module_name);
    VTR_ASSERT(true == module_manager.valid_module_id(mux_module));

    /* Get the MUX instance id from the module manager */
    size_t mux_instance_id = module_manager.num_instance(sb_module, mux_module);
    /* Instanciate the MUX Module */
    module_manager.add_child_module(sb_module, mux_module);

    std::string mux_instance_name = generate_sb_memory_instance_name(
    SWITCH_BLOCK_MUX_INSTANCE_PREFIX, chan_side, chan_node_id, std::string(""), false, direction);
    module_manager.set_child_instance_name(sb_module, mux_module, mux_instance_id,
                                         mux_instance_name);
    

    /* Link input bus port to Switch Block inputs */
    std::vector<CircuitPortId> mux_model_input_ports =
      circuit_lib.model_ports_by_type(mux_model, CIRCUIT_MODEL_PORT_INPUT, true);

    VTR_ASSERT(1 == mux_model_input_ports.size());

    ModulePortId mux_input_port_id = module_manager.find_module_port(
      mux_module, circuit_lib.port_prefix(mux_model_input_ports[0]));
    VTR_ASSERT( true == module_manager.valid_module_port_id(mux_module, mux_input_port_id));

    BasicPort mux_input_port = module_manager.module_port(mux_module, mux_input_port_id);

    VTR_ASSERT(mux_input_port.get_width() == sb_input_ports.size());

    for (size_t pin_id = 0; pin_id < sb_input_ports.size(); ++pin_id) {
      ModuleNetId net = input_port_to_module_nets.at(sb_input_ports[pin_id]);
      if (false == module_manager.net_source_exist(sb_module, net, sb_module, 0,
                                        sb_input_ports[pin_id].first,
                                        sb_input_ports[pin_id].second)) {
        module_manager.add_module_net_source(sb_module, net, sb_module, 0,
                                           sb_input_ports[pin_id].first,
                                           sb_input_ports[pin_id].second);
      }
      module_manager.add_module_net_sink(sb_module, net, mux_module,
                                       mux_instance_id, mux_input_port_id,
                                       mux_input_port.pins()[pin_id]);
    }

    /* Link output port to Switch Block outputs */
    std::vector<CircuitPortId> mux_model_output_ports =
      circuit_lib.model_ports_by_type(mux_model, CIRCUIT_MODEL_PORT_OUTPUT, true);
    VTR_ASSERT(1 == mux_model_output_ports.size());
    /* Use the port name convention in the circuit library */
    ModulePortId mux_output_port_id = module_manager.find_module_port(
      mux_module, circuit_lib.port_prefix(mux_model_output_ports[0]));
    VTR_ASSERT(true == module_manager.valid_module_port_id(mux_module,
                                                          mux_output_port_id));
    BasicPort mux_output_port =
      module_manager.module_port(mux_module, mux_output_port_id);

    ModulePortId output_chan_port_id =
            module_manager.find_module_port(sb_module, direction + "_out_TSV");
    
    ModulePinInfo sb_output_port_id = ModulePinInfo(output_chan_port_id, chan_node_id);

    BasicPort sb_output_port =
      module_manager.module_port(sb_module, sb_output_port_id.first);

    /* Check port size should match */
    VTR_ASSERT(1 == mux_output_port.get_width());
    for (size_t pin_id = 0; pin_id < mux_output_port.pins().size(); ++pin_id) {
      /* Configuring the net source */
      ModuleNetId net = create_module_source_pin_net(
        module_manager, sb_module, mux_module, mux_instance_id,
        mux_output_port_id, mux_output_port.pins()[pin_id]);
      /* Configure the net sink */
      module_manager.add_module_net_sink(sb_module, net, sb_module, 0,
                                        sb_output_port_id.first,
                                        sb_output_port_id.second);
    }

    /* Instanciate memory modules */
    /* Find the name and module id of the memory module */
    std::string mem_module_name =
      generate_mux_subckt_name(circuit_lib, mux_model, datapath_mux_size,
                              std::string(MEMORY_MODULE_POSTFIX));
    if (group_config_block) {
      mem_module_name =
        generate_mux_subckt_name(circuit_lib, mux_model, datapath_mux_size,
                                std::string(MEMORY_FEEDTHROUGH_MODULE_POSTFIX));
    }
    ModuleId mem_module = module_manager.find_module(mem_module_name);
    VTR_ASSERT(true == module_manager.valid_module_id(mem_module));

    size_t mem_instance_id = module_manager.num_instance(sb_module, mem_module);
    module_manager.add_child_module(sb_module, mem_module);
    /* Give an instance name: this name should be consistent with the block name
    * given in bitstream manager, If you want to bind the bitstream generation to
    * modules
    */
    std::string mem_instance_name = generate_sb_memory_instance_name(
      SWITCH_BLOCK_MEM_INSTANCE_PREFIX, chan_side, chan_node_id, std::string(""),
      group_config_block, direction);
    module_manager.set_child_instance_name(sb_module, mem_module, mem_instance_id,
                                          mem_instance_name);

    /* Add nets to connect regular and mode-select SRAM ports to the SRAM port of
    * memory module */
    add_module_nets_between_logic_and_memory_sram_bus(
      module_manager, sb_module, mux_module, mux_instance_id, mem_module,
      mem_instance_id, circuit_lib, mux_model);
    /* Update memory and instance list */
    size_t config_child_id = module_manager.num_configurable_children(
      sb_module, ModuleManager::e_config_child_type::LOGICAL);
    module_manager.add_configurable_child(
      sb_module, mem_module, mem_instance_id,
      group_config_block ? ModuleManager::e_config_child_type::LOGICAL
                        : ModuleManager::e_config_child_type::UNIFIED);
    /* For logical memory, define the physical memory here */
    if (group_config_block) {
      std::string physical_mem_module_name =
        generate_mux_subckt_name(circuit_lib, mux_model, datapath_mux_size,
                                std::string(MEMORY_MODULE_POSTFIX));
      ModuleId physical_mem_module =
        module_manager.find_module(physical_mem_module_name);
      VTR_ASSERT(true == module_manager.valid_module_id(physical_mem_module));
      module_manager.set_logical2physical_configurable_child(
        sb_module, config_child_id, physical_mem_module);
      std::string physical_mem_instance_name = generate_sb_memory_instance_name(
        SWITCH_BLOCK_MEM_INSTANCE_PREFIX, chan_side, chan_node_id,
        std::string(""), false, direction);
      module_manager.set_logical2physical_configurable_child_instance_name(
        sb_module, config_child_id, physical_mem_instance_name);
    }
  }

/**
 * Build the vertical Muxes for the vertical connections
 */
static void build_3d_switch_block_vertical_interc_modules(
  ModuleManager& module_manager, const ModuleId& sb_module,
  const VprDeviceAnnotation& device_annotation, const DeviceGrid& grids,
  const RRGraphView& rr_graph, const RRGSB& rr_gsb,
  const CircuitLibrary& circuit_lib, const e_side& chan_side,
  const size_t& chan_node_id,
  const std::map<ModulePinInfo, ModuleNetId>& input_port_to_module_nets,
  const bool& group_config_block, const std::string& vertical_interc_direction, const size_t& layer) {
    
    /**
     * ADD NEW MUXES TO THE SB MODULE WITH THE OUTPUT BEING THE VERTICAL CONNECTION
     * ASSUME THAT THE NEEDED MUX MODULES ARE ALREADY CREATED
     * TAKE INTO ACCOUNT THE LAYER, IF A SANDWICHED LAYER, 
     * THEN THE MUXES WILL HAVE TO TAKE IN THE INPUTS FROM THE LAYER ABOVE/BELOW ALSO
     * TO BE IMPLEMENTED
     */
    
      // assumption being made is there is more than 1 driver node for each vertical connection



      if (vertical_interc_direction == "above"){
          build_3d_switch_block_vertical_mux_module(module_manager, sb_module, device_annotation, grids, rr_graph, rr_gsb,
      circuit_lib, chan_side, chan_node_id, driver_switches[0], 
      input_port_to_module_nets, group_config_block, layer, vertical_interc_direction);
      }
      if (vertical_interc_direction == "below"){
          build_3d_switch_block_vertical_mux_module(module_manager, sb_module, device_annotation, grids, rr_graph, rr_gsb,
      circuit_lib, chan_side, chan_node_id,driver_switches[0], 
      input_port_to_module_nets, group_config_block, layer, vertical_interc_direction);
      }
  }

/********************************************************************
 * Build a module for a 3D switch block 
 * A 3D Switch Box module consists of following ports:
 * 1. Channel Y [x][y] inputs
 * 2. Channel X [x+1][y] inputs
 * 3. Channel Y [x][y-1] outputs
 * 4. Channel X [x][y] outputs
 * 5. Above Vertical Channel inputs (only if there is a layer above)
 * 6. Above Vertical Channel outputs (only if there is a layer above)
 * 7. Below Vertical Channel inputs (only if there is a layer below)
 * 8. Below Vertical Channel outputs (only if there is a layer below)
 * 9. Grid[x][y+1] Right side outputs pins
 * 10. Grid[x+1][y+1] Left side output pins
 * 11. Grid[x+1][y+1] Bottom side output pins
 * 12. Grid[x+1][y] Top side output pins
 * 13. Grid[x+1][y] Left side output pins
 * 14. Grid[x][y] Right side output pins
 * 15. Grid[x][y] Top side output pins
 * 16. Grid[x][y+1] Bottom side output pins
 *
 * Location of a Switch Box in FPGA fabric:
 *
 *    --------------          --------------
 *    |            |          |            |
 *    |    Grid    |  ChanY   |    Grid    |
 *    |  [x][y+1]  | [x][y+1] | [x+1][y+1] |
 *    |            |          |            |
 *    --------------          --------------
 *                  ----------
 *       ChanX      | Switch |     ChanX
 *       [x][y]     |   Box  |    [x+1][y]
 *                  | [x][y] |
 *                  ----------
 *    --------------          --------------
 *    |            |          |            |
 *    |    Grid    |  ChanY   |    Grid    |
 *    |   [x][y]   |  [x][y]  |  [x+1][y]  |
 *    |            |          |            |
 *    --------------          --------------
 *
 * 3D Switch Block pin location map
 *
 *                       Grid[x][y+1]   ChanY[x][y+1]  Grid[x+1][y+1] 
 *                        right_pins  inputs/outputs     left_pins
 *                            |             ^                |
 *                            |             |                |
 *                            v             v                v
 *                    +-----------------------------------------------+
 *                    |                                               |
 *                    |                                               |
 *                    |                                               |
 *                    |                                               |    
 *                    |                                               |
 *    Grid[x][y+1]    |                                               |    Grid[x+1][y+1]
 *    bottom_pins---->|                                               |<---- bottom_pins
 *                    |                                               |
 * ChanX[x][y]        |              Switch Box [x][y]                |     ChanX[x+1][y]
 * inputs/outputs<--->|                                               |<---> inputs/outputs
 *                    |                                               |
 *    Grid[x][y+1]    |                                               |    Grid[x+1][y+1]
 *       top_pins---->|                                               |<---- top_pins
 *                    |                                               |
 *                    +-----------------------------------------------+
 *                            ^             ^                ^
 *                            |             |                |
 *                            |             v                |
 *                       Grid[x][y]     ChanY[x][y]      Grid[x+1][y] 
 *                       right_pins    inputs/outputs      left_pins
 *
 *
 ********************************************************************/
void build_3d_switch_block_module(
  ModuleManager& module_manager, DecoderLibrary& decoder_lib,
  const VprDeviceAnnotation& device_annotation, const DeviceGrid& grids,
  const RRGraphView& rr_graph, const CircuitLibrary& circuit_lib,
  const e_config_protocol_type& sram_orgz_type,
  const CircuitModelId& sram_model, const DeviceRRGSB& device_rr_gsb,
  const RRGSB& rr_gsb, const bool& group_config_block, const bool& verbose, const size_t& layer) {
  /* Create a Module of Switch Block and add to module manager */
  vtr::Point<size_t> gsb_coordinate(rr_gsb.get_sb_x(), rr_gsb.get_sb_y());
  ModuleId sb_module = module_manager.add_module(
    generate_switch_block_module_name(gsb_coordinate, layer));

  /* Label module usage */
  module_manager.set_module_usage(sb_module, ModuleManager::MODULE_SB);

  VTR_LOGV(verbose, "Building module '%s'...",
           generate_switch_block_module_name(gsb_coordinate, layer).c_str());

  /* Create a cache (fast look up) for module nets whose source are input ports
   */
  std::map<ModulePinInfo, ModuleNetId> input_port_to_module_nets;

  // find the maximum input and output channel width to use for the vertical channels
  size_t input_chan_width = 0;
  size_t output_chan_width = 0;

  /* Add routing channel ports at each side of the GSB */
  for (size_t side = 0; side < rr_gsb.get_num_sides(true); ++side) {
    
    SideManager side_manager(side);

    if (side == 4) side_manager.set_side(e_side::ABOVE);
    if (side == 5) side_manager.set_side(e_side::UNDER);

    /* Count input and output port sizes */
    size_t chan_input_port_size = 0;
    size_t chan_output_port_size = 0;

    for (size_t itrack = 0;
         itrack < rr_gsb.get_chan_width(side_manager.get_side()); ++itrack) {
      switch (rr_gsb.get_chan_node_direction(side_manager.get_side(), itrack)) {
        case OUT_PORT:
          chan_output_port_size++;
          break;
        case IN_PORT:
          chan_input_port_size++;
          break;
        default:
          VTR_LOGF_ERROR(__FILE__, __LINE__,
                         "Invalid direction of chan[%d][%d][%d]_track[%d]!\n",
                         layer, rr_gsb.get_sb_x(), rr_gsb.get_sb_y(), itrack);
          exit(1);
      }
    }

    /* Do only when we have routing tracks */
    if (0 < rr_gsb.get_chan_width(side_manager.get_side())) {
      // find the maximum input and output channel width
      if(chan_input_port_size > input_chan_width) input_chan_width = chan_input_port_size;
      if(chan_output_port_size > output_chan_width) output_chan_width = chan_output_port_size;

      t_rr_type chan_type = rr_gsb.get_chan_type(side_manager.get_side());

      std::string chan_input_port_name = generate_sb_module_track_port_name(
        chan_type, side_manager.get_side(), IN_PORT);
      BasicPort chan_input_port(chan_input_port_name, chan_input_port_size);
      ModulePortId chan_input_port_id = module_manager.add_port(
        sb_module, chan_input_port, ModuleManager::MODULE_INPUT_PORT);
      /* Add side to the port */
      module_manager.set_port_side(sb_module, chan_input_port_id,
                                   side_manager.get_side());

      /* Cache the input net */
      for (const size_t& pin : chan_input_port.pins()) {
        ModuleNetId net = create_module_source_pin_net(
          module_manager, sb_module, sb_module, 0, chan_input_port_id, pin);
        input_port_to_module_nets[ModulePinInfo(chan_input_port_id, pin)] = net;
      }

      std::string chan_output_port_name = generate_sb_module_track_port_name(
        chan_type, side_manager.get_side(), OUT_PORT);
      BasicPort chan_output_port(chan_output_port_name, chan_output_port_size);
      ModulePortId chan_output_port_id = module_manager.add_port(
        sb_module, chan_output_port, ModuleManager::MODULE_OUTPUT_PORT);
      /* Add side to the port */
      module_manager.set_port_side(sb_module, chan_output_port_id,
                                   side_manager.get_side());
    }

    /* Dump OPINs of adjacent CLBs */
    for (size_t inode = 0;
         inode < rr_gsb.get_num_opin_nodes(side_manager.get_side()); ++inode) {
      vtr::Point<size_t> port_coord(rr_graph.node_xlow(rr_gsb.get_opin_node(
                                      side_manager.get_side(), inode)),
                                    rr_graph.node_ylow(rr_gsb.get_opin_node(
                                      side_manager.get_side(), inode)));
      std::string port_name = generate_3d_sb_module_grid_port_name(
        side_manager.get_side(),
        get_rr_graph_single_node_side(
          rr_graph, rr_gsb.get_opin_node(side_manager.get_side(), inode)),
        grids, device_annotation, rr_graph,
        rr_gsb.get_opin_node(side_manager.get_side(), inode));
      BasicPort module_port(port_name,
                            1); /* Every grid output has a port size of 1 */
      /* Grid outputs are inputs of switch blocks */
      ModulePortId input_port_id = module_manager.add_port(
        sb_module, module_port, ModuleManager::MODULE_INPUT_PORT);
      /* Add side to the port */
      module_manager.set_port_side(sb_module, input_port_id,
                                   side_manager.get_side());

      /* Cache the input net */
      ModuleNetId net = create_module_source_pin_net(
        module_manager, sb_module, sb_module, 0, input_port_id, 0);
      input_port_to_module_nets[ModulePinInfo(input_port_id, 0)] = net;
    }
  }

  /* Add routing multiplexers as child modules */
  for (size_t side = 0; side < rr_gsb.get_num_sides(true); ++side) {
    SideManager side_manager(side);

    if (side == 4) side_manager.set_side(e_side::ABOVE);
    if (side == 5) side_manager.set_side(e_side::UNDER);
    
    for (size_t itrack = 0;
         itrack < rr_gsb.get_chan_width(side_manager.get_side()); ++itrack) {
      /* We care OUTPUT tracks at this time only */
      if (OUT_PORT ==
          rr_gsb.get_chan_node_direction(side_manager.get_side(), itrack)) {
        build_3d_switch_block_interc_modules(
          module_manager, sb_module, device_annotation, grids, rr_graph, rr_gsb,
          circuit_lib, side_manager.get_side(), itrack,
          input_port_to_module_nets, group_config_block, layer);
      }
    }
  }

  /* Build a physical memory block */
  if (group_config_block) {
    std::string mem_module_name_prefix =
      generate_switch_block_module_name_using_index(
        device_rr_gsb.get_sb_unique_module_index(gsb_coordinate, layer));
    add_physical_memory_module(module_manager, decoder_lib, sb_module,
                               mem_module_name_prefix, circuit_lib,
                               sram_orgz_type, sram_model, verbose);
  }

  /* Add global ports to the pb_module:
   * This is a much easier job after adding sub modules (instances),
   * we just need to find all the global ports from the child modules and build
   * a list of it
   */
  add_module_global_ports_from_child_modules(module_manager, sb_module);

  /* Count shared SRAM ports from the sub-modules under this Verilog module
   * This is a much easier job after adding sub modules (instances),
   * we just need to find all the I/O ports from the child modules and build a
   * list of it
   */
  size_t module_num_shared_config_bits =
    find_module_num_shared_config_bits_from_child_modules(module_manager,
                                                          sb_module);
  if (0 < module_num_shared_config_bits) {
    add_reserved_sram_ports_to_module_manager(module_manager, sb_module,
                                              module_num_shared_config_bits);
  }

  /* Count SRAM ports from the sub-modules under this Verilog module
   * This is a much easier job after adding sub modules (instances),
   * we just need to find all the I/O ports from the child modules and build a
   * list of it
   */
  ModuleManager::e_config_child_type config_child_type =
    group_config_block ? ModuleManager::e_config_child_type::PHYSICAL
                       : ModuleManager::e_config_child_type::LOGICAL;
  size_t module_num_config_bits =
    find_module_num_config_bits_from_child_modules(
      module_manager, sb_module, circuit_lib, sram_model, sram_orgz_type,
      config_child_type);
  if (0 < module_num_config_bits) {
    add_pb_sram_ports_to_module_manager(module_manager, sb_module, circuit_lib,
                                        sram_model, sram_orgz_type,
                                        module_num_config_bits);
  }

  /* Add all the nets to connect configuration ports from memory module to
   * primitive modules This is a one-shot addition that covers all the memory
   * modules in this primitive module!
   */
  if (0 <
      module_manager.num_configurable_children(sb_module, config_child_type)) {
    add_pb_module_nets_memory_config_bus(
      module_manager, decoder_lib, sb_module, sram_orgz_type,
      circuit_lib.design_tech_type(sram_model), config_child_type);
  }

  VTR_LOGV(verbose, "Done\n");
}

}
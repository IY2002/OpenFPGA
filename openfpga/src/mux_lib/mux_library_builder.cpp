/********************************************************************
 * This file includes the functions of builders for MuxLibrary.
 *******************************************************************/
#include <cmath>

/* Headers from vtrutil library */
#include "vtr_assert.h"
#include "vtr_log.h"
#include "vtr_time.h"

/* Headers from readarchopenfpga library */
#include "circuit_library.h"
#include "circuit_library_utils.h"
#include "circuit_types.h"
#include "mux_library.h"
#include "mux_library_builder.h"
#include "mux_utils.h"
#include "openfpga_rr_graph_utils.h"
#include "pb_graph_utils.h"
#include "pb_type_utils.h"

/* Begin namespace openfpga */
namespace openfpga {

/********************************************************************
 * Update MuxLibrary with the unique multiplexer structures
 * found in the global routing architecture
 *******************************************************************/
static void build_routing_arch_mux_library(
  const RRGraphView& rr_graph, const CircuitLibrary& circuit_lib,
  const VprDeviceAnnotation& vpr_device_annotation, MuxLibrary& mux_lib, const DeviceGrid& grids) {
  /* The routing path is.
   * OPIN ----> CHAN ----> ... ----> CHAN ----> IPIN
   * Each edge is a switch, for IPIN, the switch is a connection block,
   * for the rest is a switch box
   */
  /* Count the sizes of muliplexers in routing architecture */
  for (const RRNodeId& node : rr_graph.nodes()) {
    switch (rr_graph.node_type(node)) {
      case IPIN:
      case CHANX:
      case CHANY: {
        /* Have to consider the fan_in only, it is a connection block
         * (multiplexer)*/
        if ((0 == rr_graph.node_in_edges(node).size()) ||
            (1 == rr_graph.node_in_edges(node).size())) {
          break;
        }
        /* Find the circuit_model for multiplexers in connection blocks */
        std::vector<RRSwitchId> driver_switches =
          get_rr_graph_driver_switches(rr_graph, node);
        // VTR_ASSERT(1 == driver_switches.size());
        const CircuitModelId& rr_switch_circuit_model =
          vpr_device_annotation.rr_switch_circuit_model(driver_switches[0]);
        /* we should select a circuit model for the routing resource switch */
        if (CircuitModelId::INVALID() == rr_switch_circuit_model) {
          VTR_LOG_ERROR(
            "Unable to find the circuit model for rr_switch '%s'!\n",
            rr_graph.rr_switch_inf(driver_switches[0]).name.c_str());
          VTR_LOG("Node type: %s\n", rr_graph.node_type_string(node));
          VTR_LOG("Node coordinate: %s\n",
                  rr_graph.node_coordinate_to_string(node).c_str());
          exit(1);
        }

        VTR_ASSERT(CircuitModelId::INVALID() != rr_switch_circuit_model);
        /* Add the mux to mux_library */
        std::vector<RREdgeId> in_edges = rr_graph.node_in_edges(node);

        

        // use an iterator to remove edges since the vector is modified during the loop
        // Not sure how much this is needed but idea is if I have a mixed 3D and 2D CBs, I need to account for the 2D inputs and the extra inputs of 3D
        // But if not 3D the graph shouldn't have this edge so idk if this is needed (commented out rn for testing this hypothesis)
        // for(auto it = in_edges.begin(); it != in_edges.end();) {
        //   RRNodeId src_node = rr_graph.edge_src_node(*it);
        //   RRNodeId sink_node = rr_graph.edge_sink_node(*it);

        //   // if an edge is across 2 different layers, remove it
        //   if (rr_graph.node_layer(src_node) != rr_graph.node_layer(sink_node)) {
        //     it = in_edges.erase(it);
        //   } else {
        //     ++it;
        //   }
        // }

        if (in_edges.size() < 2 ) break;
        mux_lib.add_mux(circuit_lib, rr_switch_circuit_model,
                        in_edges.size());


        /* NOTE: Old way of me doing this, the rrg didn't include the representation of the 3D sb as nodes and edges so I had to add these artifical edges. 
            Commented out right now for testing whether or not this is needed now. Making extra muxes doesn't hurt but I think it's not needed.*/
        // 3D SBs have extra inputs to muxes, inlcude both the top and bottom layer
        // 2 MUXes are added to library since we could want to have a mix of 3D and 2D SBs,
        // also CBs aren't 3D so we need to account for that as well
        // if (grids.get_num_layers() > 1) { // Need to build muxes to account for 3D SBs
        //   if (rr_graph.node_layer(node) == 0 || rr_graph.node_layer(node) == grids.get_num_layers() - 1) { // if top or bottom layer there is only 1 extra input to mux (above or below)
        //     // push invalid edge to represent the extra input, in_edges is only used for size calculation
        //     in_edges.push_back(RREdgeId::INVALID()); 
        //   } else { // else there are 2 extra inputs to mux (above and below)
        //     // push invalid edges to represent the extra input, in_edges is only used for size calculation
        //     in_edges.push_back(RREdgeId::INVALID());
        //     in_edges.push_back(RREdgeId::INVALID());
        //   }
        // }
        // mux_lib.add_mux(circuit_lib, rr_switch_circuit_model,
        //                 in_edges.size());
        break;
      }
      default:
        /* We do not care other types of rr_node */
        break;
    }
  }
}

/********************************************************************
 * For a given pin of a pb_graph_node
 * - Identify the interconnect implementation
 * - Find the number of inputs for the interconnect implementation
 * - Update the mux_library is the implementation is multiplexers
 ********************************************************************/
static void build_pb_graph_pin_interconnect_mux_library(
  t_pb_graph_pin* pb_graph_pin, t_mode* interconnect_mode,
  const CircuitLibrary& circuit_lib,
  const VprDeviceAnnotation& vpr_device_annotation, MuxLibrary& mux_lib) {
  /* Find the interconnect in the physical mode that drives this pin */
  t_interconnect* physical_interc =
    pb_graph_pin_interc(pb_graph_pin, interconnect_mode);
  /* Bypass if the interc does not indicate multiplexers */
  if (MUX_INTERC !=
      vpr_device_annotation.interconnect_physical_type(physical_interc)) {
    return;
  }
  size_t mux_size = pb_graph_pin_inputs(pb_graph_pin, physical_interc).size();
  VTR_ASSERT(true == valid_mux_implementation_num_inputs(mux_size));
  const CircuitModelId& interc_circuit_model =
    vpr_device_annotation.interconnect_circuit_model(physical_interc);
  VTR_ASSERT(CircuitModelId::INVALID() != interc_circuit_model);
  /* Add the mux model to library */
  mux_lib.add_mux(circuit_lib, interc_circuit_model, mux_size);
}

/********************************************************************
 * Update MuxLibrary with the unique multiplexer structures
 * found in programmable logic blocks
 ********************************************************************/
static void rec_build_vpr_physical_pb_graph_node_mux_library(
  t_pb_graph_node* pb_graph_node, const CircuitLibrary& circuit_lib,
  const VprDeviceAnnotation& vpr_device_annotation, MuxLibrary& mux_lib) {
  /* Find the number of inputs for each interconnect of this pb_graph_node
   * This is only applicable to each interconnect which will be implemented with
   * multiplexers
   */
  t_mode* parent_physical_mode = nullptr;

  if (false == pb_graph_node->is_root()) {
    parent_physical_mode = vpr_device_annotation.physical_mode(
      pb_graph_node->parent_pb_graph_node->pb_type);
  }

  for (int iport = 0; iport < pb_graph_node->num_input_ports; ++iport) {
    for (int ipin = 0; ipin < pb_graph_node->num_input_pins[iport]; ++ipin) {
      build_pb_graph_pin_interconnect_mux_library(
        &(pb_graph_node->input_pins[iport][ipin]), parent_physical_mode,
        circuit_lib, vpr_device_annotation, mux_lib);
    }
  }

  for (int iport = 0; iport < pb_graph_node->num_clock_ports; ++iport) {
    for (int ipin = 0; ipin < pb_graph_node->num_clock_pins[iport]; ++ipin) {
      build_pb_graph_pin_interconnect_mux_library(
        &(pb_graph_node->clock_pins[iport][ipin]), parent_physical_mode,
        circuit_lib, vpr_device_annotation, mux_lib);
    }
  }

  /* Return if this is a primitive node */
  if (true == is_primitive_pb_type(pb_graph_node->pb_type)) {
    return;
  }

  /* Get the physical mode, primitive node does not have physical mode */
  t_mode* physical_mode =
    vpr_device_annotation.physical_mode(pb_graph_node->pb_type);
  VTR_ASSERT(nullptr != physical_mode);

  /* Note that the output port interconnect should be considered for
   * non-primitive nodes */
  for (int iport = 0; iport < pb_graph_node->num_output_ports; ++iport) {
    for (int ipin = 0; ipin < pb_graph_node->num_output_pins[iport]; ++ipin) {
      build_pb_graph_pin_interconnect_mux_library(
        &(pb_graph_node->output_pins[iport][ipin]), physical_mode, circuit_lib,
        vpr_device_annotation, mux_lib);
    }
  }

  /* Visit all the child nodes in the physical mode */
  for (int ipb = 0; ipb < physical_mode->num_pb_type_children; ++ipb) {
    for (int jpb = 0; jpb < physical_mode->pb_type_children[ipb].num_pb;
         ++jpb) {
      rec_build_vpr_physical_pb_graph_node_mux_library(
        &(pb_graph_node->child_pb_graph_nodes[physical_mode->index][ipb][jpb]),
        circuit_lib, vpr_device_annotation, mux_lib);
    }
  }
}

/********************************************************************
 * Update MuxLibrary with the unique multiplexers required by
 * LUTs in the circuit library
 ********************************************************************/
static void build_lut_mux_library(MuxLibrary& mux_lib,
                                  const CircuitLibrary& circuit_lib) {
  /* Find all the circuit models which are LUTs in the circuit library */
  for (const auto& circuit_model : circuit_lib.models()) {
    /* Bypass non-LUT circuit models */
    if (CIRCUIT_MODEL_LUT != circuit_lib.model_type(circuit_model)) {
      continue;
    }
    /* Find the MUX size required by the LUT */
    /* Get input ports which are not global ports! */
    std::vector<CircuitPortId> input_ports =
      find_lut_circuit_model_input_port(circuit_lib, circuit_model, false);
    VTR_ASSERT(1 == input_ports.size());
    /* MUX size = 2^lut_size */
    size_t lut_mux_size =
      (size_t)pow(2., (double)(circuit_lib.port_size(input_ports[0])));
    /* Add mux to the mux library */
    mux_lib.add_mux(circuit_lib, circuit_model, lut_mux_size);
  }
}

/* Statistic for all the multiplexers in FPGA
 * We determine the sizes and its structure (according to spice_model) for each
 * type of multiplexers We search multiplexers in Switch Blocks, Connection
 * blocks and Configurable Logic Blocks In additional to multiplexers, this
 * function also consider crossbars. All the statistics are stored in a linked
 * list, as a return value
 */
MuxLibrary build_device_mux_library(const DeviceContext& vpr_device_ctx,
                                    const OpenfpgaContext& openfpga_ctx) {
  vtr::ScopedStartFinishTimer timer("Build a library of physical multiplexers");

  /* MuxLibrary to store the information of Multiplexers*/
  MuxLibrary mux_lib;

  /* Step 1: We should check the multiplexer spice models defined in routing
   * architecture.*/
  build_routing_arch_mux_library(vpr_device_ctx.rr_graph,
                                 openfpga_ctx.arch().circuit_lib,
                                 openfpga_ctx.vpr_device_annotation(), mux_lib, vpr_device_ctx.grid);

  /* Step 2: Count the sizes of multiplexers in complex logic blocks */
  for (const t_logical_block_type& lb_type :
       vpr_device_ctx.logical_block_types) {
    /* By pass nullptr for pb_graph head */
    if (nullptr == lb_type.pb_graph_head) {
      continue;
    }
    rec_build_vpr_physical_pb_graph_node_mux_library(
      lb_type.pb_graph_head, openfpga_ctx.arch().circuit_lib,
      openfpga_ctx.vpr_device_annotation(), mux_lib);
  }

  /* Step 3: count the size of multiplexer that will be used in LUTs*/
  build_lut_mux_library(mux_lib, openfpga_ctx.arch().circuit_lib);

  VTR_LOG("Built a multiplexer library of %lu physical multiplexers.\n",
          mux_lib.muxes().size());
  VTR_LOG("Maximum multiplexer size is %lu.\n", mux_lib.max_mux_size());
  for (auto mux_id : mux_lib.muxes()) {
    VTR_LOG("\tmodel '%s', input_size='%lu'\n",
            openfpga_ctx.arch()
              .circuit_lib.model_name(mux_lib.mux_circuit_model(mux_id))
              .c_str(),
            mux_lib.mux_graph(mux_id).num_inputs());
  }

  return mux_lib;
}

} /* End namespace openfpga*/

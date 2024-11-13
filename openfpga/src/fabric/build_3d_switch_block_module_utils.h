/**
 * Header file for the utility functions to build 3D switch block module
 * The functions in this file are modified versions of some of the ones in build_routing_module_utils.h
 */

#include <tuple>
#include <vector>

#include "device_grid.h"
#include "module_manager.h"
#include "rr_gsb.h"
#include "vpr_device_annotation.h"
#include "vpr_types.h"

namespace openfpga {
    typedef std::pair<ModulePortId, size_t> ModulePinInfo;

    ModulePinInfo find_3d_switch_block_module_chan_port(
        const ModuleManager& module_manager, const ModuleId& sb_module,
        const RRGraphView& rr_graph, const RRGSB& rr_gsb, const e_side& chan_side,
        const RRNodeId& cur_rr_node, const PORTS& cur_rr_node_direction);

    std::string generate_3d_sb_module_grid_port_name(
        const e_side& sb_side, const e_side& grid_side,
        const DeviceGrid& vpr_device_grid, const VprDeviceAnnotation& vpr_device_annotation, 
        const RRGraphView& rr_graph, const RRNodeId& rr_node);

    ModulePinInfo find_3d_switch_block_module_input_port(
        const ModuleManager& module_manager, const ModuleId& sb_module,
        const DeviceGrid& grids, const VprDeviceAnnotation& vpr_device_annotation,
        const RRGraphView& rr_graph, const RRGSB& rr_gsb, const e_side& input_side,
        const RRNodeId& input_rr_node);

    std::vector<ModulePinInfo> find_3d_switch_block_module_input_ports(
        const ModuleManager& module_manager, const ModuleId& sb_module,
        const DeviceGrid& grids, const VprDeviceAnnotation& vpr_device_annotation,
        const RRGraphView& rr_graph, const RRGSB& rr_gsb,
        const std::vector<RRNodeId>& input_rr_nodes, const size_t& layer, const size_t& node_id);
}
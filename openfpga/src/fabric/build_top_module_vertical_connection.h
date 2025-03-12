/**
 * Header file for the vertical connection of the top-level module
 * The function declarations in this file are modified versions of some of the ones in build_top_module_connection.h
 */

#ifndef BUILD_TOP_MODULE_VERTICAL_CONNECTION_H
#define BUILD_TOP_MODULE_VERTICAL_CONNECTION_H

/********************************************************************
 * Include header files that are required by function declaration
 *******************************************************************/
#include <vector>

#include "clock_network.h"
#include "config_protocol.h"
#include "device_grid.h"
#include "device_rr_gsb.h"
#include "module_manager.h"
#include "rr_clock_spatial_lookup.h"
#include "rr_graph_view.h"
#include "tile_annotation.h"
#include "vpr_device_annotation.h"
#include "vtr_geometry.h"
#include "vtr_ndmatrix.h"

namespace openfpga{
    void add_top_module_nets_connect_sb_and_sb(
        ModuleManager& module_manager, const ModuleId& top_module,
        const RRGraphView& rr_graph, const DeviceRRGSB& device_rr_gsb,
        const RRGSB& rr_gsb, const vtr::NdMatrix<size_t, 3>& sb_instance_ids,
        const bool& compact_routing_hierarchy, const size_t& layer);

    void add_top_module_nets_connect_cb_and_cb(
        ModuleManager& module_manager, const ModuleId& top_module,
        const RRGraphView& rr_graph, const DeviceRRGSB& device_rr_gsb,
        const RRGSB& rr_gsb, const vtr::NdMatrix<size_t, 3>& cb_instance_ids,
        const bool& compact_routing_hierarchy, const size_t& layer, const t_rr_type& cb_type);
}

#endif
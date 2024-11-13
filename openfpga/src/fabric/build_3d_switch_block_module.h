#ifndef BUILD_3D_SB_MODULES_H
#define BUILD_3D_SB_MODULES_H

/**
 * Header file for the functions to build 3D switch block modules
 * The functions in this file are modified versions of some of the ones in build_routing_module.h
 */

#include "circuit_library.h"
#include "decoder_library.h"
#include "device_rr_gsb.h"
#include "module_manager.h"
#include "mux_library.h"
#include "vpr_context.h"
#include "vpr_device_annotation.h"

namespace openfpga {
    void build_3d_switch_block_module(
    ModuleManager& module_manager, DecoderLibrary& decoder_lib,
    const VprDeviceAnnotation& device_annotation, const DeviceGrid& grids,
    const RRGraphView& rr_graph, const CircuitLibrary& circuit_lib,
    const e_config_protocol_type& sram_orgz_type,
    const CircuitModelId& sram_model, const DeviceRRGSB& device_rr_gsb,
    const RRGSB& rr_gsb, const bool& group_config_block, const bool& verbose, const size_t& layer);
}

#endif
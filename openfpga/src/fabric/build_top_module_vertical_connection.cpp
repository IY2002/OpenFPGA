/**
 * File containing the functions to build the vertical connections between 3D SBs in the top module
 * Functions in this file are modified versions of some of the ones in build_top_module_connection.cpp
 */

/* Headers from vtrutil library */
#include "vtr_assert.h"
#include "vtr_time.h"

/* Headers from openfpgashell library */
#include "command_exit_codes.h"

/* Headers from openfpgautil library */
#include "openfpga_side_manager.h"

/* Headers from vpr library */
#include "build_routing_module_utils.h"
#include "build_top_module_connection.h"
#include "build_top_module_utils.h"
#include "module_manager_utils.h"
#include "openfpga_device_grid_utils.h"
#include "openfpga_naming.h"
#include "openfpga_physical_tile_utils.h"
#include "openfpga_reserved_words.h"
#include "openfpga_rr_graph_utils.h"
#include "pb_type_utils.h"
#include "rr_gsb_utils.h"
#include "vpr_utils.h"
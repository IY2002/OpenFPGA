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

namespace openfpga {
    void add_top_module_nets_connect_sb_and_sb(
        ModuleManager& module_manager, const ModuleId& top_module,
        const RRGraphView& rr_graph, const DeviceRRGSB& device_rr_gsb,
        const RRGSB& rr_gsb, const vtr::NdMatrix<size_t, 3>& sb_instance_ids,
        const bool& compact_routing_hierarchy, const size_t& layer) {

            /**
             * Goal of function is to connect the vertical tracks of the switch blocks in the top module
             * If the switch block is on layer 0 then there's only the above channel where the SB at layer 0 is the source 
             * and the SB at layer 1 is the sink for the above_out_TSV track
             * 
             * If the switch block is on the last layer then there's only the below channel where the SB at the last layer is the source
             * and the SB at the second last layer is the sink for the below_out_TSV track
             * 
             * If the switch block is on any other layer then there are both above and below channels 
             * where the SB at the current layer is the source and the SB at the layer above is the sink for the above_out_TSV track
             * and the SB at the layer below is the sink for the below_out_TSV track
             * 
             * In this function each SB only adds the nets for the channels in which it is the source, since each SB is looped over
             * the nets for all channels are added to the top module eventually
             */
            
            vtr::Point<size_t> sb_coordinate = rr_gsb.get_sb_coordinate();

            if (true == compact_routing_hierarchy){
                const RRGSB& unique_mirror = device_rr_gsb.get_sb_unique_module(sb_coordinate, layer);
                sb_coordinate.set_x(unique_mirror.get_sb_x());
                sb_coordinate.set_y(unique_mirror.get_sb_y());
            }

            std::string sb_module_name = generate_switch_block_module_name(sb_coordinate, layer);

            ModuleId sb_module_id = module_manager.find_module(sb_module_name);

            size_t sb_instance_id = sb_instance_ids[layer][rr_gsb.get_sb_x()][rr_gsb.get_sb_y()];

            // ASSUMPTION BEING MADE:
            // 1. Every Switch Block has an above and under port
            // 2. The Switch in the other layer is directly above or below the current switch block
            // 3. That switch block has the same port size as the current switch block

            // TODO: Add a check for the above assumptions, and skip the SB if they are not met

            if (layer == 0){ // only worry about "above_out"
                vtr::Point<size_t> above_sb_coordinate = rr_gsb.get_sb_coordinate();
                
                if(true == compact_routing_hierarchy){
                    const RRGSB& above_sb_instance = device_rr_gsb.get_sb_unique_module(above_sb_coordinate, layer + 1);
                    above_sb_coordinate.set_x(above_sb_instance.get_sb_x());
                    above_sb_coordinate.set_y(above_sb_instance.get_sb_y());
                } 
                
                const RRGSB& above_sb = device_rr_gsb.get_gsb(above_sb_coordinate, layer + 1);

                std::string above_sb_module_name = generate_switch_block_module_name(above_sb_coordinate, layer + 1);

                std::string above_output_port_name = generate_sb_module_track_port_name(CHANX, e_side::ABOVE, PORTS::OUT_PORT);

                std::string under_input_port_name = generate_sb_module_track_port_name(CHANX, e_side::UNDER, PORTS::IN_PORT);

                ModuleId above_sb_module_id = module_manager.find_module(above_sb_module_name);

                size_t above_sb_instance_id = sb_instance_ids[layer + 1][rr_gsb.get_sb_x()][rr_gsb.get_sb_y()];

                ModulePortId sb_port_id = module_manager.find_module_port(sb_module_id, above_output_port_name);

                BasicPort sb_port = module_manager.module_port(sb_module_id, sb_port_id);

                ModulePortId above_sb_port_id = module_manager.find_module_port(above_sb_module_id, under_input_port_name);

                // connect the source SB to the sink SB (above_out to under_in)
                for (size_t itrack = 0; itrack < sb_port.get_width(); ++itrack) {
                    ModuleNetId net =
                        create_module_source_pin_net(module_manager, top_module, sb_module_id,
                                                    sb_instance_id, sb_port_id, itrack);
                    module_manager.add_module_net_sink(top_module, net, above_sb_module_id,
                                                    above_sb_instance_id, above_sb_port_id, itrack);
                }

            } else if (layer == device_rr_gsb.get_gsb_layers() - 1){ // only worry about "under_out"

                vtr::Point<size_t> under_sb_coordinate = rr_gsb.get_sb_coordinate();

                if(true == compact_routing_hierarchy){
                    const RRGSB& under_sb_instance = device_rr_gsb.get_sb_unique_module(under_sb_coordinate, layer - 1);
                    under_sb_coordinate.set_x(under_sb_instance.get_sb_x());
                    under_sb_coordinate.set_y(under_sb_instance.get_sb_y());
                }

                const RRGSB& under_sb = device_rr_gsb.get_gsb(under_sb_coordinate, layer - 1);
                std::string under_sb_module_name = generate_switch_block_module_name(under_sb_coordinate, layer - 1);

                std::string under_output_port_name = generate_sb_module_track_port_name(CHANX, e_side::UNDER, PORTS::OUT_PORT);

                std::string above_input_port_name = generate_sb_module_track_port_name(CHANX, e_side::ABOVE, PORTS::IN_PORT);

                ModuleId under_sb_module_id = module_manager.find_module(under_sb_module_name);

                size_t under_sb_instance_id = sb_instance_ids[layer - 1][rr_gsb.get_sb_x()][rr_gsb.get_sb_y()];

                ModulePortId sb_port_id = module_manager.find_module_port(sb_module_id, under_output_port_name);

                BasicPort sb_port = module_manager.module_port(sb_module_id, sb_port_id);

                ModulePortId under_sb_port_id = module_manager.find_module_port(under_sb_module_id, above_input_port_name); 

                // connect the source SB to the sink SB (under_out to above_in)
                for (size_t itrack = 0; itrack < sb_port.get_width(); ++itrack) {
                    ModuleNetId net =
                        create_module_source_pin_net(module_manager, top_module, sb_module_id,
                                                    sb_instance_id, sb_port_id, itrack);
                    module_manager.add_module_net_sink(top_module, net, under_sb_module_id,
                                                    under_sb_instance_id, under_sb_port_id, itrack);
                }


            } else{ // worry about both "above_out" and "under_out"

                vtr::Point<size_t> above_sb_coordinate = rr_gsb.get_sb_coordinate();

                if(true == compact_routing_hierarchy){
                    const RRGSB& above_sb_instance = device_rr_gsb.get_sb_unique_module(above_sb_coordinate, layer + 1);
                    above_sb_coordinate.set_x(above_sb_instance.get_sb_x());
                    above_sb_coordinate.set_y(above_sb_instance.get_sb_y());
                }

                vtr::Point<size_t> under_sb_coordinate = rr_gsb.get_sb_coordinate();

                if(true == compact_routing_hierarchy){
                    const RRGSB& under_sb_instance = device_rr_gsb.get_sb_unique_module(under_sb_coordinate, layer - 1);
                    under_sb_coordinate.set_x(under_sb_instance.get_sb_x());
                    under_sb_coordinate.set_y(under_sb_instance.get_sb_y());
                }

                const RRGSB& above_sb = device_rr_gsb.get_gsb(above_sb_coordinate, layer + 1);
                std::string above_sb_module_name = generate_switch_block_module_name(above_sb_coordinate, layer + 1);

                const RRGSB& under_sb = device_rr_gsb.get_gsb(under_sb_coordinate, layer - 1);
                std::string under_sb_module_name = generate_switch_block_module_name(under_sb_coordinate, layer - 1);

                std::string above_output_port_name = generate_sb_module_track_port_name(CHANX, e_side::ABOVE, PORTS::OUT_PORT);
                std::string under_output_port_name = generate_sb_module_track_port_name(CHANX, e_side::UNDER, PORTS::OUT_PORT);

                std::string above_input_port_name = generate_sb_module_track_port_name(CHANX, e_side::ABOVE, PORTS::IN_PORT);;
                std::string under_input_port_name = generate_sb_module_track_port_name(CHANX, e_side::UNDER, PORTS::IN_PORT);

                ModuleId above_sb_module_id = module_manager.find_module(above_sb_module_name);
                ModuleId under_sb_module_id = module_manager.find_module(under_sb_module_name);

                size_t above_sb_instance_id = sb_instance_ids[layer + 1][rr_gsb.get_sb_x()][rr_gsb.get_sb_y()];
                size_t under_sb_instance_id = sb_instance_ids[layer - 1][rr_gsb.get_sb_x()][rr_gsb.get_sb_y()];

                ModulePortId sb_port_id = module_manager.find_module_port(sb_module_id, above_output_port_name);

                BasicPort sb_port = module_manager.module_port(sb_module_id, sb_port_id);

                ModulePortId above_sb_port_id = module_manager.find_module_port(above_sb_module_id, under_input_port_name);

                // connect the source SB to the sink SB (above_out to under_in)

                for (size_t itrack = 0; itrack < sb_port.get_width(); ++itrack) {
                    ModuleNetId net =
                        create_module_source_pin_net(module_manager, top_module, sb_module_id,
                                                    sb_instance_id, sb_port_id, itrack);
                    module_manager.add_module_net_sink(top_module, net, above_sb_module_id,
                                                    above_sb_instance_id, above_sb_port_id, itrack);
                }

                sb_port_id = module_manager.find_module_port(sb_module_id, under_output_port_name);

                sb_port = module_manager.module_port(sb_module_id, sb_port_id);

                ModulePortId under_sb_port_id = module_manager.find_module_port(under_sb_module_id, above_input_port_name);

                // connect the source SB to the sink SB (under_out to above_in)
                for (size_t itrack = 0; itrack < sb_port.get_width(); ++itrack) {
                    ModuleNetId net =
                        create_module_source_pin_net(module_manager, top_module, sb_module_id,
                                                    sb_instance_id, sb_port_id, itrack);
                    module_manager.add_module_net_sink(top_module, net, under_sb_module_id,
                                                    under_sb_instance_id, under_sb_port_id, itrack);
                }

            }

    }
}
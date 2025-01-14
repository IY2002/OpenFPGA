/********************************************************************
 * This file includes functions that are used to annotate device-level
 * information, in particular the routing resource graph
 *******************************************************************/
/* Headers from vtrutil library */
#include "vtr_assert.h"
#include "vtr_log.h"
#include "vtr_time.h"

/* Headers from openfpgautil library */
#include "openfpga_side_manager.h"

/* Headers from vpr library */
#include "annotate_rr_graph.h"
#include "openfpga_rr_graph_utils.h"
#include "physical_types.h"
#include "rr_graph_view_util.h"

/* begin namespace openfpga */
namespace openfpga {

/* Build a RRChan Object with the given channel type and coorindators */
static RRChan build_one_rr_chan(const DeviceContext& vpr_device_ctx,
                                const t_rr_type& chan_type, const size_t& layer,
                                vtr::Point<size_t>& chan_coord) {
  std::vector<RRNodeId> chan_rr_nodes;

  /* Create a rr_chan object and check if it is unique in the graph */
  RRChan rr_chan;
  /* Fill the information */
  rr_chan.set_type(chan_type);

  /* Collect rr_nodes for this channel */
  chan_rr_nodes = find_rr_graph_chan_nodes(
    vpr_device_ctx.rr_graph, layer, chan_coord.x(), chan_coord.y(), chan_type);
  /* Fill the rr_chan */
  for (const RRNodeId& chan_rr_node : chan_rr_nodes) {

    // only add non vertical rr_nodes
    if (vpr_device_ctx.rr_graph.is_node_on_specific_side(chan_rr_node, e_side::TOP) || vpr_device_ctx.rr_graph.is_node_on_specific_side(chan_rr_node, e_side::BOTTOM)) {
      continue;
    }
    rr_chan.add_node(vpr_device_ctx.rr_graph, chan_rr_node,
                     vpr_device_ctx.rr_graph.node_segment(chan_rr_node));
  }

  return rr_chan;
}

/* Build a RRChan Object with the given channel type and coorindators */
static RRChan build_one_interlayer_rr_chan(const DeviceContext& vpr_device_ctx,
                                const t_rr_type& chan_type, const size_t& layer,
                                vtr::Point<size_t>& chan_coord, e_side side) {
  std::vector<RRNodeId> chan_rr_nodes;

  /* Create a rr_chan object and check if it is unique in the graph */
  RRChan rr_chan;
  /* Fill the information */
  rr_chan.set_type(chan_type);

  /* Collect rr_nodes for this channel */
  chan_rr_nodes = find_rr_graph_chan_nodes(
    vpr_device_ctx.rr_graph, layer, chan_coord.x(), chan_coord.y(), chan_type);
  /* Fill the rr_chan */
  for (const RRNodeId& chan_rr_node : chan_rr_nodes) {

    // only add interlayer rr_nodes
    if ((vpr_device_ctx.rr_graph.is_node_on_specific_side(chan_rr_node, e_side::TOP) && side == e_side::ABOVE) || 
          (vpr_device_ctx.rr_graph.is_node_on_specific_side(chan_rr_node, e_side::BOTTOM) && side == e_side::UNDER)) {
      rr_chan.add_node(vpr_device_ctx.rr_graph, chan_rr_node,
                     vpr_device_ctx.rr_graph.node_segment(chan_rr_node));
    }
  }

  return rr_chan;
}

/** 
 * Helper function to get all opin nodes on other layers for 3D CB option
 * TODO: Only get OPIN nodes that are connected to this channel, not all OPIN nodes 
*/
std::vector<RRNodeId> find_interlayer_rr_graph_grid_nodes(const RRGraphView& rr_graph,
                                               const DeviceGrid& device_grid,
                                               const size_t& layer,
                                               const int& x,
                                               const int& y,
                                               const t_rr_type& rr_type,
                                               const e_side& side,
                                               bool include_clock=false) {

    size_t num_layers = device_grid.get_num_layers();
    std::vector<RRNodeId> indices;

    VTR_ASSERT(rr_type == IPIN || rr_type == OPIN);

    /* Ensure that (x, y) is a valid location in grids */
    if (size_t(x) > device_grid.width() - 1 || size_t(y) > device_grid.height() - 1) {
      return indices;
    }

    /* Ensure we have a valid side */
    VTR_ASSERT(side != NUM_2D_SIDES);
    for (size_t ilayer = 0; ilayer < num_layers; ++ilayer) {
      if (ilayer == layer) {
        continue;
      }
      /* Find all the pins on the side of the grid */
      t_physical_tile_loc tile_loc(x, y, ilayer);
      int width_offset = device_grid.get_width_offset(tile_loc);
      int height_offset = device_grid.get_height_offset(tile_loc);
      
      for (int pin = 0; pin < device_grid.get_physical_type(tile_loc)->num_pins; ++pin) {
          /* Skip those pins have been ignored during rr_graph build-up */
          if (true == device_grid.get_physical_type(tile_loc)->is_ignored_pin[pin]) {
              /* If specified, force to include all the clock pins */
              if (!include_clock || std::find(device_grid.get_physical_type(tile_loc)->get_clock_pins_indices().begin(), device_grid.get_physical_type(tile_loc)->get_clock_pins_indices().end(), pin) == device_grid.get_physical_type(tile_loc)->get_clock_pins_indices().end()) {
                  continue;
              }
          }
          
          if (false == device_grid.get_physical_type(tile_loc)->pinloc[width_offset][height_offset][side][pin]) {
              /* Not the pin on this side, we skip */
              continue;
          }

          /* Try to find the rr node */
          RRNodeId rr_node_index = rr_graph.node_lookup().find_node(ilayer, x, y, rr_type, pin, side);
          if (rr_node_index != RRNodeId::INVALID()) {
              indices.push_back(rr_node_index);
          }
      }
    }

    return indices;
}

/* Build a General Switch Block (GSB)
 * which includes:
 * [I] A Switch Box subckt consists of following ports:
 * 1. Channel Y [x][y] inputs
 * 2. Channel X [x+1][y] inputs
 * 3. Channel Y [x][y-1] outputs
 * 4. Channel X [x][y] outputs
 * 5. Grid[x][y+1] Right side outputs pins
 * 6. Grid[x+1][y+1] Left side output pins
 * 7. Grid[x+1][y+1] Bottom side output pins
 * 8. Grid[x+1][y] Top side output pins
 * 9. Grid[x+1][y] Left side output pins
 * 10. Grid[x][y] Right side output pins
 * 11. Grid[x][y] Top side output pins
 * 12. Grid[x][y+1] Bottom side output pins
 *
 *    --------------          --------------
 *    |            |   CBY    |            |
 *    |    Grid    |  ChanY   |    Grid    |
 *    |  [x][y+1]  | [x][y+1] | [x+1][y+1] |
 *    |            |          |            |
 *    --------------          --------------
 *                  ----------
 *     ChanX & CBX  | Switch |     ChanX
 *       [x][y]     |   Box  |    [x+1][y]
 *                  | [x][y] |
 *                  ----------
 *    --------------          --------------
 *    |            |          |            |
 *    |    Grid    |  ChanY   |    Grid    |
 *    |   [x][y]   |  [x][y]  |  [x+1][y]  |
 *    |            |          |            |
 *    --------------          --------------
 * For channels chanY with INC_DIRECTION on the top side, they should be marked as outputs 
 * For channels chanY with DEC_DIRECTION on the top side, they should be marked as inputs 
 * For channels chanY with INC_DIRECTION on the bottom side, they should be marked as inputs 
 * For channels chanY with DEC_DIRECTION on the bottom side, they should be marked as outputs 
 * For channels chanX with INC_DIRECTION on the left side, they should be marked as inputs 
 * For channels chanX with DEC_DIRECTION on the left side, they should be marked as outputs
 * For channels chanX with INC_DIRECTION on the right side, they should be marked as outputs 
 * For channels chanX with DEC_DIRECTION on the right side, they should be marked as inputs
 *
 * [II] A X-direction Connection Block [x][y]
 * The connection block shares the same routing channel[x][y] with the Switch Block 
 * We just need to fill the ipin nodes at TOP and BOTTOM sides 
 * as well as properly fill the ipin_grid_side information 
 * 
 * [III] A Y-direction Connection Block [x][y+1] 
 * The connection block shares the same routing channel[x][y+1] with the Switch Block 
 * We just need to fill the ipin nodes at LEFT and RIGHT sides 
 * as well as properly fill the ipin_grid_side information
 */
static RRGSB build_rr_gsb(const DeviceContext& vpr_device_ctx,
                          const vtr::Point<size_t>& gsb_range,
                          const size_t& layer,
                          const vtr::Point<size_t>& gsb_coord,
                          const bool& perimeter_cb, const bool& include_clock,
                          const bool is_3d_cb) {
  /* Create an object to return */
  RRGSB rr_gsb;

  VTR_ASSERT(gsb_coord.x() <= gsb_range.x());
  VTR_ASSERT(gsb_coord.y() <= gsb_range.y());

  /* Coordinator initialization */
  rr_gsb.set_coordinate(gsb_coord.x(), gsb_coord.y());

  /* Basic information*/
  rr_gsb.init_num_sides(4); /* Fixed number of sides */

  /* Find all rr_nodes of channels */
  /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
  for (size_t side = 0; side < rr_gsb.get_num_sides(); ++side) {
    /* Local variables inside this for loop */
    SideManager side_manager(side);
    vtr::Point<size_t> coordinate =
      rr_gsb.get_side_block_coordinate(side_manager.get_side());
    RRChan rr_chan;
    std::vector<std::vector<RRNodeId>> temp_opin_rr_nodes(2);
    enum e_side opin_grid_side[2] = {NUM_2D_SIDES, NUM_2D_SIDES};
    enum PORTS chan_dir_to_port_dir_mapping[2] = {
      OUT_PORT, IN_PORT}; /* 0: INC_DIRECTION => ?; 1: DEC_DIRECTION => ? */

    switch (side) {
      case TOP: /* TOP = 0 */
        if (gsb_coord.y() == gsb_range.y()) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANY, layer, coordinate);
        chan_dir_to_port_dir_mapping[0] =
          OUT_PORT; /* INC_DIRECTION => OUT_PORT */
        chan_dir_to_port_dir_mapping[1] =
          IN_PORT; /* DEC_DIRECTION => IN_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x][y+1] RIGHT side outputs pins */
        opin_grid_side[0] = RIGHT;
        /* Grid[x+1][y+1] left side outputs pins */
        opin_grid_side[1] = LEFT;
        /* Include Grid[x][y+1] RIGHT side outputs pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* Include Grid[x+1][y+1] Left side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y() + 1, OPIN, opin_grid_side[1]);

        if (true == is_3d_cb){
          std::vector<std::vector<RRNodeId>> temp_opin_rr_nodes_3d(2);
          temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
            vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
            gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
          temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
            vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
            gsb_coord.y() + 1, OPIN, opin_grid_side[1]);
          temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
          temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());
        }

        break;
      case RIGHT: /* RIGHT = 1 */
        if (gsb_coord.x() == gsb_range.x()) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for top: chany[x][y+1] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANX, layer, coordinate);
        chan_dir_to_port_dir_mapping[0] =
          OUT_PORT; /* INC_DIRECTION => OUT_PORT */
        chan_dir_to_port_dir_mapping[1] =
          IN_PORT; /* DEC_DIRECTION => IN_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x+1][y+1] BOTTOM side outputs pins */
        opin_grid_side[0] = BOTTOM;
        /* Grid[x+1][y] TOP side outputs pins */
        opin_grid_side[1] = TOP;

        /* include Grid[x+1][y+1] Bottom side output pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* include Grid[x+1][y] Top side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y(), OPIN, opin_grid_side[1]);

        if (true == is_3d_cb){
          std::vector<std::vector<RRNodeId>> temp_opin_rr_nodes_3d(2);
          temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
            vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
            gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
          temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
            vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
            gsb_coord.y(), OPIN, opin_grid_side[1]);
          temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
          temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());
        }
        break;
      case BOTTOM: /* BOTTOM = 2*/
        if (!perimeter_cb && gsb_coord.y() == 0) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for bottom: chany[x][y] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANY, layer, coordinate);
        chan_dir_to_port_dir_mapping[0] =
          IN_PORT; /* INC_DIRECTION => IN_PORT */
        chan_dir_to_port_dir_mapping[1] =
          OUT_PORT; /* DEC_DIRECTION => OUT_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x+1][y] LEFT side outputs pins */
        opin_grid_side[0] = LEFT;
        /* Grid[x][y] RIGHT side outputs pins */
        opin_grid_side[1] = RIGHT;
        /* include Grid[x+1][y] Left side output pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y(), OPIN, opin_grid_side[0]);
        /* include Grid[x][y] Right side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);

        if (true == is_3d_cb){
          std::vector<std::vector<RRNodeId>> temp_opin_rr_nodes_3d(2);
          temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
            vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
            gsb_coord.y(), OPIN, opin_grid_side[0]);
          temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
            vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
            gsb_coord.y(), OPIN, opin_grid_side[1]);
          temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
          temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());
        }
        break;
      case LEFT: /* LEFT = 3 */
        if (!perimeter_cb && gsb_coord.x() == 0) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for left: chanx[x][y] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANX, layer, coordinate);
        chan_dir_to_port_dir_mapping[0] =
          IN_PORT; /* INC_DIRECTION => IN_PORT */
        chan_dir_to_port_dir_mapping[1] =
          OUT_PORT; /* DEC_DIRECTION => OUT_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Grid[x][y+1] BOTTOM side outputs pins */
        opin_grid_side[0] = BOTTOM;
        /* Grid[x][y] TOP side outputs pins */
        opin_grid_side[1] = TOP;
        /* include Grid[x][y+1] Bottom side outputs pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* include Grid[x][y] Top side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);
        
        if (true == is_3d_cb){
          std::vector<std::vector<RRNodeId>> temp_opin_rr_nodes_3d(2);
          temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
            vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
            gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
          temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
            vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
            gsb_coord.y(), OPIN, opin_grid_side[1]);
          temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
          temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());
        }
        break;
      default:
        VTR_LOG_ERROR("Invalid side index!\n");
        exit(1);
    }

    /* Organize a vector of port direction */
    if (0 < rr_chan.get_chan_width()) {
      std::vector<enum PORTS> rr_chan_dir;
      rr_chan_dir.resize(rr_chan.get_chan_width());
      for (size_t itrack = 0; itrack < rr_chan.get_chan_width(); ++itrack) {
        /* Identify the directionality, record it in rr_node_direction */
        if (Direction::INC ==
            vpr_device_ctx.rr_graph.node_direction(rr_chan.get_node(itrack))) {
          rr_chan_dir[itrack] = chan_dir_to_port_dir_mapping[0];
        } else {
          VTR_ASSERT(Direction::DEC == vpr_device_ctx.rr_graph.node_direction(
                                         rr_chan.get_node(itrack)));
          rr_chan_dir[itrack] = chan_dir_to_port_dir_mapping[1];
        }
      }
      /* Fill chan_rr_nodes */
      rr_gsb.add_chan_node(side_manager.get_side(), rr_chan, rr_chan_dir);
    }

    /* Fill opin_rr_nodes */
    /* Copy from temp_opin_rr_node to opin_rr_node */
    for (size_t opin_array_id = 0; opin_array_id < temp_opin_rr_nodes.size();
         ++opin_array_id) {
      for (const RRNodeId& inode : temp_opin_rr_nodes[opin_array_id]) {
        /* Skip those has no configurable outgoing, they should NOT appear in
         * the GSB connection This is for those grid output pins used by direct
         * connections
         */
        if (0 == vpr_device_ctx.rr_graph.num_configurable_edges(inode)) {
          continue;
        }
        /* Do not consider OPINs that directly drive an IPIN
         * they are supposed to be handled by direct connection
         */
        if (true ==
            is_opin_direct_connected_ipin(vpr_device_ctx.rr_graph, inode)) {
          continue;
        }

        rr_gsb.add_opin_node(inode, side_manager.get_side());
      }
    }

    /* Clean ipin_rr_nodes */
    /* We do not have any IPIN for a Switch Block */
    rr_gsb.clear_ipin_nodes(side_manager.get_side());

    /* Clear the temp data */
    temp_opin_rr_nodes[0].clear();
    temp_opin_rr_nodes[1].clear();
    opin_grid_side[0] = NUM_2D_SIDES;
    opin_grid_side[1] = NUM_2D_SIDES;
  }

  /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
  for (size_t side = 0; side < rr_gsb.get_num_sides(); ++side) {
    /* Local variables inside this for loop */
    SideManager side_manager(side);
    size_t ix;
    size_t iy;
    enum e_side chan_side;
    std::vector<RRNodeId> temp_ipin_rr_nodes;
    enum e_side ipin_rr_node_grid_side;

    switch (side) {
      case TOP: /* TOP = 0 */
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = LEFT;
        /* Build the connection block: ipin and ipin_grid_side */
        /* BOTTOM side INPUT Pins of Grid[x][y+1] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y() + 1;
        ipin_rr_node_grid_side = BOTTOM;
        break;
      case RIGHT: /* RIGHT = 1 */
        /* For the bording, we should take special care */
        /* Check if TOP side chan width is 0 or not */
        chan_side = BOTTOM;
        /* Build the connection block: ipin and ipin_grid_side */
        /* LEFT side INPUT Pins of Grid[x+1][y] */
        ix = rr_gsb.get_sb_x() + 1;
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = LEFT;
        break;
      case BOTTOM: /* BOTTOM = 2*/
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = LEFT;
        /* Build the connection block: ipin and ipin_grid_side */
        /* TOP side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = TOP;
        break;
      case LEFT: /* LEFT = 3 */
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = BOTTOM;
        /* Build the connection block: ipin and ipin_grid_side */
        /* RIGHT side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = RIGHT;
        break;
      default:
        VTR_LOG_ERROR("Invalid side index!\n");
        exit(1);
    }

    /* If there is no channel at this side, we skip ipin_node annotation */
    if (0 == rr_gsb.get_chan_width(chan_side)) {
      continue;
    }
    /* Collect IPIN rr_nodes*/
    temp_ipin_rr_nodes = find_rr_graph_grid_nodes(
      vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, ix, iy, IPIN,
      ipin_rr_node_grid_side, include_clock);

    if (is_3d_cb){
      std::vector<RRNodeId> temp_ipin_rr_nodes_3d = find_interlayer_rr_graph_grid_nodes(
        vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, ix, iy, IPIN, ipin_rr_node_grid_side, include_clock);
      temp_ipin_rr_nodes.insert(temp_ipin_rr_nodes.end(), temp_ipin_rr_nodes_3d.begin(), temp_ipin_rr_nodes_3d.end());
    }
    /* Fill the ipin nodes of RRGSB */
    for (const RRNodeId& inode : temp_ipin_rr_nodes) {
      /* Skip those has no configurable outgoing, they should NOT appear in the
       * GSB connection This is for those grid output pins used by direct
       * connections
       */
      if (0 ==
          vpr_device_ctx.rr_graph.node_configurable_in_edges(inode).size()) {
        continue;
      }

      /* Do not consider IPINs that are directly connected by an OPIN
       * they are supposed to be handled by direct connection
       */
      if (true ==
          is_ipin_direct_connected_opin(vpr_device_ctx.rr_graph, inode)) {
        continue;
      }

      rr_gsb.add_ipin_node(inode, side_manager.get_side());
    }
    
    /* Clear the temp data */
    temp_ipin_rr_nodes.clear();
  }

  /* Build OPIN node lists for connection blocks */
  rr_gsb.build_cb_opin_nodes(vpr_device_ctx.rr_graph, is_3d_cb);

  return rr_gsb;
}

/* Build a 3D General Switch Block (GSB)
 * which includes:
 * [I] A Switch Box subckt consists of following ports:
 * 1. Channel Y [layer][x][y] inputs
 * 2. Channel X [layer][x+1][y] inputs
 * 3. Channel Y [layer][x][y-1] outputs
 * 4. Channel X [layer][x][y] outputs
 * 5. Grid[layer][x][y+1] Right side outputs pins
 * 6. Grid[layer][x+1][y+1] Left side output pins
 * 7. Grid[layer][x+1][y+1] Bottom side output pins
 * 8. Grid[layer][x+1][y] Top side output pins
 * 9. Grid[layer][x+1][y] Left side output pins
 * 10. Grid[layer][x][y] Right side output pins
 * 11. Grid[layer][x][y] Top side output pins
 * 12. Grid[layer][x][y+1] Bottom side output pins
 * 13*. Above Channel [layer+1][x][y] inputs 
 * 14*. Below Channel [layer-1][x][y] inputs 
 * 15*. Above Channel [layer+1][x][y] outputs 
 * 16*. Below Channel [layer-1][x][y] outputs 
 * 
 * *: Only exists if there is a layer above/below
 *
 *    --------------          --------------
 *    |            |   CBY    |            |
 *    |    Grid    |  ChanY   |    Grid    |
 *    |  [x][y+1]  | [x][y+1] | [x+1][y+1] |
 *    |            |          |            |
 *    --------------          --------------
 *                  ----------
 *     ChanX & CBX  | Switch |     ChanX
 *       [x][y]     |   Box  |    [x+1][y]
 *                  | [x][y] |
 *                  ----------
 *    --------------          --------------
 *    |            |          |            |
 *    |    Grid    |  ChanY   |    Grid    |
 *    |   [x][y]   |  [x][y]  |  [x+1][y]  |
 *    |            |          |            |
 *    --------------          --------------
 * For channels chanY with INC_DIRECTION on the top side, they should be marked as outputs 
 * For channels chanY with DEC_DIRECTION on the top side, they should be marked as inputs 
 * For channels chanY with INC_DIRECTION on the bottom side, they should be marked as inputs 
 * For channels chanY with DEC_DIRECTION on the bottom side, they should be marked as outputs 
 * For channels chanX with INC_DIRECTION on the left side, they should be marked as inputs 
 * For channels chanX with DEC_DIRECTION on the left side, they should be marked as outputs
 * For channels chanX with INC_DIRECTION on the right side, they should be marked as outputs 
 * For channels chanX with DEC_DIRECTION on the right side, they should be marked as inputs
 * 
 * Vertical (Above and Below) channel nodes in the rr graph have unique properties: 
 *  1. They have no direction
 *  2. They have exactly 1 source node and 1 sink node
 *    a. One of those is on the same layer
 *    b. The other node is on a different layer
 *  3. The source and sink nodes are both channel nodes
 * 
 * How to determine whether a vertical channel should be an input or output:
 *  if the source node is on the same layer then the channel is an output of the GSB,
 *  otherwise, the channel is an input to the GSB
 * 
 * How to determine whether a vertical channel is above or below:
 *  Look at the connected node that is on a different layer,
 *  if the layer number is higher then the channel is above
 *  else, the channel is below
 *
 * [II] A X-direction Connection Block [x][y]
 * The connection block shares the same routing channel[x][y] with the Switch Block 
 * We just need to fill the ipin nodes at TOP and BOTTOM sides 
 * as well as properly fill the ipin_grid_side information 
 * 
 * [III] A Y-direction Connection Block [x][y+1] 
 * The connection block shares the same routing channel[x][y+1] with the Switch Block 
 * We just need to fill the ipin nodes at LEFT and RIGHT sides 
 * as well as properly fill the ipin_grid_side information
 */
static RRGSB build_3d_rr_gsb(const DeviceContext& vpr_device_ctx,
                          const vtr::Point<size_t>& gsb_range,
                          const size_t& layer,
                          const vtr::Point<size_t>& gsb_coord,
                          const bool& perimeter_cb, const bool& include_clock,
                          const bool is_3d_cb) {
  /* Create an object to return */
  RRGSB rr_gsb;

  VTR_ASSERT(gsb_coord.x() <= gsb_range.x());
  VTR_ASSERT(gsb_coord.y() <= gsb_range.y());

  /* Coordinator initialization */
  rr_gsb.set_coordinate(gsb_coord.x(), gsb_coord.y());

  size_t num_sides = 6;

  // // Check if there are layers above
  // if (vpr_device_ctx.grid.get_num_layers() - 1 > layer){
  //   num_sides += 1;
  // }

  // // Check if there are layers below
  // if (0 < layer){
  //   num_sides += 1;
  // }

  /* Set the number of sides of the GSB */
  rr_gsb.init_num_sides(num_sides);

  /* Find all rr_nodes of horizontal channels */
  /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3*/
  for (size_t side = 0; side < rr_gsb.get_num_sides(); ++side) {
    /* Local variables inside this for loop */
    SideManager side_manager(side);

    if (side == 4 && num_sides == 6){
      side_manager.set_side(ABOVE);
    } 
    // else if (side == 4 && num_sides == 5){ // there are only 2 scenarios where num_sides = 5, the first layer and the last layer
    //   if (layer == 0){ // first layer, only has above side
    //     side_manager.set_side(ABOVE);
    //   }
    //   if (layer == vpr_device_ctx.grid.get_num_layers() - 1){ // last layer, only has below side
    //     side_manager.set_side(UNDER);
    //   }
    // }

    if (side == 5 && num_sides == 6){
      side_manager.set_side(UNDER);
    }

    vtr::Point<size_t> coordinate = gsb_coord;
    RRChan rr_chan;
    std::vector<std::vector<RRNodeId>> temp_opin_rr_nodes(2);
      enum e_side opin_grid_side[2] = {NUM_2D_SIDES, NUM_2D_SIDES};

    // set SB coordinate for horizontal (non-interlayer) channels
    if (side < 4){
      coordinate = rr_gsb.get_side_block_coordinate(side_manager.get_side());
    }
    enum PORTS chan_dir_to_port_dir_mapping[2] = {
      OUT_PORT, IN_PORT}; /* 0: INC_DIRECTION => ?; 1: DEC_DIRECTION => ? */

    switch (side_manager.get_side()) {
      case e_side::TOP: /* TOP = 0 */
        if (gsb_coord.y() == gsb_range.y()) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANY, layer, coordinate);
        chan_dir_to_port_dir_mapping[0] =
          OUT_PORT; /* INC_DIRECTION => OUT_PORT */
        chan_dir_to_port_dir_mapping[1] =
          IN_PORT; /* DEC_DIRECTION => IN_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x][y+1] RIGHT side outputs pins */
        opin_grid_side[0] = RIGHT;
        /* Grid[x+1][y+1] left side outputs pins */
        opin_grid_side[1] = LEFT;
        /* Include Grid[x][y+1] RIGHT side outputs pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* Include Grid[x+1][y+1] Left side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y() + 1, OPIN, opin_grid_side[1]);

        break;

      case e_side::RIGHT: /* RIGHT = 1 */
        if (gsb_coord.x() == gsb_range.x()) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for top: chany[x][y+1] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANX, layer, coordinate);
        chan_dir_to_port_dir_mapping[0] =
          OUT_PORT; /* INC_DIRECTION => OUT_PORT */
        chan_dir_to_port_dir_mapping[1] =
          IN_PORT; /* DEC_DIRECTION => IN_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x+1][y+1] BOTTOM side outputs pins */
        opin_grid_side[0] = BOTTOM;
        /* Grid[x+1][y] TOP side outputs pins */
        opin_grid_side[1] = TOP;

        /* include Grid[x+1][y+1] Bottom side output pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* include Grid[x+1][y] Top side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y(), OPIN, opin_grid_side[1]);

        break;

      case e_side::BOTTOM: /* BOTTOM = 2*/
        if (!perimeter_cb && gsb_coord.y() == 0) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for bottom: chany[x][y] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANY, layer, coordinate);
        chan_dir_to_port_dir_mapping[0] =
          IN_PORT; /* INC_DIRECTION => IN_PORT */
        chan_dir_to_port_dir_mapping[1] =
          OUT_PORT; /* DEC_DIRECTION => OUT_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x+1][y] LEFT side outputs pins */
        opin_grid_side[0] = LEFT;
        /* Grid[x][y] RIGHT side outputs pins */
        opin_grid_side[1] = RIGHT;
        /* include Grid[x+1][y] Left side output pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y(), OPIN, opin_grid_side[0]);
        /* include Grid[x][y] Right side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);

        break;

      case e_side::LEFT: /* LEFT = 3 */
        if (!perimeter_cb && gsb_coord.x() == 0) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for left: chanx[x][y] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANX, layer, coordinate);
        chan_dir_to_port_dir_mapping[0] =
          IN_PORT; /* INC_DIRECTION => IN_PORT */
        chan_dir_to_port_dir_mapping[1] =
          OUT_PORT; /* DEC_DIRECTION => OUT_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Grid[x][y+1] BOTTOM side outputs pins */
        opin_grid_side[0] = BOTTOM;
        /* Grid[x][y] TOP side outputs pins */
        opin_grid_side[1] = TOP;
        /* include Grid[x][y+1] Bottom side outputs pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* include Grid[x][y] Top side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);
        break;

      case e_side::ABOVE: /* ABOVE = 5 */
        // if (gsb_coord.y() == gsb_range.y() || gsb_coord.x() == gsb_range.x()) {
        //   rr_gsb.clear_one_side(side_manager.get_side());
        //   break;
        // }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for left: chanx[x][y] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_interlayer_rr_chan(vpr_device_ctx, CHANX, layer, coordinate, (e_side)side);
        chan_dir_to_port_dir_mapping[0] =
          OUT_PORT; /* INC_DIRECTION => OUT_PORT */
        chan_dir_to_port_dir_mapping[1] =
          IN_PORT; /* DEC_DIRECTION => IN_PORT */

        break;

      case e_side::UNDER: /* UNDER = 7 */
        // if (gsb_coord.y() == gsb_range.y() || gsb_coord.x() == gsb_range.x()) {
        //   rr_gsb.clear_one_side(side);
        //   break;
        // }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for left: chanx[x][y] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_interlayer_rr_chan(vpr_device_ctx, CHANX, layer, coordinate, (e_side) side);
        chan_dir_to_port_dir_mapping[0] =
          IN_PORT; /* INC_DIRECTION => IN_PORT */
        chan_dir_to_port_dir_mapping[1] =
          OUT_PORT; /* DEC_DIRECTION => OUT_PORT */

        break;
      default:
        VTR_LOG_ERROR("Invalid side index!\n");
        exit(1);
    }

    /* Organize a vector of port direction */
    if (0 < rr_chan.get_chan_width()) {
      std::vector<enum PORTS> rr_chan_dir;
      rr_chan_dir.resize(rr_chan.get_chan_width());

      for (size_t itrack = 0; itrack < rr_chan.get_chan_width(); ++itrack) {
        /* Identify the directionality, record it in rr_node_direction */
        if (Direction::INC ==
            vpr_device_ctx.rr_graph.node_direction(rr_chan.get_node(itrack))) {
          rr_chan_dir[itrack] = chan_dir_to_port_dir_mapping[0];
        } else {
          VTR_ASSERT(Direction::DEC == vpr_device_ctx.rr_graph.node_direction(
                                         rr_chan.get_node(itrack)));
          rr_chan_dir[itrack] = chan_dir_to_port_dir_mapping[1];
        }
      }
      /* Fill chan_rr_nodes */
      rr_gsb.add_chan_node((e_side) side, rr_chan, rr_chan_dir);
    }

    /* Fill opin_rr_nodes */
    /* Copy from temp_opin_rr_node to opin_rr_node */
    for (size_t opin_array_id = 0; opin_array_id < temp_opin_rr_nodes.size();
         ++opin_array_id) {
      for (const RRNodeId& inode : temp_opin_rr_nodes[opin_array_id]) {
        /* Skip those has no configurable outgoing, they should NOT appear in
         * the GSB connection This is for those grid output pins used by direct
         * connections
         */
        if (0 == vpr_device_ctx.rr_graph.num_configurable_edges(inode)) {
          continue;
        }
        /* Do not consider OPINs that directly drive an IPIN
         * they are supposed to be handled by direct connection
         */
        if (true ==
            is_opin_direct_connected_ipin(vpr_device_ctx.rr_graph, inode)) {
          continue;
        }

        rr_gsb.add_opin_node(inode, side_manager.get_side());
      }
    }

    /* Clean ipin_rr_nodes */

    /* We do not have any IPIN for a Switch Block */
    rr_gsb.clear_ipin_nodes(side_manager.get_side());
    
    /* Clear the temp data */
    temp_opin_rr_nodes[0].clear();
    temp_opin_rr_nodes[1].clear();
    opin_grid_side[0] = NUM_2D_SIDES;
    opin_grid_side[1] = NUM_2D_SIDES;
  }

  /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
  for (size_t side = 0; side < rr_gsb.get_num_sides(); ++side) {
    // only consider the 4 sides of the GSB that are not interlayer
    // if (side > 3){
    //   break;
    // }

    /* Local variables inside this for loop */
    SideManager side_manager(side);
    if (side == 4){
      side_manager.set_side(ABOVE);
    } else if (side == 5){
      side_manager.set_side(UNDER);
    }
    size_t ix;
    size_t iy;
    enum e_side chan_side;
    std::vector<RRNodeId> temp_ipin_rr_nodes;
    enum e_side ipin_rr_node_grid_side;

    switch (side_manager.get_side()) {
      case TOP: /* TOP = 0 */
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = LEFT;
        /* Build the connection block: ipin and ipin_grid_side */
        /* BOTTOM side INPUT Pins of Grid[x][y+1] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y() + 1;
        ipin_rr_node_grid_side = BOTTOM;
        break;
      case RIGHT: /* RIGHT = 1 */
        /* For the bording, we should take special care */
        /* Check if TOP side chan width is 0 or not */
        chan_side = BOTTOM;
        /* Build the connection block: ipin and ipin_grid_side */
        /* LEFT side INPUT Pins of Grid[x+1][y] */
        ix = rr_gsb.get_sb_x() + 1;
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = LEFT;
        break;
      case BOTTOM: /* BOTTOM = 2*/
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = LEFT;
        /* Build the connection block: ipin and ipin_grid_side */
        /* TOP side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = TOP;
        break;
      case LEFT: /* LEFT = 3 */
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = BOTTOM;
        /* Build the connection block: ipin and ipin_grid_side */
        /* RIGHT side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = RIGHT;
        break;
      case ABOVE: /* ABOVE = 4 */
        chan_side = ABOVE;
        /* Build the connection block: ipin and ipin_grid_side */
        /* RIGHT side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = UNDER;
        break;
      case UNDER: /* UNDER = 5 */
        chan_side = UNDER;
        /* Build the connection block: ipin and ipin_grid_side */
        /* RIGHT side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = ABOVE;
        break;
      default:
        VTR_LOG_ERROR("Invalid side index!\n");
        exit(1);
    }

    /* If there is no channel at this side, we skip ipin_node annotation */
    if (0 == rr_gsb.get_chan_width(chan_side)) {
      continue;
    }
    /* Collect IPIN rr_nodes*/
    temp_ipin_rr_nodes = find_rr_graph_grid_nodes(
      vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, ix, iy, IPIN,
      ipin_rr_node_grid_side, include_clock);


    /* Fill the ipin nodes of RRGSB */
    for (const RRNodeId& inode : temp_ipin_rr_nodes) {
      /* Skip those has no configurable outgoing, they should NOT appear in the
       * GSB connection This is for those grid output pins used by direct
       * connections
       */
      if (0 ==
          vpr_device_ctx.rr_graph.node_configurable_in_edges(inode).size()) {
        continue;
      }

      /* Do not consider IPINs that are directly connected by an OPIN
       * they are supposed to be handled by direct connection
       */
      if (true ==
          is_ipin_direct_connected_opin(vpr_device_ctx.rr_graph, inode)) {
        continue;
      }

      rr_gsb.add_ipin_node(inode, side_manager.get_side());
    }
    
    /* Clear the temp data */
    temp_ipin_rr_nodes.clear();
  }

  /* Build OPIN node lists for connection blocks */
  rr_gsb.build_cb_opin_nodes(vpr_device_ctx.rr_graph, is_3d_cb);

  return rr_gsb;
}

/********************************************************************
 * Build the annotation for the routing resource graph
 * by collecting the nodes to the General Switch Block context
 *******************************************************************/
void annotate_device_rr_gsb(const DeviceContext& vpr_device_ctx,
                            DeviceRRGSB& device_rr_gsb,
                            const bool& include_clock,
                            const bool& verbose_output,
                            const bool is_3d_cb,
                            const bool is_3d_sb) {
  vtr::ScopedStartFinishTimer timer(
    "Build General Switch Block(GSB) annotation on top of routing resource "
    "graph");

  /* Note that the GSB array is smaller than the grids by 1 column and 1 row!!!
   */
  vtr::Point<size_t> gsb_range(vpr_device_ctx.grid.width() - 1,
                               vpr_device_ctx.grid.height() - 1);
  size_t layers = vpr_device_ctx.grid.get_num_layers();
  if (vpr_device_ctx.arch->perimeter_cb) {
    gsb_range.set(vpr_device_ctx.grid.width(), vpr_device_ctx.grid.height());
  }
  device_rr_gsb.reserve(gsb_range, layers);

  VTR_LOGV(verbose_output, "Start annotation GSB up to [%lu][%lu][%lu]\n",
           layers, gsb_range.x(), gsb_range.y());

  size_t gsb_cnt = 0;
  /* For each switch block, determine the size of array */
  for (size_t ilayer = 0; ilayer < layers; ++ilayer){
    for (size_t ix = 0; ix < gsb_range.x(); ++ix) {
      for (size_t iy = 0; iy < gsb_range.y(); ++iy) {
        /* Here we give the builder the fringe coordinates so that it can handle
        * the GSBs at the borderside correctly sort drive_rr_nodes should be
        * called if required by users
        */
        vtr::Point<size_t> sub_gsb_range(vpr_device_ctx.grid.width() - 1,
                                        vpr_device_ctx.grid.height() - 1);

        if (is_3d_sb)
        {
          const RRGSB& rr_gsb = build_3d_rr_gsb(
            vpr_device_ctx, sub_gsb_range, ilayer, vtr::Point<size_t>(ix, iy),
            vpr_device_ctx.arch->perimeter_cb, include_clock, is_3d_cb);

          /* Add to device_rr_gsb */
          vtr::Point<size_t> gsb_coordinate = rr_gsb.get_sb_coordinate();

          device_rr_gsb.add_rr_gsb(gsb_coordinate, rr_gsb, ilayer);

          gsb_cnt++; /* Update counter */
          /* Print info */
          VTR_LOG("[%lu%] Backannotated GSB[%lu][%lu][%lu]\r",
                  100 * gsb_cnt / (gsb_range.x() * gsb_range.y() * layers), ilayer, ix, iy);
        } 

        else
        {
          const RRGSB& rr_gsb = build_rr_gsb(
            vpr_device_ctx, sub_gsb_range, ilayer, vtr::Point<size_t>(ix, iy),
            vpr_device_ctx.arch->perimeter_cb, include_clock, is_3d_cb);

          /* Add to device_rr_gsb */
          vtr::Point<size_t> gsb_coordinate = rr_gsb.get_sb_coordinate();

          device_rr_gsb.add_rr_gsb(gsb_coordinate, rr_gsb, ilayer);

          gsb_cnt++; /* Update counter */
          /* Print info */
          VTR_LOG("[%lu%] Backannotated GSB[%lu][%lu][%lu]\r",
                  100 * gsb_cnt / (gsb_range.x() * gsb_range.y() * layers), ilayer, ix, iy);
        }

      }
    }
  }
  /* Report number of unique mirrors */
  VTR_LOG("Backannotated %d General Switch Blocks (GSBs).\n",
          gsb_range.x() * gsb_range.y() * layers);
}

/********************************************************************
 * Sort all the incoming edges for each channel node which are
 * output ports of the GSB
 *******************************************************************/
void sort_device_rr_gsb_chan_node_in_edges(const RRGraphView& rr_graph,
                                           DeviceRRGSB& device_rr_gsb,
                                           const bool& verbose_output,
                                           const bool is_3d_cb) {
  vtr::ScopedStartFinishTimer timer(
    "Sort incoming edges for each routing track output node of General Switch "
    "Block(GSB)");

  /* Note that the GSB array is smaller than the grids by 1 column and 1 row!!!
   */
  vtr::Point<size_t> gsb_range = device_rr_gsb.get_gsb_range();
  size_t layers = device_rr_gsb.get_gsb_layers();

  VTR_LOGV(verbose_output, "Start sorting edges for GSBs up to [%lu][%lu][%lu]\n",
          layers, gsb_range.x(), gsb_range.y());

  size_t gsb_cnt = 0;

  /* For each switch block, determine the size of array */
  for (size_t ilayer = 0; ilayer < layers; ++ilayer){
    for (size_t ix = 0; ix < gsb_range.x(); ++ix) {
      for (size_t iy = 0; iy < gsb_range.y(); ++iy) {
        vtr::Point<size_t> gsb_coordinate(ix, iy);
        RRGSB& rr_gsb = device_rr_gsb.get_mutable_gsb(gsb_coordinate, ilayer);
        rr_gsb.sort_chan_node_in_edges(rr_graph, is_3d_cb);

        gsb_cnt++; /* Update counter */

        /* Print info */
        VTR_LOG(
          "[%lu%] Sorted incoming edges for each routing track output node of "
          "GSB[%lu][%lu][%lu]\r",
          100 * gsb_cnt / (layers * gsb_range.x() * gsb_range.y()), ilayer, ix, iy);
      }
    }
  }

  /* Report number of unique mirrors */
  VTR_LOG(
    "Sorted incoming edges for each routing track output node of %d General "
    "Switch Blocks (GSBs).\n",
    layers * gsb_range.x() * gsb_range.y());
}

/********************************************************************
 * Sort all the incoming edges for each input pin node which are
 * output ports of the GSB
 *******************************************************************/
void sort_device_rr_gsb_ipin_node_in_edges(const RRGraphView& rr_graph,
                                           DeviceRRGSB& device_rr_gsb,
                                           const bool& verbose_output) {
  vtr::ScopedStartFinishTimer timer(
    "Sort incoming edges for each input pin node of General Switch Block(GSB)");

  /* Note that the GSB array is smaller than the grids by 1 column and 1 row!!!
   */
  vtr::Point<size_t> gsb_range = device_rr_gsb.get_gsb_range();
  size_t layers = device_rr_gsb.get_gsb_layers();

  VTR_LOGV(verbose_output, "Start sorting edges for GSBs up to [%lu][%lu][%lu]\n",
           layers, gsb_range.x(), gsb_range.y());

  size_t gsb_cnt = 0;

  /* For each switch block, determine the size of array */
  for(size_t ilayer = 0; ilayer < layers; ++ilayer){
    for (size_t ix = 0; ix < gsb_range.x(); ++ix) {
      for (size_t iy = 0; iy < gsb_range.y(); ++iy) {
        vtr::Point<size_t> gsb_coordinate(ix, iy);
        RRGSB& rr_gsb = device_rr_gsb.get_mutable_gsb(gsb_coordinate, ilayer);
        rr_gsb.sort_ipin_node_in_edges(rr_graph);

        gsb_cnt++; /* Update counter */

        /* Print info */
        VTR_LOG(
          "[%lu%] Sorted incoming edges for each input pin node of "
          "GSB[%lu][%lu][%lu]\r",
          100 * gsb_cnt / (layers * gsb_range.x() * gsb_range.y()), ilayer, ix, iy);
      }
    }
  }

  /* Report number of unique mirrors */
  VTR_LOG(
    "Sorted incoming edges for each input pin node of %d General Switch Blocks "
    "(GSBs).\n",
    layers * gsb_range.x() * gsb_range.y());
}

/********************************************************************
 * Build the link between rr_graph switches to their physical circuit models
 * The binding is done based on the name of rr_switches defined in the
 * OpenFPGA arch XML
 *******************************************************************/
static void annotate_rr_switch_circuit_models(
  const DeviceContext& vpr_device_ctx, const Arch& openfpga_arch,
  VprDeviceAnnotation& vpr_device_annotation, const bool& verbose_output) {
  size_t count = 0;

  for (size_t rr_switch_id = 0;
       rr_switch_id < vpr_device_ctx.rr_graph.rr_switch().size();
       rr_switch_id++) {
    std::string switch_name(
      vpr_device_ctx.rr_graph.rr_switch()[RRSwitchId(rr_switch_id)].name);
    /* Skip the delayless switch, which is only used by the edges between
     * - SOURCE and OPIN
     * - IPIN and SINK
     */
    if (switch_name == std::string(VPR_DELAYLESS_SWITCH_NAME)) {
      continue;
    }

    CircuitModelId circuit_model = CircuitModelId::INVALID();
    /* The name-to-circuit mapping is stored in either cb_switch-to-circuit or
     * sb_switch-to-circuit, Try to find one and update the device annotation
     */
    if (0 < openfpga_arch.cb_switch2circuit.count(switch_name)) {
      circuit_model = openfpga_arch.cb_switch2circuit.at(switch_name);
    }
    if (0 < openfpga_arch.sb_switch2circuit.count(switch_name)) {
      if (CircuitModelId::INVALID() != circuit_model) {
        VTR_LOG_WARN(
          "Found a connection block and a switch block switch share the same "
          "name '%s' and binded to different circuit models '%s' and "
          "'%s'!\nWill use the switch block switch binding!\n",
          switch_name.c_str(),
          openfpga_arch.circuit_lib.model_name(circuit_model).c_str(),
          openfpga_arch.circuit_lib
            .model_name(openfpga_arch.sb_switch2circuit.at(switch_name))
            .c_str());
      }
      circuit_model = openfpga_arch.sb_switch2circuit.at(switch_name);
    }

    /* Cannot find a circuit model, error out! */
    if (CircuitModelId::INVALID() == circuit_model) {
      VTR_LOG_ERROR(
        "Fail to find a circuit model for a routing resource graph switch "
        "'%s'!\nPlease check your OpenFPGA architecture XML!\n",
        switch_name.c_str());
      exit(1);
    }

    /* Check the circuit model type */
    if (CIRCUIT_MODEL_MUX !=
        openfpga_arch.circuit_lib.model_type(circuit_model)) {
      VTR_LOG_ERROR(
        "Require circuit model type '%s' for a routing resource graph switch "
        "'%s'!\nPlease check your OpenFPGA architecture XML!\n",
        CIRCUIT_MODEL_TYPE_STRING[CIRCUIT_MODEL_MUX], switch_name.c_str());
      exit(1);
    }

    /* Now update the device annotation */
    vpr_device_annotation.add_rr_switch_circuit_model(RRSwitchId(rr_switch_id),
                                                      circuit_model);
    VTR_LOGV(
      verbose_output,
      "Binded a routing resource graph switch '%s' to circuit model '%s'\n",
      switch_name.c_str(),
      openfpga_arch.circuit_lib.model_name(circuit_model).c_str());
    count++;
  }

  VTR_LOG("Binded %lu routing resource graph switches to circuit models\n",
          count);
}

/********************************************************************
 * Build the link between rr_graph routing segments to their physical circuit
 *models The binding is done based on the name of rr_segment defined in the
 * OpenFPGA arch XML
 *******************************************************************/
static void annotate_rr_segment_circuit_models(
  const DeviceContext& vpr_device_ctx, const Arch& openfpga_arch,
  VprDeviceAnnotation& vpr_device_annotation, const bool& verbose_output) {
  size_t count = 0;

  for (size_t iseg = 0; iseg < vpr_device_ctx.arch->Segments.size(); ++iseg) {
    std::string segment_name = vpr_device_ctx.arch->Segments[iseg].name;
    CircuitModelId circuit_model = CircuitModelId::INVALID();
    /* The name-to-circuit mapping is stored in either cb_switch-to-circuit or
     * sb_switch-to-circuit, Try to find one and update the device annotation
     */
    if (0 < openfpga_arch.routing_seg2circuit.count(segment_name)) {
      circuit_model = openfpga_arch.routing_seg2circuit.at(segment_name);
    }
    /* Cannot find a circuit model, error out! */
    if (CircuitModelId::INVALID() == circuit_model) {
      VTR_LOG_ERROR(
        "Fail to find a circuit model for a routing segment '%s'!\nPlease "
        "check your OpenFPGA architecture XML!\n",
        segment_name.c_str());
      exit(1);
    }

    /* Check the circuit model type */
    if (CIRCUIT_MODEL_CHAN_WIRE !=
        openfpga_arch.circuit_lib.model_type(circuit_model)) {
      VTR_LOG_ERROR(
        "Require circuit model type '%s' for a routing segment '%s'!\nPlease "
        "check your OpenFPGA architecture XML!\n",
        CIRCUIT_MODEL_TYPE_STRING[CIRCUIT_MODEL_CHAN_WIRE],
        segment_name.c_str());
      exit(1);
    }

    /* Now update the device annotation */
    vpr_device_annotation.add_rr_segment_circuit_model(RRSegmentId(iseg),
                                                       circuit_model);
    VTR_LOGV(verbose_output,
             "Binded a routing segment '%s' to circuit model '%s'\n",
             segment_name.c_str(),
             openfpga_arch.circuit_lib.model_name(circuit_model).c_str());
    count++;
  }

  VTR_LOG("Binded %lu routing segments to circuit models\n", count);
}

/********************************************************************
 * Build the link between rr_graph direct connection to their physical circuit
 *models The binding is done based on the name of directs defined in the
 * OpenFPGA arch XML
 *******************************************************************/
static void annotate_direct_circuit_models(
  const DeviceContext& vpr_device_ctx, const Arch& openfpga_arch,
  VprDeviceAnnotation& vpr_device_annotation, const bool& verbose_output) {
  size_t count = 0;

  for (int idirect = 0; idirect < vpr_device_ctx.arch->num_directs; ++idirect) {
    std::string direct_name = vpr_device_ctx.arch->Directs[idirect].name;
    /* The name-to-circuit mapping is stored in either cb_switch-to-circuit or
     * sb_switch-to-circuit, Try to find one and update the device annotation
     */
    ArchDirectId direct_id = openfpga_arch.arch_direct.direct(direct_name);
    /* Cannot find a direct, no annotation needed for this direct */
    if (ArchDirectId::INVALID() == direct_id) {
      continue;
    }

    CircuitModelId circuit_model =
      openfpga_arch.arch_direct.circuit_model(direct_id);
    /* Cannot find a circuit model, error out! */
    if (CircuitModelId::INVALID() == circuit_model) {
      VTR_LOG_ERROR(
        "Fail to find a circuit model for a direct connection '%s'!\nPlease "
        "check your OpenFPGA architecture XML!\n",
        direct_name.c_str());
      exit(1);
    }

    /* Check the circuit model type */
    if (openfpga_arch.arch_direct.type(direct_id) !=
          e_direct_type::PART_OF_CB &&
        CIRCUIT_MODEL_WIRE !=
          openfpga_arch.circuit_lib.model_type(circuit_model)) {
      VTR_LOG_ERROR(
        "Require circuit model type '%s' for a direct connection '%s'!\nPlease "
        "check your OpenFPGA architecture XML!\n",
        CIRCUIT_MODEL_TYPE_STRING[CIRCUIT_MODEL_WIRE], direct_name.c_str());
      exit(1);
    }
    if (openfpga_arch.arch_direct.type(direct_id) ==
          e_direct_type::PART_OF_CB &&
        CIRCUIT_MODEL_MUX !=
          openfpga_arch.circuit_lib.model_type(circuit_model)) {
      VTR_LOG_ERROR(
        "Require circuit model type '%s' for a direct connection '%s'!\nPlease "
        "check your OpenFPGA architecture XML!\n",
        CIRCUIT_MODEL_TYPE_STRING[CIRCUIT_MODEL_MUX], direct_name.c_str());
      exit(1);
    }

    /* Now update the device annotation */
    vpr_device_annotation.add_direct_annotation(idirect, direct_id);
    VTR_LOGV(verbose_output,
             "Binded a direct connection '%s' to circuit model '%s'\n",
             direct_name.c_str(),
             openfpga_arch.circuit_lib.model_name(circuit_model).c_str());
    count++;
  }

  VTR_LOG("Binded %lu direct connections to circuit models\n", count);
}

/********************************************************************
 * Build the link between
 * - rr_graph switches
 * - rr_graph segments
 * - directlist
 * to their physical circuit models
 *******************************************************************/
void annotate_rr_graph_circuit_models(
  const DeviceContext& vpr_device_ctx, const Arch& openfpga_arch,
  VprDeviceAnnotation& vpr_device_annotation, const bool& verbose_output) {
  /* Iterate over each rr_switch in the device context and bind with names */
  annotate_rr_switch_circuit_models(vpr_device_ctx, openfpga_arch,
                                    vpr_device_annotation, verbose_output);

  /* Iterate over each rr_segment in the device context and bind with names */
  annotate_rr_segment_circuit_models(vpr_device_ctx, openfpga_arch,
                                     vpr_device_annotation, verbose_output);

  /* Iterate over each direct connection in the device context and bind with
   * names */
  annotate_direct_circuit_models(vpr_device_ctx, openfpga_arch,
                                 vpr_device_annotation, verbose_output);
}

// Function to check if a given node is a vertical channel node or not
bool is_vertical_node(const RRGraphView& rr_graph, RRNodeId node_id){
  if ((rr_graph.node_type(node_id) == CHANX || rr_graph.node_type(node_id) == CHANY) 
                                  && rr_graph.node_direction(node_id) == Direction::NONE){
    return true;
  }
  return false;
}

// Function to get the source node for a vertical channel node
RRNodeId get_src_node(const RRGraphView& rr_graph, RRNodeId node_id){
  for (auto edge : rr_graph.node_in_edges(node_id)){
    if (rr_graph.edge_sink_node((RREdgeId) edge) == node_id) return rr_graph.edge_src_node((RREdgeId) edge);
  }
  return (RRNodeId)rr_graph.num_nodes();
}

// Function to get the sink node for a vertical channel node
RRNodeId get_sink_node(const RRGraphView& rr_graph, RRNodeId node_id){
  for (auto edge : rr_graph.edge_range(node_id)){
    if (rr_graph.edge_src_node((RREdgeId) edge) == node_id) return rr_graph.edge_sink_node((RREdgeId) edge);
  }
  return (RRNodeId) rr_graph.num_nodes();
}


/** 
 * Function to label the side and direction of vertical channel nodes
 * 
 *  The side and direction of a vertical channel will determine if 
 * its an input to the layer or an output based on the following table:
 * 
 *          +-----------------------------------------+
 *          | Side  |  Direction  |  input or output? | 
 *          |-------+-------------+-------------------|
 *          | ABOVE |     DEC     |      Input        | 
 *          |-------+-------------+-------------------|
 *          | ABOVE |     INC     |      Output       |
 *          |-------+-------------+-------------------| 
 *          | UNDER |     DEC     |      Output       | 
 *          |-------+-------------+-------------------|
 *          | UNDER |     INC     |      Input        |
 *          +-----------------------------------------+
 * 
 * These new side and direction labels will make it easier to create 3D GSBs
 * 
 * Vertical (Above and Below) channel nodes in the rr graph have unique properties: 
 *  1. They have no direction
 *  2. They have exactly 1 source node and 1 sink node
 *    a. One of those is on the same layer
 *    b. The other node is on a different layer
 *  3. The source and sink nodes are both channel nodes
 * 
 * These properties are used to identify them and label these vertical channels
 */
void annotate_interlayer_channels(RRGraphBuilder& rr_graph_builder, const RRGraphView& rr_graph){
  RRNodeId starting_node = (RRNodeId) 0;

  for (auto it = rr_graph_builder.rr_nodes().begin(); it != rr_graph_builder.rr_nodes().end(); ++it){
      RRNodeId node_id = it->id();

      bool vertical_node = is_vertical_node(rr_graph, node_id);

      if(!vertical_node) {
        rr_graph_builder.add_node_side(node_id, e_side::NUM_2D_SIDES);
        continue;
      }

      /**
       * Plan of action:
       *  1. Get all the edges for the node
       *    a. there will be exactly 2, one where the node is a sink and the other it's a source
       *  2. If the source node is on a different layer then the node, then the node is an input
       *  3. Otherwise, its an output
       *  4. if the layer number of the differring layer is higher then label the nodes side as ABOVE
       *  5. if the layer number is smaller than label the node as BELOW
       *  6. use table above to determine what direction the node should have
       */

      RRNodeId src_node = get_src_node(rr_graph, node_id);
      RRNodeId sink_node = get_sink_node(rr_graph, node_id);

      size_t node_layer = rr_graph.node_layer(node_id);
      size_t src_layer = rr_graph.node_layer(src_node);
      size_t sink_layer = rr_graph.node_layer(sink_node);

      if (node_layer != src_layer){
        // node is an input to the layer
        VTR_ASSERT(node_layer == sink_layer);

        if (src_layer > node_layer){
          // node is an input from layer above so set side and direction according to table above
          // rr_graph_builder.add_node_side(node_id, e_side::ABOVE);
          rr_graph_builder.add_node_side(node_id, e_side::TOP); // Adding Top instead of Above since the node storage only uses the 2D sides not the 3D sides
          rr_graph_builder.set_node_direction(node_id, Direction::DEC);
        }
        else {
          // node is an input from layer below so set side and direction according to table above
          // rr_graph_builder.add_node_side(node_id, e_side::UNDER);
          rr_graph_builder.add_node_side(node_id, e_side::BOTTOM); // Adding Bottom instead of Under since the node storage only uses the 2D sides not the 3D sides
          rr_graph_builder.set_node_direction(node_id, Direction::INC);
        }

      } else{
        // node is an output of the layer
        VTR_ASSERT(node_layer != sink_layer);

        if (sink_layer > node_layer){
          // node is an output from layer above so set side and direction according to table above
          // rr_graph_builder.add_node_side(node_id, e_side::ABOVE);
          rr_graph_builder.add_node_side(node_id, e_side::TOP); // Adding Top instead of Above since the node storage only uses the 2D sides not the 3D sides
          rr_graph_builder.set_node_direction(node_id, Direction::INC);
        }
        else {
          // node is an output from layer below so set side and direction according to table above
          // rr_graph_builder.add_node_side(node_id, e_side::UNDER);
          rr_graph_builder.add_node_side(node_id, e_side::BOTTOM); // Adding Bottom instead of Under since the node storage only uses the 2D sides not the 3D sides
          rr_graph_builder.set_node_direction(node_id, Direction::DEC);
        }
      }
  }
}

} /* end namespace openfpga */

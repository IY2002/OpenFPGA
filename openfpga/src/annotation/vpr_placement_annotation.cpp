/************************************************************************
 * Member functions for class VprPlacementAnnotation
 ***********************************************************************/
#include "vpr_placement_annotation.h"

#include "vtr_assert.h"
#include "vtr_log.h"

/* namespace openfpga begins */
namespace openfpga {

/************************************************************************
 * Constructors
 ***********************************************************************/

/************************************************************************
 * Public accessors
 ***********************************************************************/
std::vector<ClusterBlockId> VprPlacementAnnotation::grid_blocks(
  const vtr::Point<size_t>& grid_coord, const size_t& layer) const {
  return blocks_[layer][grid_coord.x()][grid_coord.y()];
}

/************************************************************************
 * Public mutators
 ***********************************************************************/
void VprPlacementAnnotation::init_mapped_blocks(const DeviceGrid& grids) {
  /* Size the block array with grid sizes */
  blocks_.resize({grids.get_num_layers(), grids.width(), grids.height()});

  /* Resize the number of blocks allowed per grid by the capacity of the type */
  for (size_t layer = 0; layer < grids.get_num_layers(); ++layer){
    for (size_t x = 0; x < grids.width(); ++x) {
      for (size_t y = 0; y < grids.height(); ++y) {
        /* Deposit invalid ids and we will fill later */
        blocks_[layer][x][y].resize(
          grids.get_physical_type(t_physical_tile_loc(x, y, 0))->capacity,
          ClusterBlockId::INVALID());
      }
    }
  }
}

void VprPlacementAnnotation::add_mapped_block(
  const vtr::Point<size_t>& grid_coord, const size_t& z,
  const ClusterBlockId& mapped_block, const size_t& layer) {
  VTR_ASSERT(z < grid_blocks(grid_coord, layer).size());
  if (ClusterBlockId::INVALID() != blocks_[layer][grid_coord.x()][grid_coord.y()][z]) {
    VTR_LOG("Override mapped blocks at grid[%lu][%lu][%lu][%lu]!\n", layer, grid_coord.x(),
            grid_coord.y(), z);
  }
  blocks_[layer][grid_coord.x()][grid_coord.y()][z] = mapped_block;
}

} /* End namespace openfpga*/

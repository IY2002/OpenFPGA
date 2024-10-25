/************************************************************************
 * Member functions for class DeviceRRGSB
 ***********************************************************************/
#include "device_rr_gsb.h"

#include "rr_gsb_utils.h"
#include "vtr_assert.h"
#include "vtr_log.h"

/* namespace openfpga begins */
namespace openfpga {

/************************************************************************
 * Constructors
 ***********************************************************************/
DeviceRRGSB::DeviceRRGSB(const VprDeviceAnnotation& device_annotation)
  : device_annotation_(device_annotation) {}

/************************************************************************
 * Public accessors
 ***********************************************************************/
/* get the max coordinate of the switch block array */
vtr::Point<size_t> DeviceRRGSB::get_gsb_range() const {
  size_t max_x = 0;
  size_t max_y = 0;
  /* Get the largest size of sub-arrays */
  for (size_t layer = 0; layer < rr_gsb_.size(); ++layer) {
    max_x = std::max(max_x, rr_gsb_[layer].size());
    for (size_t x = 0; x < rr_gsb_[layer].size(); ++x){
      max_y = std::max(max_y, rr_gsb_[layer][x].size());
    }
  }

  vtr::Point<size_t> coordinate(max_x, max_y);
  return coordinate;
}

size_t DeviceRRGSB::get_gsb_layers() const{
  return rr_gsb_.size();
}

/* Get a rr switch block in the array with a coordinate */
const RRGSB& DeviceRRGSB::get_gsb(const vtr::Point<size_t>& coordinate, const size_t& layer) const {
  VTR_ASSERT(validate_coordinate(coordinate, layer));
  return rr_gsb_[layer][coordinate.x()][coordinate.y()];
}

/* Get a rr switch block in the array with a coordinate */
const RRGSB& DeviceRRGSB::get_gsb(const size_t& x, const size_t& y, const size_t& layer) const {
  vtr::Point<size_t> coordinate(x, y);
  return get_gsb(coordinate, layer);
}

/* Get a rr switch block in the array with a coordinate */
const RRGSB& DeviceRRGSB::get_gsb_by_cb_coordinate(
  const vtr::Point<size_t>& coordinate, const size_t& layer) const {
  vtr::Point<size_t> gsb_coord = coordinate;
  VTR_ASSERT(validate_coordinate(gsb_coord, layer));

  return rr_gsb_[layer][gsb_coord.x()][gsb_coord.y()];
}

/* get the number of unique mirrors of switch blocks */
size_t DeviceRRGSB::get_num_cb_unique_module(const t_rr_type& cb_type) const {
  VTR_ASSERT(validate_cb_type(cb_type));
  switch (cb_type) {
    case CHANX:
      return cbx_unique_module_.size();
    case CHANY:
      return cby_unique_module_.size();
    default:
      VTR_LOG_ERROR("Invalid type of connection block!\n");
      exit(1);
  }
}
/* Identify if unique blocks are preloaded or built */
bool DeviceRRGSB::is_compressed() const { return is_compressed_; }

/* Identify if a GSB actually exists at a location */
bool DeviceRRGSB::is_gsb_exist(const RRGraphView& rr_graph,
                               const vtr::Point<size_t> coord, const size_t layer) const {
  /* Out of range, does not exist */
  if (false == validate_coordinate(coord, layer)) {
    return false;
  }

  /* If the GSB is empty, it does not exist */
  if (true == get_gsb(coord, layer).is_cb_exist(CHANX)) {
    return true;
  }

  if (true == get_gsb(coord, layer).is_cb_exist(CHANY)) {
    return true;
  }

  if (true == get_gsb(coord, layer).is_sb_exist(rr_graph)) {
    return true;
  }

  return false;
}

/* get the number of unique mirrors of switch blocks */
size_t DeviceRRGSB::get_num_sb_unique_module() const {
  return sb_unique_module_.size();
}

/* get the coordinate of unique mirrors of switch blocks */
PointWithLayer DeviceRRGSB::get_sb_unique_block_coord(size_t id) const {
  return sb_unique_module_[id];
}

/* get the coordinates of the instances of a unique switch block */
std::vector<PointWithLayer> DeviceRRGSB::get_sb_unique_block_instance_coord(
  const vtr::Point<size_t>& unique_block_coord, const size_t& layer) const {
  auto unique_module_id =
    sb_unique_module_id_[layer][unique_block_coord.x()][unique_block_coord.y()];
  std::vector<PointWithLayer> instance_map;
  for (size_t location_layer = 0; location_layer < sb_unique_module_id_.size(); ++location_layer){
    for (size_t location_x = 0; location_x < sb_unique_module_id_[0].size();
        ++location_x) {
      for (size_t location_y = 0; location_y < sb_unique_module_id_[0][0].size();
          ++location_y) {
        auto unique_module_id_instance =
          sb_unique_module_id_[location_layer][location_x][location_y];
        if (unique_module_id_instance == unique_module_id) {
          PointWithLayer new_point;
          vtr::Point<size_t> instance_coord(location_x, location_y);
          new_point.coordinates = instance_coord;
          new_point.layer = location_layer;
          instance_map.push_back(new_point);
        }
      }
    }
  }
  return instance_map;
}

/* get the coordinate of unique mirrors of connection blocks of CHANX type */
PointWithLayer DeviceRRGSB::get_cbx_unique_block_coord(size_t id) const {
  return cbx_unique_module_[id];
}

/* get the coordinates of the instances of a unique connection block of CHANX
 * type */
std::vector<PointWithLayer>
DeviceRRGSB::get_cbx_unique_block_instance_coord(
  const vtr::Point<size_t>& unique_block_coord, const size_t& layer) const {
  auto unique_module_id =
    cbx_unique_module_id_[layer][unique_block_coord.x()][unique_block_coord.y()];
  std::vector<PointWithLayer> instance_map;
  for (size_t location_layer = 0; location_layer < cbx_unique_module_id_.size(); ++location_layer){  
    for (size_t location_x = 0; location_x < cbx_unique_module_id_[0].size();
        ++location_x) {
      for (size_t location_y = 0; location_y < cbx_unique_module_id_[0][0].size();
          ++location_y) {
        auto unique_module_id_instance =
          cbx_unique_module_id_[location_layer][location_x][location_y];
        if (unique_module_id_instance == unique_module_id) {
          vtr::Point<size_t> instance_coord(location_x, location_y);
          PointWithLayer new_point;
          new_point.coordinates = instance_coord;
          new_point.layer = location_layer;
          instance_map.push_back(new_point);
        }
      }
    }
  }
  return instance_map;
}

/* get the coordinate of unique mirrors of connection blocks of CHANY type */
PointWithLayer DeviceRRGSB::get_cby_unique_block_coord(size_t id) const {
  return cby_unique_module_[id];
}

/* get the coordinates of the instances of a unique connection block of CHANY
 * type */
std::vector<PointWithLayer>
DeviceRRGSB::get_cby_unique_block_instance_coord(
  const vtr::Point<size_t>& unique_block_coord, const size_t& layer) const {
  auto unique_module_id =
    cby_unique_module_id_[layer][unique_block_coord.x()][unique_block_coord.y()];
  std::vector<PointWithLayer> instance_map;
  for (size_t location_layer = 0; location_layer < cby_unique_module_id_.size(); ++location_layer){
    for (size_t location_x = 0; location_x < cby_unique_module_id_[0].size();
        ++location_x) {
      for (size_t location_y = 0; location_y < cby_unique_module_id_[0][0].size();
          ++location_y) {
        auto unique_module_id_instance =
          cby_unique_module_id_[location_layer][location_x][location_y];
        if (unique_module_id_instance == unique_module_id) {
          vtr::Point<size_t> instance_coord(location_x, location_y);
          PointWithLayer new_point;
          new_point.coordinates = instance_coord;
          new_point.layer = location_layer;
          instance_map.push_back(new_point);
        }
      }
    }
  }
  return instance_map;
}

/* get the number of unique mirrors of switch blocks */
size_t DeviceRRGSB::get_num_gsb_unique_module() const {
  return gsb_unique_module_.size();
}

/* Get a rr switch block which a unique mirror */
const RRGSB& DeviceRRGSB::get_gsb_unique_module(const size_t& index) const {
  VTR_ASSERT(validate_gsb_unique_module_index(index));

  return rr_gsb_[gsb_unique_module_[index].layer][gsb_unique_module_[index].coordinates.x()][gsb_unique_module_[index].coordinates.y()];
}

/* Get layer of the unique GSB module at index*/
size_t DeviceRRGSB::get_gsb_unique_module_layer(const size_t& index) const {
  VTR_ASSERT(validate_gsb_unique_module_index(index));

  return gsb_unique_module_[index].layer;
}

/* Get a rr switch block which a unique mirror */
const RRGSB& DeviceRRGSB::get_sb_unique_module(const size_t& index) const {
  VTR_ASSERT(validate_sb_unique_module_index(index));

  return rr_gsb_[sb_unique_module_[index].layer][sb_unique_module_[index].coordinates.x()][sb_unique_module_[index].coordinates.y()];
}

/* Get layer of unique SB module at index */
size_t DeviceRRGSB::get_sb_unique_module_layer(const size_t& index) const {
  VTR_ASSERT(validate_sb_unique_module_index(index));

  return sb_unique_module_[index].layer;
}

/* Get a rr switch block which a unique mirror */
const RRGSB& DeviceRRGSB::get_cb_unique_module(const t_rr_type& cb_type,
                                               const size_t& index) const {
  VTR_ASSERT(validate_cb_unique_module_index(cb_type, index));
  VTR_ASSERT(validate_cb_type(cb_type));
  switch (cb_type) {
    case CHANX:
      return rr_gsb_[cbx_unique_module_[index].layer]
                    [cbx_unique_module_[index].coordinates.x()]
                    [cbx_unique_module_[index].coordinates.y()];
    case CHANY:
      return rr_gsb_[cby_unique_module_[index].layer]
                    [cby_unique_module_[index].coordinates.x()]
                    [cby_unique_module_[index].coordinates.y()];
    default:
      VTR_LOG_ERROR("Invalid type of connection block!\n");
      exit(1);
  }
}

/* Get layer of unique CB module at index */
size_t DeviceRRGSB::get_cb_unique_module_layer(const t_rr_type& cb_type,
                                               const size_t& index) const {
  VTR_ASSERT(validate_cb_unique_module_index(cb_type, index));
  VTR_ASSERT(validate_cb_type(cb_type));
  switch (cb_type) {
    case CHANX:
      return cbx_unique_module_[index].layer;
    case CHANY:
      return cby_unique_module_[index].layer;
    default:
      VTR_LOG_ERROR("Invalid type of connection block!\n");
      exit(1);
  }
}

/* Give a coordinate of a rr switch block, and return its unique mirror */
const RRGSB& DeviceRRGSB::get_cb_unique_module(
  const t_rr_type& cb_type, const vtr::Point<size_t>& coordinate, const size_t& layer) const {
  return get_cb_unique_module(cb_type,
                              get_cb_unique_module_index(cb_type, coordinate, layer));
}

/* Give a coordinate of a rr switch block, and return its unique mirror */
const RRGSB& DeviceRRGSB::get_sb_unique_module(
  const vtr::Point<size_t>& coordinate, const size_t& layer) const {
  return get_sb_unique_module(get_sb_unique_module_index(coordinate, layer));
}

/************************************************************************
 * Public mutators
 ***********************************************************************/

/* Pre-allocate the rr_switch_block array that the device requires */
void DeviceRRGSB::reserve(const vtr::Point<size_t>& coordinate, const size_t& layer) {
  rr_gsb_.resize(layer);

  gsb_unique_module_id_.resize(layer);

  sb_unique_module_id_.resize(layer);

  cbx_unique_module_id_.resize(layer);
  cby_unique_module_id_.resize(layer);

  for (size_t cur_layer = 0; cur_layer < layer; ++cur_layer){
    rr_gsb_[cur_layer].resize(coordinate.x());

    gsb_unique_module_id_[cur_layer].resize(coordinate.x());

    sb_unique_module_id_[cur_layer].resize(coordinate.x());

    cbx_unique_module_id_[cur_layer].resize(coordinate.x());
    cby_unique_module_id_[cur_layer].resize(coordinate.x());

    for (size_t x = 0; x < coordinate.x(); ++x) {
      rr_gsb_[cur_layer][x].resize(coordinate.y());

      gsb_unique_module_id_[cur_layer][x].resize(coordinate.y());

      sb_unique_module_id_[cur_layer][x].resize(coordinate.y());

      cbx_unique_module_id_[cur_layer][x].resize(coordinate.y());
      cby_unique_module_id_[cur_layer][x].resize(coordinate.y());
    }
  }
}

void DeviceRRGSB::reserve_unique_modules() {
  /* As rr_gsb_ has been built, it has valid size. Will reserve space for
   * unique blocks according to rr_gsb_'s size*/

  //Space for each layer
  sb_unique_module_id_.resize(rr_gsb_.size());
  cbx_unique_module_id_.resize(rr_gsb_.size());
  cby_unique_module_id_.resize(rr_gsb_.size());

  for (std::size_t layer = 0; layer < rr_gsb_.size(); ++layer) {
    // space for each X coord
    sb_unique_module_id_[layer].resize(rr_gsb_[layer].size());
    cbx_unique_module_id_[layer].resize(rr_gsb_[layer].size());
    cby_unique_module_id_[layer].resize(rr_gsb_[layer].size());
    for (size_t x = 0; x < rr_gsb_[layer].size(); ++x){
      // space for each y coord
      sb_unique_module_id_[layer][x].resize(rr_gsb_[layer][x].size());
      cbx_unique_module_id_[layer][x].resize(rr_gsb_[layer][x].size());
      cby_unique_module_id_[layer][x].resize(rr_gsb_[layer][x].size());
    }
  }
}

/* Resize rr_switch_block array is needed*/
void DeviceRRGSB::resize_upon_need(const vtr::Point<size_t>& coordinate, const size_t& layer) {
  if (layer + 1 > rr_gsb_.size()){
    rr_gsb_.resize(layer + 1);

    sb_unique_module_id_.resize(layer + 1);

    cbx_unique_module_id_.resize(layer + 1);
    cby_unique_module_id_.resize(layer + 1);
  }
  
  if (coordinate.x() + 1 > rr_gsb_[layer].size()) {
    rr_gsb_[layer].resize(coordinate.x() + 1);

    sb_unique_module_id_[layer].resize(coordinate.x() + 1);

    cbx_unique_module_id_[layer].resize(coordinate.x() + 1);
    cby_unique_module_id_[layer].resize(coordinate.x() + 1);
  }

  if (coordinate.y() + 1 > rr_gsb_[layer][coordinate.x()].size()) {
    rr_gsb_[layer][coordinate.x()].resize(coordinate.y() + 1);
    sb_unique_module_id_[layer][coordinate.x()].resize(coordinate.y() + 1);

    cbx_unique_module_id_[layer][coordinate.x()].resize(coordinate.y() + 1);
    cby_unique_module_id_[layer][coordinate.x()].resize(coordinate.y() + 1);
  }
}

/* Add a switch block to the array, which will automatically identify and
 * update the lists of unique mirrors and rotatable mirrors */
void DeviceRRGSB::add_rr_gsb(const vtr::Point<size_t>& coordinate,
                             const RRGSB& rr_gsb, const size_t& layer) {
  /* Resize upon needs*/
  resize_upon_need(coordinate, layer);

  /* Add the switch block into array */
  rr_gsb_[layer][coordinate.x()][coordinate.y()] = rr_gsb;
}

/* Get a rr switch block in the array with a coordinate */
RRGSB& DeviceRRGSB::get_mutable_gsb(const vtr::Point<size_t>& coordinate, const size_t& layer) {
  VTR_ASSERT(validate_coordinate(coordinate, layer));
  return rr_gsb_[layer][coordinate.x()][coordinate.y()];
}

/* Get a rr switch block in the array with a coordinate */
RRGSB& DeviceRRGSB::get_mutable_gsb(const size_t& x, const size_t& y, const size_t& layer) {
  vtr::Point<size_t> coordinate(x, y);
  return get_mutable_gsb(coordinate, layer);
}

/* Add a switch block to the array, which will automatically identify and
 * update the lists of unique mirrors and rotatable mirrors */
void DeviceRRGSB::build_cb_unique_module(const RRGraphView& rr_graph,
                                         const t_rr_type& cb_type) {
  /* Make sure a clean start */
  clear_cb_unique_module(cb_type);

  for (size_t ilayer = 0; ilayer < rr_gsb_.size(); ilayer++){
    for (size_t ix = 0; ix < rr_gsb_[ilayer].size(); ++ix) {
      for (size_t iy = 0; iy < rr_gsb_[ilayer][ix].size(); ++iy) {
        bool is_unique_module = true;
        vtr::Point<size_t> gsb_coordinate(ix, iy);

        /* Bypass non-exist CB */
        if (false == rr_gsb_[ilayer][ix][iy].is_cb_exist(cb_type)) {
          continue;
        }

        /* Traverse the unique_mirror list and check it is an mirror of another
        */
        for (size_t id = 0; id < get_num_cb_unique_module(cb_type); ++id) {
          const RRGSB& unique_module = get_cb_unique_module(cb_type, id);
          if (true == is_cb_mirror(rr_graph, device_annotation_, rr_gsb_[ilayer][ix][iy],
                                  unique_module, cb_type)) {
            /* This is a mirror, raise the flag and we finish */
            is_unique_module = false;
            /* Record the id of unique mirror */
            set_cb_unique_module_id(cb_type, gsb_coordinate, id, ilayer);
            break;
          }
        }
        /* Add to list if this is a unique mirror*/
        if (true == is_unique_module) {
          add_cb_unique_module(cb_type, gsb_coordinate, ilayer);
          /* Record the id of unique mirror */
          set_cb_unique_module_id(cb_type, gsb_coordinate,
                                  get_num_cb_unique_module(cb_type) - 1, ilayer);
        }
      }
    }
  }
}

/* Add a switch block to the array, which will automatically identify and
 * update the lists of unique mirrors and rotatable mirrors */
void DeviceRRGSB::build_sb_unique_module(const RRGraphView& rr_graph) {
  /* Make sure a clean start */
  clear_sb_unique_module();

  /* Build the unique module */
  for (size_t ilayer = 0; ilayer <  rr_gsb_.size(); ++ilayer){
    for (size_t ix = 0; ix < rr_gsb_[ilayer].size(); ++ix) {
      for (size_t iy = 0; iy < rr_gsb_[ilayer][ix].size(); ++iy) {
        bool is_unique_module = true;
        vtr::Point<size_t> sb_coordinate(ix, iy);

        /* Traverse the unique_mirror list and check it is an mirror of another
        */
        for (size_t id = 0; id < get_num_sb_unique_module(); ++id) {
          /* Check if the two modules have the same submodules,
          * if so, these two modules are the same, indicating the sb is not
          * unique. else the sb is unique
          */
          const RRGSB& unique_module = get_sb_unique_module(id);
          if (true == is_sb_mirror(rr_graph, device_annotation_, rr_gsb_[ilayer][ix][iy],
                                  unique_module)) {
            /* This is a mirror, raise the flag and we finish */
            is_unique_module = false;
            /* Record the id of unique mirror */
            sb_unique_module_id_[ilayer][ix][iy] = id;
            break;
          }
        }

        /* Add to list if this is a unique mirror*/
        if (true == is_unique_module) {
          PointWithLayer new_point;
          new_point.coordinates = sb_coordinate;
          new_point.layer = ilayer;
          sb_unique_module_.push_back(new_point);
          /* Record the id of unique mirror */
          sb_unique_module_id_[ilayer][ix][iy] = sb_unique_module_.size() - 1;
        }
      }
    }
  }
}

/* Add a switch block to the array, which will automatically identify and
 * update the lists of unique mirrors and rotatable mirrors */

/* Find repeatable GSB block in the array */
void DeviceRRGSB::build_gsb_unique_module() {
  /* Make sure a clean start */
  clear_gsb_unique_module();

  for(size_t ilayer=0; ilayer < rr_gsb_.size(); ++ilayer){
    for (size_t ix = 0; ix < rr_gsb_[ilayer].size(); ++ix) {
      for (size_t iy = 0; iy < rr_gsb_[ilayer][ix].size(); ++iy) {
        bool is_unique_module = true;
        vtr::Point<size_t> gsb_coordinate(ix, iy);

        /* Traverse the unique_mirror list and check it is an mirror of another
        */
        for (size_t id = 0; id < get_num_gsb_unique_module(); ++id) {
          /* We have alreay built sb and cb unique module list
          * We just need to check if the unique module id of SBs, CBX and CBY
          * are the same or not
          */
          const vtr::Point<size_t>& gsb_unique_module_coordinate =
            gsb_unique_module_[id].coordinates;
          size_t gsb_unique_module_layer = gsb_unique_module_[id].layer;
          if ((sb_unique_module_id_[ilayer][ix][iy] ==
              sb_unique_module_id_[gsb_unique_module_layer][gsb_unique_module_coordinate.x()]
                                  [gsb_unique_module_coordinate.y()]) &&
              (cbx_unique_module_id_[ilayer][ix][iy] ==
              cbx_unique_module_id_[gsb_unique_module_layer][gsb_unique_module_coordinate.x()]
                                    [gsb_unique_module_coordinate.y()]) &&
              (cby_unique_module_id_[ilayer][ix][iy] ==
              cby_unique_module_id_[gsb_unique_module_layer][gsb_unique_module_coordinate.x()]
                                    [gsb_unique_module_coordinate.y()])) {
            /* This is a mirror, raise the flag and we finish */
            is_unique_module = false;
            /* Record the id of unique mirror */
            gsb_unique_module_id_[ilayer][ix][iy] = id;
            break;
          }
        }
        /* Add to list if this is a unique mirror*/
        if (true == is_unique_module) {
          add_gsb_unique_module(gsb_coordinate, ilayer);
          /* Record the id of unique mirror */
          gsb_unique_module_id_[ilayer][ix][iy] = get_num_gsb_unique_module() - 1;
        }
      }
    }
  }
  is_compressed_ = true;
}

void DeviceRRGSB::build_unique_module(const RRGraphView& rr_graph) {
  build_sb_unique_module(rr_graph);

  build_cb_unique_module(rr_graph, CHANX);
  build_cb_unique_module(rr_graph, CHANY);

  build_gsb_unique_module(); /*is_compressed_ flip inside
                                build_gsb_unique_module*/
}

void DeviceRRGSB::add_gsb_unique_module(const vtr::Point<size_t>& coordinate, const size_t& layer) {
  PointWithLayer new_point;
  new_point.coordinates = coordinate;
  new_point.layer = layer;
  gsb_unique_module_.push_back(new_point);
}

void DeviceRRGSB::add_cb_unique_module(const t_rr_type& cb_type,
                                       const vtr::Point<size_t>& coordinate, const size_t& layer) {
  VTR_ASSERT(validate_cb_type(cb_type));
  PointWithLayer new_point;
  new_point.coordinates = coordinate;
  new_point.layer = layer;
  switch (cb_type) {
    case CHANX:
      cbx_unique_module_.push_back(new_point);
      return;
    case CHANY:
      cby_unique_module_.push_back(new_point);
      return;
    default:
      VTR_LOG_ERROR("Invalid type of connection block!\n");
      exit(1);
  }
}

void DeviceRRGSB::set_cb_unique_module_id(const t_rr_type& cb_type,
                                          const vtr::Point<size_t>& coordinate,
                                          size_t id,
                                          const size_t& layer) {
  VTR_ASSERT(validate_cb_type(cb_type));
  size_t x = coordinate.x();
  size_t y = coordinate.y();
  switch (cb_type) {
    case CHANX:
      cbx_unique_module_id_[layer][x][y] = id;
      return;
    case CHANY:
      cby_unique_module_id_[layer][x][y] = id;
      return;
    default:
      VTR_LOG_ERROR("Invalid type of connection block!\n");
      exit(1);
  }
}

/************************************************************************
 * Public clean-up functions:
 ***********************************************************************/
/* clean the content */
void DeviceRRGSB::clear() {
  clear_gsb();

  clear_gsb_unique_module();
  clear_gsb_unique_module_id();

  /* clean unique module lists */
  clear_cb_unique_module(CHANX);
  clear_cb_unique_module_id(CHANX);

  clear_cb_unique_module(CHANY);
  clear_cb_unique_module_id(CHANY);

  clear_sb_unique_module();
  clear_sb_unique_module_id();
  is_compressed_ = false;
}

void DeviceRRGSB::clear_unique_modules() {
  /* clean unique module lists */
  clear_cb_unique_module(CHANX);
  clear_cb_unique_module_id(CHANX);

  clear_cb_unique_module(CHANY);
  clear_cb_unique_module_id(CHANY);

  clear_sb_unique_module();
  clear_sb_unique_module_id();
  is_compressed_ = false;
}

void DeviceRRGSB::clear_gsb() {
  /* clean gsb array */
  for (size_t layer = 0; layer < rr_gsb_.size(); ++layer){
    for (size_t x = 0; x < rr_gsb_[0].size(); ++x){
      rr_gsb_[layer][x].clear();
    }
    rr_gsb_[layer].clear();
  }
  rr_gsb_.clear();
}

void DeviceRRGSB::clear_gsb_unique_module_id() {
  /* clean rr_switch_block array */
  for (size_t layer = 0; layer < rr_gsb_.size(); ++layer){
    for (size_t x = 0; x < rr_gsb_.size(); ++x) {
      gsb_unique_module_id_[layer][x].clear();
    }
    gsb_unique_module_id_[layer].clear();
  }
  gsb_unique_module_id_.clear();
}

void DeviceRRGSB::clear_sb_unique_module_id() {
  /* clean rr_switch_block array */
  for (size_t layer = 0; layer < rr_gsb_.size(); ++layer){
    for (size_t x = 0; x < rr_gsb_.size(); ++x) {
      sb_unique_module_id_[layer][x].clear();
    }
    sb_unique_module_id_[layer].clear();
  }
  sb_unique_module_id_.clear();
}

void DeviceRRGSB::clear_cb_unique_module_id(const t_rr_type& cb_type) {
  VTR_ASSERT(validate_cb_type(cb_type));
  switch (cb_type) {
    case CHANX:
      for (size_t layer = 0; layer < rr_gsb_.size(); ++layer){
        for (size_t x = 0; x < rr_gsb_.size(); ++x) {
          cbx_unique_module_id_[layer][x].clear();
        }
        cbx_unique_module_id_[layer].clear();
      }
      cbx_unique_module_id_.clear();
      return;
    case CHANY:
      for (size_t layer = 0; layer < rr_gsb_.size(); ++layer){
        for (size_t x = 0; x < rr_gsb_.size(); ++x) {
          cby_unique_module_id_[layer][x].clear();
        }
        cby_unique_module_id_[layer].clear();
      }
      cby_unique_module_id_.clear();
      return;
    default:
      VTR_LOG_ERROR("Invalid type of connection block!\n");
      exit(1);
  }
}

/* clean the content related to unique_mirrors */
void DeviceRRGSB::clear_gsb_unique_module() {
  /* clean unique mirror */
  gsb_unique_module_.clear();
}

/* clean the content related to unique_mirrors */
void DeviceRRGSB::clear_sb_unique_module() {
  /* clean unique mirror */
  sb_unique_module_.clear();
}

void DeviceRRGSB::clear_cb_unique_module(const t_rr_type& cb_type) {
  VTR_ASSERT(validate_cb_type(cb_type));
  switch (cb_type) {
    case CHANX:
      cbx_unique_module_.clear();
      return;
    case CHANY:
      cby_unique_module_.clear();
      return;
    default:
      VTR_LOG_ERROR("Invalid type of connection block!\n");
      exit(1);
  }
}

/************************************************************************
 * Internal validators
 ***********************************************************************/
/* Validate if the (x,y) is the range of this device */
bool DeviceRRGSB::validate_coordinate(
  const vtr::Point<size_t>& coordinate, const size_t& layer) const {
  if (layer > rr_gsb_.capacity()){
    return false;
  }
  if (coordinate.x() >= rr_gsb_[0].capacity()) {
    return false;
  }
  return (coordinate.y() < rr_gsb_[layer][coordinate.x()].capacity());
}

/* Validate if the index in the range of unique_mirror vector*/
bool DeviceRRGSB::validate_gsb_unique_module_index(const size_t& index) const {
  return (index < gsb_unique_module_.size());
}

/* Validate if the index in the range of unique_mirror vector*/
bool DeviceRRGSB::validate_sb_unique_module_index(const size_t& index) const {
  return (index < sb_unique_module_.size());
}

bool DeviceRRGSB::validate_cb_unique_module_index(const t_rr_type& cb_type,
                                                  const size_t& index) const {
  VTR_ASSERT(validate_cb_type(cb_type));
  switch (cb_type) {
    case CHANX:
      return (index < cbx_unique_module_.size());
    case CHANY:
      return (index < cby_unique_module_.size());
    default:
      VTR_LOG_ERROR("Invalid type of connection block!\n");
      exit(1);
  }

  return false;
}

bool DeviceRRGSB::validate_cb_type(const t_rr_type& cb_type) const {
  return ((CHANX == cb_type) || (CHANY == cb_type));
}

size_t DeviceRRGSB::get_sb_unique_module_index(
  const vtr::Point<size_t>& coordinate, const size_t& layer) const {
  VTR_ASSERT(validate_coordinate(coordinate, layer));
  size_t sb_unique_module_id =
    sb_unique_module_id_[layer][coordinate.x()][coordinate.y()];
  return sb_unique_module_id;
}

size_t DeviceRRGSB::get_cb_unique_module_index(
  const t_rr_type& cb_type, const vtr::Point<size_t>& coordinate, const size_t& layer) const {
  VTR_ASSERT(validate_cb_type(cb_type));
  VTR_ASSERT(validate_coordinate(coordinate, layer));
  size_t cb_unique_module_id;

  switch (cb_type) {
    case CHANX:
      cb_unique_module_id =
        cbx_unique_module_id_[layer][coordinate.x()][coordinate.y()];
      break;
    case CHANY:
      cb_unique_module_id =
        cby_unique_module_id_[layer][coordinate.x()][coordinate.y()];
      break;
    default:
      VTR_LOG_ERROR("Invalid type of connection block!\n");
      exit(1);
  }

  return cb_unique_module_id;
}

/************************************************************************
 * Preload unique blocks
 ***********************************************************************/
/* preload unique cbx blocks and their corresponding instance information. This
 * function will be called when read_unique_blocks command invoked */
void DeviceRRGSB::preload_unique_cbx_module(
  const PointWithLayer& block_coordinate,
  const std::vector<PointWithLayer>& instance_coords) {
  /*check whether the preloaded value exceeds the limit */
  size_t limit_layer = cbx_unique_module_id_.size();
  size_t limit_x = cbx_unique_module_id_[0].size();
  size_t limit_y = cbx_unique_module_id_[0][0].size();
  VTR_ASSERT(block_coordinate.layer < limit_layer);
  VTR_ASSERT(block_coordinate.coordinates.x() < limit_x);
  VTR_ASSERT(block_coordinate.coordinates.y() < limit_y);
  add_cb_unique_module(CHANX, block_coordinate.coordinates, block_coordinate.layer);
  /* preload the unique block */
  set_cb_unique_module_id(CHANX, block_coordinate.coordinates, block_coordinate.layer,
                          get_num_cb_unique_module(CHANX) - 1);

  /* preload the instances of the unique block. Instance will have the same id
   * as the unique block */
  for (auto instance_location : instance_coords) {
    VTR_ASSERT(instance_location.layer < limit_layer);
    VTR_ASSERT(instance_location.coordinates.x() < limit_x);
    VTR_ASSERT(instance_location.coordinates.y() < limit_y);
    set_cb_unique_module_id(
      CHANX, instance_location.coordinates, instance_location.layer,
      cbx_unique_module_id_[block_coordinate.layer][block_coordinate.coordinates.x()][block_coordinate.coordinates.y()]);
  }
}

/* preload unique cby blocks and their corresponding instance information. This
 * function will be called when read_unique_blocks command invoked */
void DeviceRRGSB::preload_unique_cby_module(
  const PointWithLayer& block_coordinate,
  const std::vector<PointWithLayer>& instance_coords) {
  /*check whether the preloaded value exceeds the limit */
  size_t limit_layer = cby_unique_module_id_.size();
  size_t limit_x = cby_unique_module_id_[0].size();
  size_t limit_y = cby_unique_module_id_[0][0].size();

  VTR_ASSERT(block_coordinate.layer < limit_layer);
  VTR_ASSERT(block_coordinate.coordinates.x() < limit_x);
  VTR_ASSERT(block_coordinate.coordinates.y() < limit_y);
  add_cb_unique_module(CHANY, block_coordinate.coordinates, block_coordinate.layer);
  /* preload the unique block */
  set_cb_unique_module_id(CHANY, block_coordinate.coordinates, block_coordinate.layer,
                          get_num_cb_unique_module(CHANY) - 1);

  /* preload the instances of the unique block. Instance will have the same id
   * as the unique block */
  for (auto instance_location : instance_coords) {
    VTR_ASSERT(instance_location.layer < limit_layer);
    VTR_ASSERT(instance_location.coordinates.x() < limit_x);
    VTR_ASSERT(instance_location.coordinates.y() < limit_y);
    set_cb_unique_module_id(
      CHANY, instance_location.coordinates, instance_location.layer,
      cby_unique_module_id_[block_coordinate.layer][block_coordinate.coordinates.x()][block_coordinate.coordinates.y()]);
  }
}

/* preload unique sb blocks and their corresponding instance information. This
 * function will be called when read_unique_blocks command invoked */
void DeviceRRGSB::preload_unique_sb_module(
  const PointWithLayer& block_coordinate,
  const std::vector<PointWithLayer>& instance_coords) {
  /*check whether the preloaded value exceeds the limit */
  size_t limit_layer = sb_unique_module_id_.size();
  size_t limit_x = sb_unique_module_id_[0].size();
  size_t limit_y = sb_unique_module_id_[0][0].size();

  VTR_ASSERT(block_coordinate.layer < limit_layer);
  VTR_ASSERT(block_coordinate.coordinates.x() < limit_x);
  VTR_ASSERT(block_coordinate.coordinates.y() < limit_y);

  sb_unique_module_.push_back(block_coordinate);
  /* Record the id of unique module */
  sb_unique_module_id_[block_coordinate.layer][block_coordinate.coordinates.x()][block_coordinate.coordinates.y()] =
    sb_unique_module_.size() - 1;

  /* each mirror instance of the unique module will have the same module id as
   * the unique module */
  for (auto instance_location : instance_coords) {
    VTR_ASSERT(instance_location.coordinates.x() < sb_unique_module_id_.size());
    VTR_ASSERT(instance_location.coordinates.y() < sb_unique_module_id_[0].size());
    sb_unique_module_id_[instance_location.layer][instance_location.coordinates.x()][instance_location.coordinates.y()] =
      sb_unique_module_id_[block_coordinate.layer][block_coordinate.coordinates.x()][block_coordinate.coordinates.y()];
  }
}
} /* End namespace openfpga*/

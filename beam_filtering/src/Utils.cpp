#include <beam_filtering/Utils.h>

namespace beam_filtering {

void PrintFilterParamsOptions() {
  BEAM_ERROR("Invalid filter params, see bellow for options and requirements:");
  std::cout << "FILTER TYPES SUPPORTED:\n"
            << "----------------------:\n"
            << "Type: DROR\n"
            << "Required inputs:\n"
            << "- radius_multiplier (required >= 1)\n"
            << "- azimuth_angle (required > 0.01)\n"
            << "- min_neighbors (required > 1)\n"
            << "- min_search_radius (required > 0)\n"
            << "Type: ROR\n"
            << "Required inputs:\n"
            << "- search_radius (required > 0)\n"
            << "- min_neighbors (required > 1)\n"
            << "Type: VOXEL\n"
            << "Required inputs:\n"
            << "- cell_size (required vector of size 3)\n"
            << "Type: CROPBOX\n"
            << "Required inputs:\n"
            << "- min (required vector of size 3)\n"
            << "- max (required vector of size 3)\n"
            << "- remove_outside_points (required 0 or 1)";
}

bool LoadDRORParams(const nlohmann::json& J, std::vector<double>& params) {
  params.clear();

  beam::ValidateJsonKeysOrThrow({"radius_multiplier", "azimuth_angle",
                                 "min_neighbors", "min_search_radius"},
                                J);

  double radius_multiplier, azimuth_angle, min_neighbors, min_search_radius;

  radius_multiplier = J["radius_multiplier"];
  azimuth_angle = J["azimuth_angle"];
  min_neighbors = J["min_neighbors"];
  min_search_radius = J["min_search_radius"];
  params.push_back(radius_multiplier);
  params.push_back(azimuth_angle);
  params.push_back(min_neighbors);
  params.push_back(min_search_radius);

  if (!(radius_multiplier >= 1)) {
    BEAM_ERROR("Invalid radius_multiplier parameter in config");
    return false;
  }
  if (!(azimuth_angle > 0.01)) {
    BEAM_ERROR("Invalid azimuth_angle parameter in config");
    return false;
  }
  if (!(min_neighbors > 1)) {
    BEAM_ERROR("Invalid min_neighbors parameter in config");
    return false;
  }
  if (!(min_search_radius > 0)) {
    BEAM_ERROR("Invalid min_search_radius parameter in config");
    return false;
  }

  return true;
}

bool LoadRORParams(const nlohmann::json& J, std::vector<double>& params) {
  params.clear();

  beam::ValidateJsonKeysOrThrow({"search_radius", "min_neighbors"}, J);

  double search_radius, min_neighbors;
  search_radius = J["search_radius"];
  min_neighbors = J["min_neighbors"];
  params.push_back(search_radius);
  params.push_back(min_neighbors);

  if (!(search_radius > 0)) {
    BEAM_ERROR("Invalid search_radius parameter in config");
    return false;
  }
  if (!(min_neighbors > 1)) {
    BEAM_ERROR("Invalid min_neighbors parameter in config");
    return false;
  }

  return true;
}

bool LoadCropBoxParams(const nlohmann::json& J, std::vector<double>& params) {
  params.clear();

  std::vector<double> min, max;
  bool remove_outside_points;

  beam::ValidateJsonKeysOrThrow({"min", "max", "remove_outside_points"}, J);
  std::vector<double> min_tmp = J["min"];
  min = min_tmp;
  std::vector<double> max_tmp = J["max"];
  max = max_tmp;
  remove_outside_points = J["remove_outside_points"];

  if (min.size() != 3 || max.size() != 3) {
    BEAM_ERROR(
        "Invalid size min or max parameter in cropbox from map builder config");
    return false;
  }

  params.push_back(min[0]);
  params.push_back(min[1]);
  params.push_back(min[2]);
  params.push_back(max[0]);
  params.push_back(max[1]);
  params.push_back(max[2]);
  if (remove_outside_points) {
    params.push_back(1);
  } else {
    params.push_back(0);
  }

  if (J.contains("quaternion_wxyz") && J.contains("translation_xyz")) {
    std::vector<double> q = J["quaternion_wxyz"];
    std::vector<double> t = J["translation_xyz"];
    if (q.size() != 4) {
      BEAM_ERROR("Invalid size quaternion parameter in cropbox from map "
                 "builder config");
      return false;
    }
    if (t.size() != 3) {
      BEAM_ERROR("Invalid size for translation parameter in cropbox from map "
                 "builder config");
      return false;
    }
    params.push_back(q[0]);
    params.push_back(q[1]);
    params.push_back(q[2]);
    params.push_back(q[3]);
    params.push_back(t[0]);
    params.push_back(t[1]);
    params.push_back(t[2]);
  }
  return true;
}

bool LoadVoxelDownsamplingParams(const nlohmann::json& J,
                                 std::vector<double>& params) {
  params.clear();

  beam::ValidateJsonKeysOrThrow({"cell_size"}, J);

  std::vector<double> cell_size;
  for (const auto param : J["cell_size"]) { cell_size.push_back(param); }
  if (cell_size.size() != 3) {
    BEAM_ERROR("Invalid cell size parameter in voxel grid filter from map "
               "builder config");
    return false;
  }
  if (cell_size[0] < 0.001 || cell_size[2] < 0.001 || cell_size[2] < 0.001) {
    BEAM_ERROR("Invalid cell_size parameter in config");
    return false;
  }
  params = cell_size;

  return true;
}

FilterParamsType GetFilterParams(const nlohmann::json& J) {
  std::cout << J.dump() << "\n";

  beam::ValidateJsonKeysOrThrow({"filter_type"}, J);
  std::string filter_type_str = J["filter_type"];
  std::map<std::string, FilterType>::iterator filter_type_iter =
      FilterTypeStringMap.find(filter_type_str);

  if (filter_type_iter == FilterTypeStringMap.end()) {
    std::string filter_options = GetFilterTypes();
    BEAM_CRITICAL("Invalid filter type. Input: {}, options: {}",
                  filter_type_str, filter_options);
    PrintFilterParamsOptions();
    throw std::invalid_argument{"Invalid filter params."};
  }

  FilterType filter_type = filter_type_iter->second;

  std::vector<double> params;

  if (filter_type == FilterType::DROR) {
    if (!LoadDRORParams(J, params)) {
      throw std::invalid_argument{"Invalid filter params."};
    }
  } else if (filter_type == FilterType::ROR) {
    if (!LoadRORParams(J, params)) {
      throw std::invalid_argument{"Invalid filter params."};
    }
  } else if (filter_type == FilterType::VOXEL) {
    if (!LoadVoxelDownsamplingParams(J, params)) {
      throw std::invalid_argument{"Invalid filter params."};
    }
  } else if (filter_type == FilterType::CROPBOX) {
    if (!LoadCropBoxParams(J, params)) {
      throw std::invalid_argument{"Invalid filter params."};
    }
  }

  return std::make_pair(filter_type, params);
}

std::vector<FilterParamsType> LoadFilterParamsVector(const nlohmann::json& J) {
  std::vector<FilterParamsType> filter_params_vec;
  for (const auto& J_filter : J) {
    FilterParamsType filter_params = GetFilterParams(J_filter);
    filter_params_vec.push_back(filter_params);
  }
  return filter_params_vec;
}

} // namespace beam_filtering
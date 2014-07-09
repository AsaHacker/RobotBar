#include "ssb_utils_model.h"

namespace ssb_utils_model {

void TentacleModel::setParams(ssb_common_enum::Config settings) {
  /*
  ssb_common_vec::VecTentacle min_input_settings(-45, -45, -45, -45, -45, -45, -45);
  ssb_common_vec::VecTentacle max_input_settings(45, 45, 45, 45, 45, 45, 45);
  ssb_common_vec::VecTentacle min_output_settings(1100, 1000, 1100, 900, 1100, 1400, 1100);
  ssb_common_vec::VecTentacle max_output_settings(1700, 1800, 1700, 1500, 1700, 1800, 1700);
  ssb_common_vec::VecTentacle home_output_settings(1400, 1500, 1400, 1200, 1400, 1600, 1470);
  ssb_common_vec::VecTentacle direction_settings(-1, 1, -1, 1, -1, 1, -1);
  */
  /*
  ssb_common_vec::VecTentacle min_input_settings(-45, -45, -45, -45, -45, -45, -45);
  ssb_common_vec::VecTentacle max_input_settings(45, 45, 45, 45, 45, 45, 45);
  ssb_common_vec::VecTentacle min_output_settings(1000, 1000, 1160, 1280, 1170, 1510, 1680);
  ssb_common_vec::VecTentacle max_output_settings(1700, 1625, 1560, 1980, 1900, 1910, 2000);
  ssb_common_vec::VecTentacle home_output_settings(1250, 1425, 1360, 1580, 1370, 1710, 1880);
  ssb_common_vec::VecTentacle direction_settings(-1, 1, -1, 1, -1, 1, -1);
  */
  ssb_common_vec::VecTentacle min_input_settings(-45, -45, -45, -45, -45, -45, -45);
  ssb_common_vec::VecTentacle max_input_settings(45, 45, 45, 45, 45, 45, 45);
  /*
  ssb_common_vec::VecTentacle min_output_settings(1000, 1200, 1260, 1330, 1220, 1510, 1680);
  ssb_common_vec::VecTentacle max_output_settings(1700, 1825, 1660, 2000, 1940, 1910, 2000);
  ssb_common_vec::VecTentacle home_output_settings(1180, 1625, 1460, 1630, 1420, 1710, 1840);
  */
  ssb_common_vec::VecTentacle min_output_settings(1000, 1200, 1070, 1680, 1080, 1625, 1600);
  ssb_common_vec::VecTentacle max_output_settings(1700, 1825, 1470, 2000, 1780, 2000, 2000);
  ssb_common_vec::VecTentacle home_output_settings(1020, 1625, 1270, 1980, 1280, 1825, 1800);
  ssb_common_vec::VecTentacle direction_settings(-1, 1, -1, 1, -1, 1, -1);
  min_limit_input_ = min_input_settings;
  max_limit_input_ = max_input_settings;
  min_limit_output_ = min_output_settings;
  max_limit_output_ = max_output_settings;
  home_output_ = home_output_settings;
  sgn_direction_ = direction_settings;
  for (int i = 0; i < 7; ++i)
    k_input2output_.joint[i] = (max_limit_output_.joint[i] - min_limit_output_.joint[i])
                               / (max_limit_input_.joint[i] - min_limit_input_.joint[i]);
  object_.set_motor_id(ssb_common_vec::VecTentacle(0, 1, 2, 3, 4, 5, 6));
}

void TentacleModel::Input2Output() {
  ssb_common_vec::VecTentacle tmp_output;
  for (int i = 0; i < 7; ++i) {
    // Normalize input to output direction.
    tmp_output.joint[i] = sgn_direction_.joint[i] * input_.joint[i];
    // Convert input to local amount.
    tmp_output.joint[i] -= min_limit_input_.joint[i];
    // Convert local input to local output.
    tmp_output.joint[i] *= k_input2output_.joint[i];
    // Convert output to absolute amount.
    tmp_output.joint[i] += min_limit_output_.joint[i];
    // In case output is out of range.
    if (tmp_output.joint[i] > max_limit_output_.joint[i])
      tmp_output.joint[i] = max_limit_output_.joint[i];
    else if (tmp_output.joint[i] < min_limit_output_.joint[i])
      tmp_output.joint[i] = min_limit_output_.joint[i];
  }
  output_ = tmp_output;
}

ssb_common_vec::VecTentacle TentacleModel::getHomeAsInput() {
  ssb_common_vec::VecTentacle tmp_home_as_input;
  tmp_home_as_input = home_output_;
  for (int i = 0; i < 7; ++i) {
    tmp_home_as_input.joint[i] -= min_limit_output_.joint[i];
    tmp_home_as_input.joint[i] /= k_input2output_.joint[i];
    tmp_home_as_input.joint[i] += min_limit_input_.joint[i];
    tmp_home_as_input.joint[i] *= sgn_direction_.joint[i];
  }
  return tmp_home_as_input;
}


void QuadOEyeModel::setParams(ssb_common_enum::Config settings) {
  ssb_common_vec::VecEye min_input_settings(-1.0, -1.0);
  min_limit_input_ = min_input_settings;
  ssb_common_vec::VecEye max_input_settings(1.0, 1.0);
  max_limit_input_ = max_input_settings;
  ssb_common_vec::VecQuadOEye min_output_settings(1200, 1450, 1650, 1450);
  min_limit_output_ = min_output_settings;
  ssb_common_vec::VecQuadOEye max_output_settings(1450, 1550, 1900, 1550);
  max_limit_output_ = max_output_settings;
  ssb_common_vec::VecQuadOEye home_output_settings(1350, 1500, 1750, 1500);
  home_output_ = home_output_settings;
  /* ssb_common_vec::VecQuadOEye direction_settings(1, -1, 1, 1); */
  ssb_common_vec::VecQuadOEye direction_settings(1, 1, 1, -1);
  sgn_direction_ = direction_settings;
  object_.set_motor_id(ssb_common_vec::VecQuadOEye(0, 2, 1, 3));
  k_input2output4positive_input_ = max_limit_output_;
  k_input2output4positive_input_ -= home_output_;
  k_input2output4negative_input_ = home_output_;
  k_input2output4negative_input_ -= min_limit_output_;
}

void QuadOEyeModel::Input2Output() {
  ssb_common_vec::VecQuadOEye tmp_output;
  // Normalize input to output direction.
  tmp_output.horizontal_l = sgn_direction_.horizontal_l * input_.horizontal;
  tmp_output.vertical_l = sgn_direction_.vertical_l * input_.vertical;
  tmp_output.horizontal_r = sgn_direction_.horizontal_r * input_.horizontal;
  tmp_output.vertical_r = sgn_direction_.vertical_r * input_.vertical;
  // Convert input to local output.
  tmp_output.horizontal_l *= input_.horizontal > 0 ?
                           k_input2output4positive_input_.horizontal_l
                           : k_input2output4negative_input_.horizontal_l;
  tmp_output.vertical_l *= input_.vertical > 0 ?
                           k_input2output4positive_input_.vertical_l
                           : k_input2output4negative_input_.vertical_l;
  tmp_output.horizontal_r *= input_.horizontal > 0 ?
                           k_input2output4positive_input_.horizontal_r
                           : k_input2output4negative_input_.horizontal_r;
  tmp_output.vertical_r *= input_.vertical > 0 ?
                           k_input2output4positive_input_.vertical_r
                           : k_input2output4negative_input_.vertical_r;
  // Convert output to absolute amount.
  tmp_output.horizontal_l += home_output_.horizontal_l;
  tmp_output.vertical_l += home_output_.vertical_l;
  tmp_output.horizontal_r += home_output_.horizontal_r;
  tmp_output.vertical_r += home_output_.vertical_r;
  // In case output is out of range.
  if (tmp_output.horizontal_l > max_limit_output_.horizontal_l)
    tmp_output.horizontal_l = max_limit_output_.horizontal_l;
  else if (tmp_output.horizontal_l < min_limit_output_.horizontal_l)
    tmp_output.horizontal_l = min_limit_output_.horizontal_l;
  if (tmp_output.vertical_l > max_limit_output_.vertical_l)
    tmp_output.vertical_l = max_limit_output_.vertical_l;
  else if (tmp_output.vertical_l < min_limit_output_.vertical_l)
    tmp_output.vertical_l = min_limit_output_.vertical_l;
  if (tmp_output.horizontal_r > max_limit_output_.horizontal_r)
    tmp_output.horizontal_r = max_limit_output_.horizontal_r;
  else if (tmp_output.horizontal_r < min_limit_output_.horizontal_r)
    tmp_output.horizontal_r = min_limit_output_.horizontal_r;
  if (tmp_output.vertical_r > max_limit_output_.vertical_r)
    tmp_output.vertical_r = max_limit_output_.vertical_r;
  else if (tmp_output.vertical_r < min_limit_output_.vertical_r)
    tmp_output.vertical_r = min_limit_output_.vertical_r;
  output_ = tmp_output;
}

ssb_common_vec::VecEye QuadOEyeModel::getHomeAsInput() {
  return ssb_common_vec::VecEye(0, 0);
}


void NeckModel::setParams(ssb_common_enum::Config settings) {
  ssb_common_vec::VecNeck direction_settings(1, -1, 1);
  sgn_direction_ = direction_settings;
}

void NeckModel::Input2Output() {
  ssb_common_vec::VecNeck tmp_output;
  tmp_output.roll = sgn_direction_.roll * input_.roll;
  tmp_output.pitch = sgn_direction_.pitch * input_.pitch;
  tmp_output.yaw = sgn_direction_.yaw * input_.yaw;
  output_ = tmp_output;
}

ssb_common_vec::VecNeck NeckModel::getHomeAsInput() {
  return ssb_common_vec::VecNeck(0, 0, 0);
}

} // namespace ssb_utils_model

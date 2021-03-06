#ifndef SSB_COMMON_VEC_H_
#define SSB_COMMON_VEC_H_

#include <string>
#include <vector>
#include <ofVec2f.h>

namespace ssb_common_vec {

struct None {
  None() {};
  ~None() {};
};

struct VecTentacle {
  VecTentacle();
  VecTentacle(const float init_joint0,
                 const float init_joint1,
                 const float init_joint2,
                 const float init_joint3,
                 const float init_joint4,
                 const float init_joint5,
                 const float init_joint6);
  VecTentacle(const VecTentacle &obj);
  VecTentacle& operator=(const VecTentacle &obj);
  VecTentacle& operator+=(const VecTentacle &obj);
  VecTentacle& operator-=(const VecTentacle &obj);
  VecTentacle& operator/=(const float kGain);
  VecTentacle& operator*=(const float kGain);
  // Keeps the angle[degree] of each tentacle joint.
  // 0 is the base while 6 is the tail.
  float joint[7];
};


struct VecEye {
  VecEye();
  VecEye(const float init_horizontal, const float init_vertical);
  VecEye(const VecEye &obj);
  VecEye& operator=(const VecEye &obj);
  VecEye& operator+=(const VecEye &obj);
  VecEye& operator-=(const VecEye &obj);
  VecEye& operator/=(const float kGain);
  VecEye& operator*=(const float kGain);
  // Keeps the horizontal position of the eye.
  // Takes the UI input value which is -1.0 to 1.0.
  float horizontal;
  // Keeps the vertical position of the eye.
  float vertical;
};


struct VecQuadOEye {
  VecQuadOEye();
  VecQuadOEye(const float init_horizontal_l, const float init_vertical_l,
              const float init_horizontal_r, const float init_vertical_r);
  VecQuadOEye(const VecQuadOEye &obj);
  VecQuadOEye& operator=(const VecQuadOEye &obj);
  VecQuadOEye& operator+=(const VecQuadOEye &obj);
  VecQuadOEye& operator-=(const VecQuadOEye &obj);
  VecQuadOEye& operator/=(const float kGain);
  VecQuadOEye& operator*=(const float kGain);
  // Takes the UI input value which is -1.0 to 1.0.
  float horizontal_l;
  float horizontal_r;
  float vertical_l;
  float vertical_r;
};


struct VecNeck {
  VecNeck();
  VecNeck(const float init_roll, const float init_pitch, const float init_yaw);
  VecNeck(const VecNeck &obj);
  VecNeck& operator=(const VecNeck &obj);
  VecNeck& operator+=(const VecNeck &obj);
  VecNeck& operator-=(const VecNeck &obj);
  VecNeck& operator/=(const float kGain);
  VecNeck& operator*=(const float kGain);
  // Keeps the roll, pitch, yaw position of the neck.
  // Takes the dynamixel input value which is -2.0 to 2.0.
  float roll;
  float pitch;
  float yaw;
};

struct VecTime {
  VecTime();
  VecTime(const float init_start_time, const float init_play_time);
  VecTime(const VecTime &obj);
  VecTime& operator=(const VecTime &obj);
  VecTime& operator+=(const VecTime &obj);
  VecTime& operator-=(const VecTime &obj);
  VecTime& operator/=(const float kGain);
  VecTime& operator*=(const float kGain);
  // Keeps the starting time[seconds] of a motion.
  float start_time;
  // Keeps the playing time[seconds] of a motion.
  float play_time;
};


// This is a unit Vector used for media.
struct VecEvent {
  VecEvent();
  VecEvent(const std::string init_msg,
           const float init_start_time, const float init_end_time,
           const std::string init_args);
  VecEvent(const VecEvent &obj);
  VecEvent& operator=(const VecEvent &obj);
  VecEvent& operator+=(const VecEvent &obj);
  VecEvent& operator-=(const VecEvent &obj);
  VecEvent& operator+=(const VecTime &obj);
  VecEvent& operator*=(const VecTime &obj);
  VecEvent& operator/=(const float kGain);
  VecEvent& operator*=(const float kGain);
  // Main event message.
  std::string msg;
  float start_time;
  float end_time;
  // Any other messages needed for event.
  std::string args;
};


// Specified initialization and distincts xml handling among events.
struct VecVoice : public VecEvent {
  VecVoice();
  VecVoice(const std::string init_msg,
           const float init_start_time, const float init_end_time,
           const std::string init_args);
  VecVoice(const VecVoice &obj);
};


// This is a unit vector used for interpolation.
struct VecInterpolation {
  VecInterpolation();
  VecInterpolation(const int init_type, const std::vector<ofVec2f> init_p);
  VecInterpolation(const VecInterpolation &obj);
  int type;
  std::vector<ofVec2f> p;
};


// This is a unit vector used for keyframes.
template<typename T> struct VecSequence {
  VecSequence();
  VecSequence(const int _number_of_frames,
              const bool _is_interpolation,
              const int _interpolation_type,
              const int _key_ref,
              const T _diff);
  VecSequence(const VecSequence<T> &obj);
  VecSequence& operator=(const VecSequence<T> &obj);
  int number_of_frames;
  bool is_interpolation;
  int interpolation_type;
  int key_ref;
  T diff;
};

} // namespace ssb_common_vec

#endif

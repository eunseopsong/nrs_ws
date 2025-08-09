/* Mode data definition start */
// Rule the mode from 0 ~ 9999
// the main mode must be under 10 (1000,2000,300 ... ,9000, 0000)
// if you want sub command add to the additional room like 5000 -> 5001
#define Posture_control_mode_cmd 1000

#define Joint_control_mode_cmd  2000

#define EE_Posture_control_mode_cmd 3000

#define Hand_guiding_mode_cmd 4000

#define Data_recording_mode_cmd 5000
#define Continuous_reording_start 5001
#define Continusous_recording_end 5002
#define Discrete_reording_start 5101
#define Discrete_recording_end 5102
#define Discrete_recording_save 5103

#define VRCali_reording_start 5201
#define VRCali_recording_end 5202
#define VRCali_recording_save 5203
#define VRTeac_reording_start 5211
#define VRTeac_recording_end 5212
#define VRTeac_recording_save 5213

#define Playback_mode_cmd 7000

#define Motion_stop_cmd 0000

/* Mode data definition end */

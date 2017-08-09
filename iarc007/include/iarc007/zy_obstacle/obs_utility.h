#ifndef _OBS_UTILITY_H_
#define _OBS_UTILITY_H_


/**
* @enum  obs_camera_index
* @brief Define logical direction of vbus of guidance and realsense, i.e. the direction of selected Guidance/Realsense Sensor. Note that they are only defined by the VBUS ports on Guidance Core/Nuc Core, not by the Guidance/Realsense Sensors. The comment of each element indicates the default direction when Guidance is installed on Matrice 100. However the developers can install Guidance in any manner on any device, thus the directions might also be different
*/
enum obs_camera_index
{
	obs_g1_left = 1,	/** guidance_left_camera on M100 */
	obs_g2_left = 3,	
	obs_g3_left = 5,
        obs_g4_left = 7,
        obs_r5_left = 9,        /** realsense_left_camera on M100 */
        obs_r6_left = 11, 
        
        obs_g1_right = 0,	/** guidance_right_camera on M100 */
        obs_g2_right = 2,
        obs_g3_right = 4,
        obs_g4_right = 6,
        obs_r5_right = 8,      /** realsense_right_camera on M100 */
        obs_r6_right = 10
};

/**
* @enum  obs_depth_index
* @brief Define logical direction of vbus of guidance and realsense, i.e. the direction of selected Guidance/Realsense Sensor(depth image). Note that they are only defined by the VBUS ports on Guidance Core/Nuc Core, not by the Guidance/Realsense Sensors. The comment of each element indicates the default direction when Guidance is installed on Matrice 100. However the developers can install Guidance in any manner on any device, thus the directions might also be different
*/
enum obs_depth_index
{
	obs_g1 = 0,	/** guidance_left_camera on M100 */
	obs_g2 = 1,	
	obs_g3 = 2,
        obs_g4 = 3,
        obs_r5 = 4,        /** realsense_left_camera on M100 */
        obs_r6 = 5
};


#endif

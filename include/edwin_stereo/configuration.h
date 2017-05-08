#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#include <dynamic_reconfigure/server.h>
#include "edwin_stereo/ObjectCriteria.h"
#include "edwin_stereo/EdwinStereoConfig.h"
#include "edwin_stereo/EdwinStereoCoarseConfig.h"
#include "edwin_stereo/EdwinStereoFineConfig.h"
#include "edwin_stereo/EdwinFramesConfig.h"


using namespace cv;
using namespace std;

class Configuration{
	private:
		ros::NodeHandle nh_global;
		ros::NodeHandle nh_coarse;
		ros::NodeHandle nh_fine;
		ros::NodeHandle nh_frames;

	public:
		edwin_stereo::EdwinStereoFineConfig c_f;
		edwin_stereo::EdwinStereoCoarseConfig c_c;
		edwin_stereo::EdwinFramesConfig c_fr;
	private:
		dynamic_reconfigure::Server<edwin_stereo::EdwinStereoConfig> s_g;
		dynamic_reconfigure::Server<edwin_stereo::EdwinStereoFineConfig> s_f;
		dynamic_reconfigure::Server<edwin_stereo::EdwinStereoCoarseConfig> s_c;
		dynamic_reconfigure::Server<edwin_stereo::EdwinFramesConfig> s_fr;

	public:
		Configuration();
		ObjectCriteria params;

		bool verbose;
		bool pcl;
		bool coarse;
		float solidity;

		void global_cb(edwin_stereo::EdwinStereoConfig& config, uint32_t level);
		void coarse_cb(edwin_stereo::EdwinStereoCoarseConfig& config, uint32_t level);
		void fine_cb(edwin_stereo::EdwinStereoFineConfig& config, uint32_t level);
		void frames_cb(edwin_stereo::EdwinFramesConfig& config, uint32_t level);
};

#endif

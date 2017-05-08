
/* STD */
#include <iostream>
#include <string>
#include <vector>

/* OpenCV */
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

/* ROS */
#include "ros/ros.h"
#include "ros/package.h"
#include "edwin_stereo/configuration.h"

Configuration::Configuration():
	nh_global("~"), nh_coarse("~/coarse"), nh_fine("~/fine"), nh_frames("~/frames"),
	s_g(nh_global), s_c(nh_coarse), s_f(nh_fine), s_fr(nh_frames)
{

	// global
	dynamic_reconfigure::Server<edwin_stereo::EdwinStereoConfig>::CallbackType f_g;
	f_g = boost::bind(&Configuration::global_cb, this, _1, _2);
	s_g.setCallback(f_g);

	//coarse
	dynamic_reconfigure::Server<edwin_stereo::EdwinStereoCoarseConfig>::CallbackType f_c;
	f_c = boost::bind(&Configuration::coarse_cb, this, _1, _2);
	s_c.setCallback(f_c);

	// fine	
	dynamic_reconfigure::Server<edwin_stereo::EdwinStereoFineConfig>::CallbackType f_f;
	f_f = boost::bind(&Configuration::fine_cb, this, _1, _2);
	s_f.setCallback(f_f);

	dynamic_reconfigure::Server<edwin_stereo::EdwinFramesConfig>::CallbackType f_fr;
	f_fr = boost::bind(&Configuration::frames_cb, this, _1, _2);
	s_fr.setCallback(f_fr);
}

void Configuration::global_cb(edwin_stereo::EdwinStereoConfig& config, uint32_t level){
	verbose = config.verbose;

	if(verbose){
		//namedWindow("left", WINDOW_AUTOSIZE);
		//namedWindow("right", WINDOW_AUTOSIZE);
		//namedWindow("raw_disp", WINDOW_AUTOSIZE);
		//namedWindow("disp", WINDOW_AUTOSIZE);
		//namedWindow("filtered", WINDOW_AUTOSIZE);
	}else{
		destroyAllWindows();
	}

	pcl = config.pcl;

	if(coarse != config.coarse){
		coarse = config.coarse;
		if(coarse){
			coarse_cb(this->c_c, 0);
		}else{
			fine_cb(this->c_f, 0);
		}
	}
	solidity = config.solidity;
}

void Configuration::coarse_cb(edwin_stereo::EdwinStereoCoarseConfig& config, uint32_t level){
	this->c_c = config;
	if( !coarse)
		return;

	auto& hsv1_l = params.hsv_range[0].min;
	auto& hsv1_h = params.hsv_range[0].max;
	auto& hsv2_l = params.hsv_range[1].min;
	auto& hsv2_h = params.hsv_range[1].max;

	switch(config.color){
		case edwin_stereo::EdwinStereoCoarse_Red: // Red
			// H
			hsv1_l[0] = 0;
			hsv1_h[0] = 10;
			hsv2_l[0] = 165;
			hsv2_h[0] = 180;

			// S
			hsv1_l[1] = hsv2_l[1] = 210;
			hsv1_h[1] = hsv2_h[1] = 220;

			// V
			hsv1_l[2] = hsv2_l[2] = 165;
			hsv1_h[2] = hsv2_h[2] = 200;
			break;
		case edwin_stereo::EdwinStereoCoarse_Green: // Green
			// H
			hsv1_l[0] = hsv2_l[0] = 45;
			hsv1_h[0] = hsv2_h[0] = 80;

			// S
			hsv1_l[1] = hsv2_l[1] = 40;
			hsv1_h[1] = hsv2_h[1] = 190;

			// V
			hsv1_l[2] = hsv2_l[2] = 140;
			hsv1_h[2] = hsv2_h[2] = 215;
			break;
		case edwin_stereo::EdwinStereoCoarse_Blue: // Blue
			// H
			hsv1_l[0] = hsv2_l[0] = 100;
			hsv1_h[0] = hsv2_h[0] = 120;

			// S
			hsv1_l[1] = hsv2_l[1] = 120;
			hsv1_h[1] = hsv2_h[1] = 200;

			// V
			hsv1_l[2] = hsv2_l[2] = 45;
			hsv1_h[2] = hsv2_h[2] = 95;
			break;
		case edwin_stereo::EdwinStereoCoarse_Orange: // Orange
			// H
			hsv1_l[0] = hsv2_l[0] = 0;
			hsv1_h[0] = hsv2_h[0] = 16;

			// S
			hsv1_l[1] = hsv2_l[1] = 145;
			hsv1_h[1] = hsv2_h[1] = 256;

			// V
			hsv1_l[2] = hsv2_l[2] = 210;
			hsv1_h[2] = hsv2_h[2] = 256;
			break;
		case edwin_stereo::EdwinStereoCoarse_Yellow: // Yellow
			// H
			hsv1_l[0] = hsv2_l[0] = 20;
			hsv1_h[0] = hsv2_h[0] = 30;

			// S
			hsv1_l[1] = hsv2_l[1] = 95;
			hsv1_h[1] = hsv2_h[1] = 190;

			// V
			hsv1_l[2] = hsv2_l[2] = 180;
			hsv1_h[2] = hsv2_h[2] = 256;
			break;
		case edwin_stereo::EdwinStereoCoarse_Pink: // Hot Pink
			// H
			hsv1_l[0] = hsv2_l[0] = 160;
			hsv1_h[0] = hsv2_h[0] = 180;

			// S
			hsv1_l[1] = hsv2_l[1] = 160;
			hsv1_h[1] = hsv2_h[1] = 200;

			// V
			hsv1_l[2] = hsv2_l[2] = 200;
			hsv1_h[2] = hsv2_h[2] = 256;
			break;
		case edwin_stereo::EdwinStereoCoarse_Purple: // Violet
			// H
			hsv1_l[0] = hsv2_l[0] = 130;
			hsv1_h[0] = hsv2_h[0] = 150;

			// S
			hsv1_l[1] = hsv2_l[1] = 80;
			hsv1_h[1] = hsv2_h[1] = 150;

			// V
			hsv1_l[2] = hsv2_l[2] = 0;
			hsv1_h[2] = hsv2_h[2] = 100;
			break;
		case edwin_stereo::EdwinStereoCoarse_Black: // Black
			// H
			hsv1_l[0] = hsv2_l[0] = 0;
			hsv1_h[0] = hsv2_h[0] = 180;

			// S
			hsv1_l[1] = hsv2_l[1] = 0;
			hsv1_h[1] = hsv2_h[1] = 256;

			// V
			hsv1_l[2] = hsv2_l[2] = 0;
			hsv1_h[2] = hsv2_h[2] = 35;
			break;
		case edwin_stereo::EdwinStereoCoarse_White:
			break;
		case edwin_stereo::EdwinStereoCoarse_Grey:
			break;

	}

	switch(config.size){
		case edwin_stereo::EdwinStereoCoarse_Small:
			params.area_range.min = 0.0005;
			params.area_range.max = 0.0015;
			break;
		case edwin_stereo::EdwinStereoCoarse_Medium:
			params.area_range.min = 0.001;
			params.area_range.max = 0.005;
			break;
		case edwin_stereo::EdwinStereoCoarse_Large:
			params.area_range.min = 0.004;
			params.area_range.max = 0.01;
			break;
		case edwin_stereo::EdwinStereoCoarse_ExtraLarge:
			params.area_range.min = 0.008;
			params.area_range.max = 0.02;
			break;
	}

	params.dist_range.min = 0.0;
	params.dist_range.max = 5.0;
}

void Configuration::fine_cb(edwin_stereo::EdwinStereoFineConfig& config, uint32_t level){

	this->c_f = config;
	if(coarse)
		return;

	auto& hsv1_l = params.hsv_range[0].min;
	auto& hsv1_h = params.hsv_range[0].max;
	auto& hsv2_l = params.hsv_range[1].min;
	auto& hsv2_h = params.hsv_range[1].max;
	
	if(config.h_l < config.h_h){
		hsv1_l[0] = hsv2_l[0] = config.h_l;
		hsv1_h[0] = hsv2_h[0] = config.h_h;
	}else{
		// wrap around
		hsv1_l[0] = 0;
		hsv1_h[0] = config.h_h;

		hsv2_l[0] = config.h_l;
		hsv2_h[0] = 180;
	}

	hsv1_l[1] = config.s_l;
	hsv1_h[1] = config.s_h;
	hsv1_l[2] = config.v_l;
	hsv1_h[2] = config.v_h;

	hsv2_l[1] = config.s_l;
	hsv2_h[1] = config.s_h;
	hsv2_l[2] = config.v_l;
	hsv2_h[2] = config.v_h;

	params.area_range.min = config.min_area;
	params.area_range.max = config.max_area;
	params.dist_range.min = config.min_dist;
	params.dist_range.max = config.max_dist;
}

void Configuration::frames_cb(edwin_stereo::EdwinFramesConfig& config, uint32_t level){
	this->c_fr = config;
	if(!config.left)
		destroyWindow("left");
	if(!config.right)
		destroyWindow("right");
	if(!config.disp)
		destroyWindow("disp");
	if(!config.raw_disp)
		destroyWindow("raw_disp");
	if(!config.filtered)
		destroyWindow("filtered");
}


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
	nh_global("~"), nh_coarse("~/coarse"), nh_fine("~/fine"),
	s_g(nh_global), s_c(nh_coarse), s_f(nh_fine)
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
			hsv2_l[0] = 170;
			hsv2_h[0] = 180;

			// S
			hsv1_l[1] = hsv2_l[1] = 50;
			hsv1_h[1] = hsv2_h[1] = 255;

			// V
			hsv1_l[2] = hsv2_l[2] = 50;
			hsv1_h[2] = hsv2_h[2] = 255;
			break;
		case edwin_stereo::EdwinStereoCoarse_Green: // Blue
			// H
			hsv1_l[0] = hsv2_l[0] = 40;
			hsv1_h[0] = hsv2_h[0] = 80;

			// S
			hsv1_l[1] = hsv2_l[1] = 80;
			hsv1_h[1] = hsv2_h[1] = 255;

			// V
			hsv1_l[2] = hsv2_l[2] = 100;
			hsv1_h[2] = hsv2_h[2] = 255;
			break;
		case edwin_stereo::EdwinStereoCoarse_Blue: // Green
			// H
			hsv1_l[0] = hsv2_l[0] = 85;
			hsv1_h[0] = hsv2_h[0] = 135;

			// S
			hsv1_l[1] = hsv2_l[1] = 80;
			hsv1_h[1] = hsv2_h[1] = 255;

			// V
			hsv1_l[2] = hsv2_l[2] = 100;
			hsv1_h[2] = hsv2_h[2] = 255;
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

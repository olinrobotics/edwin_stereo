
/* STD */
#include <iostream>
#include <string>
#include <vector>
/* OpenCV */
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

/* Custom OpenCV */
#include "edwin_stereo/rectifier.h"
#include "edwin_stereo/f_sgbm.h"

/* ROS */
#include "ros/ros.h"
#include "ros/package.h"

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/point_cloud2_iterator.h"

#include "geometry_msgs/PointStamped.h"

#include <dynamic_reconfigure/server.h>
#include <edwin_stereo/EdwinStereoConfig.h>


using namespace cv;
using namespace std;

template<typename T>
struct MinMaxRange{
	T min;
	T max;
};

struct ObjectCriteria{
	ObjectCriteria(){
		// setup default parameters
		area_range.min = 0.005;
		area_range.max = 1.0;
		dist_range.min = 0.0;
		dist_range.max = 1.5;

		MinMaxRange<Vec3b> hsv1;
		hsv1.min = Vec3b(0,0,0);
		hsv1.max = Vec3b(255,255,255);

		MinMaxRange<Vec3b> hsv2;
		hsv2.min = Vec3b(0,0,0);
		hsv2.max = Vec3b(0,0,0);

		hsv_range.push_back(hsv1);
		hsv_range.push_back(hsv2);
	}
	// color
	std::vector<MinMaxRange<Vec3b>> hsv_range;

	// distance
	MinMaxRange<float> dist_range;

	MinMaxRange<float> area_range;
};

struct LabelData{
	int cX, cY;
	float d, a, p_a;
};

void putCenteredText(Mat& frame,const std::string& text,const Point& center){

	double fontScale = 0.5;
	int thickness = 1;

	int baseline=0;
	auto font = FONT_HERSHEY_SIMPLEX;
	auto textSize = getTextSize(text, font, fontScale, thickness, &baseline);

	Point origin(center.x - textSize.width/2, center.y - textSize.height/2);
	baseline += thickness;

	putText(frame, text, origin, font, fontScale, Scalar::all(255), thickness);
}

void hline(const std::string& tag){
	std::cout << "=======================" << tag << "=============================" << std::endl;

}

void apply_criteria(Mat& dist, Mat& frame, ObjectCriteria& params, Mat& output, std::vector<LabelData>& labels){

	Mat blur;
	GaussianBlur(frame, blur, Size(7,7), 0);

	Mat mask;

	// color
	Mat c_mask(frame.rows,frame.cols,CV_8U,Scalar(0));

	Mat hsv;
	cv::cvtColor(blur,hsv,COLOR_BGR2HSV);

	Mat t;
	Mat t2;
	for(auto& r : params.hsv_range){
		inRange(hsv,r.min,r.max,t);
		c_mask.setTo(255, t);
	}

	// distance
	Mat d_mask;
	inRange(dist, params.dist_range.min, params.dist_range.max, d_mask);

	vector<vector<Point> > ctrs;
	vector<Vec4i> hrch;

	cv::bitwise_and(c_mask, d_mask, mask);

	cv::findContours(mask, ctrs, hrch, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	mask.setTo(Scalar::all(0));

	labels.clear();
	int ctr_idx = 0;
	for(auto& ctr : ctrs){
		auto m = moments(ctr);
		if(fabs(m.m00) > 1e-6){
			float p_a = contourArea(ctr);
			//std::cout << p_a << std::endl;
			int cX = m.m10/m.m00;
			int cY = m.m01/m.m00;
			float d = dist.at<float>(cY,cX);
			float a = p_a * d * 1.5e-6; // magic scaling factor
			if(params.area_range.min <= a && a < params.area_range.max){
				// fill in mask
				drawContours(mask, ctrs, ctr_idx, Scalar::all(255), -1);
				labels.push_back({
						cX,cY,d,a,p_a
						});
			}
		}
		++ctr_idx;
	}

	Mat res(frame.rows, frame.cols, frame.type()); // todo : check valid?
	res.setTo(Scalar::all(0));

	frame.copyTo(res,mask);

	// draw Labels
	for(auto& label : labels){
		char text_d[64] = {};
		char text_a[64] = {};

		std::sprintf(text_d, "d : %.2f", label.d);
		std::sprintf(text_a, "a : %.2f", label.a);

		auto center = Point(label.cX, label.cY);
		circle(res, center, sqrt(label.p_a), Scalar(255,255,255), 2);

		putCenteredText(res, (string)text_d, center);
		putCenteredText(res, (string)text_a, Point(center.x,center.y+20));
	}

	output = res.clone();
}

struct pcl_layout{
	float x;
	float y;
	float z;
	union{
		float rgb;
		struct{ //endian-ness
			uint8_t b;
			uint8_t g;
			uint8_t r;
		};
	};
};

class Parallel_cvImg2ROSPCL: public cv::ParallelLoopBody{
	private:
		const Mat& xyz;
		const Mat& bgr;
		sensor_msgs::PointCloud2& msg;
	public:
		Parallel_cvImg2ROSPCL(const Mat& xyz, const Mat& bgr, sensor_msgs::PointCloud2& msg):
			xyz(xyz),bgr(bgr),msg(msg){}
		virtual void operator()(const cv::Range &r) const {
			pcl_layout* msg_ptr = (pcl_layout*) msg.data.data();
			const uint8_t* bgr_ptr = (const uint8_t*) bgr.data;
			const float* xyz_ptr = (const float*) xyz.data;

			float bad = std::numeric_limits<float>::quiet_NaN();

			for(int i = r.start; i < r.end; ++i){ // essentially index of point
				int i_3 = i*3;
				float z = xyz_ptr[i_3+2];
				auto& m= msg_ptr[i];
				if (z <= 100 && !std::isinf(z)){ // 10000 = MISSING_Z
					m.x = xyz_ptr[i_3+2];
					m.y = -xyz_ptr[i_3];
					m.z = -xyz_ptr[i_3+1];
					m.b = bgr_ptr[i_3];
					m.g = bgr_ptr[i_3+1];;
					m.r = bgr_ptr[i_3+2];;
				}else{
					m.x = m.y = m.z = bad;
				}
			}
		}

};

#define INIT_PFIELD(n, o, t) \
	sensor_msgs::PointField n; \
n.datatype = n.t; \
n.offset = o; \
n.count = 1; \
n.name = #n;

void format_pointfields(std::vector<sensor_msgs::PointField>& fields){
	fields.clear();

	//sensor_msgs::PointField x,y,z,b,g,r;
	/* Instantiate Fields */
	INIT_PFIELD(x,0,FLOAT32);
	INIT_PFIELD(y,4,FLOAT32);
	INIT_PFIELD(z,8,FLOAT32);

	INIT_PFIELD(rgb,12,FLOAT32);
	//INIT_PFIELD(a,15,UINT8);

	fields.push_back(x);
	fields.push_back(y);
	fields.push_back(z);
	fields.push_back(rgb);
}

void dist2pcl(Mat& dist, Mat& frame, sensor_msgs::PointCloud2& msg){
	//dist = (row,col,(x,y,z))
	msg.header.stamp = ros::Time::now();

	msg.height = dist.rows;
	msg.width  = dist.cols;

	format_pointfields(msg.fields);

	msg.is_bigendian = false;
	msg.point_step = sizeof(pcl_layout);
	msg.row_step = msg.width * sizeof(pcl_layout);

	int n = dist.rows * dist.cols;
	msg.data.resize(n*sizeof(pcl_layout));

	msg.is_dense = false; // there may be invalid points	

	parallel_for_(Range{0,dist.rows * dist.cols}, Parallel_cvImg2ROSPCL(dist, frame, msg), 8);
}

bool verbose = false;
bool pcl = false;
ObjectCriteria params;

void callback(edwin_stereo::EdwinStereoConfig& config, uint32_t level){
	verbose = config.verbose;
	pcl = config.pcl;
	ROS_INFO("[Reconfigure Request] Verbose : %s; PCL : %s", (verbose?"True":"False"),(pcl?"True":"False"));

	if(verbose){
		namedWindow("left", WINDOW_AUTOSIZE);
		namedWindow("right", WINDOW_AUTOSIZE);
		namedWindow("raw_disp", WINDOW_AUTOSIZE);
		namedWindow("disp", WINDOW_AUTOSIZE);
		namedWindow("filtered", WINDOW_AUTOSIZE);
	}else{
		destroyAllWindows();
	}

	auto& hsv1_l = params.hsv_range[0].min;
	auto& hsv1_h = params.hsv_range[0].max;
	hsv1_l[0] = config.h1_l;
	hsv1_h[0] = config.h1_h;
	hsv1_l[1] = config.s1_l;
	hsv1_h[1] = config.s1_h;
	hsv1_l[2] = config.v1_l;
	hsv1_h[2] = config.v1_h;
}

string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch ( depth ) {
		case CV_8U:  r = "8U"; break;
		case CV_8S:  r = "8S"; break;
		case CV_16U: r = "16U"; break;
		case CV_16S: r = "16S"; break;
		case CV_32S: r = "32S"; break;
		case CV_32F: r = "32F"; break;
		case CV_64F: r = "64F"; break;
		default:     r = "User"; break;
	}

	r += "C";
	r += (chans+'0');

	return r;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "edwin_stereo");
	ros::NodeHandle nh;

	ros::Publisher obj_pub;
	geometry_msgs::PointStamped obj_msg;
	obj_msg.header.frame_id = "camera_link";
	obj_pub = nh.advertise<geometry_msgs::PointStamped>("obj_point", 10, false);

	ros::Publisher pcl_pub;
	sensor_msgs::PointCloud2 pcl_msg;
	pcl_msg.header.frame_id = "camera_link";
	pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl", 10, false);

	std::string path = ros::package::getPath("edwin_stereo");

	dynamic_reconfigure::Server<edwin_stereo::EdwinStereoConfig> server;
	dynamic_reconfigure::Server<edwin_stereo::EdwinStereoConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	double vis_mult = 2.0;

	auto cap_l = cv::VideoCapture(0);
	if(!cap_l.isOpened()){
		ROS_ERROR("LEFT CAMERA CANNOT BE OPENED");
		return -1;
	}
	auto cap_r = cv::VideoCapture(2);
	if(!cap_l.isOpened()){
		ROS_ERROR("RIGHT CAMERA CANNOT BE OPENED");
		return -1;
	}
	Mat left,right;
	while (true){
		if(cap_l.read(left) && cap_r.read(right))
			break;	
	}

	FilteredSGBM bm;
	Rectifier r(path + "/params/left_camera.yaml", path + "/params/right_camera.yaml");
	Mat disp, raw_disp;
	Mat dist;
	Mat rect_left, rect_right;
	Mat filtered;

	// TODO : Dynamically Configure
	
	Mat filtered_disp_vis;
	Mat filtered_raw_disp_vis;
	Mat xyz[3];
	std::vector<LabelData> labels;

	while(ros::ok()){

		if( !cap_l.read(left) || !cap_r.read(right)){
			ROS_ERROR("FAILED TO READ IMAGE STREAM!!");
			return -1;
		}

		r.apply(left, right, rect_left, rect_right);
		bm.compute(rect_left, rect_right, disp, &raw_disp);
		r.convert(disp, dist);
		split(dist, xyz);

		apply_criteria(xyz[2],rect_left,params,filtered,labels);

		getDisparityVis(disp,filtered_disp_vis,vis_mult);
		getDisparityVis(raw_disp,filtered_raw_disp_vis,vis_mult);

		if(verbose){
			imshow("left", rect_left);
			imshow("right", right);
			imshow("raw_disp", filtered_raw_disp_vis);
			imshow("disp", filtered_disp_vis);
			imshow("filtered", filtered);
		}

		if(pcl){
			dist2pcl(dist, left, pcl_msg);
			pcl_pub.publish(pcl_msg);
		}
		if(labels.size() > 0){

			// biggest area comes first
			std::sort(labels.begin(), labels.end(), [](const LabelData& a,const LabelData& b){return a.a > b.a;});

			auto& label = labels.front();
			auto& mpt = obj_msg.point;
			auto& pt = dist.at<Vec3f>(label.cY, label.cX);
			//std::cout << label.a << std::endl;
			//camera coord. to ROS coord
			mpt.x = pt[2];
			mpt.y = -pt[0];
			mpt.z = -pt[1];
			obj_msg.header.stamp = ros::Time::now();
			obj_pub.publish(obj_msg);
		}


		int k = waitKey(10);
		if(k == 27)
			break;
		ros::spinOnce();
	}

	return 0;
}

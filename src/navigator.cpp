// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <std_msgs/Empty.h>

// IMAGE PROCESSING
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

// IMAGE RENDERING
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

// MISC.
#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <pthread.h>
#include <math.h>
#include <cmath>

// CONFIGURATION
#include "ConfigFile.cpp"

// ROS CONFIGURATION
#include <yaml-cpp/yaml.h>

// THREAD DATA MANAGEMENT
class DataManagement
{
	private:
	
		cv::Mat tvec;
		cv::Mat rvec;
		cv::Mat frame;
		bool transformation_ready = false;
		bool image_ready = false;
		bool thread_continue = true;
		float altitude = 0;
		
	public:
	
		void loadTransform(cv::Mat t, cv::Mat r);
		void loadFrame(cv::Mat f);
		void loadAltitude(float a);
		
		void getTransform(cv::Mat& t, cv::Mat& r);
		bool getFrame(cv::Mat& f);
		void getAltitude(float& a);
		bool getThreadStatus();
		
		bool statusTransform();
		
		void stopThread();
};

void DataManagement::loadTransform(cv::Mat t, cv::Mat r){
	tvec = t;
	rvec = r;
	transformation_ready = true;
}

void DataManagement::loadFrame(cv::Mat f){
	frame = f.clone();
	image_ready = true;
}

void DataManagement::loadAltitude(float a){
	altitude = a;
}

void DataManagement::getTransform(cv::Mat& t, cv::Mat& r){
	t = tvec;
	r = rvec;
}

bool DataManagement::getFrame(cv::Mat& f){
	if (image_ready){
		f = frame.clone();
		image_ready = false;
		return true;
	}
	else
		return false;
}

void DataManagement::getAltitude(float& a){
	altitude = a;
}

bool DataManagement::getThreadStatus(){
	return thread_continue;
}

bool DataManagement::statusTransform(){
	return transformation_ready;
}

void DataManagement::stopThread(){
	thread_continue = false;
}

// For Data Management 
DataManagement dm;

// For Debugging
bool debug_rendering_ = false;
//~ bool debug_rendering_ = true;

//~ bool debug_calibration_ = false;
bool debug_calibration_ = true;

bool debug_ = false;

// For Texturing
GLuint textures[2];
cv::Mat mat_vertex;
cv::Mat obj_tex_mat;

// For Camera Parameters
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
cv::Mat camera_matrix;
cv::Mat dist_coeffs;

// For ARUCO
int target_id = 1;	
float aruco_size = 0.082/2;

// For Camera Frame Rendering
int window_height = 480;
int window_width = 856;

int image_height = window_height;
int image_width = window_width;

float frame_height;
float frame_width;

float clipping_length;
float focal_length;

float camera_distance;
int framecount = 0;
bool frame_ready = false;

// For Virtual Object Loading
int vertex_type = 4;
bool extract_colors = false;
bool extract_normals = true;

std::vector<std::vector<float> > vertex;
std::vector<std::vector<float> > normals;
std::vector<std::vector<float> > vertex_color;
std::vector<std::vector<int> > vertex_index;

std::vector<std::vector<float> > tex_vertex;
std::vector<std::vector<float> > tex_normals;
std::vector<std::vector<int> > tex_vertex_index;

// For Virtual Object Rendering
cv::Mat virtual_tvec;
cv::Mat virtual_rvec;
float x_correction = 0.0;
float y_correction = 0.0;
float z_correction = 0.0;
float f_correction = 0.0;
float correct_increment = 0.001;

// For ROS .yaml calibration
std::string yaml_path_;
std::string package_path_;
std::string image_topic_;
std::string odom_topic_;
std::string takeoff_topic_;
std::string landing_topic_;
std::string cmd_vel_topic_;
float cmd_vel_increment_;

// ROS Interface
ros::Publisher takeoff_pub;
ros::Publisher landing_pub;
ros::Publisher cmd_vel_pub;
std_msgs::Empty empty_msg;
geometry_msgs::Twist geo_msg;

void updateParameters(YAML::Node config){
	if (config["fx"])
    camera_matrix.at<double>(0,0) = config["fx"].as<double>();
	if (config["fx"])
    camera_matrix.at<double>(1,1) = config["fx"].as<double>();
	if (config["x0"])
    camera_matrix.at<double>(0,2) = config["x0"].as<double>();
	if (config["y0"])
    camera_matrix.at<double>(1,2) = config["y0"].as<double>();
  if (config["k1"])
    dist_coeffs.at<double>(0,0) = config["k1"].as<double>();
  if (config["k2"])
    dist_coeffs.at<double>(1,0) = config["k2"].as<double>();
  if (config["k3"])
    dist_coeffs.at<double>(4,0) = config["k3"].as<double>();
  if (config["p1"])
    dist_coeffs.at<double>(2,0) = config["p1"].as<double>();
  if (config["p2"])
    dist_coeffs.at<double>(3,0) = config["p2"].as<double>();
}

void loadCalibrationMatrix(std::string camera_name_){
	// Updates Parameter with .yaml file
	camera_matrix = cv::Mat::eye(3, 3, CV_64F);
	dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
	camera_matrix.at<double>(2,2) = 1;
  yaml_path_ = ros::package::getPath("ivan_aruco") + "/config/camera_info.yaml";
  YAML::Node config;
  try 
  {
    config = YAML::LoadFile(yaml_path_);
  } 
  catch (YAML::Exception &e) 
  {
    ROS_ERROR_STREAM("YAML Exception: " << e.what());
    exit(EXIT_FAILURE);
  }
  if (!config[camera_name_])
  {
    ROS_ERROR("Cannot find default parameters in yaml file: %s", yaml_path_.c_str());
    exit(EXIT_FAILURE);
  }
  updateParameters(config[camera_name_]);
}

bool arucoPoseEstimation(cv::Mat& input_image, int id, cv::Mat& tvec, cv::Mat& rvec, cv::Mat& mtx, cv::Mat& dist, bool draw_axis){
	// Contextual Parameters
	//~ std::cout << "Pose estimation called" << std::endl;
	float aruco_square_size = aruco_size*2;
	bool marker_found = false;
	std::vector< int > marker_ids;
	std::vector< std::vector<cv::Point2f> > marker_corners, rejected_candidates;
	cv::Mat gray;
	
	cv::cvtColor(input_image, gray, cv::COLOR_BGR2GRAY);
	cv::aruco::detectMarkers(gray, dictionary, marker_corners, marker_ids);	
	
	if (marker_ids.size() > 0){
		for (int i = 0 ; i < marker_ids.size() ; i++){
			//~ std::cout << "Marker IDs found: " << marker_ids[i] << std::endl;
			if (marker_ids[i] == id){
				std::vector< std::vector<cv::Point2f> > single_corner(1);
				single_corner[0] = marker_corners[i];
				cv::aruco::estimatePoseSingleMarkers(single_corner, aruco_square_size, mtx, dist, rvec, tvec);
				if (draw_axis){
					cv::aruco::drawDetectedMarkers(input_image, marker_corners, marker_ids);
					cv::aruco::drawAxis(input_image, mtx, dist, rvec, tvec, aruco_square_size/2);
				}
				//~ std::cout << "Marker found : aruco pose estimation" << std::endl;
				marker_found = true;
			}
		}
	}
	else{
		//~ std::cout << "No markers detected" << std::endl;
	}
	
	return marker_found;
}

void init_texture(std::string package_path_){
	
	ROS_DEBUG_STREAM("Initiating Vertex");
	
	float aspect_ratio = image_width/image_height;
	ROS_DEBUG_STREAM("Aspect Ratio: " << aspect_ratio);
	clipping_length = (1.0 / 2.0) * ( 1.0 - 1.0/ aspect_ratio);
	float pixel_ratio = camera_distance/focal_length;
	frame_height = image_height * pixel_ratio / 2;
	frame_width = image_width * pixel_ratio / 2;
	
	ROS_DEBUG_STREAM("Camera Distance: " << camera_distance);
	ROS_DEBUG_STREAM("Focal Length: " << focal_length);
	ROS_DEBUG_STREAM("Frame Height: " << frame_height);
	ROS_DEBUG_STREAM("Frame Width: " << frame_width);
	
	cv::flip(mat_vertex, mat_vertex, 0);
	
	glGenTextures(2, textures);
	
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mat_vertex.cols, mat_vertex.rows,  0, GL_BGR, GL_UNSIGNED_BYTE, mat_vertex.ptr());

	glBindTexture(GL_TEXTURE_2D, 0); 
	
}

void update_texture(void){
	frame_ready = dm.getFrame(mat_vertex);
	if (frame_ready){
		glBindTexture(GL_TEXTURE_2D, textures[0]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mat_vertex.cols, mat_vertex.rows,  0, GL_BGR, GL_UNSIGNED_BYTE, mat_vertex.ptr());
	}
}

void init_lighting(void){
	GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_shininess[] = { 50.0 };
	GLfloat light_position[] = { 0.0, 1.0, 0.0, 0.0 };
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glShadeModel (GL_SMOOTH);

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
}

std::string type2str(int type) {
  std::string r;

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

// OPENGL CALLBACK
void changeSize(int w, int h) {

	ROS_DEBUG_STREAM("Change Size Callback");
	ROS_DEBUG_STREAM("Size detected: " << w << " x " << h);
	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (h == 0)
		h = 1;

	float ratio =  w * 1.0 / h;
	
	// Use the Projection Matrix
	glMatrixMode(GL_PROJECTION);

	// Reset Matrix
	glLoadIdentity();

	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	// Set the correct perspective.
	// Original FOV is 45 deg.
	ROS_DEBUG_STREAM("Aspect ratio: " << ratio);
	gluPerspective(32.0f, ratio, 0.1f, 100.0f);

	// Get Back to the Modelview
	glMatrixMode(GL_MODELVIEW);
	
}

void drawBitmapText(char *string,float x,float y,float z) {  
	std::cout << "Draw Bitmap" << std::endl;
	char *c;
	//~ glRasterPos3f(x, y,z);
	glRasterPos2f(x, y);

	for (c=string; *c != (char) 0; c++) 
	{
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, *c);
	}
}

void renderScene(void) {
	
	// Clear Color and Depth Buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// Reset transformations
	glLoadIdentity();
	
	// Draw Text
	//~ drawBitmapText("Hello World",200,200,camera_distance);
	
	// Set the camera
	gluLookAt(	0.0f, 0.0f, 0.0f,
				0.0f, 0.0f,  1.0f,
				0.0f, 1.0f,  0.0f);
					
	// For Camera Background
	update_texture();
	
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	glBegin(GL_QUADS);
	 glTexCoord2f(1.0, 1.0);
	 glVertex3f(-frame_width, -frame_height, camera_distance);
	 glTexCoord2f(1.0, 0.0);
	 glVertex3f(-frame_width, frame_height, camera_distance);
	 glTexCoord2f(0.0, 0.0);
	 glVertex3f(frame_width, frame_height, camera_distance);
	 glTexCoord2f(0.0, 1.0);
	 glVertex3f(frame_width, -frame_height, camera_distance);
	glEnd();
	
	glBindTexture(GL_TEXTURE_2D, 0); 
	
	glutSwapBuffers();
	
}

void compute_camera_frame_size(){
	float width = mat_vertex.cols;
	float height = mat_vertex.rows;
	float pixel_ratio = camera_distance/focal_length;
	frame_height = height * pixel_ratio / 2;
	frame_width = width * pixel_ratio / 2;
}

void take_off_command(){
	ROS_INFO("Initiate Takeoff");
	std::cout << "Initiate Takeoff" << std::endl;
	takeoff_pub.publish(empty_msg);
}

void land_command(){
	ROS_INFO("Initiate Landing");
	std::cout << "Initiate Landing" << std::endl;
	landing_pub.publish(empty_msg);
}

void descend(){
	std::cout << "Descend" << std::endl;
	geo_msg.linear.z = -cmd_vel_increment_;
	cmd_vel_pub.publish(geo_msg);
}

void hover(){
	std::cout << "Hover" << std::endl;
	geo_msg.linear.z = 0;
	cmd_vel_pub.publish(geo_msg);
}

void ascend(){
	std::cout << "Ascend" << std::endl;
	geo_msg.linear.z = cmd_vel_increment_;
	cmd_vel_pub.publish(geo_msg);
}

void processNormalKeys(unsigned char key, int x, int y) {
	std::cout << "key press: " << key << std::endl;
	if (key == 'q'){
		take_off_command();
	}
	if (key == 'a'){
		land_command();
	}
	if (key == 'w'){
		ascend();
	}
	if (key == 's'){
		hover();
	}
	if (key == 'x'){
		descend();
	}
	if (key == 'd'){
		z_correction = z_correction - correct_increment;
	}
	if (key == 'r'){
		focal_length++;
		std::cout << "focal_length: " << focal_length << std::endl;
		compute_camera_frame_size();
	}
	if (key == 'f'){
		focal_length--;
		std::cout << "focal_length: " << focal_length << std::endl;
		compute_camera_frame_size();
	}
	if (key == 'z')
		exit(0);
}

// THREAD FUNCTIONS
void *start_gl_main(void *threadid){
	long tid;
  tid = (long)threadid;
  ROS_DEBUG_STREAM("Start gl main thread id : " << tid);
	glutMainLoop();
	ROS_DEBUG_STREAM("Ending gl main thread");
	//~ pthread_exit(NULL);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //~ ROS_INFO("Seq: [%d]", msg->header.seq);
  //~ ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //~ ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //~ ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  dm.loadAltitude(msg->pose.pose.position.z);
  
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	
	try{
		cv::Mat image;
		cv::Mat unDistort;
		cv::Mat rvec;
		cv::Mat tvec;
		bool marker_found;
		image = cv_bridge::toCvShare(msg, "bgr8")->image;
		//~ cv::undistort(image, unDistort, camera_matrix, dist_coeffs);
		//~ marker_found = false;
  	//~ marker_found = arucoPoseEstimation(image, target_id, tvec, rvec, camera_matrix, dist_coeffs, true);
		//~ cv::undistort(image, unDistort, camera_matrix, dist_coeffs);
		dm.loadFrame(image);
		//~ if (marker_found==true){
			//~ dm.loadTransform(tvec, rvec);			
		//~ }
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	
}

//ROS Loop
void *start_cv_main(void *threadid){
	ROS_DEBUG_STREAM("CV Main Thread Start");
  ros::NodeHandle n;
	image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe(image_topic_, 1, imageCallback);
  ros::Subscriber sub2 = n.subscribe(odom_topic_, 50, odomCallback);
  
  landing_pub = n.advertise<std_msgs::Empty>(landing_topic_, 1);
  takeoff_pub = n.advertise<std_msgs::Empty>(takeoff_topic_, 1);
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
  ros::spin();
}

// MAIN
int main (int argc, char** argv){
  
  ros::init(argc, argv, "navigator");
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_("~");
  
  // VARIABLE INITIALISATION    
  package_path_ = ros::package::getPath("kingfisher_drone");
  nh_private_.getParam("image_topic_", image_topic_);
  nh_private_.getParam("debug_", debug_);
  nh_private_.getParam("camera_distance_", camera_distance);
  nh_private_.getParam("focal_length_", focal_length);
  nh_private_.getParam("takeoff_topic_", takeoff_topic_);
  nh_private_.getParam("landing_topic_", landing_topic_);
  nh_private_.getParam("cmd_vel_topic_", cmd_vel_topic_);
  nh_private_.getParam("cmd_vel_increment_", cmd_vel_increment_);
  
  if (debug_)
	{
		ROS_INFO("Debug Mode ON");
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	}
	else
	{
		ROS_INFO("Debug Mode OFF");
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	}
	
  ROS_DEBUG("Kingfisher Navigator");  
  ROS_DEBUG_STREAM("Package path: " << package_path_);
	
	// GLUT INITIALISATION
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100,100);
	glutInitWindowSize(window_width,window_height);
	glutCreateWindow("Kingfisher Navigator");
	init_texture(package_path_);
	init_lighting();
	
	// GLUT CALLBACK REGISTRATION
	glutDisplayFunc(renderScene);
	glutReshapeFunc(changeSize);
	glutIdleFunc(renderScene);
	glutKeyboardFunc(processNormalKeys);
  
	// THREAD INITIALISATION	
	pthread_t thread[2];
	int threadError;
	int i=0;

	// START GL THREAD
	threadError = pthread_create(&thread[i], NULL, start_gl_main, (void *)i);

	if (threadError){
		ROS_DEBUG_STREAM("Error:unable to create thread," << threadError);
		exit(-1);
	}

	// START CV THREAD
	i++;
	threadError = pthread_create(&thread[i], NULL, start_cv_main, (void *)i);

	if (threadError){
		ROS_DEBUG_STREAM("Error:unable to create thread," << threadError);
		exit(-1);
	}
	
	pthread_exit(NULL);

  return 0;
}

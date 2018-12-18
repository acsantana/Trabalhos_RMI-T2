
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"


#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include "tf/transform_datatypes.h"

#define PARTICLE_NUM	1000		//numero de particulas parametrizavel
#define MAX_DEV			300		//janela de distribuicao das amostras
#define MAX_DEV_T		20		//Desvio maximo de theta
#define RESOLUTION		100		//100 pixels por metro
#define MEASURES 		640		//amostras do laser
#define NOISE			10		//Noise added on copied particles
#define WINDOW 			10		//Resampling window
#define SCAN_INCR		10
#define AREA_PIXEL		5

#define OFFSET_X		0 //70
#define OFFSET_Y		0 //200

#define INIT_W			1.0		//1.0 para multiplicacao, 0.0 para soma logaritmica

using namespace std;

struct Particle {
	float w; //weight
	float x;
	float y;
	float theta;
};

struct Odometry {
	float x;
	float y;
	float theta;
};

struct Position {
	int x;
	int y;
	float theta;
};

struct SensorData {
	float angle;
	float distance;
};

void init_particle(struct Particle *p, float w, float x, float y, float theta){
	p->w = w;
	p->x = x;
	p->y = y;
	p->theta = theta;
}

void init_cloud(struct Particle *p, float x, float y, float theta){
	int i;

	for (i = 0; i < PARTICLE_NUM; i++){
		p[i].w = 0.0;
		p[i].x = x + float(rand()%MAX_DEV*2 - MAX_DEV + OFFSET_X)/RESOLUTION;
		p[i].y = y + float(rand()%MAX_DEV*2 - MAX_DEV + OFFSET_Y)/RESOLUTION;
		p[i].theta = theta + float(rand()%MAX_DEV_T*2 - MAX_DEV_T)/RESOLUTION;

		if(p[i].theta > M_PI) p[i].theta = p[i].theta - 2*M_PI;
		if(p[i].theta < -M_PI) p[i].theta = p[i].theta + 2*M_PI;
	}
}

void copy_cloud(struct Particle *p, struct Particle *np){
	int i;

	for(i = 0; i < PARTICLE_NUM; i++){
		p[i].x = np[i].x;
		p[i].y = np[i].y;
		p[i].theta = np[i].theta;
		p[i].w = np[i].w;
	}
}

void motion_update(struct Particle *p, Odometry current_pose, Odometry last_pose){
	int i;
	float dx, dy, dtheta;

	dx = current_pose.x - last_pose.x;
	dy = current_pose.y - last_pose.y;
	dtheta = current_pose.theta - last_pose.theta;

	for (i = 0; i < PARTICLE_NUM; i++){
		p[i].x = p[i].x + dx;
		p[i].y = p[i].y + dy;
		p[i].theta = p[i].theta + dtheta;

		if(p[i].theta > M_PI) p[i].theta = p[i].theta - 2*M_PI;
		if(p[i].theta < -M_PI) p[i].theta = p[i].theta + 2*M_PI;
	}
}

int check_bounds(int sx, int sy, int w, int h){
	if(sx < 0 || sy < 0)
		return 0;
	if(sx >= w || sy >= h)
		return 0;
	return 1;
}

void sensor_update(struct Particle *p, struct SensorData *sensor_data, char *map, int w, int h){
	int i, j;

	int sx, sy;		//sensor coordinates
	int px, py;		//particle coordinates

	float angle;

	float prob;

	for (i = 0; i < PARTICLE_NUM; i++){

		px = w/2 +  p[i].x*RESOLUTION;
		py = h/2 +  p[i].y*RESOLUTION;

		prob = INIT_W;
		for (j = 0; j < MEASURES; j++){
			if (sensor_data[j].distance != 0){
				sx = px + (int)(cos(p[i].theta + sensor_data[j].angle)*sensor_data[j].distance*RESOLUTION);
				sy = py + (int)(sin(p[i].theta + sensor_data[j].angle)*sensor_data[j].distance*RESOLUTION);
				if (check_bounds(sx, sy, w, h))
					prob *= (float)(255 - (uchar)map[sy*w + sx])/255;
			}
		}
		p[i].w += prob;
	}
}

// Calculates probability according to the gaussian distribution
double measureProb(double x, double mu, double sigma){
	return exp( -pow(x-mu,2)/(2*pow(sigma,2)) / sqrt(2*M_PI*pow(sigma,2)) );
}

float calc_particle_weight(struct Particle p, struct SensorData *sensor_data, char *map, int w, int h, float range_min, float range_max){
	int j;

	int sx, sy;		//sensor coordinates
	int px, py;		//particle coordinates

	float angle, range;

	float prob;

	px = w/2 +  p.x*RESOLUTION;
	py = h/2 +  p.y*RESOLUTION;

	prob = INIT_W;
	for (j = 0; j < MEASURES; j += SCAN_INCR){
		for (range = range_min; range <= range_max; range += 0.01){
			sx = px + (int)(cos(p.theta + sensor_data[j].angle)*range*RESOLUTION);
			sy = py + (int)(sin(p.theta + sensor_data[j].angle)*range*RESOLUTION);
			if (check_bounds(sx, sy, w, h) && map[sy*w + sx] == 0)
				break;
		}
		if (sensor_data[j].distance == 0)
			prob += log(measureProb(range, 10, 1)); //(abs(sensor_data[j].distance - range));
		else
			prob += log(measureProb(range, sensor_data[j].distance, 1));
		//ROS_INFO("prob %d = %f", j, prob);
	}
	return prob;

}

void sensor_update_trace(struct Particle *p, struct SensorData *sensor_data, char *map, int w, int h, float range_min, float range_max){
	int i;

	for (i = 0; i < PARTICLE_NUM; i++)
		p[i].w = exp(calc_particle_weight(p[i], sensor_data, map, w, h, range_min, range_max));

}

void trace_img(struct Particle p, struct SensorData *sensor_data, char *map, int w, int h, float range_min, float range_max, cv::Mat image){
	int i, j;

	int sx, sy;		//sensor coordinates
	int px, py;		//particle coordinates

	float angle, range;

	float diff_sum;

	px = w/2 +  p.x*RESOLUTION;
	py = h/2 +  p.y*RESOLUTION;

	diff_sum = 0;
	for (j = 0; j < MEASURES; j += SCAN_INCR){
		for (range = range_min; range <= range_max; range += 0.01){
			sx = px + (int)(cos(p.theta + sensor_data[j].angle)*range*RESOLUTION);
			sy = py + (int)(sin(p.theta + sensor_data[j].angle)*range*RESOLUTION);
			//image.at<uchar>(cv::Point(sx,sy)) = cv::Scalar(200,200,200);

			cv::Point mc(sx, sy);
			cv::circle(image, mc, 1, cv::Scalar(200,200,200), -1);
			if (check_bounds(sx, sy, w, h) && map[sy*w + sx] == 0)
				break;
		}
	}
}

void smooth(float *distances, int h, int w){
	for (int i = 1; i < h-1; i++){
		for (int j = 1; j < w-1; j++){
			if (distances[i*w + j] != 0){
				distances[i*w + j] = (distances[(i+1)*w + j] + distances[(i-1)*w + j] + distances[i*w + j+1] + distances[i*w + j-1])/4;
			}
		}
	}
}

void normalize_weights(struct Particle *p){
	float total_weight = 0;
	float average;
	int i;

	for(i = 0; i < PARTICLE_NUM; i++) total_weight += p[i].w;
	for(i = 0; i < PARTICLE_NUM; i++) p[i].w = p[i].w/total_weight;
}

void filter(struct Particle *p, struct Particle *np){
	int i;
	int n;
	float cumulative_sum[PARTICLE_NUM];
	float index;

	cumulative_sum[0] = p[0].w;
	for(i = 1; i < PARTICLE_NUM; i++){
		cumulative_sum[i] = p[i].w + cumulative_sum[i-1];
	}

	for(i = 0; i < PARTICLE_NUM; i++){
		index = (float)(rand()%1000)/1000;
		n = 0;
		while(index > cumulative_sum[n])
			n++;
		np[i].x = p[n].x + float(rand()%NOISE*2 - NOISE)/RESOLUTION;
		np[i].y = p[n].y + float(rand()%NOISE*2 - NOISE)/RESOLUTION;
		np[i].theta = p[n].theta + float(rand()%NOISE*2 - NOISE)/RESOLUTION;
		np[i].w = 0.0;
	}
}

struct Odometry compute_predicted_pose(struct Particle *p){
	int i;
	float sum_x, sum_y;
	Odometry pose;

	sum_x = 0;
	sum_y = 0;
	for(i = 0; i < PARTICLE_NUM; i++){
		sum_x += p[i].x * p[i].w;
		sum_y += p[i].y * p[i].w;
	}
	pose.x = sum_x;
	pose.y = sum_y;
	return pose;
}

int particle_sum(struct Particle p, struct SensorData *sensor_data, char *map, int w, int h){

	int d, j;

	int sx, sy;		//sensor coordinates
	int px, py;		//particle coordinates


	px = w/2 +  p.x*RESOLUTION;
	py = h/2 +  p.y*RESOLUTION;

	d = 0;
	for (j = 0; j < MEASURES; j++){
		if (sensor_data[j].distance != 0){
			sx = px + (int)(cos(p.theta + sensor_data[j].angle)*sensor_data[j].distance*RESOLUTION);
			sy = py + (int)(sin(p.theta + sensor_data[j].angle)*sensor_data[j].distance*RESOLUTION);
			if (check_bounds(sx, sy, w, h))
				d += (255 - (uchar)map[sy*w + sx]);
		}
	}
	return d;

}

int particle_laser(struct Particle p, struct SensorData *sensor_data, cv::Mat image, int w, int h, int color, char *map){
	int i;
	float lx, ly;

	int sx, sy;		//sensor coordinates
	int px, py;		//particle coordinates

	px = w/2 +  p.x*RESOLUTION;
	py = h/2 +  p.y*RESOLUTION;

	int d = 0;
	for (i = 0; i < MEASURES; i++){
		if (sensor_data[i].distance != 0){
			//lx = cos(p.theta + sensor_data[i].angle)*sensor_data[i].distance;
			//ly = sin(p.theta + sensor_data[i].angle)*sensor_data[i].distance;
			sx = px + (int)(cos(p.theta + sensor_data[i].angle)*sensor_data[i].distance*RESOLUTION);
			sy = py + (int)(sin(p.theta + sensor_data[i].angle)*sensor_data[i].distance*RESOLUTION);
			if (check_bounds(sx, sy, w, h))
				d += (255 - (uchar)map[sy*w + sx]);
			//cv::Point mc((int)(lx*RESOLUTION) + (int)(p.x*RESOLUTION) + w/2, (int)(ly*RESOLUTION) + (int)(p.y*RESOLUTION) + h/2);
			cv::Point mc(sx, sy);
			cv::circle(image, mc, 3, cv::Scalar(0,0,color), -1);
			//if (color == 255) ROS_INFO("laser %d, %d, map = %d", sx, sy, (uchar)map[sy*w + sx]);
		}
	}
	return d;
}

class ParticleFilter
{
private:
	ros::NodeHandle n;
	ros::Subscriber target_sub;
	ros::Subscriber map_sub;
	ros::Subscriber sensor_sub;
	ros::Subscriber odom_sub;
	ros::Publisher cloud_pub;
	ros::Publisher pose_pub;
	ros::Publisher cmd_vel_pub;

	geometry_msgs::PoseStamped target;
	geometry_msgs::Twist twist;
	sensor_msgs::LaserScan laser;
	nav_msgs::OccupancyGrid map;
	nav_msgs::Odometry odom;


	cv::Mat image;
	cv::Mat Source_Target;
	cv::Mat Source_Target_small;
	cv::Mat potential_fields;

	Particle particle_cloud[PARTICLE_NUM];
	SensorData sensor_measures[MEASURES];
	Odometry current_pose, last_pose, predicted_pose;
	
	Odometry target_pose;
	char *objects;
	float *map_float;

	bool init = false;
	bool moving = false;
	int best_particle, iterations, miterations;

public:
	ParticleFilter();
	void target_callback(const geometry_msgs::PoseStamped& goal);
	void sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	void move_bot(float* map_float);
	void publish_cloud();
	void publish_pose();
};

ParticleFilter::ParticleFilter() {
	iterations = 0;
	miterations = 0;
	moving = false;
	target_sub = n.subscribe("move_base_simple/goal", 5, &ParticleFilter::target_callback, this);
	sensor_sub = n.subscribe("scan", 5, &ParticleFilter::sensor_callback, this);
	map_sub = n.subscribe("map", 1, &ParticleFilter::map_callback, this);
	odom_sub = n.subscribe("odom", 5, &ParticleFilter::odom_callback, this);
	//cloud_pub = n.advertise<geometry_msgs::PoseArray>("particle_cloud", 1);
	pose_pub = n.advertise<geometry_msgs::Pose>("pose", 1);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
}

void ParticleFilter::sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg){

	cv::Mat image;

	int ox, oy, otheta;					//odometer pixel position
	int last_ox, last_oy, last_otheta;	//last odometer pixel position
	int px, py, ptheta;					//predicted pixel position

	float lx, ly, angle;				//laser variables

	float best_val;

	Position current_xytheta;
	Particle new_cloud[PARTICLE_NUM];
	Particle odom_p;

	ox = map.info.width/2 + (int)(current_pose.x*RESOLUTION) + OFFSET_X;
	oy = map.info.height/2 + (int)(current_pose.y*RESOLUTION) + OFFSET_Y;
	otheta = (int)(current_pose.theta*RESOLUTION);

	last_ox = map.info.width/2 + (int)(last_pose.x*RESOLUTION) + OFFSET_X;
	last_oy = map.info.height/2 + (int)(last_pose.y*RESOLUTION) + OFFSET_Y;
	last_otheta = (int)(last_pose.theta*RESOLUTION);

	current_xytheta.x = ox;
	current_xytheta.y = oy;
	current_xytheta.theta = current_pose.theta;

	cv::cvtColor(this->image, image, CV_GRAY2BGR);

	int i = 0;
	for (angle = msg->angle_min; angle < msg->angle_max; angle += msg->angle_increment){
		if (msg->ranges[i] == msg->ranges[i]){
			lx = cos(angle + current_pose.theta)*msg->ranges[i];
			ly = sin(angle + current_pose.theta)*msg->ranges[i];

			sensor_measures[i].angle = angle;
			sensor_measures[i].distance = msg->ranges[i];

			//cv::Point mc((int)(lx*RESOLUTION) + ox, (int)(ly*RESOLUTION) + oy);
			//cv::circle(image, mc, 3, cv::Scalar(0,0,255), -1);
		}
		else {
			sensor_measures[i].angle = angle;
			sensor_measures[i].distance = 0;			
		}
		i++;
	}

	odom_p.x = current_pose.x + (float)OFFSET_X/RESOLUTION;
	odom_p.y = current_pose.y + (float)OFFSET_Y/RESOLUTION;
	odom_p.theta = current_pose.theta;

	particle_laser(particle_cloud[best_particle], sensor_measures, image, map.info.width, map.info.height, 0, objects);
	int test = particle_laser(odom_p, sensor_measures, image, map.info.width, map.info.height, 255, objects);

	/* verifica movimento */
	if (ox != last_ox || oy != last_oy || otheta != last_otheta){
		iterations++;
		motion_update(particle_cloud, current_pose, last_pose);
		sensor_update(particle_cloud, sensor_measures, objects, map.info.width, map.info.height);
		//sensor_update_trace(particle_cloud, sensor_measures, objects, map.info.width, map.info.height, msg->range_min, msg->range_max);
		best_val = particle_cloud[0].w;
		best_particle = 0;
		for (i = 1; i < PARTICLE_NUM; i++){
			if (particle_cloud[i].w > best_val){
				best_val = particle_cloud[i].w;
				best_particle = i;
			}
			//ROS_INFO("particle %d = %f", i, particle_cloud[i].w);
		}
		//ROS_INFO("best_particle %d", best_particle);
		//ROS_INFO("particle info = %fx%f %f", particle_cloud[best_particle].x, particle_cloud[best_particle].y, particle_cloud[best_particle].theta);
		//ROS_INFO("best_particle weight %f", particle_cloud[best_particle].w);

		if(iterations == WINDOW){
			iterations = 0;

			normalize_weights(particle_cloud);
			filter(particle_cloud, new_cloud);
			copy_cloud(particle_cloud, new_cloud);
		}
		last_pose = current_pose;
	}
	predicted_pose.x = particle_cloud[best_particle].x;
	predicted_pose.y = particle_cloud[best_particle].y;
	predicted_pose.theta = particle_cloud[best_particle].theta;
	//publish_cloud();
	publish_pose();

	//trace_img(odom_p, sensor_measures, objects, map.info.width, map.info.height, msg->range_min, msg->range_max, image);

	/* Desenha Particulas */
	for (i = 0; i < PARTICLE_NUM; i++){
		int particle_x = map.info.width/2 + (int)(particle_cloud[i].x*RESOLUTION);
		int particle_y = map.info.height/2 + (int)(particle_cloud[i].y*RESOLUTION);
		cv::Point p1(particle_x, particle_y);
		cv::Point p2(particle_x+10*cos(particle_cloud[i].theta), particle_y+10*sin(particle_cloud[i].theta));

		cv::arrowedLine(image, p1, p2, cv::Scalar(255,0,0), 2, 8, 0, 0.5);
		if (i == best_particle) cv::arrowedLine(image, p1, p2, cv::Scalar(0,255,0), 2, 8, 0, 0.5);
	}

	/* Desenha posicao de odometria */
	cv::Point p1(ox, oy);
	cv::Point p2(ox+10*cos(current_pose.theta), oy+10*sin(current_pose.theta));
	cv::arrowedLine(image, p1, p2, cv::Scalar(0,0,255), 2, 8, 0, 0.5);

	cv::Mat image_small(map.info.height/2, map.info.width/2, CV_8UC3, 255);
	cv::resize(image, image_small, image_small.size());

	cv::imshow("view", image_small);
	cv::waitKey(3);

	miterations++;
	if(miterations == 1){
		miterations = 0;
		if (moving) move_bot(map_float);
	}
}

void ParticleFilter::publish_cloud(){
	int i;
	/*geometry_msgs::Quaternion q;
	geometry_msgs::Pose p;

	for (i = 0; i < PARTICLE_NUM; i++){
		q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, particle_cloud[i].theta);

		p.position.x = particle_cloud[i].x;
		p.position.y = particle_cloud[i].y;
		p.position.z = 0;
		p.orientation = q;

		poseArray.poses[i] = p;
	}
	//cloud_pub.publish(poseArray);*/
}

void ParticleFilter::publish_pose(){
	int i;

	geometry_msgs::Quaternion q;
	geometry_msgs::Pose p;

	q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, predicted_pose.theta);

	p.position.x = predicted_pose.x;
	p.position.y = predicted_pose.y;
	p.position.z = 0;
	p.orientation = q;

	pose_pub.publish(p);
}

void ParticleFilter::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

	ROS_INFO("received map: %d x %d", msg->info.width, msg->info.height);
	map = *msg;

	cv::Mat image(map.info.height, map.info.width, CV_8UC1, 255);
	cv::Mat smooth_map(map.info.height, map.info.width, CV_8UC3, 255);
	cv::Mat input = cv::imread("smooth_map.pgm", 1);
	cv::cvtColor(input, smooth_map, CV_BGR2GRAY);

	objects = (char*)malloc(map.info.height * map.info.width);

	for (int i = 0; i < map.info.height; i++){
		for (int j = 0; j < map.info.width; j++){
			objects[i*map.info.width + j] = 255;
			if (map.data[i*map.info.width + j] == -1){
				image.at<uchar>(cv::Point(j,i)) = 128;
				objects[i*map.info.width + j] = 0;
			}
			if (map.data[i*map.info.width + j] == 100){
				image.at<uchar>(cv::Point(j,i)) = 0;
				objects[i*map.info.width + j] = 0;
			}

			objects[i*map.info.width + j] = smooth_map.at<uchar>(cv::Point(j,i));
		}
	}

	/* Procedimentos para gerar mapa de distancias (lento)*/
	/*float *distances = (float*)malloc(map.info.height * map.info.width*sizeof(float));
	for (int i = 0; i < map.info.height; i++){
		for (int j = 0; j < map.info.width; j++){
			if (map.data[i*map.info.width + j] == 100)
				distances[i*map.info.width + j] = 0.0;
			else
				distances[i*map.info.width + j] = 1.0;
		}
	}

	for (int i = 0; i < 170; i++)
		smooth(distances, map.info.height, map.info.width);

	for (int i = 0; i < map.info.height; i++)
		for (int j = 0; j < map.info.width; j++){
			image.at<uchar>(cv::Point(j,i)) = distances[i*map.info.width + j]*255;
			objects[i*map.info.width + j] = distances[i*map.info.width + j]*255;
		}
	cv::imwrite("smooth_map.pgm", image);*/

	this->image = image.clone();
}

void ParticleFilter::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
	odom = *msg;
	geometry_msgs::Quaternion q = odom.pose.pose.orientation;

	current_pose.x = odom.pose.pose.position.x;
	current_pose.y = odom.pose.pose.position.y;
	current_pose.theta = tf::getYaw(q);

	if(!init){
		init = true;
		float theta = tf::getYaw(q);
		init_cloud(particle_cloud, current_pose.x, current_pose.y, theta);
		last_pose = current_pose;
	}
}




//Anderson dev
void ParticleFilter::target_callback(const geometry_msgs::PoseStamped& goal){

	int px, py, ptheta;					//target position
	int cx, cy, ctheta;					//current position
	int smoothing_level = 180;


/*----Variaveis para o potential fields*/

	int area_pixel=AREA_PIXEL; //x cm2 por pixel

	int height_potential_fields = map.info.height/area_pixel; //convertendo para 10 vezes menor
	int width_potential_fields = map.info.width/area_pixel;

	int px_pf, py_pf, ptheta_pf;					//target position
	int cx_pf, cy_pf, ctheta_pf;					//current position

	target = goal;
	geometry_msgs::Quaternion q = target.pose.orientation;

	target_pose.x = target.pose.position.x;
	target_pose.y = target.pose.position.y;
	target_pose.theta = tf::getYaw(q);

	ROS_INFO("new_target = %fx%f %f",target_pose.x, target_pose.y, target_pose.theta);

	px = map.info.width/2 + (int)(target_pose.x*RESOLUTION);
	py = map.info.height/2 + (int)(target_pose.y*RESOLUTION);
	ptheta = (int)(target_pose.theta*RESOLUTION);

	cx = map.info.width/2 + (int)(predicted_pose.x*RESOLUTION);
	cy = map.info.height/2 + (int)(predicted_pose.y*RESOLUTION);
	ctheta = (int)(predicted_pose.theta*RESOLUTION);

	ROS_INFO("atual = %dx%d", cx, cy);
	ROS_INFO("destino = %dx%d", px, py);

	//Criando a imagem
	cv::Mat Source_Target(map.info.height, map.info.width, CV_8UC1, 255); //inicializou com branco

	for (int i = 0; i < map.info.height; i++){
		for (int j = 0; j < map.info.width; j++){
			if (map.data[i*map.info.width + j] == -1){
				Source_Target.at<uchar>(cv::Point(j,i)) = 128; //eh cinza
			}
			if (map.data[i*map.info.width + j] == 100){
				Source_Target.at<uchar>(cv::Point(j,i)) = 0; //0 eh preto
			}

		}
	}




	map_float = (float*)malloc(height_potential_fields*width_potential_fields*sizeof(float));


	cv::Point destiny(px,py);
	cv::circle(Source_Target, destiny, 10, 200, -1);

	cv::Point current(cx,cy);
	cv::circle(Source_Target, current, 20, 1, -1);

//imprimindo a imagem
	//cv::imshow("Source_Target", Source_Target);
	//cv::waitKey(10);


	//versao original do mapa eh 1 cm quadrado por pixel

	cv::Mat potential_fields(height_potential_fields, width_potential_fields, CV_8UC1, 255);

	cv::Mat Source_Target_small(height_potential_fields, width_potential_fields, CV_8UC1, 255);
	cv::resize(Source_Target, Source_Target_small, Source_Target_small.size());

//imprimindo a imagem
	cv::imshow("Source_Target", Source_Target_small);
	cv::waitKey(10);


//Convertendo as cordenadas 
	px_pf=px/area_pixel;
	py_pf=py/area_pixel;
	cx_pf=cx/area_pixel;
	cy_pf=cy/area_pixel;
	//Atribuindo pesos do tipo float para a imagem
	//
	//

	for (int i = 0; i < height_potential_fields; i++){
		for (int j = 0; j < width_potential_fields; j++){

			if (Source_Target_small.at<uchar>(cv::Point(j,i)) == 200){ //destino
				map_float[i*width_potential_fields + j] = 0.0; //0 eh preto
			}
			else if (Source_Target_small.at<uchar>(cv::Point(j,i)) == 128){ //estruturas desconhecidas, interior dos obstaculos
				map_float[i*width_potential_fields + j] = 1.0; 
			}
			else if (Source_Target_small.at<uchar>(cv::Point(j,i)) == 0){ //obstáculos
				map_float[i*width_potential_fields + j] = 1.0; 
			}
			else{
				map_float[i*width_potential_fields + j] = 0.5; // area a ser trafegada

			}

		}
	}

	//suavizacao da imagem
	for (int k = 0; k < smoothing_level; k++){
		for (int i = 1; i < (height_potential_fields-1); i++){
			for (int j = 1; j < (width_potential_fields-1); j++){

				if ((map_float[i*width_potential_fields + j] != 1.0) && (map_float[i*width_potential_fields + j] != 0.0) ){ //se nao for
					map_float[i*width_potential_fields + j] = (map_float[(i+1)*width_potential_fields + j] + 
															   map_float[(i-1)*width_potential_fields + j] + 
															   map_float[i*width_potential_fields + j+1] + 
															   map_float[i*width_potential_fields + j-1]) /4; //Atribuo a media das particulas
				}
			}
		}
	}



	/* Indices dos vetores
	_____________
	|	|	|	|
	__2___4___7__
	|	|	|	|
	__1__XY___6__
	|	|	|	|
	__0___3___5__
	*/
	float vector[8];

	int arrive=0;
	int n=7;

	while(arrive==10000){

		vector[0]=map_float[(cy_pf+1)*width_potential_fields+ cx_pf-1];
		vector[1]=map_float[(cy_pf)*width_potential_fields+ cx_pf-1];
		vector[2]=map_float[(cy_pf-1)*width_potential_fields+ cx_pf-1];
		vector[3]=map_float[(cy_pf+1)*width_potential_fields+ cx_pf];
		vector[4]=map_float[(cy_pf-1)*width_potential_fields+ cx_pf];
		vector[5]=map_float[(cy_pf+1)*width_potential_fields+ cx_pf+1];
		vector[6]=map_float[(cy_pf)*width_potential_fields+ cx_pf+1];
		vector[7]=map_float[(cy_pf-1)*width_potential_fields+ cx_pf+1];

	arrive++;

		if(map_float[cy_pf*width_potential_fields + cx_pf] < -0.5){
			//arrive=1;
			//break;
		}

		else{



			int c = 0, d, trocou = 1;
			float troca;
		    while (c < (n - 1) && trocou)
		    {
		        trocou = 0;
		        for (d = 0; d < n - c - 1; d++){
		           
					if (vector[d] < vector[d + 1])
					{
						troca = vector[d];
						vector[d] = vector[d + 1];
						vector[d + 1] = troca;
						trocou = 1;
					}
					
				}
		        c++;
		    }



			if (vector[0]==map_float[(cy_pf+1)*width_potential_fields+ cx_pf-1]){
				cy_pf++;
				cx_pf--;
				map_float[cy_pf*width_potential_fields+ cx_pf] =0;
			}
			else if (vector[0]==map_float[(cy_pf)*width_potential_fields+ cx_pf-1]){
				cy_pf;
				cx_pf--;
				map_float[cy_pf*width_potential_fields+ cx_pf] =0;
			}
			else if (vector[0]==map_float[(cy_pf-1)*width_potential_fields+ cx_pf-1]){
				cy_pf--;
				cx_pf--;
				map_float[cy_pf*width_potential_fields+ cx_pf] =0;
			}
			else if (vector[0]==map_float[(cy_pf+1)*width_potential_fields+ cx_pf]){
				cy_pf++;
				cx_pf;
				map_float[cy_pf*width_potential_fields+ cx_pf] =0;
			}
			else if (vector[0]==map_float[(cy_pf-1)*width_potential_fields+ cx_pf]){
				cy_pf--;
				cx_pf;
				map_float[cy_pf*width_potential_fields+ cx_pf] =0;
			}
			else if (vector[0]==map_float[(cy_pf+1)*width_potential_fields+ cx_pf+1]){
				cy_pf++;
				cx_pf++;
				map_float[cy_pf*width_potential_fields+ cx_pf] =0;
			}
			else if (vector[0]==map_float[(cy_pf)*width_potential_fields+ cx_pf+1]){
				cy_pf;
				cx_pf++;
				map_float[cy_pf*width_potential_fields+ cx_pf] =0;
			}
			else if (vector[0]==map_float[(cy_pf-1)*width_potential_fields+ cx_pf+1]){
				cy_pf--;
				cx_pf++;
				map_float[cy_pf*width_potential_fields+ cx_pf] =0;
			}

			else {
				// arrive=1;
				// break;
				ROS_INFO("atual y sss %d",cy_pf);
			}
		}

	}



// while=(arrive==0){




// 			if(map_float[cy_pf*width_potential_fields + cx_pf] == -1){
// 					arrive=1;
// 					break;
// 			}

// 			if(map_float[cy_pf*width_potential_fields + cx_pf] > map_float[(cy_pf+1)*width_potential_fields+ cx_pf] ){ //acima
// 				//map_float[cy_pf*width_potential_fields+ cx_pf] = map_float[(cy+1)*map.info.width+ cx];

// 				cy_pf++;
// 				map_float[cy_pf*width_potential_fields + cx_pf] =1.0;
// 			}		


// 			else if(map_float[cy_pf*width_potential_fields + cx_pf] > map_float[cy_pf*width_potential_fields+ cx_pf+1]){ //diteita
// 				//map_float[cy_pf*width_potential_fields+ cx_pf] =map_float[cy*map.info.width + cx+1];
// 				map_float[cy_pf*width_potential_fields+ cx_pf] =1.0;
// 				cx_pf++;
				
// 			}	



// 			else if (map_float[cy_pf*width_potential_fields + cx_pf] > map_float[(cy_pf-1)*width_potential_fields + cx_pf]){ //abaixo
// 				//map_float[cy_pf*width_potential_fields+ cx_pf] = map_float[(cy-1)*map.info.width + cx];
// 				map_float[cy_pf*width_potential_fields+ cx_pf] =1.0;
	
// 				cy_pf--;
// 			}	

// 			else if (map_float[cy_pf*width_potential_fields + cx_pf] > map_float[cy_pf*width_potential_fields + cx_pf-1]){ //esquerda
// 				//map_float[cy_pf*width_potential_fields+ cx_pf] = map_float[cy*map.info.width + cx-1];
// 				map_float[cy_pf*width_potential_fields+ cx_pf] =1.0;
// 				cx_pf--;
				
// 			}	
// 			else{



// 				// if (cx_pf < px_pf ){ //esquerda
// 				// 	map_float[cy_pf*width_potential_fields+ cx_pf]=1.0;
// 				// 	cx--;

				
// 				// }
// 				// else if (cx_pf < px_pf)  //direita
// 				// {
// 				// 	map_float[cy_pf*width_potential_fields+ cx_pf] =1.0;
// 				// 	cx++;

// 				// }

// 				// else
// 				// {
// 				// 	if(cy_pf<py_pf){ //acima
// 				// 		map_float[cy_pf*width_potential_fields+ cx_pf] =1.0;
// 				// 		cy--;
// 				// 	}
// 				// 	else if (cy_pf>py_pf){
// 				// 		map_float[cy_pf*width_potential_fields+ cx_pf] =1.0;
// 				// 		cy++;
// 				// 	}
// 				// 	else{
// 				// 		arrive=1;
// 				// 		break;
// 				// 	}

// 				// }
// 			arrive=1;
// 					break;
		


// 				ROS_INFO("atual x sdsss%d",cx_pf);
// 				ROS_INFO("atual y sss %d",cy_pf);
// 			}
	//}



	for (int i = 0; i < height_potential_fields; i++){
		for (int j = 0; j < width_potential_fields; j++){			
			potential_fields.at<uchar>(cv::Point(j,i)) =255*(map_float[i*width_potential_fields+ j]); //Normalizando
		}
	}


	cv::imshow("potential_fields", potential_fields);
	cv::waitKey(3);

	moving = true;
}

float gradient_x(float matrix[3][3]){
	float kernel[3][3] = {{-1, 0, 1}, {-1, 0, 1}, {-1, 0, 1}};
	float acc = 0;

	for(int y = 0; y < 3; y++)
		for(int x = 0; x < 3; x++)
			acc +=  (kernel[y][x] * matrix[y][x]);

	return acc;
}

float gradient_y(float matrix[3][3]){
	float kernel[3][3] = {{-1, -1, -1}, {0, 0, 0}, {1, 1, 1}};
	float acc = 0;

	for(int y = 0; y < 3; y++)
		for(int x = 0; x < 3; x++)
			acc +=  (kernel[y][x] * matrix[y][x]);

	return acc;
}

void ParticleFilter::move_bot(float* map_float){
	int height = map.info.height/AREA_PIXEL; //convertendo para 10 vezes menor
	int width = map.info.width/AREA_PIXEL;

	float vector[8];
	float matrix[3][3];
	float desired_theta, theta_error, current_theta;

	//while(true){

	int px = width/2  + (int)(predicted_pose.x*RESOLUTION/AREA_PIXEL);
	int py = height/2 + (int)(predicted_pose.y*RESOLUTION/AREA_PIXEL);

	int tx = width/2 + (int)(target_pose.x*RESOLUTION/AREA_PIXEL);
	int ty = height/2 + (int)(target_pose.y*RESOLUTION/AREA_PIXEL);


	for(int y = 0; y < 3; y++)
		for(int x = 0; x < 3; x++)
			matrix[y][x] = map_float[(py-1+y)*width  + px-1+x];

	for(int y = 0; y < 3; y++){
		for(int x = 0; x < 3; x++)
			cout << matrix[y][x] << " ";
		cout << endl;
	}

	float gx = -gradient_x(matrix);
	float gy = -gradient_y(matrix);

	cout << gx << " " << gy << " = " << atan2(gy, gx) << endl;


	/*vector[0]=map_float[(py)*width + px+1];		//0 graus, 0 rad
	vector[1]=map_float[(py-1)*width + px+1];	//45,	M_PI/4
	vector[2]=map_float[(py-1)*width + px];		//90,	M_PI/2
	vector[3]=map_float[(py-1)*width + px-1];	//135,	3*M_PI/4
	vector[4]=map_float[(py)*width + px-1];		//180,	M_PÌ
	vector[5]=map_float[(py+1)*width + px-1];	//225,	-3*M_PÌ/4
	vector[6]=map_float[(py+1)*width + px];		//270,	-M_PI/2
	vector[7]=map_float[(py+1)*width + px+1];	//315, 	-M_PI/4

	int lowest_index = 0;
	for (int i = 1; i < 8; i++)
		if (vector[i] < vector[lowest_index])
			lowest_index = i;*/

	ROS_INFO("current field position %dx%d", px, py);
	ROS_INFO("target field position %dx%d", tx, ty);

	desired_theta = atan2(gy, gx);
	current_theta = predicted_pose.theta;

	/*if (desired_theta < 0)
		desired_theta = desired_theta + 2*M_PI;*/

	/*if (current_theta < 0)
		current_theta = current_theta + 2*M_PI;*/

	theta_error = desired_theta - current_theta;

	//ROS_INFO("gradients %f %f %f %f %f %f %f %f", vector[0], vector[1], vector[2], vector[3], vector[4], vector[5], vector[6], vector[7]);
	ROS_INFO("desired_theta %f, current_theta %f, theta_error %f", desired_theta, current_theta, theta_error);

	twist.linear.x = 0.0;
	twist.angular.z = theta_error;

	if (theta_error < 0.5 && theta_error > -0.5){
		twist.linear.x = 0.3;
		twist.angular.z = theta_error;
	}
	cmd_vel_pub.publish(twist);

	if(tx == px && ty == py)
		moving = false; //condição de parada

	/*cv::Point curr_pos(px,py);
	cv::circle(potential_fields, curr_pos, 2, 1, -1);

	cv::imshow("potential_fields", potential_fields);
	cv::waitKey(3);*/
	//}

}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "particle");

	ParticleFilter pf;

	cv::namedWindow("view");
	cv::startWindowThread();

	cv::namedWindow("Source_Target");
	cv::startWindowThread();

	cv::namedWindow("potential_fields");
	cv::startWindowThread();

	ros::spin();

	return 0;
}

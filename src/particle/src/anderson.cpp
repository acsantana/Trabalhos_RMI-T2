#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include "tf/transform_datatypes.h"


#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>



#define PARTICLE_NUM	100		//numero de particulas parametrizavel
#define MAX_DEV			200		//janela de distribuicao das amostras
#define MAX_DEV_T		100		//Desvio maximo de theta
#define RESOLUTION		100		//100 pixels por metro
#define MEASURES 		640		//amostras do laser
#define NOISE			10		//Noise added on copied particles


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
		p[i].w = 1.0;
		p[i].x = x + float(rand()%MAX_DEV*2 - MAX_DEV)/RESOLUTION;
		p[i].y = y + float(rand()%MAX_DEV*2 - MAX_DEV)/RESOLUTION;
		p[i].theta = theta + float(rand()%MAX_DEV_T*2 - MAX_DEV_T)/RESOLUTION;

		if(p[i].theta > M_PI) p[i].theta = p[i].theta - 2*M_PI;
		if(p[i].theta < -M_PI) p[i].theta = p[i].theta + 2*M_PI;
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

void sensor_update(struct Particle *p, struct SensorData *sensor_data, struct Position current_xytheta, char *map, int w, int h){
	int i, j;

	int sx, sy;		//sensor coordinates
	int px, py;		//particle coordinates

	float angle;

	int d;

	for (i = 0; i < PARTICLE_NUM; i++){
		
		px = current_xytheta.x + p[i].x*RESOLUTION;
		py = current_xytheta.y + p[i].y*RESOLUTION;

		d = 0;
		for (j = 0; j < MEASURES; j++){
			if (sensor_data[j].distance != 0){
				sx = px + (int)(cos(p[i].theta + sensor_data[j].angle)*sensor_data[j].distance*RESOLUTION);
				sy = py + (int)(sin(p[i].theta + sensor_data[j].angle)*sensor_data[j].distance*RESOLUTION);
				if (check_bounds(sx, sy, w, h))
					d += (uchar)map[sy*w + sx];
			}
		}
		p[i].w = d;
	}
}

void smooth(float *distances, int h, int w){ //pega os vizinhos e suaviliza
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

	for(i = 0; i < PARTICLE_NUM; i++) p[i].w = 1/p[i].w;
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
		index = (float)(rand()%100)/100;
		n = 0;
		while(index > cumulative_sum[n])
			n++;
		np[i].x = p[n].x + float(rand()%NOISE*2 - NOISE)/RESOLUTION;
		np[i].y = p[n].y + float(rand()%NOISE*2 - NOISE)/RESOLUTION;
		np[i].theta = p[n].theta + float(rand()%NOISE*2 - NOISE)/RESOLUTION;
		np[i].w = 1.0;
	}
}

void copy_particles(struct Particle *p, struct Particle *np){
	int i;

	for(i = 0; i < PARTICLE_NUM; i++){
		p[i].x = np[i].x;
		p[i].y = np[i].y;
		p[i].theta = np[i].theta;
		p[i].w = np[i].w;
	}
}






class ParticleFilter
{
private:
	ros::NodeHandle n;
	ros::Subscriber target_sub;
	ros::Subscriber map_sub;
	ros::Subscriber sensor_sub;
	ros::Subscriber odom_sub;

	sensor_msgs::LaserScan laser;
	nav_msgs::OccupancyGrid map;
	nav_msgs::Odometry odom;

	geometry_msgs::PoseStamped target;

	cv::Mat image;
	cv::Mat Source_Target;
	cv::Mat Source_Target_small;
	cv::Mat potential_fields;

	Particle particle_cloud[PARTICLE_NUM];
	SensorData sensor_measures[MEASURES];
	Odometry current_pose, last_pose;
	
	Odometry target_pose;
	char *objects;

	bool init = false;
	int best_particle;

public:
	ParticleFilter();
	void target_callback(const geometry_msgs::PoseStamped& goal);
	void sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	
};

ParticleFilter::ParticleFilter() {
	target_sub = n.subscribe("move_base_simple/goal", 5, &ParticleFilter::target_callback, this);
	sensor_sub = n.subscribe("scan", 5, &ParticleFilter::sensor_callback, this);
	map_sub = n.subscribe("map", 1, &ParticleFilter::map_callback, this);
	odom_sub = n.subscribe("odom", 5, &ParticleFilter::odom_callback, this);

}

void ParticleFilter::sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg){

	cv::Mat image;

	int px, py, ptheta;					//pixel position
	int last_px, last_py, last_ptheta;

	float lx, ly, angle;				//laser variables

	float best_val;

	Position current_xytheta;
	Particle new_cloud[PARTICLE_NUM];

	px = map.info.width/2 + (int)(current_pose.x*RESOLUTION);
	py = map.info.height/2 + (int)(current_pose.y*RESOLUTION);
	ptheta = (int)(current_pose.theta*RESOLUTION);

	last_px = map.info.width/2 + (int)(last_pose.x*RESOLUTION);
	last_py = map.info.height/2 + (int)(last_pose.y*RESOLUTION);
	last_ptheta = (int)(last_pose.theta*RESOLUTION);

	current_xytheta.x = px;
	current_xytheta.y = py;
	current_xytheta.theta = current_pose.theta;

	cv::cvtColor(this->image, image, CV_GRAY2BGR);

	int i = 0;
	for (angle = msg->angle_min; angle < msg->angle_max; angle += msg->angle_increment){
		if (msg->ranges[i] == msg->ranges[i]){
			lx = cos(angle + current_pose.theta)*msg->ranges[i];
			ly = sin(angle + current_pose.theta)*msg->ranges[i];

			sensor_measures[i].angle = angle;
			sensor_measures[i].distance = msg->ranges[i];

			cv::Point mc((int)(lx*RESOLUTION) + px, (int)(ly*RESOLUTION) + py);
			cv::circle(image, mc, 3, cv::Scalar(0,0,255), -1);
		}
		else {
			sensor_measures[i].angle = 0;
			sensor_measures[i].distance = 0;			
		}
		i++;
	}

	/* desenha laser para uma particula */
	for (i = 0; i < MEASURES; i++){
		if (sensor_measures[i].distance != 0){
			lx = cos(particle_cloud[best_particle].theta + sensor_measures[i].angle)*sensor_measures[i].distance;
			ly = sin(particle_cloud[best_particle].theta + sensor_measures[i].angle)*sensor_measures[i].distance;
			cv::Point mc((int)(lx*RESOLUTION) + (int)(particle_cloud[best_particle].x*RESOLUTION) + px, (int)(ly*RESOLUTION) + (int)(particle_cloud[best_particle].y*RESOLUTION) + py);
			cv::circle(image, mc, 3, cv::Scalar(0,0,0), -1);
		}
		//cv::circle(image, laser_part[i], 3, cv::Scalar(0,0,0), -1);
	}

	/* verifica movimento */
	if (px != last_px || py != last_py || ptheta != last_ptheta){
		motion_update(particle_cloud, current_pose, last_pose);
		sensor_update(particle_cloud, sensor_measures, current_xytheta, objects, map.info.width, map.info.height);
		normalize_weights(particle_cloud);
		filter(particle_cloud, new_cloud);
		last_pose = current_pose;
		
		best_val = particle_cloud[0].w;
		best_particle = 0;
		for (i = 0; i < PARTICLE_NUM; i++){
			if (particle_cloud[i].w > best_val){
				best_val = particle_cloud[i].w;
				best_particle = i;
			}
		}
		ROS_INFO("best_particle %d", best_particle);
		copy_particles(particle_cloud, new_cloud);
	}

	/* Desenha Particulas */
	for (i = 0; i < PARTICLE_NUM; i++){
		int particle_x = map.info.width/2 + (int)(particle_cloud[i].x*RESOLUTION);
		int particle_y = map.info.height/2 + (int)(particle_cloud[i].y*RESOLUTION);
		cv::Point p1(particle_x, particle_y);
		cv::Point p2(particle_x+10*cos(particle_cloud[i].theta), particle_y+10*sin(particle_cloud[i].theta));
		uchar pcollor = (int)(particle_cloud[i].w*2555);
		cv::arrowedLine(image, p1, p2, cv::Scalar(255,255-pcollor,255-pcollor), 2, 8, 0, 0.5);
		if (i == best_particle) cv::arrowedLine(image, p1, p2, cv::Scalar(0,255,0), 2, 8, 0, 0.5);
	}

	/* Desenha posicao de odometria */
	cv::Point p1(px, py);
	cv::Point p2(px+10*cos(current_pose.theta), py+10*sin(current_pose.theta));
	cv::arrowedLine(image, p1, p2, cv::Scalar(0,0,255), 2, 8, 0, 0.5);

	cv::Mat image_small(map.info.height/2, map.info.width/2, CV_8UC3, 255);
	cv::resize(image, image_small, image_small.size());

	cv::imshow("view", image_small);
	cv::waitKey(3);

}

void ParticleFilter::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

	ROS_INFO("received map: %d x %d", msg->info.width, msg->info.height);
	map = *msg;

	cv::Mat image(map.info.height, map.info.width, CV_8UC1, 255);
	cv::Mat smooth_map = cv::imread("smooth_map.pgm", 1);

	objects = (char*)malloc(map.info.height * map.info.width);

	for (int i = 0; i < map.info.height; i++){
		for (int j = 0; j < map.info.width; j++){
			objects[i*map.info.width + j] = smooth_map.at<uchar>(cv::Point(j,i));
			if (map.data[i*map.info.width + j] == -1)
				image.at<uchar>(cv::Point(j,i)) = 128;
			if (map.data[i*map.info.width + j] == 100)
				image.at<uchar>(cv::Point(j,i)) = 0;
		}
	}






	/* Procedimentos para gerar mapa de distancias (lento)*/
	// float *distances = (float*)malloc(map.info.height * map.info.width*sizeof(float));
	// for (int i = 0; i < map.info.height; i++){
	// 	for (int j = 0; j < map.info.width; j++){
	// 		if (map.data[i*map.info.width + j] == 100)
	// 			distances[i*map.info.width + j] = 0.0;
	// 		else
	// 			distances[i*map.info.width + j] = 1.0;
	// 	}
	// }

	// for (int i = 0; i < 1700; i++) //parametro de 
	// 	smooth(distances, map.info.height, map.info.width);

	// for (int i = 0; i < map.info.height; i++)
	// 	for (int j = 0; j < map.info.width; j++)
	// 		image.at<uchar>(cv::Point(j,i)) = distances[i*map.info.width + j]*255;
	// cv::imwrite("smooth_map.pgm", image);

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
	int smoothing_level = 3000;


/*----Variaveis para o potential fields*/

	int area_pixel=10; //x cm2 por pixel

	int height_potential_fields = map.info.height/area_pixel; //convertendo para 10 vezes menor
	int width_potential_fields = map.info.width/area_pixel;

	int px_pf, py_pf, ptheta_pf;					//target position
	int cx_pf, cy_pf, ctheta_pf;					//current position



	target = goal;
	geometry_msgs::Quaternion q = target.pose.orientation;

	target_pose.x = target.pose.position.x;
	target_pose.y = target.pose.position.y;
	target_pose.theta = tf::getYaw(q);

	ROS_INFO("new_target x %f",target_pose.x);
	ROS_INFO("new_target y %f",target_pose.y);
	ROS_INFO("new_target theta %f",target_pose.theta);
	ROS_INFO("new_target");


	px = map.info.width/2 + (int)(target_pose.x*RESOLUTION);
	py = map.info.height/2 + (int)(target_pose.y*RESOLUTION);
	ptheta = (int)(target_pose.theta*RESOLUTION);

	cx = map.info.width/2 + (int)(current_pose.x*RESOLUTION);
	cy = map.info.height/2 + (int)(current_pose.y*RESOLUTION);
	ctheta = (int)(current_pose.theta*RESOLUTION);

	ROS_INFO("atual x %d",cx);
	ROS_INFO("atual y %d",cy);
	ROS_INFO("destino x %d",px);
	ROS_INFO("destino y %d",py);
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




	float *map_float = (float*)malloc(height_potential_fields*width_potential_fields*sizeof(float));


	cv::Point destiny(px,py);
	cv::circle(Source_Target, destiny, 10, 200, -1);

	cv::Point current(cx,cy);
	cv::circle(Source_Target, current, 20, 0, -1);

//imprimindo a imagem
	cv::imshow("Source_Target", Source_Target);
	cv::waitKey(10);


	//versao original do mapa eh 1 cm quadrado por pixel

	cv::Mat potential_fields(height_potential_fields, width_potential_fields, CV_8UC1, 255);

	cv::Mat Source_Target_small(height_potential_fields, width_potential_fields, CV_8UC1, 255);
	cv::resize(Source_Target,Source_Target_small, Source_Target_small.size());

 

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
			if (Source_Target_small.at<uchar>(cv::Point(j,i)) == 0){ //origem
				map_float[i*width_potential_fields + j] = 0.0; //eh branco
			}
			else if (Source_Target_small.at<uchar>(cv::Point(j,i)) == 200){ //destino
				map_float[i*width_potential_fields + j] = -1.0; //0 eh preto
			}
			else if (Source_Target_small.at<uchar>(cv::Point(j,i)) == 128){ //estruturas desconhecidas, interior dos obstaculos
				map_float[i*width_potential_fields + j] = 1.0; 
			}
			else{
				map_float[i*width_potential_fields + j] = 0; // area a ser trafegada

			}

		}
	}



	//suavizacao da imagem
	for (int k = 0; k < smoothing_level; k++){
		for (int i = 1; i < (height_potential_fields-1); i++){
			for (int j = 1; j < (width_potential_fields-1); j++){

				if ((map_float[i*width_potential_fields + j] != 1.0) && (map_float[i*width_potential_fields + j] != -1.0) ){ //se nao for
					map_float[i*width_potential_fields + j] = (map_float[(i+1)*width_potential_fields + j]+ map_float[(i-1)*width_potential_fields + j] + map_float[i*width_potential_fields + j+1]+ map_float[i*width_potential_fields + j-1]) /4; //Atribuo a media das particulas
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
float vector[7];

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
			potential_fields.at<uchar>(cv::Point(j,i)) = 127*(map_float[i*width_potential_fields+ j]+1); //Normalizando
		}
	}


	cv::imshow("potential_fields", potential_fields);
	cv::waitKey(10);


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


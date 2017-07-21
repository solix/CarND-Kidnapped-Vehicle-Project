/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	default_random_engine gen;
	double std_x, std_y, std_psi; // Standard deviations for x, y, and psi

	// Set standard deviations for x, y, and psi.
	 num_particles = 100;
	

	 	
	// normal distributions for x, y and psi
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_psi(theta, std[2]);


	for (int i = 0; i < num_particles; ++i) {
		double sample_x, sample_y, sample_psi;
		
		//Sample  and from these normal distrubtions like this: 
		Particle p;
		p.id =i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_psi(gen);
		p.weight = 1;
		particles.push_back(p);
		weights.push_back(1);
		// where "gen" is the random engine initialized earlier .
		// cout<<particles[i]>>endl;

	}
		is_initialized = true;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;

	for(int i = 0; i < num_particles; i++){
		
		//initialize new variables for new state t	
		double x_f ;
		double y_f ;
		double yaw_rate_f ;

		//calculate new distance according to formula given in the literature

		if(yaw_rate == 0){

			x_f = 	particles[i].x + velocity*delta_t*cos(particles[i].theta);
			y_f =  particles[i].y + velocity*delta_t*sin(particles[i].theta);
			yaw_rate_f = particles[i].theta;	

		}else{

			x_f = 	particles[i].x + velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
			y_f =  particles[i].y + velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t));
			yaw_rate_f = particles[i].theta + yaw_rate*delta_t;	

		}


		// normal distributions for x, y and psi
	normal_distribution<double> dist_x_f(x_f, std_pos[0]);
	normal_distribution<double> dist_y_f(y_f, std_pos[1]);
	normal_distribution<double> dist_yawrate_f(yaw_rate_f, std_pos[2]);

	particles[i].x = dist_x_f(gen);
	particles[i].y = dist_y_f(gen);
	particles[i].theta = dist_yawrate_f(gen);




	}





}


/*
We will first need to transform the car's measurements from its local car coordinate system to the map's
 coordinate system. Next each measurement will need to be associated with a landmark identifier, for doing this 
 part we will simply take the closest landmark to each transformed observation. Finally we will then have everything 
 we need to calculate the particles weight value.
*/

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


		LandmarkObs landmark_obj;

		for(int i = 0; i < observations.size(); i++){
			landmark_obj = observations[i];

			//minimum distance		
			double min_dis = std::numeric_limits<double>::max();

			//calculate distance for every landmark
			for(int j = 0 ; j < predicted.size(); j++  ){


				//create variables to hold current input values 
				double current_dist;
				double meas_x = landmark_obj.x;
				double meas_y = landmark_obj.y;
				double mu_x = predicted[j].x;
				double mu_y = predicted[j].y;
				int id = predicted[j].id;
				current_dist = dist(meas_x,meas_y,mu_x,mu_y);

				if(current_dist < min_dis){
					min_dis = current_dist;
					landmark_obj.id = id;

				}
				landmark_obj.id = id;



			}





		}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	for(int p = 0; p < num_particles; p++){

		vector<int> associations;
		vector<double> sense_x;
		std::vector<double> sense_y;
		double p_x = particles[p].x;
		double p_y = particles[p].y;
		double p_theta = particles[p].theta;
		int p_id = particles[p].id;
		double p_weight = particles[p].weight;

		//filter landmarks
		//ditch the too far landmarks

		vector<LandmarkObs> predicted;

		for(int j =0; j < map_landmarks.landmark_list.size(); j++)
		{
			
			double mu_x = map_landmarks.landmark_list[j].x_f;
			double mu_y = map_landmarks.landmark_list[j].y_f;
			int id = map_landmarks.landmark_list[j].id_i;

			if(fabs(p_x - mu_x) <= sensor_range  && fabs(p_y - mu_y) <= sensor_range){
				LandmarkObs landmark;
				landmark.x = mu_x;
				landmark.y = mu_y;
				landmark.id = id;
				predicted.push_back(landmark);

			}

	   }

		//transform  
		std::vector<LandmarkObs> obs_transform2d;
		LandmarkObs landmark_obj;

		for(int i=0;i<observations.size();i++){
			LandmarkObs landmark_obj_trans;
			landmark_obj = observations[i];

			landmark_obj_trans.x = particles[p].x+(landmark_obj.x*cos(particles[p].theta)-landmark_obj.y*sin(particles[p].theta));
			landmark_obj_trans.y = particles[p].y+(landmark_obj.x*sin(particles[p].theta)+landmark_obj.y*cos(particles[p].theta));

			obs_transform2d.push_back(landmark_obj_trans);

		}


		dataAssociation(predicted,obs_transform2d);

		particles[p].weight = 1.0;
		double x,y,x_mu,y_mu;
		int landmark_id , map_id;

		for(int t = 0; t < obs_transform2d.size(); t++){

			x = obs_transform2d[t].x;
			y = obs_transform2d[t].y;
			landmark_id = obs_transform2d[t].id;

			for(int j = 0 ; j < predicted.size(); j++  ){
				map_id = predicted[j].id;
				if(landmark_id == map_id){
					x_mu = predicted[j].x;
					y_mu = predicted[j].y;
				}
		}
	
		double x_xmu_pow = pow(x-x_mu,2); 
		double y_ymu_pow = pow(y-y_mu,2);
		double std_x = std_landmark[0];
		double std_y = std_landmark[1];
		double std_x_pow = pow(std_x,2);
		double std_y_pow = pow(std_y,2);
		double normalizer = 1/(2*M_PI*std_x*std_y);
		double e_num = exp(-1*((x_xmu_pow/(2*std_x_pow))+(y_ymu_pow/(2*std_y_pow)))); 

		//weight for particular measurement
		double gauss_pr = normalizer*e_num;

		if(gauss_pr>0)
		{
		p_weight *= gauss_pr;
		}
		sense_x.push_back(x);
		sense_y.push_back(y);
		associations.push_back(t+1);
	}


		particles[p]= SetAssociations(particles[p],associations,sense_x,sense_y);

	
	}

}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;
	discrete_distribution<int> dst_weights(weights.begin(),weights.end());

	vector<Particle> particles_resampled;

	for(int i = 0; i < num_particles; ++i){

		particles_resampled.push_back(particles[dst_weights(gen)]);
	}


}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

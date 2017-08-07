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
	 num_particles = 50;
	

	 	
	// normal distributions for x, y and psi
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_psi(theta, std[2]);


	for (int i = 0; i < num_particles; i++) {

		//Sample  and from these normal distrubtions like this: 
		Particle p;
		p.id =i+1;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_psi(gen);
		p.weight = 1;
		weights.push_back(1.0);

		particles.push_back(p);
		// where "gen" is the random engine initialized earlier .

	}
		is_initialized = true;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	
	// default_random_engine gen;
	// normal_distribution<double> dist_x_f(0, std_pos[0]);
	// normal_distribution<double> dist_y_f(0 , std_pos[1]);
	// normal_distribution<double> dist_yawrate_f(0, std_pos[2]);

	for(int i = 0; i < num_particles; i++){

        
		if(yaw_rate == 0){

			particles[i].x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
			particles[i].y =  particles[i].y + velocity*delta_t*sin(particles[i].theta);

		}else{

			particles[i].x = 	particles[i].x + velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
			particles[i].y =  particles[i].y + velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t));
			particles[i].theta = particles[i].theta + yaw_rate*delta_t;	

		}

	// 	// normal distributions for x, y and psi
	//    particles[i].x = dist_x_f(gen);
	//    particles[i].y = dist_y_f(gen);
	//    particles[i].theta += dist_yawrate_f(gen);




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



		for(int i = 0; i < observations.size(); i++){
			LandmarkObs obs;
			obs = observations[i];

			//minimum distance		
			double min_dis = std::numeric_limits<double>::max();
							//create variables to hold current input values 
			double current_dist;
			int closest_id =-1;	
			//calculate distance for every landmark
			for(int j = 0 ; j < predicted.size(); j++  ){



				double meas_x = obs.x;
				double meas_y = obs.y;
				double mu_x = predicted[j].x;
				double mu_y = predicted[j].y;

				//calculate current distance of landmark in map 
				current_dist = dist(meas_x,meas_y,mu_x,mu_y);

				if(current_dist < min_dis){
					min_dis = current_dist;
					closest_id = j;
                }


			}

			observations[i].id = closest_id;



		}

}
/**
	Calculate weights for each observation
**/
double Multivariate_Gaussian_Probability(double x,double y, double x_mu, double y_mu,double std_x, double std_y){
		
		double dx= x-x_mu;
		double dy= y-y_mu;
		double x_xmu_pow = pow(dx,2); 
		double y_ymu_pow = pow(dy,2);
		double std_x_pow = pow(std_x,2);
		double std_y_pow = pow(std_y,2);
		double normalizer = 2.0*M_PI*std_x*std_y;
		double e_num = exp(-(x_xmu_pow/(2*std_x_pow) + y_ymu_pow/(2*std_y_pow))); 

		//weight for particular measurement
		double gauss_pr = e_num/normalizer;

		return gauss_pr;
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

			if(dist(p_x,p_y,mu_x,mu_y) <= pow(sensor_range,2)){
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
			landmark_obj_trans.id = i;

			obs_transform2d.push_back(landmark_obj_trans);
	   
		}

		//ASSOCIATION

		dataAssociation(predicted,obs_transform2d);

		double probability = 1.0f;
		
		//weight calculation
		for(int t = 0; t < obs_transform2d.size(); t++){	
		   LandmarkObs obs = obs_transform2d[t];
		   LandmarkObs predicted_landmark = predicted[obs.id];
		   associations.push_back(predicted_landmark.id);
		   sense_x.push_back(predicted_landmark.x);
		   sense_y.push_back(predicted_landmark.y);
		   double weight = Multivariate_Gaussian_Probability(obs.x,obs.y,predicted_landmark.x,predicted_landmark.y,std_landmark[0],std_landmark[1]);
		   probability *= weight;				

		    }

		    particles[p].weight = probability;
		    weights[p] = probability;
		    particles[p] = SetAssociations(particles[p],associations,sense_x,sense_y);

		    // cout<<"paticle probability"<<probability<<endl;
		

	
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

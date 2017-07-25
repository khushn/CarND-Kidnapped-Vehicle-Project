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
	

	num_particles=1000;

	// This (Gaussian) distributions for x, y and theta.
	default_random_engine gen;	
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for(int i=0; i<num_particles; i++){
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1./ num_particles;
		particles.push_back(p);
	}

	is_initialized = true;
	cout << "Init done." << endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	// This (Gaussian) distributions for x, y and theta.
	// We assume that creating it around 0, and then adding, 
	// will have the same effect as adding around the exact mean
	cout << "inside prediction()." << endl;
	default_random_engine gen;	
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for(int i=0; i<num_particles; i++){
		Particle & p = particles[i];
		p.x += (velocity/yaw_rate)*(sin(p.theta+yaw_rate*delta_t)-sin(p.theta)) + dist_x(gen);
		p.y += (velocity/yaw_rate)*(cos(p.theta) - cos(p.theta+yaw_rate*delta_t)) + dist_y(gen);
		p.theta += yaw_rate*delta_t + dist_theta(gen);
	}
	cout << "prediction done." << endl;
}

Map::single_landmark_s ParticleFilter::dataAssociation(Map & map_landmarks, LandmarkObs& lobs) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
		
	double min_dist=-1.0;
	int sel_id=-1;
	Map::single_landmark_s ret;
	for(int i=0; i<map_landmarks.landmark_list.size(); i++) {
		Map::single_landmark_s &sl = map_landmarks.landmark_list[i];
		double dx = sl.x_f - lobs.x;
		double dy = sl.y_f = lobs.y;
		double dist = sqrt(dx*dx+dy*dy);
		if (min_dist == -1 || min_dist > dist) {
			min_dist = dist;
			sel_id = sl.id_i;
			ret = sl;
		}
	}
	lobs.id = sel_id;
	return ret;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map & map_landmarks) {
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
	cout << "inside updateWeights()." << endl;
	double sigma_x = std_landmark[0];
	double sigma_y = std_landmark[1];
	
	for(int i=0; i<num_particles; i++){
		Particle & p = particles[i];
		double weight=1.0;
		// For each particle, we do the following steps

		// Step 1: transform the particle co-ordinates to map co-ordinates
		for(int j=0; j<observations.size(); j++){
			LandmarkObs &lobs = observations.at(j);
			double tx = p.x + lobs.x*cos(p.theta) - lobs.y*sin(p.theta);
			double ty = p.y + lobs.x*sin(p.theta) + lobs.y*cos(p.theta);
			lobs.x = tx;
			lobs.y = ty;

			// Step 2: Assign the nearest landmark ids
			Map::single_landmark_s assoc_lm = dataAssociation(map_landmarks, lobs);
			//cout << "associated lm: " << assoc_lm.id_i << endl;

			// Step 3: Calculate mult-variate gaussian probabilities for associated landmark
			// to have this measurement
			double dx = assoc_lm.x_f - tx;
			double dy = assoc_lm.y_f -ty;
			double prob = exp(-(dx*dx/(2*sigma_x*sigma_x) + dy*dy/(2*sigma_y*sigma_y))) / (2*M_PI*sigma_x*sigma_y);
			//out << "prob: " << prob << endl;
			weight*=prob;
		}
		p.weight = weight;
		//cout << "probability weight of particle id: "<< p.id << " is: " << p.weight << endl;
	}

	cout << "updateWeights() done." << endl;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	cout << "inside resample()." << endl;
	std::vector<Particle> new_particles;
	//get a random index into the particles
	int index = rand() % num_particles + 1;
	double beta = 0.0;
	double mw = getMaxWeight();
	cout << "max weight of the particles: " << mw << endl;
	for(int i=0; i<num_particles; i++){
		beta += ((double) rand() / (RAND_MAX)) * 2*mw;
		while (beta > particles[index].weight) {
			beta -= particles[index].weight;
			index = (index+1)%num_particles;
			//cout << "beta: " << beta << ", weight at index: " << particles[index].weight << endl;
		}
		new_particles.push_back(particles[index]);
	}
	particles = new_particles;
	cout << "resample() done." << endl;

}

double ParticleFilter::getMaxWeight() {
	double max_weight = -1.0;
	for(int i=0; i<num_particles; i++){
		Particle & p = particles[i];
		if (p.weight > max_weight) {
			max_weight = p.weight;
		}
	}
	return max_weight;
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

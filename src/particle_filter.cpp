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
#include <unordered_set>
#include <map>

#include "particle_filter.h"

// macro for distance
#define DIST(x1, y1, x2, y2) sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

using namespace std;

double normalize_angle(double theta) {
  //angle normalization
    while (theta> M_PI) theta-=2.*M_PI;
    while (theta<-M_PI) theta+=2.*M_PI;
    return theta;
}

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
		if (yaw_rate!=0.) {
			p.x += (velocity/yaw_rate)*(sin(p.theta+yaw_rate*delta_t)-sin(p.theta)) + dist_x(gen);
			p.y += (velocity/yaw_rate)*(cos(p.theta) - cos(p.theta+yaw_rate*delta_t)) + dist_y(gen);
		} else {
			p.x += velocity*cos(p.theta);
			p.y += velocity*sin(p.theta);
		}
		p.theta += yaw_rate*delta_t + dist_theta(gen);
		p.theta = normalize_angle(p.theta);
	}
	cout << "prediction done." << endl;
}

std::vector<LandmarkObs> ParticleFilter::dataAssociation(double sensor_range, 
	Map & map_landmarks, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	map<int, LandmarkObs> obs_map;
	
	
	for(int i=0; i<observations.size(); i++) {
		LandmarkObs lobs = observations[i];
		double min_dist=-1.0;
		int sel_id=-1;
		Map::single_landmark_s sel_lm;
		for(map<int, Map::single_landmark_s>::iterator it = map_landmarks.landmark_map.begin(); it != map_landmarks.landmark_map.end(); ++it) {
			Map::single_landmark_s sl = it->second;
			double dx = sl.x_f - lobs.x;
			double dy = sl.y_f - lobs.y;
			// double dist = sqrt(dx*dx+dy*dy);
			double dist = DIST(lobs.x, lobs.y, sl.x_f, sl.y_f);
			if (dist <= sensor_range && (min_dist == -1 || min_dist > dist) ) {
				min_dist = dist;
				sel_id = sl.id_i;
				sel_lm = sl;			
			}
		}

		if (sel_id != -1) {
			map<int, LandmarkObs>::iterator it = obs_map.find(sel_id);
			if ( it == obs_map.end()) {
				lobs.id = sel_id;
				obs_map[sel_id] = lobs;
			} else {
				// we need to retain the closest one of the two
				LandmarkObs old_obs = it->second;
				double old_dist = DIST(old_obs.x, old_obs.y, sel_lm.x_f, sel_lm.y_f);
				double dist = DIST(lobs.x, lobs.y, sel_lm.x_f, sel_lm.y_f);
				if (dist < old_dist) {
					// replace only if new dist is less than the old one
					lobs.id = sel_id;
					obs_map[sel_id] = lobs;
				}
			}
		}
	}

	std::vector<LandmarkObs> good_obs;
	for( map<int, LandmarkObs>::iterator it = obs_map.begin(); it != obs_map.end(); ++it ) {
        good_obs.push_back( it->second );
    }
	return good_obs;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> & observations, Map & map_landmarks) {
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

	double total_weights=0.0;
	
	for(int i=0; i<num_particles; i++){
		Particle & p = particles.at(i);
		double weight=1.0;
		// For each particle, we do the following steps

		// Step 1: transform the particle co-ordinates to map co-ordinates
		vector<int> associations;
		vector<double> sense_x;
		vector<double> sense_y;
		for(int j=0; j<observations.size(); j++){
			LandmarkObs &lobs = observations.at(j);			
			double r = sqrt(lobs.x*lobs.x+lobs.y*lobs.y);
			double theta2 = atan2(lobs.y, lobs.x);
			double sum_theta = normalize_angle(p.theta + theta2);
			double tx = p.x + r*cos(sum_theta);
			double ty = p.y + r*sin(sum_theta);
			lobs.x = tx;
			lobs.y = ty;
		}

		// Step 2: Assign the nearest landmark ids
		std::vector<LandmarkObs> good_obs = dataAssociation(sensor_range, map_landmarks, observations);

			//cout << "associated lm: " << assoc_lm.id_i << endl;

		// Step 3: Calculate mult-variate gaussian probabilities for associated landmark
		// to have this measurement
		for(int j=0; j<good_obs.size(); j++){
			LandmarkObs lobs = good_obs[j];	
			Map::single_landmark_s assoc_lm = map_landmarks.landmark_map[lobs.id];
			double dx = assoc_lm.x_f - lobs.x;
			double dy = assoc_lm.y_f - lobs.y;
			double prob = exp(-(dx*dx/(2*sigma_x*sigma_x) + dy*dy/(2*sigma_y*sigma_y))) / (2*M_PI*sigma_x*sigma_y);
			//out << "prob: " << prob << endl;
			weight*=prob;
			associations.push_back(lobs.id);
			sense_x.push_back(lobs.x);
			sense_y.push_back(lobs.y);						
		}

		if (associations.size() > 0) {
			p.weight = weight;	
			SetAssociations(p, associations, sense_x, sense_y);		
		} else {
			p.weight =0.;
		}
		total_weights+= p.weight;
		//cout << "probability weight of particle id: "<< p.id << " is: " << p.weight << endl;
	}

	cout << "total_weights: " << total_weights << endl;

	
	// Not needed as weights already are probabilities
	// To ensuure the weights reflect probabilites
	// NOTE: We can get rid of that later on, if not needed by the logic
	//       To save on computation

	/**
	if (total_weights > 0.0) {

		for(int i=0; i<num_particles; i++){
			Particle & p = particles.at(i);
			p.weight/=total_weights;
		}
	}
	**/
	

	cout << "updateWeights() done." << endl;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	cout << "inside resample()." << endl;
	std::vector<Particle> new_particles;
	double new_weight_sum=0.;
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
		Particle sel_p = particles[index];
		new_particles.push_back(sel_p);
		new_weight_sum += sel_p.weight;
	}
	particles = new_particles;

	/**
	if (new_weight_sum > 0.0) {

		for(int i=0; i<num_particles; i++){
			Particle & p = particles.at(i);
			p.weight/=new_weight_sum;
		}
	}
	**/

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

void ParticleFilter::SetAssociations(Particle &particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
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
}

string ParticleFilter::getAssociations(const Particle &best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(const Particle &best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(const Particle &best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

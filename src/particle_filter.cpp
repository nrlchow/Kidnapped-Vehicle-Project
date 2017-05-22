/*
 * particle_filter.cpp
 *
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	
	num_particles = 20;

	//standard deviation for x,y and theta
	double std_x,std_y,std_theta;
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	//Generate noise using normal distributions for x,y and theta
	//std::default_random_engine gen;

	std::random_device rd;
	std::default_random_engine gen1(rd());
	std::normal_distribution<double>gps_uncertainities_x(x,std_x);
	std::normal_distribution<double>gps_uncertainities_y(y,std_y);
	std::normal_distribution<double>gps_uncertainities_theta(theta,std_theta);

	for(int i = 0; i<num_particles;++i) {
		Particle newParticle;
		newParticle.id = i;
		newParticle.x = gps_uncertainities_x(gen1);
		newParticle.y = gps_uncertainities_y(gen1);
		newParticle.theta = gps_uncertainities_theta(gen1);
		newParticle.weight = 1.0;
		particles.push_back(newParticle);

	}

	is_initialized = true;

}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	
	// Add measurements to each particle and add random Gaussian noise.
	
	std::random_device rd;
	std::default_random_engine gen2(rd());
	std::normal_distribution<double>noise_x(0,std_pos[0]);
	std::normal_distribution<double>noise_y(0,std_pos[1]);
	std::normal_distribution<double>noise_theta(0,std_pos[2]);

	//apply motion measurements and add noise to each particle based on Yaw rate
	// ================

	for (int i=0;i<particles.size();++i) {
		if (yaw_rate==0) {
			particles[i].x += velocity * delta_t * cos(particles[i].theta) + noise_x(gen2);
			particles[i].y += velocity * delta_t * sin(particles[i].theta) + noise_y(gen2);
		}

		else {
			particles[i].x += (velocity / yaw_rate) *
							  (sin(particles[i].theta + delta_t * yaw_rate) - sin(particles[i].theta)) + noise_x(gen2);
			particles[i].y += (velocity / yaw_rate) *
							  (cos(particles[i].theta) - cos(particles[i].theta + delta_t * yaw_rate)) + noise_y(gen2);
			particles[i].theta +=  delta_t * yaw_rate + noise_theta(gen2);
		}
	}
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
								   std::vector<LandmarkObs> observations, Map map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution. 

	//clear the weights
	weights.clear();

	// loop through all particles
	for (int i = 0; i < particles.size(); ++i) {


		// Transform the observed points from vehicle coordinates into map coordinates
		// ================

		for (int j = 0; j < observations.size(); ++j) {
			// Transform - Translation and Rotation only
			double transOBS_x =
					observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta) +
					particles[i].x;
			double transOBS_y =
					observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta) +
					particles[i].y;

			// Loop over a list of landmarks in the map and find close-by predicted landmarks
			// Look at only those in sensor range of the particle
			// Associate them with transformed observation
			// ================
			double curr_dist = 1e5;
			int closest_id;

			for (int k = 0; k < map_landmarks.landmark_list.size(); ++k) {
				Map::single_landmark_s landmark_in_range = map_landmarks.landmark_list[k];
				double single_landmark_dist = dist(particles[i].x, particles[i].y, landmark_in_range.x_f,
												   landmark_in_range.y_f);
				if (single_landmark_dist <= sensor_range) {
					double OBS_to_Pred_x_dist = transOBS_x - landmark_in_range.x_f;
					double OBS_to_Pred_y_dist = transOBS_y - landmark_in_range.y_f;
					double dist = sqrt(OBS_to_Pred_x_dist * OBS_to_Pred_x_dist + OBS_to_Pred_y_dist * OBS_to_Pred_y_dist);
					if (dist < curr_dist) {
						curr_dist = dist;
						closest_id = k;
					}
				}
			}

			// Calculate weight of each particle
			// ================

			particles[i].weight = 1;
			double x = transOBS_x - map_landmarks.landmark_list[closest_id].x_f;
			double y = transOBS_y - map_landmarks.landmark_list[closest_id].y_f;
			double std_x = 2*std_landmark[0]*std_landmark[0];
			double std_y = 2*std_landmark[1]*std_landmark[1];
			particles[i].weight *=  exp(-1*(x*x/std_x+ y*y/std_y))/1/(2*M_PI * std_landmark[0] * std_landmark[1]);

		}
		weights.push_back(particles[i].weight);

	}
}


void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
	
	std::random_device rd;
	std::mt19937 gen3(rd());
	std::discrete_distribution<> ddist(weights.begin(),weights.end());
	std::vector<Particle>resampled_particles;
	std::vector<double>resampled_weights;
	resampled_particles.clear();
	for (int i = 0; i < num_particles; ++i){
		int new_index = ddist(gen3);
		resampled_particles.push_back(particles[new_index]);
	}
	particles = resampled_particles;

}

void ParticleFilter::write(std::string filename) {
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}

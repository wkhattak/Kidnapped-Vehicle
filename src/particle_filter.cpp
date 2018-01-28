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
  
  num_particles = 150; 

  default_random_engine gen;  
   
  // Initialize normal distributions for x,y,theta noise
  normal_distribution<double> dist_n_x(x, std[0]);
  normal_distribution<double> dist_n_y(y, std[1]);
  normal_distribution<double> dist_n_theta(theta, std[2]);
  
  // Initialize each particle
  for (int i = 0; i < num_particles; i++) { 
    // Set all weights to 1
    double weight = 1.0;    
    weights.push_back(weight);  
    
    Particle particle;
    particle.id = i;
    particle.x = dist_n_x(gen);
    particle.y = dist_n_y(gen);
    particle.theta = dist_n_theta(gen);
    particle.weight = weight;
    
    particles.push_back(particle);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  
  default_random_engine gen; 

	for (int i = 0; i < num_particles; i++) {    
	  // Check if yaw_rate is almost zero
	  if (fabs(yaw_rate) < zero_threshold) {
	    particles[i].x += velocity * cos(particles[i].theta) * delta_t;
	    particles[i].y += velocity * sin(particles[i].theta) * delta_t;
	  } else {
	    particles[i].x += ((velocity/yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta)));
	    particles[i].y += ((velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t))));
	    particles[i].theta += (yaw_rate * delta_t);
	  }
	  
    // Add random noise based on updated particle position   
    normal_distribution<double> dist_n_x(particles[i].x, std_pos[0]);
	  normal_distribution<double> dist_n_y(particles[i].y, std_pos[1]);
	  normal_distribution<double> dist_n_theta(particles[i].theta, std_pos[2]);
    particles[i].x = dist_n_x(gen);
	  particles[i].y = dist_n_y(gen);
	  particles[i].theta = dist_n_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  
  for (unsigned int i = 0; i < observations.size(); i++) {
    // Set distance to max posible for comparison purposes
    double min_dist = numeric_limits<double>::max();

    // Landmark id to be asscoiated with the observation id
    int landmark_id = -9999999;
    
    for (unsigned int j = 0; j < predicted.size(); j++) {    
      // Find distance between landmark & observation
      double temp_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

      // Pick current landmark if it's the closest so far
      if (temp_dist < min_dist) {
        min_dist = temp_dist;
        landmark_id = predicted[j].id;
      }
    }
    // Assign the closest found landmark id to the observation id
    observations[i].id = landmark_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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
  
  double weights_sum = 0.0;  

  for (int i = 0; i < num_particles; i++) {
    /***************************************************************************************************
     * STEP 1: Transform observations' coordinate space to the map's coordinate space
    ***************************************************************************************************/ 
    vector<LandmarkObs> transf_observs;
    for (unsigned int j = 0; j < observations.size(); j++) {
      LandmarkObs transf_observ;
      transf_observ.id = observations[j].id;
      transf_observ.x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
      transf_observ.y = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);
      transf_observs.push_back(transf_observ);
    }

    /***************************************************************************************************
     * STEP 2: Filter out landmarks that are outside the sensor's range of the current particle
    ***************************************************************************************************/ 
    vector<LandmarkObs> in_range_landmarks;
    for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); k++) {
      // Cut down version of Euclidean distance to reduce computation time - NOT Used
      //if ((fabs((particles[i].x - map_landmarks.landmark_list[k].x_f)) <= sensor_range) && (fabs((particles[i].y - map_landmarks.landmark_list[k].y_f)) <= sensor_range)) {
        
      if (dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f) <= sensor_range) {
        in_range_landmarks.push_back(LandmarkObs {map_landmarks.landmark_list[k].id_i, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f});
      }
    }

    /***************************************************************************************************
     * STEP 3: Associate each transformed observation with the closest in-range landmark
    ***************************************************************************************************/ 
    dataAssociation(in_range_landmarks, transf_observs);

    /***************************************************************************************************
     * STEP 4: Calculate particle weight
    ***************************************************************************************************/
    // Reset particle weight
    particles[i].weight = 1.0;  

    // Pre-compute for faster execution
    double std_x_square = pow(std_landmark[0], 2);
    double std_y_square = pow(std_landmark[1], 2);
    // Calculate normalization term
    double gaussian_norm = (1.0/(2.0 * M_PI * std_landmark[0] * std_landmark[1]));
      
    // Loop through all obervations and add weight for all associated landmarks 
    for (unsigned int l = 0; l < transf_observs.size(); l++) {
      double transf_observ_x = transf_observs[l].x;
      double transf_observ_y = transf_observs[l].y;
      double transf_observ_id = transf_observs[l].id;
      double weight = 1.0;

      for (unsigned int m = 0; m < in_range_landmarks.size(); m++) {
        if (transf_observ_id == in_range_landmarks[m].id) {
          weight = gaussian_norm * exp(-1.0 * ((pow((transf_observ_x - in_range_landmarks[m].x), 2)/(2.0 * std_x_square)) + (pow((transf_observ_y - in_range_landmarks[m].y), 2)/(2.0 * std_y_square))));
          if (weight < zero_threshold) particles[i].weight *= zero_threshold;
          else particles[i].weight *= weight;
        }
      }
    }
    weights_sum += particles[i].weight;  
  }

  /***************************************************************************************************
   * STEP 5: Normalize weights so that their values are between 0-1 (these weights will be used as probabilities during resampling)
  ***************************************************************************************************/
  for (int i = 0; i < num_particles; i++) {
    particles[i].weight /= weights_sum;
    // Also update global weight vector for all particles
    weights[i] = particles[i].weight;  
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  vector<Particle> resampled_particles;

  // Create a random number generator for selecting a particle index based on particle weight
  default_random_engine gen;
  discrete_distribution<int> particle_index(weights.begin(), weights.end());
  for (int i = 0; i < num_particles; i++) {
    resampled_particles.push_back(particles[particle_index(gen)]);
  }
  
  // Replace with resampled particles
  particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations = associations;
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
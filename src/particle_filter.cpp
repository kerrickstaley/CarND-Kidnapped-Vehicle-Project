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

#define NUM_PARTICLES 100

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    num_particles = NUM_PARTICLES;

    default_random_engine generator;
    normal_distribution<double> x_dist(x, std[0]);
    normal_distribution<double> y_dist(y, std[1]);
    normal_distribution<double> theta_dist(theta, std[2]);

    for (int i = 0; i < num_particles; i++) {
        Particle part = {
            .id = i,
            .x = x_dist(generator),
            .y = y_dist(generator),
            .theta = theta_dist(generator),
            .weight = 1,
        };

        particles.push_back(part);
    }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    default_random_engine generator;
    normal_distribution<double> dx_dist(0, std_pos[0]);
    normal_distribution<double> dy_dist(0, std_pos[1]);
    normal_distribution<double> dtheta_dist(0, std_pos[2]);

    for (auto& part : particles) {
        part.x += delta_t * velocity * cos(part.theta) + dx_dist(generator);
        part.y += delta_t * velocity * sin(part.theta) + dy_dist(generator);
        part.theta += delta_t * yaw_rate + dtheta_dist(generator);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
        const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

    // filter down landmark list to reduce time spent in inner loop
    double min_x = numeric_limits<double>::infinity(),
           max_x = -numeric_limits<double>::infinity(),
           min_y = numeric_limits<double>::infinity(),
           max_y = -numeric_limits<double>::infinity();

    for (auto& part : particles) {
        min_x = min(min_x, part.x);
        max_x = max(max_x, part.x);
        min_y = min(min_y, part.y);
        max_y = max(max_y, part.y);
    }

    vector<Map::single_landmark_s> landmarks;
    for (auto& land : map_landmarks.landmark_list) {
        if (land.x_f < min_x - sensor_range
                || land.x_f > max_x + sensor_range
                || land.y_f < min_y - sensor_range
                || land.y_f > max_y + sensor_range) {
            continue;
        }
        landmarks.push_back(land);
    }

    #ifdef DEBUG
    cerr << "filtered landmarks from " << map_landmarks.landmark_list.size() << " elements to " << landmarks.size()
         << " elements" << endl;
    cerr << "there are " << observations.size() << " observations" << endl;
    cerr << "there are " << particles.size() << " particles" << endl;
    cerr << "expect " << particles.size() * observations.size() * landmarks.size() << " inner loop iterations" << endl;
    int inner_loop_iters = 0;
    #endif

    double sigma_x2 = std_landmark[0] * std_landmark[0];
    double sigma_y2 = std_landmark[0] * std_landmark[0];

    for (auto& part : particles) {
        // reset particle weight
        part.weight = 1;

        for (auto& obs : observations) {
            // convert observations to absolute positions based on particle position
            double xabs = cos(part.theta) * obs.x - sin(part.theta) * obs.y + part.x;
            double yabs = sin(part.theta) * obs.x + cos(part.theta) * obs.y + part.y;

            // find most likely object in map
            // find the object that minimizes the "normalized" distance squared, which I define as
            // dx^2 / sigma_x^2 + dy^2 / sigma_y^2
            double min_norm_dist_sq = numeric_limits<double>::infinity();
            for (auto& land : landmarks) {
                #ifdef DEBUG
                inner_loop_iters++;
                #endif
                double dx = xabs - land.x_f;
                double dy = yabs - land.y_f;
                double norm_dist_sq = dx * dx / sigma_x2 + dy * dy / sigma_y2;
                min_norm_dist_sq = min(min_norm_dist_sq, norm_dist_sq);
            }

            double prob = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1])
                          * exp(-0.5 * min_norm_dist_sq);

            // update particle weight
            part.weight *= prob;
        }
    }

    #ifdef DEBUG
    cerr << "did " << inner_loop_iters << " iterations in the inner loop of ParticleFilter::updateWeights" << endl;
    #endif
}

void ParticleFilter::resample() {
    // do a copy, sigh
    // asked for a better way to do this here:
    // https://stackoverflow.com/questions/49715255/given-a-vector-of-structs-how-can-i-get-an-iterator-for-a-specific-member-of-ea
    vector<double> weights;
    for (auto& part : particles) {
        weights.push_back(part.weight);
    }

    default_random_engine generator;
    std::discrete_distribution<> dist(weights.begin(), weights.end());

    vector<Particle> new_particles;
    for (size_t i = 0; i < NUM_PARTICLES; i++) {
        new_particles.push_back(particles[dist(generator)]);
    }

    particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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

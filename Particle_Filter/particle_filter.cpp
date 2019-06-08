#include "particle_filter.h"
#include <vector>
#include <random>
#include <math.h>

ParticleFilter::ParticleFilter(int N, float sample_time, std::vector<float> covs, std::vector<float> init_state, std::vector<float> init_cov) {
	num_particles = N;
	dt = sample_time;
	covariances = covs;
	std::default_random_engine generator;

	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < init_state.size(); ++j){
			std::normal_distribution<float> distribution(init_state[j], init_cov[j]);
			particles[i].state[j] = distribution(generator);
		}
	}
}

std::vector<float> ParticleFilter::estimate_state() {

	std::default_random_engine generator;

	for (int i = 0; i < particles.size(); ++i) {
		// predict x position
		float x_predict = particles[i].state[0] + particles[i].state[3] * std::cos(particles[i].state[2] + particles[i].state[4]) * dt;
		std::normal_distribution<float> distribution(x_predict, covariances[0]);
		x_predict = distribution(generator);

		//predict y position
		float y_predict = particles[i].state[1] + particles[i].state[3] * std::sin(particles[i].state[2] + particles[i].state[4]) * dt;
		std::normal_distribution<float> distribution(y_predict, covariances[1]);
		y_predict = distribution(generator);

		//predict heading
		float psi_predict = particles[i].state[2] + particles[i].state[3] * std::cos(particles[i].state[4])*std::tan(steering_angle)*dt;
		std::normal_distribution<float> distribution(y_predict, covariances[1]);
		y_predict = distribution(generator);

	}
}

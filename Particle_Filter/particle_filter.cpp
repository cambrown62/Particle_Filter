#include "particle_filter.h"
#include <vector>
#include <random>

ParticleFilter::ParticleFilter(int N, std::vector<float> init_state, std::vector<float> init_cov) {
	num_particles = N;

	std::default_random_engine generator;

	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < init_state.size(); ++j){
			std::normal_distribution<float> distribution(init_state[j], init_cov[j]);
			particles[i].state[j] = distribution(generator);
		}
	}
}

std::vector<float> ParticleFilter::estimate_state() {
	for (int i = 0; i < num_particles; ++i) {
		for (int j = 0; j < particles[0].state.size(); ++j) {

		}
	}
}

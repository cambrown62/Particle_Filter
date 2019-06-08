#pragma once
#include <vector>
#include <iostream>

struct Particle {
	std::vector<float> state;
	float weight;
};

class ParticleFilter {
	int num_particles;
	std::vector<Particle> particles;
	float dt;
	std::vector<float> covariances;
	float steering_angle;

public:
	ParticleFilter(int N, float sample_time, std::vector<float> covs, std::vector<float> init_state, std::vector<float> init_cov);
	//std::vector<Particle> initialize();
	std::vector<Particle> resample();
	std::vector<float> estimate_state();
};
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

public:
	ParticleFilter(int N, std::vector<float> init_state, std::vector<float> init_cov);
	//std::vector<Particle> initialize();
	std::vector<Particle> resample();
	std::vector<float> estimate_state();
};
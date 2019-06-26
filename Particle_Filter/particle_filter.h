#pragma once
#include <vector>
#include <iostream>

struct Particle {
	//std::vector<float> state;
	float x;
	float y;
	float psi;
	float v;
	float beta;
	float weight;
};

class ParticleFilter {
	int num_particles;
	std::vector<Particle> particles;
	float dt;
	std::vector<float> state_covariances;
	std::vector<float> msmt_covariances;
	float L;
	std::vector<float> state_estimate;


	// these are placeholders, they would come from their respective publishers
	float steering_angle;
	float acceleration;
	std::vector<float> measurements;
	

public:
	ParticleFilter(int N, float sample_time, std::vector<float> state_covs, std::vector<float> msmt_covs, std::vector<float> init_state, std::vector<float> init_cov);
	//std::vector<Particle> initialize();
	void resample();
	void predict();
	void calc_weights();
	void estimate_state();
};
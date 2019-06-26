#include "particle_filter.h"
#include <vector>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>


ParticleFilter::ParticleFilter(int N, float sample_time, std::vector<float> state_covs, std::vector<float> msmt_covs, std::vector<float> init_state, std::vector<float> init_cov) {
	num_particles = N;
	dt = sample_time;
	state_covariances = state_covs;
	msmt_covariances = msmt_covs;
	std::default_random_engine generator;
	std::normal_distribution<float> x_dist(init_state[0], init_cov[0]);
	std::normal_distribution<float> y_dist(init_state[1], init_cov[1]);
	std::normal_distribution<float> psi_dist(init_state[2], init_cov[2]);
	std::normal_distribution<float> v_dist(init_state[3], init_cov[3]);
	std::normal_distribution<float> beta_dist(init_state[4], init_cov[4]);

	for (int i = 0; i < N; ++i) {
		particles[i].x = x_dist(generator);
		particles[i].y = y_dist(generator);
		particles[i].psi = psi_dist(generator);
		particles[i].v = v_dist(generator);
		particles[i].beta = beta_dist(generator);
		particles[i].weight = 1.0/N;
	}
}

void ParticleFilter::predict() {

	std::default_random_engine generator;

	//steering_angle = msg from steering angle publisher
	//acceleration = msg from acceleration publisher

	for (int i = 0; i < particles.size(); ++i) {
		// predict x position
		float x_predict = particles[i].x + particles[i].v * std::cos(particles[i].psi + particles[i].beta)*dt;
		std::normal_distribution<float> x_dist(x_predict, state_covariances[0]);
		particles[i].x = x_dist(generator);

		//predict y position
		float y_predict = particles[i].y + particles[i].v * std::sin(particles[i].psi + particles[i].beta)*dt;
		std::normal_distribution<float> y_dist(y_predict, state_covariances[1]);
		particles[i].y = y_dist(generator);

		//predict heading
		float psi_predict = particles[i].psi + (particles[i].v / L)*std::cos(particles[i].beta)*std::tan(steering_angle)*dt;
		std::normal_distribution<float> psi_dist(psi_predict, state_covariances[2]);
		particles[i].psi = psi_dist(generator);

		//predict velocity
		float v_predict = particles[i].v + acceleration * dt;
		std::normal_distribution<float> v_dist(v_predict, state_covariances[3]);
		particles[i].v = v_dist(generator);

		//predict slip angle
		float beta_predict = std::atan(L/2 * std::tan(steering_angle)/L);
		std::normal_distribution<float> beta_dist(beta_predict, state_covariances[4]);
		particles[i].beta = beta_dist(generator);
	}
}

void ParticleFilter::calc_weights() {
	//in real life:
	//integrate IMU twice to get one (x,y) reading with IMU covariance
	//read gps for second (x,y) reading with GPS covariance
	//multiply these 2 distributions
	//evalute resulting distribution at each particle (x,y) to find weight


	//for our purposes, pretend we only have GPS measurement
	std::default_random_engine generator;

	float weight_sum = 0;

	for (int i = 0; i < particles.size(); ++i) {
		// predict x measurement
		std::normal_distribution<float> x_dist(particles[i].x, msmt_covariances[0]);
		float x_msmt_predict = x_dist(generator);

		// predict y measurement
		std::normal_distribution<float> y_dist(particles[i].y, msmt_covariances[1]);
		float y_msmt_predict = y_dist(generator);

		//calculate weights
		float x_part = 1 / (sqrt(2 * M_PI * msmt_covariances[0] * msmt_covariances[0])) * exp(-pow(x_msmt_predict - measurements[0], 2) / (2 * msmt_covariances[0] * msmt_covariances[0]);
		float y_part = 1 / (sqrt(2 * M_PI * msmt_covariances[1] * msmt_covariances[1])) * exp(-pow(y_msmt_predict - measurements[1], 2) / (2 * msmt_covariances[1] * msmt_covariances[1]);
		particles[i].weight = x_part * y_part;
		weight_sum += particles[i].weight;
	}

	//normalize weights
	for (int i = 0; i <= num_particles - 1; ++i) {
		particles[i].weight /= weight_sum;
	}

}

void ParticleFilter::resample() {
	// systematic resampling

	std::vector<Particle> resamped_parts;
	std::vector<float> cum_sum;
	cum_sum.push_back(particles[0].weight);

	for (int i = 1; i <= num_particles - 1; ++i) {
		cum_sum.push_back(cum_sum[i - 1] + particles[i].weight);
	}

	std::default_random_engine generator;
	std::uniform_real_distribution<float> dist(0, 1.0/(float)num_particles);

	float u = dist(generator);
	int index = 0;

	for (int j = 1; j <= num_particles - 1; ++j) {
		while (u > cum_sum[index]) {
			index++;
		}
		resamped_parts.push_back(particles[index]);
		u += (1.0 / (float)num_particles);
	}

}

void ParticleFilter::estimate_state() {
	//estimate state with expected value of particles
	float x_hat = 0;
	float y_hat = 0;
	for (int i = 0; i <= num_particles - 1; ++i) {
		x_hat += (particles[i].x*particles[i].weight);
		y_hat += (particles[i].y*particles[i].weight);
	}
	state_estimate[0] = x_hat;
	state_estimate[1] = y_hat;
}


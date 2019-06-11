#include "particle_filter.h"
#include <vector>
#include <random>
#include <math.h>

ParticleFilter::ParticleFilter(int N, float sample_time, std::vector<float> covs, std::vector<float> init_state, std::vector<float> init_cov) {
	num_particles = N;
	dt = sample_time;
	covariances = covs;
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
		std::normal_distribution<float> x_dist(x_predict, covariances[0]);
		particles[i].x = x_dist(generator);

		//predict y position
		float y_predict = particles[i].y + particles[i].v * std::sin(particles[i].psi + particles[i].beta)*dt;
		std::normal_distribution<float> y_dist(y_predict, covariances[1]);
		particles[i].y = y_dist(generator);

		//predict heading
		float psi_predict = particles[i].psi + (particles[i].v / L)*std::cos(particles[i].beta)*std::tan(steering_angle)*dt;
		std::normal_distribution<float> psi_dist(psi_predict, covariances[2]);
		particles[i].psi = psi_dist(generator);

		//predict velocity
		float v_predict = particles[i].v + acceleration * dt;
		std::normal_distribution<float> v_dist(v_predict, covariances[3]);
		particles[i].v = v_dist(generator);

		//predict slip angle
		float beta_predict = std::atan(L/2 * std::tan(steering_angle)/L);
		std::normal_distribution<float> beta_dist(beta_predict, covariances[4]);
		particles[i].beta = beta_dist(generator);
	}
}

void ParticleFilter::calc_weights() {
	//integrate IMU twice to get one (x,y) reading with IMU covariance
	//read gps for second (x,y) reading with GPS covariance
	//multiply these 2 distributions
	//evalute resulting distribution at each particle (x,y) to find weight
}



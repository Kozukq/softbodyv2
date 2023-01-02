#pragma once

#include "cgp/cgp.hpp"

struct gui_parameters {
	bool display_frame = false;
	float gy = -9.81f; // gravity
};

struct spring {

	cgp::vec3* posOther;
	float K; 				// spring stiffness
	float mu; 				// damping coefficient
	float L0; 				// rest-length of spring
	bool isHidden;

	spring(cgp::vec3* _posOther, float _K, float _mu, float _L0, bool _isHidden = false): posOther(_posOther), K(_K), mu(_mu), L0(_L0), isHidden(_isHidden) {};
};

struct particle {

	float mass;
	cgp::vec3 pos;
	cgp::vec3 vel;
	std::vector<spring> springs;

	particle(float _mass, cgp::vec3 _pos, cgp::vec3 _vel): mass(_mass), pos(_pos), vel(_vel) {};
};

struct scene_structure {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //

	// Particles:
	std::vector<particle> particles;

	void simulation_step(float dt);
	void draw_segment(cgp::vec3 const& a, cgp::vec3 const& b);

	// Drawable structure to display the particles and the spring
	cgp::mesh_drawable particle_sphere;
	cgp::segments_drawable segment;


	// Standard elements of the scene
	cgp::mesh_drawable global_frame;          // The standard global frame
	cgp::scene_environment_basic environment; // Standard environment controler
	gui_parameters gui;                       // Standard GUI element storage
	cgp::timer_basic timer;


	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();  // Standard initialization to be called before the animation loop
	void display();     // The frame display to be called within the animation loop
	void display_gui(); // The display of the GUI, also called within the animation loop
};






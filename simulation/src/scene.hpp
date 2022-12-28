#pragma once

#include "cgp/cgp.hpp"





struct gui_parameters {
	bool display_frame = false;
	float m  = 0.01f;        // particle mass
	float K  = 5.0f;        // spring stiffness
	float mu = 0.01f;       // damping coefficient
	float gy = -9.81f; // gravity
};




struct scene_structure {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //

	// Particles:
	cgp::vec3 pA; // position of particle A
	cgp::vec3 pB; // position of particle B
	cgp::vec3 vA; // velocity of particle A
	cgp::vec3 vB; // velocity of particle B
	cgp::vec3 pC;
	cgp::vec3 vC;
	float L0; // Rest-length of spring


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






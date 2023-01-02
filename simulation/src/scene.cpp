#include "scene.hpp"

using namespace cgp;

// Spring force applied on particle p_i with respect to position p_j.
vec3 spring_force(const vec3& p_i, const vec3& p_j, float L0, float K) {

	vec3 const p = p_i - p_j;
	float const L = norm(p);
	vec3 const u = p / L;

	vec3 const F = -K * (L - L0) * u;
	return F;
}


void scene_structure::simulation_step(float dt) {

	vec3 const g = { 0,0,gui.gy };

	for(particle& p : particles) {

		// Colliding with ground plane at arbitrary Z height
		if(p.pos.z < -1.5f) {
			p.pos.z = -1.5f;
			p.vel = -p.vel * dt;
		}

		for(const spring& s : p.springs) {

			// Forces
			const vec3 Fspring = spring_force(p.pos,*s.posOther,s.L0,s.K);
			const vec3 Fweight = p.mass * g;
			const vec3 Fdamping = -s.mu * p.vel;
			vec3 F = Fspring + Fweight + Fdamping;

			// Velocity-Verlet integration
			const vec3 halfVel = p.vel + dt / 2 * F / p.mass;
			p.pos = p.pos + dt * halfVel;
			F = spring_force(p.pos,*s.posOther,s.L0,s.K) + Fweight + Fdamping;
			p.vel = halfVel + dt / 2 * F / p.mass;
		}
	}
}

void scene_structure::display() {
	// Basics common elements
	// ***************************************** //
	timer.update();
	environment.light = environment.camera.position();
	if (gui.display_frame)
		draw(global_frame, environment);

	simulation_step(timer.scale * 0.01f);

	for(const particle& p : particles) {

		particle_sphere.transform.translation = p.pos;
		particle_sphere.shading.color = { 0,0,0 };
		draw(particle_sphere,environment);

		for(const spring& s : p.springs) {

			if(!s.isHidden) draw_segment(p.pos,*s.posOther);
		}
	}
}



void scene_structure::initialize() {

	// AUTO CHAIN
	// int len = 25;
	// float dl = 6.0f / float(len);
	// float curpos = 4.0f;
	// for(int i = 0; i < len; i++) {

	// 	particles.push_back(particle(0.01f,vec3(0,i*0.01f,curpos),vec3(0,0,0)));
	// 	curpos -= dl;
	// }
	// for(int i = 1; i < len; i++) {

	// 	particles[i].springs.push_back(spring(&particles[i-1].pos,len*1.0f,0.01f,dl));
	// 	if(i < len-1) particles[i].springs.push_back(spring(&particles[i+1].pos,len*1.0f,0.01f,dl));
	// }

	// CUBE
	//   4----5
	//  /|   /| 
	// 0----1 |
	// | 6--|-7
	// |/   |/
	// 2----3	
	particles.push_back(particle(0.01f,vec3(1,-1,1),vec3(0,0,0)));
	particles.push_back(particle(0.01f,vec3(1,1,1),vec3(0,0,0)));
	particles.push_back(particle(0.01f,vec3(1,-1,-1),vec3(0,0,0)));
	particles.push_back(particle(0.01f,vec3(1,1,-1),vec3(0,0,0)));
	particles.push_back(particle(0.01f,vec3(-1,-1,1),vec3(0,0,0)));
	particles.push_back(particle(0.01f,vec3(-1,1,1),vec3(0,0,0)));
	particles.push_back(particle(0.01f,vec3(-1,-1,-1),vec3(0,0,0)));
	particles.push_back(particle(0.01f,vec3(-1,1,-1),vec3(0,0,0)));
	particles[0].springs.push_back(spring(&particles[4].pos,5.0f,0.01f,2));
	particles[0].springs.push_back(spring(&particles[1].pos,5.0f,0.01f,2));
	particles[0].springs.push_back(spring(&particles[2].pos,5.0f,0.01f,2));
	particles[1].springs.push_back(spring(&particles[5].pos,5.0f,0.01f,2));
	particles[1].springs.push_back(spring(&particles[0].pos,5.0f,0.01f,2));
	particles[1].springs.push_back(spring(&particles[3].pos,5.0f,0.01f,2));
	particles[2].springs.push_back(spring(&particles[6].pos,5.0f,0.01f,2));
	particles[2].springs.push_back(spring(&particles[3].pos,5.0f,0.01f,2));
	particles[2].springs.push_back(spring(&particles[0].pos,5.0f,0.01f,2));
	particles[3].springs.push_back(spring(&particles[7].pos,5.0f,0.01f,2));
	particles[3].springs.push_back(spring(&particles[2].pos,5.0f,0.01f,2));
	particles[3].springs.push_back(spring(&particles[1].pos,5.0f,0.01f,2));
	particles[4].springs.push_back(spring(&particles[0].pos,5.0f,0.01f,2));
	particles[4].springs.push_back(spring(&particles[5].pos,5.0f,0.01f,2));
	particles[4].springs.push_back(spring(&particles[6].pos,5.0f,0.01f,2));
	particles[5].springs.push_back(spring(&particles[1].pos,5.0f,0.01f,2));
	particles[5].springs.push_back(spring(&particles[4].pos,5.0f,0.01f,2));
	particles[5].springs.push_back(spring(&particles[7].pos,5.0f,0.01f,2));
	particles[6].springs.push_back(spring(&particles[2].pos,5.0f,0.01f,2));
	particles[6].springs.push_back(spring(&particles[7].pos,5.0f,0.01f,2));
	particles[6].springs.push_back(spring(&particles[4].pos,5.0f,0.01f,2));
	particles[7].springs.push_back(spring(&particles[3].pos,5.0f,0.01f,2));
	particles[7].springs.push_back(spring(&particles[6].pos,5.0f,0.01f,2));
	particles[7].springs.push_back(spring(&particles[5].pos,5.0f,0.01f,2));

	float cubeDiag = 2 * sqrt(3);
	cubeDiag = 4;
	particles[0].springs.push_back(spring(&particles[7].pos,5.0f,0.01f,cubeDiag,true));
	particles[7].springs.push_back(spring(&particles[0].pos,5.0f,0.01f,cubeDiag,true));
	particles[1].springs.push_back(spring(&particles[6].pos,5.0f,0.01f,cubeDiag,true));
	particles[6].springs.push_back(spring(&particles[1].pos,5.0f,0.01f,cubeDiag,true));
	particles[2].springs.push_back(spring(&particles[5].pos,5.0f,0.01f,cubeDiag,true));
	particles[3].springs.push_back(spring(&particles[4].pos,5.0f,0.01f,cubeDiag,true));
	particles[5].springs.push_back(spring(&particles[2].pos,5.0f,0.01f,cubeDiag,true));
	particles[4].springs.push_back(spring(&particles[3].pos,5.0f,0.01f,cubeDiag,true));

	particle_sphere.initialize(mesh_primitive_sphere(0.05f));

	segments_drawable::default_shader = curve_drawable::default_shader;

	segment = segments_drawable();
	segment.initialize({{0,0,0},{1,0,0}});

	global_frame.initialize(mesh_primitive_frame(), "Frame");
	environment.camera.look_at({ 10.0f,0.5f,0.0f }, { 0,0,0 }, { 0,0,1 });
}

void scene_structure::display_gui() {

	ImGui::Checkbox("Frame", &gui.display_frame);
	ImGui::SliderFloat("Gravity",&gui.gy,-10.0f,10.0f);
}



void scene_structure::draw_segment(vec3 const& a, vec3 const& b) {

	segment.update({ a, b });
	draw(segment, environment);
}
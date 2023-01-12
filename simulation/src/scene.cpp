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

	bool onClickXneg = ImGui::Button("-X Force"); ImGui::SameLine();
	bool onClickXpos = ImGui::Button("+X Force");
	bool onClickYneg = ImGui::Button("-Y Force"); ImGui::SameLine();
	bool onClickYpos = ImGui::Button("+Y Force");
	bool onClickZneg = ImGui::Button("-Z Force"); ImGui::SameLine();
	bool onClickZpos = ImGui::Button("+Z Force");

	for(particle& p : particles) {

		if(onClickXneg) p.vel += vec3(-10,0,0);
		if(onClickXpos) p.vel += vec3(10,0,0);
		if(onClickYneg) p.vel += vec3(0,-10,0);
		if(onClickYpos) p.vel += vec3(0,10,0);
		if(onClickZneg) p.vel += vec3(0,0,-10);
		if(onClickZpos) p.vel += vec3(0,0,10);

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

	// Update simulation parameters
	if(ImGui::Button("Update parameters")) {

		for(particle& p : particles) {

			// p.mass = gui.pM;

			// Edge springs
			for(int i = 0; i < 2; i++) {

				p.springs[i].K = gui.sK;
				p.springs[i].mu = gui.sMu;
				// p.springs[i].L0 = gui.sL0;
			}

			// Cube diagonal spring
			if(p.springs.size() > 2) {

				// float cubeDiag = gui.sL0 * sqrt(3);
				// cubeDiag = 4;
				p.springs[2].K = gui.sK;
				p.springs[2].mu = gui.sMu;
				// p.springs[2].L0 = cubeDiag;
			}
		}
	}

	simulation_step(timer.scale * 0.01f);

	if(gui.displayParticles || gui.displaySprings) {

		for(const particle& p : particles) {

			particle_sphere.transform.translation = p.pos;
			particle_sphere.shading.color = { 0,0,0 };

			if(gui.displayParticles) draw(particle_sphere,environment);

			if(gui.displaySprings) {

				for(const spring& s : p.springs) {

					if(s.isDrawn) draw_segment(p.pos,*s.posOther);
				}
			}
		}
	}

	if(gui.displayMesh) {

		mesh shape;
		shape.push_back(mesh_primitive_quadrangle(particles[0].pos, particles[1].pos, particles[3].pos, particles[2].pos));
		shape.push_back(mesh_primitive_quadrangle(particles[1].pos, particles[5].pos, particles[7].pos, particles[3].pos));
		shape.push_back(mesh_primitive_quadrangle(particles[5].pos, particles[4].pos, particles[6].pos, particles[7].pos));
		shape.push_back(mesh_primitive_quadrangle(particles[4].pos, particles[0].pos, particles[2].pos, particles[6].pos));
		shape.push_back(mesh_primitive_quadrangle(particles[2].pos, particles[3].pos, particles[7].pos, particles[6].pos));
		shape.push_back(mesh_primitive_quadrangle(particles[1].pos, particles[0].pos, particles[4].pos, particles[5].pos));
		cube.clear();
		cube.initialize(shape);
		cube.shading.color = vec3(1,0,0);
		draw(cube,environment);
	}

	draw(ground,environment);
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
	float zPosCube = 5;
	float pM = 0.01f;	
	float sK = 3.0f;
	float sMu = 0.01f;
	float sL0 = 2.0f;
	particles.push_back(particle(pM,vec3(1,-1,1+zPosCube),vec3(0,0,0)));
	particles.push_back(particle(pM,vec3(1,1,1+zPosCube),vec3(0,0,0)));
	particles.push_back(particle(pM,vec3(1,-1,-1+zPosCube),vec3(0,0,0)));
	particles.push_back(particle(pM,vec3(1,1,-1+zPosCube),vec3(0,0,0)));
	particles.push_back(particle(pM,vec3(-1,-1,1+zPosCube),vec3(0,0,0)));
	particles.push_back(particle(pM,vec3(-1,1,1+zPosCube),vec3(0,0,0)));
	particles.push_back(particle(pM,vec3(-1,-1,-1+zPosCube),vec3(0,0,0)));
	particles.push_back(particle(pM,vec3(-1,1,-1+zPosCube),vec3(0,0,0)));
	particles[0].springs.push_back(spring(&particles[4].pos,sK,sMu,sL0));
	particles[0].springs.push_back(spring(&particles[1].pos,sK,sMu,sL0));
	particles[0].springs.push_back(spring(&particles[2].pos,sK,sMu,sL0));
	particles[1].springs.push_back(spring(&particles[5].pos,sK,sMu,sL0));
	particles[1].springs.push_back(spring(&particles[0].pos,sK,sMu,sL0));
	particles[1].springs.push_back(spring(&particles[3].pos,sK,sMu,sL0));
	particles[2].springs.push_back(spring(&particles[6].pos,sK,sMu,sL0));
	particles[2].springs.push_back(spring(&particles[3].pos,sK,sMu,sL0));
	particles[2].springs.push_back(spring(&particles[0].pos,sK,sMu,sL0));
	particles[3].springs.push_back(spring(&particles[7].pos,sK,sMu,sL0));
	particles[3].springs.push_back(spring(&particles[2].pos,sK,sMu,sL0));
	particles[3].springs.push_back(spring(&particles[1].pos,sK,sMu,sL0));
	particles[4].springs.push_back(spring(&particles[0].pos,sK,sMu,sL0));
	particles[4].springs.push_back(spring(&particles[5].pos,sK,sMu,sL0));
	particles[4].springs.push_back(spring(&particles[6].pos,sK,sMu,sL0));
	particles[5].springs.push_back(spring(&particles[1].pos,sK,sMu,sL0));
	particles[5].springs.push_back(spring(&particles[4].pos,sK,sMu,sL0));
	particles[5].springs.push_back(spring(&particles[7].pos,sK,sMu,sL0));
	particles[6].springs.push_back(spring(&particles[2].pos,sK,sMu,sL0));
	particles[6].springs.push_back(spring(&particles[7].pos,sK,sMu,sL0));
	particles[6].springs.push_back(spring(&particles[4].pos,sK,sMu,sL0));
	particles[7].springs.push_back(spring(&particles[3].pos,sK,sMu,sL0));
	particles[7].springs.push_back(spring(&particles[6].pos,sK,sMu,sL0));
	particles[7].springs.push_back(spring(&particles[5].pos,sK,sMu,sL0));

	float cubeDiag = sL0 * sqrt(3);
	cubeDiag = 4;
	particles[0].springs.push_back(spring(&particles[7].pos,sK,sMu,cubeDiag));
	particles[7].springs.push_back(spring(&particles[0].pos,sK,sMu,cubeDiag));
	particles[1].springs.push_back(spring(&particles[6].pos,sK,sMu,cubeDiag));
	particles[6].springs.push_back(spring(&particles[1].pos,sK,sMu,cubeDiag));
	particles[2].springs.push_back(spring(&particles[5].pos,sK,sMu,cubeDiag));
	particles[3].springs.push_back(spring(&particles[4].pos,sK,sMu,cubeDiag));
	particles[5].springs.push_back(spring(&particles[2].pos,sK,sMu,cubeDiag));
	particles[4].springs.push_back(spring(&particles[3].pos,sK,sMu,cubeDiag));

	mesh groundMesh = mesh_primitive_quadrangle(vec3(1000,-1000,-1.5f),vec3(1000,1000,-1.5f),vec3(-1000,1000,-1.5f),vec3(-1000,-1000,-1.5f));
	ground.initialize(groundMesh);
	ground.shading.color = vec3(0.9,0.9,0.9);

	particle_sphere.initialize(mesh_primitive_sphere(0.05f));

	segments_drawable::default_shader = curve_drawable::default_shader;

	segment = segments_drawable();
	segment.initialize({{0,0,0},{1,0,0}});

	global_frame.initialize(mesh_primitive_frame(), "Frame");
	environment.camera.look_at({ 10.0f,0.5f,0.0f }, { 0,0,0 }, { 0,0,1 });

	// Initialize GUI
	// gui.pM = pM;
	gui.sK = sK;
	gui.sMu = sMu;
	// gui.sL0 = sL0;
}

void scene_structure::display_gui() {

	// ImGui::Checkbox("Frame", &gui.display_frame);
	ImGui::Checkbox("Draw mesh", &gui.displayMesh);
	ImGui::Checkbox("Draw particles", &gui.displayParticles);
	ImGui::Checkbox("Draw springs", &gui.displaySprings);
	ImGui::SliderFloat("Gravity",&gui.gy,-10.0f,10.0f);
	// ImGui::SliderFloat("Cube mass",&gui.pM,0.01f,1.0f);
	ImGui::SliderFloat("Springs stiffness",&gui.sK,1.0f,5.0f);
	ImGui::SliderFloat("Springs damping coefficient",&gui.sMu,0.001f,0.1f);
	// ImGui::SliderFloat("Springs rest-length",&gui.sL0,1.0f,5.0f);
}



void scene_structure::draw_segment(vec3 const& a, vec3 const& b) {

	segment.update({ a, b });
	draw(segment, environment);
}
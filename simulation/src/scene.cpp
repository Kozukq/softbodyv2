#include "scene.hpp"


using namespace cgp;



// Spring force applied on particle p_i with respect to position p_j.
vec3 spring_force(const vec3& p_i, const vec3& p_j, float L0, float K)
{
	vec3 const p = p_i - p_j;
	float const L = norm(p);
	vec3 const u = p / L;

	vec3 const F = -K * (L - L0) * u;
	return F;
}


void scene_structure::simulation_step(float dt)
{

	// Simulation parameters
	float const m  = gui.m;        // particle mass
	float const K  = gui.K;        // spring stiffness
	float const mu = gui.mu;       // damping coefficient

	vec3 const g = { 0,0,gui.gy }; // gravity

	// Forces
	vec3 fB_spring  = spring_force(pB, pA, L0, K);
	vec3 fB_weight  = m * g;
	vec3 fB_damping = -mu * vB;
	vec3 fB = fB_spring + fB_weight + fB_damping;

	// Numerical Integration
	//   To do: Change this relation to compute a semi-implicit integration (instead of explicit Euler)
	// pB = pB + dt * vB;
	// vB = vB + dt * fB / m;

	// Velocity-Verlet Integration
	vec3 vBhalf = vB + dt / 2 * fB / m;
	pB = pB + dt * vBhalf;
	fB = spring_force(pB,pA,L0,K) + fB_weight + -mu * vBhalf;
	vB = vBhalf + dt / 2 * fB / m;


	// 2nd spring B to C

	// Forces
	vec3 fC_spring  = spring_force(pC, pB, L0, K);
	vec3 fC_weight  = m * g;
	vec3 fC_damping = -mu * vC;
	vec3 fC = fC_spring + fC_weight + fC_damping;

	// Velocity-Verlet Integration
	vec3 vChalf = vC + dt / 2 * fC / m;
	pC = pC + dt * vChalf;
	fC = spring_force(pC,pB,L0,K) + fC_weight + -mu * vChalf;
	vC = vChalf + dt / 2 * fC / m;

	// 2nd spring C to B

	// Forces
	fB_spring  = spring_force(pB, pC, L0, K);
	fB_weight  = m * g;
	fB_damping = -mu * vB;
	fB = fB_spring + fB_weight + fB_damping;

	// Velocity-Verlet Integration
	vBhalf = vB + dt / 2 * fB / m;
	pB = pB + dt * vBhalf;
	fB = spring_force(pB,pC,L0,K) + fB_weight + -mu * vBhalf;
	vB = vBhalf + dt / 2 * fB / m;
}

void scene_structure::display()
{
	// Basics common elements
	// ***************************************** //
	timer.update();
	environment.light = environment.camera.position();
	if (gui.display_frame)
		draw(global_frame, environment);


	simulation_step(timer.scale * 0.01f);

	particle_sphere.transform.translation = pA;
	particle_sphere.shading.color = { 0,0,0 };
	draw(particle_sphere, environment);

	particle_sphere.transform.translation = pB;
	particle_sphere.shading.color = { 1,0,0 };
	draw(particle_sphere, environment);

	particle_sphere.transform.translation = pC;
	particle_sphere.shading.color = { 0,1,0 };
	draw(particle_sphere,environment);

	draw_segment(pA, pB);
	draw_segment(pB,pC);
}



void scene_structure::initialize()
{
	// Initial position and speed of particles
	// ******************************************* //
	pA = { 0,0,0 };     
	vB = { 0,0,0 };     

	pB = { 0.0f,0.45f,0.0f }; 
	vB = { 0,0,0 };    

	pC = pB + cgp::vec3(0,0.5,0);
	vC = { 0,0,0 };

	L0 = 0.4f; 


	particle_sphere.initialize(mesh_primitive_sphere(0.05f));

	segments_drawable::default_shader = curve_drawable::default_shader;

	segment = segments_drawable();
	segment.initialize({{0,0,0},{1,0,0}});

	global_frame.initialize(mesh_primitive_frame(), "Frame");
	environment.camera.look_at({ 3.0f,0.5f,0.0f }, { 0,0,0 }, { 0,0,1 });
}

void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);
	ImGui::SliderFloat("Particle mass",&gui.m,0.01f,1.0f);
	ImGui::SliderFloat("Spring stiffness",&gui.K,1.0f,10.0f);
	ImGui::SliderFloat("Damping coefficient",&gui.mu,0.0f,0.05f);
	ImGui::SliderFloat("Gravity",&gui.gy,-10.0f,10.0f);
}



void scene_structure::draw_segment(vec3 const& a, vec3 const& b)
{
	segment.update({ a, b });
	draw(segment, environment);
}
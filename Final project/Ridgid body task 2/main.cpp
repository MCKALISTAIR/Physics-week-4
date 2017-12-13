#pragma once
// Math constants
#define _USE_MATH_DEFINES
#include <cmath>  
#include <random>

// Std. Includes
#include <string>
#include <time.h>
#include <vector>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include "glm/ext.hpp"


// Other Libs
#include "SOIL2/SOIL2.h"

// project includes
#include "Application.h"
#include "Shader.h"
#include "Mesh.h"
//#include "Body.h"
#include "Particle.h"
#include "RigidBody.h"
const double dtime = 0.01;

//colisoon functions
std::vector <Vertex> Collision_Detect(GLfloat y, RigidBody &rb)
{
	std::vector <Vertex> transformlist;
	for each (Vertex v in rb.getMesh().getVertices())
	{
		glm::vec3 collided = glm::mat3(rb.getMesh().getModel()) * v.getCoord() + rb.getPos();
		if (collided[1] < y)
		{
			transformlist.push_back(collided);
		}
	}
	return transformlist;
}

int Collison(RigidBody a, RigidBody b)
{
	//.e = scale
	//.u = rotation
	//.c = pos
	float ra, rb;
	glm::mat3x3 R, absR;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = glm::dot(a.getRotate()[i], b.getRotate()[j]);
	glm::vec3 t = b.getPos() - a.getPos();
	t = glm::vec3(glm::dot(t, a.getRotate()[0]), glm::dot(t, a.getRotate()[1]), glm::dot(t, a.getRotate()[2]));
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			absR[i][j] = abs(R[i][j]) + 0.001f;
	for (int i = 0; i < 3; i++) {
		ra = a.getScale()[i][i];
		rb = b.getScale()[0][0] * absR[0][i] + a.getScale()[1][1] * absR[1][i];
		if (abs(t[i]) > ra + rb) return 0;
	}
	for (int i = 0; i < 3; i++) {
		ra = a.getScale()[0][0] * absR[0][i] + a.getScale()[1][1] * absR[1][i] + a.getScale()[2][2] * absR[2][i];
		rb = b.getScale()[i][i];
		if (abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] + R[2][i]) > ra + rb) return 0;
	}
	ra = a.getScale()[1][1] * absR[2][0] + a.getScale()[2][2] * absR[1][0];
	rb = b.getScale()[1][1] * absR[0][2] + b.getScale()[2][2] * absR[0][1];
	if (abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb)  return 0;

	ra = a.getScale()[1][1] * absR[2][1] + a.getScale()[2][2] * absR[1][1];
	rb = b.getScale()[0][0] * absR[0][2] + b.getScale()[2][2] * absR[0][0];
	if (abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb) return 0;

	ra = a.getScale()[1][1] * absR[2][2] + a.getScale()[2][2] * absR[1][2];
	rb = b.getScale()[0][0] * absR[0][1] + b.getScale()[1][1] * absR[0][0];
	if (abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[2][0] + a.getScale()[2][2] * absR[0][0];
	rb = b.getScale()[1][1] * absR[1][2] + b.getScale()[2][2] * absR[1][1];
	if (abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[2][1] + a.getScale()[2][2] * absR[0][1];
	rb = b.getScale()[0][0] * absR[1][2] + b.getScale()[1][1] * absR[1][0];
	if (abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[2][2] + a.getScale()[2][2] * absR[0][2];
	rb = b.getScale()[0][0] * absR[1][1] + b.getScale()[1][1] * absR[1][0];
	if (abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[1][0] + a.getScale()[1][1] * absR[0][0];
	rb = b.getScale()[1][1] * absR[2][2] + b.getScale()[2][2] * absR[2][1];
	if (abs(t[2] * R[0][0] - t[0] * R[1][0]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[1][1] + a.getScale()[1][1] * absR[0][1];
	rb = b.getScale()[0][0] * absR[2][2] + b.getScale()[2][2] * absR[2][0];
	if (abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[1][2] + a.getScale()[1][1] * absR[0][2];
	rb = b.getScale()[0][0] * absR[2][1] + b.getScale()[1][1] * absR[2][0];
	if (abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb) return 0;

	return 1;
}

int PCollison(Particle a, Particle b)
{
	//.e = scale
	//.u = rotation
	//.c = pos
	float ra, rb;
	glm::mat3x3 R, absR;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = glm::dot(a.getRotate()[i], b.getRotate()[j]);
	glm::vec3 t = b.getPos() - a.getPos();
	t = glm::vec3(glm::dot(t, a.getRotate()[0]), glm::dot(t, a.getRotate()[1]), glm::dot(t, a.getRotate()[2]));
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			absR[i][j] = abs(R[i][j]) + 0.001f;
	for (int i = 0; i < 3; i++) {
		ra = a.getScale()[i][i];
		rb = b.getScale()[0][0] * absR[0][i] + a.getScale()[1][1] * absR[1][i];
		if (abs(t[i]) > ra + rb) return 0;
	}
	for (int i = 0; i < 3; i++) {
		ra = a.getScale()[0][0] * absR[0][i] + a.getScale()[1][1] * absR[1][i] + a.getScale()[2][2] * absR[2][i];
		rb = b.getScale()[i][i];
		if (abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] + R[2][i]) > ra + rb) return 0;
	}
	ra = a.getScale()[1][1] * absR[2][0] + a.getScale()[2][2] * absR[1][0];
	rb = b.getScale()[1][1] * absR[0][2] + b.getScale()[2][2] * absR[0][1];
	if (abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb)  return 0;

	ra = a.getScale()[1][1] * absR[2][1] + a.getScale()[2][2] * absR[1][1];
	rb = b.getScale()[0][0] * absR[0][2] + b.getScale()[2][2] * absR[0][0];
	if (abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb) return 0;

	ra = a.getScale()[1][1] * absR[2][2] + a.getScale()[2][2] * absR[1][2];
	rb = b.getScale()[0][0] * absR[0][1] + b.getScale()[1][1] * absR[0][0];
	if (abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[2][0] + a.getScale()[2][2] * absR[0][0];
	rb = b.getScale()[1][1] * absR[1][2] + b.getScale()[2][2] * absR[1][1];
	if (abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[2][1] + a.getScale()[2][2] * absR[0][1];
	rb = b.getScale()[0][0] * absR[1][2] + b.getScale()[1][1] * absR[1][0];
	if (abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[2][2] + a.getScale()[2][2] * absR[0][2];
	rb = b.getScale()[0][0] * absR[1][1] + b.getScale()[1][1] * absR[1][0];
	if (abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[1][0] + a.getScale()[1][1] * absR[0][0];
	rb = b.getScale()[1][1] * absR[2][2] + b.getScale()[2][2] * absR[2][1];
	if (abs(t[2] * R[0][0] - t[0] * R[1][0]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[1][1] + a.getScale()[1][1] * absR[0][1];
	rb = b.getScale()[0][0] * absR[2][2] + b.getScale()[2][2] * absR[2][0];
	if (abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb) return 0;

	ra = a.getScale()[0][0] * absR[1][2] + a.getScale()[1][1] * absR[0][2];
	rb = b.getScale()[0][0] * absR[2][1] + b.getScale()[1][1] * absR[2][0];
	if (abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb) return 0;

	return 1;
}
void closestpointOBB(Vertex p, RigidBody b, Vertex &q)
{
	glm::vec3 d = p.getCoord() - b.getPos();
	q = b.getPos();

	for (int i = 0; i < 3; i++) {
		float dist = glm::dot(d, b.getRotate()[i]);
		if (dist > b.getScale()[i][i]) dist = b.getScale()[i][i];
		if (dist < b.getScale()[i][i]) dist = -b.getScale()[i][i];

		q.getCoord() += dist * b.getRotate()[i];

	}
}
float sqdistpointOBB(Vertex p, RigidBody b)
{
	Vertex closest;
	closestpointOBB(p, b, closest);
	float sqDist = glm::dot(closest.getCoord() - p.getCoord(), closest.getCoord() - p.getCoord());
	return sqDist;
}
void Collide(glm::vec3 corner, glm::vec3 wall, Particle &particle)
{
	for (int i = 0; i < 3; i++)
	{
		if (particle.getPos()[i] < corner[i])
		{
			//set the particle in line with wall
			particle.setPos(i, corner[i]);
			//makes ball go opposite direction
			particle.setVel(i, particle.getVel()[i] *= -1.0f);
		}
		if (particle.getPos()[i] < corner[i])
		{
			//set the particle in line with wall
			particle.setPos(i, corner[i]);
			//makes ball go opposite direction
			particle.setVel(i, particle.getVel()[i] *= -1.0f);
		}
		if (particle.getPos()[i] > corner[i] + wall[i])
		{
			//same as above
			particle.setPos(i, corner[i] + wall[i]);
			particle.setVel(i, particle.getVel()[i] *= -1.0f);
		}
	}
}
/*void Collision_Detect(glm::vec3 corner, glm::vec3 wall, Particle &particle)
{
		for (int i = 0; i < 3; i++)
		{
			if (particle.getPos()[i] < corner[i])
			{
				particle.setPos(i, corner[i]);
				particle.setVel(i, particle.getVel()[i] * -0.5f);
			}
			if (particle.getPos()[i] > corner[i] + wall[i])
			{
				particle.setPos(i, corner[i] + wall[i]);
			particle.setVel(i, particle.getVel()[i] * -0.5f);
			}
		}
}
*/
// time
//GLfloat dtime = 0.0f;
GLfloat lastFrame = 0.0f;

// main function
int main()
{

	// create application
	Application app = Application::Application();
	app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 5.0f, 20.0f));
	
	// create ground plane
	Mesh plane = Mesh::Mesh();
	// scale it up x5
		plane.scale(glm::vec3(25.0f, 0.0f, 25.0f));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));

	Particle particle1 = Particle::Particle();
	//particle1.translate(glm::vec3(-10005.0f, -10.0f, -10000.0f));
	particle1.scale(glm::vec3(4.1f, 4.1f, 4.1f));
	particle1.rotate((GLfloat)M_PI_2, glm::vec3(0.0f, 2.0f, 0.0f));
	particle1.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_odd.frag"));
	particle1.setPos(glm::vec3(-5.0f, 5.0f, 1.0f));
	particle1.translate(glm::vec3(-10.0f, 999995.0f, -91.0f));
	particle1.setVel(glm::vec3(1.0f, 2.0f, 0.0f));

	Particle particle2 = Particle::Particle();
	particle2.scale(glm::vec3(4.1f, 4.1f, 4.1f));
	particle2.rotate((GLfloat)M_PI_2, glm::vec3(0.0f, 2.0f, 0.0f));
	particle2.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	particle2.setPos(glm::vec3(5.0f, 5.0f, 1.0f));
	particle2.translate(glm::vec3(-5.0f, 999995.0f, -91.0f));
	particle2.setVel(glm::vec3(1.0f, 2.0f, 0.0f));

	glm::vec3 corner = glm::vec3(-2.5, 0.0f, 2.5f);
	glm::vec3 wall = glm::vec3(5.0f, 5.0f, 5.0f);
	glm::vec3 force = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 gra = glm::vec3(0.0f, -9.8f, 0.0f);
	glm::vec3 horizont = glm::vec3(1.5f, 0.0f, 0.0f);
	
	// Set up a cubic rigid bodys.
	RigidBody ridgid = RigidBody();
	Mesh m = Mesh::Mesh(Mesh::CUBE);
	ridgid.setMesh(m);
	Shader ridgidShader = Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag");
	ridgid.getMesh().setShader(ridgidShader);

	RigidBody ridgid2 = RigidBody();
	Mesh m2 = Mesh::Mesh(Mesh::CUBE);
	ridgid2.setMesh(m);
	Shader ridgidShader2 = Shader("resources/shaders/core.vert", "resources/shaders/core_red.frag");
	ridgid2.getMesh().setShader(ridgidShader2);

	RigidBody ridgid3 = RigidBody();
	Mesh m3 = Mesh::Mesh(Mesh::CUBE);
	ridgid3.setMesh(m);
	Shader ridgidShader3 = Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag");
	ridgid3.getMesh().setShader(ridgidShader3);
	
	ridgid.translate(glm::vec3(0.5f, 0.0f, 2.0f));
	ridgid.scale(glm::vec3(1.0f, 3.0f, 1.0f));
	ridgid.setAngVel(glm::vec3(0.0f, 0.0f, -1.5f));

	ridgid2.translate(glm::vec3(3.5f, 0.0f, 2.0f));
	ridgid2.scale(glm::vec3(1.0f, 3.0f, 1.0f));

	ridgid3.translate(glm::vec3(9.5f, 0.0f, 2.0f));
	ridgid3.scale(glm::vec3(1.0f, 3.0f, 1.0f));
	//mass and gravity stuff
	NegGravity ng = NegGravity();
	Gravity g = Gravity();
	ridgid.setMass(2.0f);
	ridgid.addForce(&g);
	ridgid2.setMass(2.0f);
	ridgid2.addForce(&g);
	ridgid3.setMass(2.0f);
	ridgid3.addForce(&g);
	int firstCollision = 1;
	int secondCollision = 0;
	
	
	
	// time

	double t = 0.0f;
	const double dtime = 0.01f;
	float current_time = (GLfloat)glfwGetTime();
	double accumalator = 0.0f;
	bool invInertiaShowed = false;
	//glm::vec3 impulse = glm::vec3(-4.0f, 0.0f, 0.0f);
	bool impulseapply = false;
	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{
		/*
		if (current_time > 2.0f && impulseapply == false)
		{
			glm::vec3 application = ridgid.getPos() + glm::vec3(1.0f, -1.0f, 0.0f);
			ridgid.setVel(ridgid.getVel() + impulse / ridgid.getMass());
			impulseapply = true;
			//ridgid.setVel(ridgid.getVel() + impulse / ridgid.getMass());
			ridgid.setAngVel(ridgid.getAngVel() + ridgid.getInvInertia() * cross(application - ridgid.getPos(), impulse));
		}
		*/
		//
double new_time = (GLfloat)glfwGetTime();
double frame = new_time - current_time;
current_time = new_time;

accumalator += frame;


//**	INTERACTION
//*/
// Manage interaction
app.doMovement(dtime);
//	SIMULATION
while (accumalator >= dtime)
{

	//seting up basic movements
	particle1.setAcc(gra);
	particle1.setVel(particle1.getVel() + dtime*particle1.getAcc());
	glm::vec3 movep = dtime*particle1.getVel();
	particle1.translate(movep);

	particle2.setAcc(gra);
	particle2.setVel(particle2.getVel() + dtime*particle2.getAcc());
	glm::vec3 movep1 = dtime*particle2.getVel();
	particle2.translate(movep1);

	ridgid.setAcc(ridgid.applyForces(ridgid.getPos(), ridgid.getVel(), t, dtime));
	ridgid.setVel(ridgid.getVel() + dtime*ridgid.getAcc());
	glm::vec3 move = dtime * ridgid.getVel();
	ridgid.translate(move);

	ridgid2.setAcc(ridgid2.applyForces(ridgid2.getPos(), ridgid2.getVel(), t, dtime));
	ridgid2.setVel(ridgid2.getVel() + dtime*ridgid2.getAcc());
	glm::vec3 move2 = dtime * ridgid2.getVel();
	ridgid2.translate(move2);


	ridgid3.setAcc(ridgid3.applyForces(ridgid3.getPos(), ridgid3.getVel(), t, dtime));
	ridgid3.setVel(ridgid3.getVel() + dtime*ridgid3.getAcc());
	glm::vec3 move3 = dtime * ridgid3.getVel();
	ridgid3.translate(move3);
	accumalator -= dtime;
	//integration for individula bodys
	// integration ( rotation ) - 3D
	ridgid.setAngVel(ridgid.getAngVel() + dtime * ridgid.getAngAcc());
	// create skew symmetric matrix for w
	glm::mat3 angVelSkew = glm::matrixCross3(ridgid.getAngVel());
	// create 3x3 rotation matrix from rb rotation matrix
	glm::mat3 R = glm::mat3(ridgid.getRotate());
	// update rotation matrix
	R += dtime * angVelSkew *R;
	R = glm::orthonormalize(R);
	ridgid.getMesh().setRotate(glm::mat4(R));

	// integration ( rotation ) - 3D
	ridgid2.setAngVel(ridgid2.getAngVel() + dtime * ridgid2.getAngAcc());
	// create skew symmetric matrix for w
	glm::mat3 angVelSkew2 = glm::matrixCross3(ridgid2.getAngVel());
	// create 3x3 rotation matrix from rb rotation matrix
	glm::mat3 R2 = glm::mat3(ridgid2.getRotate());
	// update rotation matrix
	R2 += dtime * angVelSkew2 *R2;
	R2 = glm::orthonormalize(R2);
	ridgid2.getMesh().setRotate(glm::mat4(R2));

	ridgid3.setAngVel(ridgid3.getAngVel() + dtime * ridgid3.getAngAcc());
	// create skew symmetric matrix for w
	glm::mat3 angVelSkew3 = glm::matrixCross3(ridgid3.getAngVel());
	// create 3x3 rotation matrix from rb rotation matrix
	glm::mat3 R3 = glm::mat3(ridgid3.getRotate());
	// update rotation matrix
	R3 += dtime * angVelSkew2 *R3;
	R3 = glm::orthonormalize(R3);
	ridgid3.getMesh().setRotate(glm::mat4(R3));

}
//keypress actions
if (glfwGetKey(app.getWindow(), GLFW_KEY_UP))
{
	ridgid.addForce(&ng);
	ridgid2.addForce(&ng);
}
if (glfwGetKey(app.getWindow(), GLFW_KEY_DOWN))
{
	ridgid.addForce(&g);
	ridgid2.addForce(&g);
}
//collision detection functions for different bodys
		std::vector<Vertex> collidingVertices4 = Collision_Detect(particle1.getPos()[1], ridgid);
		bool collisionDetected4 = collidingVertices4.size() > 0;


		std::vector<Vertex> collidingVertices2 = Collision_Detect(plane.getPos()[1], ridgid2);
		bool collisionDetected2 = collidingVertices2.size() > 0;

		std::vector<Vertex> collidingVertices3 = Collision_Detect(ridgid.getPos()[1], ridgid2);
		bool collisionDetected3 = collidingVertices2.size() > 0;
		//allowing particles to bounce off each other
		if (PCollison(particle2, particle1))
		{
			particle1.setVel(glm::vec3(-1.0));
			particle2.setVel(glm::vec3(0.0f, 0.0f, -1.0f));
		}

		if (Collison(ridgid, ridgid2))
		{
			/*
			getchar();
			*/
			std::vector<Vertex> listofsmallest;
			float shortest = 100.0f;
			//for first body
			for (Vertex v : ridgid.getMesh().getVertices())
			{
				v = Vertex(glm::mat3(ridgid.getMesh().getModel()) * v.getCoord() + ridgid.getPos());
				float closestcollisionpoint = sqdistpointOBB(v, ridgid2);
				if (closestcollisionpoint < shortest)
				{
					listofsmallest.clear();

					shortest = closestcollisionpoint;
				}
				if (closestcollisionpoint = shortest)
				{
					listofsmallest.push_back(v);
				}
			}
			//for second body
			for (Vertex v : ridgid2.getMesh().getVertices())
			{
				v = Vertex(glm::mat3(ridgid2.getMesh().getModel()) * v.getCoord() + ridgid2.getPos());
				float closestcollisionpoint = sqdistpointOBB(v, ridgid);
				if (closestcollisionpoint < shortest)
				{
					listofsmallest.clear();

					shortest = closestcollisionpoint;
				}
				if (closestcollisionpoint = shortest)
				{
					listofsmallest.push_back(v);
				}
			}
			//working out impulse and appoint
			glm::vec3 sumofvert;
			for (Vertex v : listofsmallest)
			{
				sumofvert += v.getCoord();
			}

			glm::vec3 apppoint = sumofvert / listofsmallest.size();

			glm::vec3 r = apppoint - ridgid2.getPos();
			glm::vec3 vr = ridgid2.getVel() + cross(ridgid2.getAngVel(), r);
			glm::vec3 n = glm::vec3(0.0f, 1.0f, 0.0f);
			float e = 0.1f;
			glm::vec3 impulse = (-(1 + e)* vr * n) / (pow(ridgid2.getMass(), -1) + n * cross(ridgid2.getInvInertia()* cross(r, n), r));

			ridgid2.setVel(ridgid.getVel() + (impulse / ridgid.getMass()));
			ridgid2.setAngVel(ridgid.getAngVel() + ridgid.getInvInertia() * cross(r, impulse));

			/*
			ridgid.setAngAccl(glm::vec3(0.0f));
			ridgid.setAngVel(glm::vec3(0.0f));
			/*
			ridgid2.setVel(ridgid.getVel() + (horizont / ridgid.getMass()));
			ridgid2.setAngVel(glm::vec3(0.0f, 0.0f, -1.5f));
			ridgid2.setAcc(gra);
			/*
			if (collisionDetected3)
			{
				ridgid2.setAcc(glm::vec3(0.0f));
				ridgid2.setAngAccl(glm::vec3(0.0f)); 
				ridgid2.setAngVel(glm::vec3(0.0f));
			}
			*/

		}
			
		
//particle1.setAcc(g);
			/*//if particle1.getpos = pasrticle2.getpos reverse the velocity
			for (int i = 0; i < list.size(); i++)
			{
				list[i].setAcc(list[i].applyForces(list[i].getPos(), list[i].getVel(), t, dtime));

				list[i].setVel(list[i].getVel() + dtime*list[i].getAcc());

					glm::vec3 move = dtime*list[i].getVel();

				list[i].translate(move);


			}
				accumalator -= dtime;
				*/
		//}
		/*
		// integration (rotation)
		glm::vec3 dRot = ridgid.getAngVel() * dtime;
		if (glm::dot(dRot, dRot) > 0) {
			ridgid.rotate(sqrt(glm::dot(dRot, dRot)), dRot);

		}
		*/
		//////////////////////////////////////////////////////////////////////t = Vector(Dot(t, a.u[0]), Dot(t, a.u[1]), Dot(t, a.u[2]));
		

		
		std::vector<Vertex> collidingVertices = Collision_Detect(plane.getPos()[1], ridgid);
		bool collisionDetected = collidingVertices.size() > 0;

		std::vector<Vertex> collidingVertices5 = Collision_Detect(plane.getPos()[1], ridgid3);
		bool collisionDetected5 = collidingVertices5.size() > 0;
		
		
		if (collisionDetected2)
		{
			Vertex lowVert = collidingVertices2[0].getCoord();
			for (Vertex v : collidingVertices2)
			{
				if (v.getCoord().y < lowVert.getCoord().y)
				{
					lowVert = v;
				}
			}
			glm::vec3 displacement = glm::vec3(0.0f);
			displacement.y = abs(lowVert.getCoord().y);
			ridgid2.translate(displacement);
			glm::vec3 sumOfVertices;
			for (Vertex v : collidingVertices2)
			{
				sumOfVertices += v.getCoord();
			}

			Vertex average = Vertex(sumOfVertices / collidingVertices2.size());
			glm::vec3 r = average.getCoord() - ridgid2.getPos();
			glm::vec3 vr = ridgid2.getVel() + cross(ridgid2.getAngVel(), r);
			glm::vec3 n = normalize(glm::vec3(0.0f, 1.0f, 0.0f));
			float e = 0.2f;

			glm::vec3 j = (-(1 + e) * vr * n) / (pow(ridgid2.getMass(), -1) + n * cross(ridgid2.getInvInertia()* cross(r, n), r));

			ridgid2.setVel(ridgid2.getVel() + j / ridgid2.getMass());
			ridgid2.setAngVel(ridgid2.getAngVel() + ridgid2.getInvInertia() * glm::cross(r, j));
		}
		if (collisionDetected5)
		{
			Vertex lowVert = collidingVertices5[0].getCoord();
			for (Vertex v : collidingVertices5)
			{
				if (v.getCoord().y < lowVert.getCoord().y)
				{
					lowVert = v;
				}
			}
			glm::vec3 displacement = glm::vec3(0.0f);
			displacement.y = abs(lowVert.getCoord().y);
			ridgid3.translate(displacement);
			glm::vec3 sumOfVertices;
			for (Vertex v : collidingVertices5)
			{
				sumOfVertices += v.getCoord();
			}

			Vertex average = Vertex(sumOfVertices / collidingVertices2.size());
			glm::vec3 r = average.getCoord() - ridgid3.getPos();
			glm::vec3 vr = ridgid3.getVel() + cross(ridgid3.getAngVel(), r);
			glm::vec3 n = normalize(glm::vec3(0.0f, 1.0f, 0.0f));
			float e = 0.2f;

			glm::vec3 j = (-(1 + e) * vr * n) / (pow(ridgid3.getMass(), -1) + n * cross(ridgid3.getInvInertia()* cross(r, n), r));

			ridgid3.setVel(ridgid3.getVel() + j / ridgid3.getMass());
			ridgid3.setAngVel(ridgid3.getAngVel() + ridgid3.getInvInertia() * glm::cross(r, j));
		}
		//colision ( rigid falls)
		if (collisionDetected)
		{
			Vertex lowVert = collidingVertices[0].getCoord();
			for (Vertex v : collidingVertices)
			{
				if (v.getCoord().y < lowVert.getCoord().y)
				{
					lowVert = v;
				}

			}
			glm::vec3 displacement = glm::vec3(0.0f);
			displacement.y = abs(lowVert.getCoord().y);
			ridgid.translate(displacement);

			glm::vec3 sumOfVertices;
			for (Vertex v : collidingVertices)
			{
				sumOfVertices += v.getCoord();
			}

			Vertex average = Vertex(sumOfVertices / collidingVertices.size());
			glm::vec3 r = average.getCoord() - ridgid.getPos();
			glm::vec3 vr = ridgid.getVel() + cross(ridgid.getAngVel(), r);
			glm::vec3 n = normalize(glm::vec3(0.0f, 1.0f, 0.0f));
			float e = 0.1f;

			glm::vec3 j = (-(1 + e) * vr * n) / (pow(ridgid.getMass(), -1) + n * cross(ridgid.getInvInertia()* cross(r, n), r));
			ridgid.setVel(ridgid.getVel() + j / ridgid.getMass());
			ridgid.setAngVel(ridgid.getAngVel() + ridgid.getInvInertia() * glm::cross(r, j));
		}




		/*
		for (int i = 0; i < list.size(); i++)
		{
			Collision_Detect(o, d, list[i]);
		}
		*/
		Collide(corner, wall, particle1);
		Collide(corner, wall, particle2);
		/*
		**	RENDER
		*/
		// clear buffer
		app.clear();
		// draw groud plane
		app.draw(plane);
		// draw particles
		//for (int i = 0; i < list.size(); i++)
		//{
		//	app.draw(list[i].getMesh());
		//}
		app.draw(ridgid.getMesh());
		app.draw(ridgid2.getMesh());
		app.draw(ridgid3.getMesh());
		app.draw(particle1.getMesh());
		app.draw(particle2.getMesh());


		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}


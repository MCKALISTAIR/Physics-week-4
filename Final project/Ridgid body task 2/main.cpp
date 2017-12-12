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
	particle1.translate(glm::vec3(0.0f, 2.5f, 0.0f));
	particle1.scale(glm::vec3(4.1f, 4.1f, 4.1f));
	particle1.rotate((GLfloat)M_PI_2, glm::vec3(0.0f, 2.0f, 0.0f));
	particle1.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	particle1.setPos(glm::vec3(0.0f, 5.0f, -2.0f));
	particle1.setVel(glm::vec3(1.0f, 2.0f, 0.0f));

	glm::vec3 corner = glm::vec3(-2.5, 0.0f, 2.5f);
	glm::vec3 wall = glm::vec3(5.0f, 5.0f, 5.0f);
	const int particleNum = 100;
	glm::vec3 force = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 gra = glm::vec3(0.0f, -9.8f, 0.0f);
	/*
	float numberparticles = 10;
	std::vector<Particle> list;
	//initial position for first particle
	glm::vec3 start = glm::vec3(-2.5f, 5.0f, 0.0f);
	float distance = 0.5f;
	//loop to add particles
	for (int i = 0; i < numberparticles; i++)
	{
		Particle particle = Particle::Particle();
		particle.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
		particle.setMass(0.3f);
		if (i == 0)
		{
			///set velocity and position of non moving particle
			particle.setVel(glm::vec3(0.0f));
			particle.setPos(glm::vec3(start));
		}
		else if (i > 0)
		{
			//set velocity and position of particles
			particle.setVel(glm::vec3(0.01f));
			particle.setPos(start + glm::vec3(distance, 0.0f, 0.0f) * i);
		}
		list.push_back(particle);
	}
	*/
	// Set up a cubic rigid body.
	RigidBody ridgid = RigidBody();
	Mesh m = Mesh::Mesh(Mesh::CUBE);
	ridgid.setMesh(m);
	Shader ridgidShader = Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag");
	ridgid.getMesh().setShader(ridgidShader);

	RigidBody ridgid2 = RigidBody();
	Mesh m2 = Mesh::Mesh(Mesh::CUBE);
	ridgid2.setMesh(m);
	ridgid2.getMesh().setShader(ridgidShader);
	
	ridgid.translate(glm::vec3(0.0f, 0.0f, 0.0f));
	ridgid.scale(glm::vec3(1.0f, 3.0f, 1.0f));
	ridgid.setAngVel(glm::vec3(0.0f, 0.0f, -1.5f));

	ridgid2.translate(glm::vec3(3.0f, 4.0f, 0.0f));
	ridgid2.scale(glm::vec3(1.0f, 3.0f, 1.0f));
	/*
	ridgid.translate(glm::vec3(0.0f, 15.0f, 0.0f));
	ridgid.setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	ridgid.setAngVel(glm::vec3(0.3f, 0.6f, 0.8f));
	ridgid.setAngAccl(glm::vec3(0.0f, 0.0f, 0.0f));
	ridgid.scale(glm::vec3(1.0f, 3.0f, 1.0f));
	*/
	Gravity g = Gravity();
	ridgid.setMass(2.0f);
	ridgid.addForce(&g);
	ridgid2.setMass(2.0f);
	ridgid2.addForce(&g);
	int firstCollision = 1;
	int secondCollision = 0;
	/*
	//Apply forces to particls
	for (int i = 0; i < list.size(); i++)
	{
		//k
		if (i > 0 && i < list.size() - 1)
		{
				list[i].addForce(&g);
				list[i].addForce(new Drag());
				list[i].addForce(new Hook(&list[i], &list[i + 1], 10.0f, 0.8f, 0.5f));
				list[i].addForce(new Hook(&list[i], &list[i - 1], 10.0f, 0.8f, 0.5f));
		}
	}
	glm::vec3 o = glm::vec3(-2.5, 0.0f, 2.5f);
	glm::vec3 d = glm::vec3(5.0f, 5.0f, 5.0f);
	*/
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

			particle1.setAcc(gra);
			particle1.setVel(particle1.getVel() + dtime*particle1.getAcc());
			glm::vec3 movep = dtime*particle1.getVel();
			particle1.translate(movep);
			if (!invInertiaShowed)
			{
				std::cout << "Inverse inertia: " << glm::to_string(ridgid.getInvInertia()) << std::endl << std::endl;
				invInertiaShowed = true;
			}
			ridgid.setAcc(ridgid.applyForces(ridgid.getPos(), ridgid.getVel(), t, dtime));
			ridgid.setVel(ridgid.getVel() + dtime*ridgid.getAcc());
			glm::vec3 move = dtime * ridgid.getVel();
			ridgid.translate(move);

			ridgid2.setAcc(ridgid2.applyForces(ridgid2.getPos(), ridgid2.getVel(), t, dtime));
			ridgid2.setVel(ridgid2.getVel() + dtime*ridgid2.getAcc());
			glm::vec3 move2 = dtime * ridgid2.getVel();
			ridgid2.translate(move2);
			accumalator -= dtime;


		}

		if (Collison(ridgid, ridgid2))
		{
			ridgid.setAngAccl(glm::vec3(0.0f));
			ridgid.setAngVel(glm::vec3(0.0f));
			//ridgid2.setAngVel(glm::vec3(0.0f, 0.0f, -1.5f));
		}
			//particle1.setAcc(g);
			/*
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

		std::vector<Vertex> collidingVertices = Collision_Detect(plane.getPos()[1], ridgid);
		bool collisionDetected = collidingVertices.size() > 0;
		
		std::vector<Vertex> collidingVertices2 = Collision_Detect(plane.getPos()[1], ridgid2);
		bool collisionDetected2 = collidingVertices2.size() > 0;
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
			glm::vec3 n = normalize(glm::vec3(1.0f, 0.0f, 0.0f));
			float e = 0.2f;

			glm::vec3 j = (-(1 + e) * vr * n) / (pow(ridgid2.getMass(), -1) + n * cross(ridgid2.getInvInertia()* cross(r, n), r));

			ridgid.setVel(ridgid.getVel() + j / ridgid.getMass());
			ridgid.setAngVel(ridgid.getAngVel() + ridgid.getInvInertia() * glm::cross(r, j));
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
		app.draw(particle1.getMesh());


		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}


#pragma once
#define GLEW_STATIC

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
#include "Particle.h"
#include "Body.h"


const double dtime = 0.001f;
double currentTime = glfwGetTime();
double accumulator = 0.0f;


void Collide(glm::vec3 corner, glm::vec3 wall, Particle &particle)
{
	//for (int i = 0; i < 3; i++)
	//{
	if (particle.getPos()[1] < corner[1])
	{
		//set the particle in line with wall
		particle.setPos(1, corner[1]);
		//makes ball go opposite direction
		particle.setVel(1, particle.getVel()[1] *= -0.5f);
	}
	if (particle.getPos()[1] > corner[1] + wall[1])
	{
		//same as above
		particle.setPos(1, corner[1] + wall[1]);
		particle.setVel(1, particle.getVel()[1] *= -0.5f);
	}
	//}
}
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
	plane.scale(glm::vec3(5.0f, 5.0f, 5.0f));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));

	float numberparticles = 10;
	std::vector<Particle> list;
	glm::vec3 start = glm::vec3(-2.5f, 5.0f, 0.0f);
	float distance = 0.5f;
	for (int i = 0; i < numberparticles; i++)
	{
		Particle particle = Particle::Particle();
		particle.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
		particle.setMass(0.3f);
		if (i == 0)
		{
			particle.setVel(glm::vec3(0.0f));
			particle.setPos(glm::vec3(start));
		}
		else if (i > 0 && i < numberparticles -1)
		{
			particle.setVel(glm::vec3(0.01f));
			particle.setPos(start + glm::vec3(distance, 0.0f, 0.0f) * i);
		}
		else if (i == numberparticles - 1)
		{
			particle.setVel(glm::vec3(0.0f));
			particle.setPos(start + glm::vec3(distance, 0.0f, 0.0f) * i);
		}
			list.push_back(particle);
	}



	// create particle's
	
	Gravity g = Gravity();
for (int i = 0; i < list.size(); i++)
	{
	
		if (i > 0 && i < list.size() - 1)
		{
			list[i].addForce(&g);
			list[i].addForce(new Drag());
			list[i].addForce(new Hook(&list[i], &list[i - 1], 5.0f, 0.8f, 0.5f));
			list[i].addForce(new Hook(&list[i], &list[i + 1], 5.0f, 0.8f, 0.5f));
		}
	
	}
//dimensions of cube
glm::vec3 corner = glm::vec3(-2.5, 0.0f, 2.5f);
glm::vec3 wall = glm::vec3(5.0f, 5.0f, 5.0f);
const int particleNum = 100;
glm::vec3 force = glm::vec3(0.0f, 0.0f, 0.0f);
// time
GLfloat firstFrame = (GLfloat)glfwGetTime();
// Game loop
while (!glfwWindowShouldClose(app.getWindow()))
{
	// Set frame time
	//// Set frame time
	double newTime = glfwGetTime();
	double frameTime = newTime - currentTime;
	currentTime = newTime;
	accumulator += frameTime;
	// the animation can be sped up or slowed down by multiplying currentFrame by a factor.
	//	INTERACTION
	// Manage interaction
	app.doMovement(dtime);

	while (accumulator > dtime)
	{
		//for loop
		// list
		for (int i = 0; i < list.size(); i++)
		{
			list[i].setAcc(list[i].applyForces(list[i].getPos(), list[i].getVel(), currentTime, dtime));
			list[i].setVel(list[i].getVel() + dtime*list[i].getAcc());
			glm::vec3 move = dtime*list[i].getVel();
			list[i].translate(move);

		}
		//add particle to list

		/*
		**	SIMULATION
		*/
		/*

	
	/*
	for (int i = 0; i < 3; i++)
	{
		if (particle1.getTranslate()[3][i] < o[i])
		{
			v[i] = v[i] * -1.0f;
		}
		if (particle1.getTranslate()[3][i] > o[i] + d[i])
		{
			v[i] =v[i] * -1.0f;
		}
	}
	*/

		accumulator -= dtime;

	}
	for (int i = 0; i < list.size(); i++)
		Collide(corner, wall, list[i]);

	//Collide(corner, wall, particle1);
	//Collide(corner, wall, particle2);
	//particle1.setPos();
	/*
	**	RENDER
	*/
	// clear buffer
	app.clear();
	// draw groud plane
	app.draw(plane);
	// draw particles
	for (int i = 0; i < list.size(); i++)
		{
		app.draw(list[i].getMesh());
}
		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}


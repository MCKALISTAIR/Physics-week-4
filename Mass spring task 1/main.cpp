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
//Method for Collision Detection
void Collision(glm::vec3 c, glm::vec3 w, Particle &particle)
{
	for (int i = 0; i < 3; i++)
	{
		//if the particle goes outside the floor of the cube
		if (particle.getPos()[i] < c[i])
		{
			//set the position level with the cube
			particle.setPos(i, c[i]);
			//reverse the velocity
			particle.setVel(i, particle.getVel()[i] *= -0.4f);
		}
		if (particle.getPos()[i] > c[i] + w[i])
		{
			//set the position level with the cube
			particle.setPos(i, c[i] + w[i]);
			//reverse the velocity
			particle.setVel(i, particle.getVel()[i] *= -0.4f);
		}
	}
}

// main function
int main()
{
	// create application
	Application app = Application::Application();
	app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 5.0f, 20.0f));
	float particlenum = 5;
	float distance = -0.5f;
	float spring = 40.0f;
	std::vector<Particle> parts;
	glm::vec3 start = glm::vec3(0.0f, 10.0f, 0.0f);
	for (int i = 0; i < particlenum; i++)
	{
		Particle particle = Particle::Particle();
		particle.setMass(0.3f);
		particle.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
		if (i > 0)
		{
			particle.setVel(glm::vec3(0.01f));
			particle.setPos(start + glm::vec3(0.0f, distance, 0.0f)* i);
		}
		else
	{
			particle.setVel(glm::vec3(0.0f));
			particle.setPos(start);
	}
		parts.push_back(particle);
	}
	Mesh plane = Mesh::Mesh();
	// scale it up x5
	plane.scale(glm::vec3(5.0f, 5.0f, 5.0f));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));

	Gravity g = Gravity();
	//add forces..

	for (int i = 0; i < parts.size(); i++)
	{
		if (i == parts.size() - 1)
		{
			parts[i].addForce(&g);
			parts[i].addForce(new Drag());
			parts[i].addForce(new Hook(&parts[i], &parts[i - 1], spring, 0.8f, 0.5f));
		}
		else if (i > 0)
		{
			parts[i].addForce(&g);
			parts[i].addForce(new Drag());
			parts[i].addForce(new Hook(&parts[i], &parts[i - 1], spring, 0.5f, 0.5f));
			parts[i].addForce(new Hook(&parts[i], &parts[i + 1], spring, 0.5f, 0.5f));
		}
	}

	glm::vec3 c = glm::vec3(-2.5f, 0.0f, -2.5f);

	glm::vec3 gravity = glm::vec3(0.0f, -9.8f, 0.0f);
	glm::vec3 w = glm::vec3(5.0f, 5.0f, 5.0f);
	// time
	

	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{

		double newTime = glfwGetTime();
		double frameTime = newTime - currentTime;
		currentTime = newTime;
		accumulator += frameTime;
		/*
		**	INTERACTION
		*/
		// Manage interaction
		app.doMovement(dtime);

		while (accumulator > dtime)
		{
			/*
			**	SIMULATION
			*/
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
			for (int i = 0; i < parts.size(); i++)
			{
				parts[i].setAcc(parts[i].applyForces(parts[i].getPos(), parts[i].getVel(), currentTime, dtime));
				parts[i].setVel(parts[i].getVel() + dtime*parts[i].getAcc());
				glm::vec3 move = dtime*parts[i].getVel();
				parts[i].translate(move);
			}
			accumulator -= dtime;
		}
		// COLLISION DETECTION
		for (int i = 0; i < parts.size(); i++)
			Collision(c, w, parts[i]);
		/*
		**	RENDER
		*/
		// clear buffer
		app.clear();
		// draw groud plane
		app.draw(plane);
		// draw particles
		for (int i = 0; i < parts.size(); i++)
			app.draw(parts[i].getMesh());
		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}


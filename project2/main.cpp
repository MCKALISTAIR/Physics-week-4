#pragma once
#define GLEW_STATIC

// Math constants
#define _USE_MATH_DEFINES
#include <cmath>  
#include <random>

// Std. Includes
#include <string>
#include <time.h>

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


const double dtime = 0.01;
double currentTime = glfwGetTime();
double accumulator = 0.0f;


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
	
	
	
	// create particle's
	
	Particle particle1 = Particle::Particle();
	Particle particle2 = Particle::Particle();
	Particle particle3 = Particle::Particle();
	Particle particle4 = Particle::Particle();
	//scale it down (x.1), translate it up by 2.5 and rotate it by 90 degrees around the x axis
	particle1.translate(glm::vec3(0.0f, 2.5f, 0.0f));
	particle1.scale(glm::vec3(4.1f, 4.1f, 4.1f));
	particle1.rotate((GLfloat) M_PI_2, glm::vec3(0.0f, 2.0f, 0.0f));
	particle1.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	
	particle2.scale(glm::vec3(5.0f, 5.0f, 5.0f));
	particle2.translate(glm::vec3(0.0f, 2.0f, 0.0f));
	particle2.rotate((GLfloat)M_PI_2, glm::vec3(1.0f, 0.0f, 0.0f));
	particle2.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	

	particle3.scale(glm::vec3(5.0f, 5.0f, 5.0f));
	particle3.translate(glm::vec3(2.0f, 2.0f, 0.0f));
	particle3.rotate((GLfloat)M_PI_2, glm::vec3(1.0f, 0.0f, 0.0f));
	particle3.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));

	particle4.scale(glm::vec3(5.0f, 5.0f, 5.0f));
	particle4.translate(glm::vec3(-2.0f, 2.0f, 0.0f));
	particle4.rotate((GLfloat)M_PI_2, glm::vec3(1.0f, 0.0f, 0.0f));
	particle4.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	//Add vectors for particle movement
	/*
	glm::vec3 p = glm::vec3(0.0f, 5.0f, 0.0f);
	glm::vec3 v = glm::vec3(1.0f, 2.0f, 0.0f);
	glm::vec3 acc = glm::vec3(0.0f, 0.0f, 0.0f);
	*/
	 particle1.setPos(glm::vec3(0.0f, 5.0f, -2.0f));
	 particle1.setVel(glm::vec3(1.0f, 2.0f, 0.0f));
	 Gravity g = Gravity(glm::vec3(0.0f, -9.8f, 0.0f));
	 particle1.addForce(&g);
	//dimensions of cube
	glm::vec3 corner = glm::vec3(-2.5, 0.0f, 2.5f);
	glm::vec3 wall = glm::vec3(5.0f, 5.0f, 5.0f);
	const int particleNum = 100;
	glm::vec3 force = glm::vec3(0.0f, 0.0f, 0.0f);
	// time
	GLfloat firstFrame = (GLfloat) glfwGetTime();
	//float mass = 2.0f;
	/*
	glm::vec3 p0 = glm::vec3(0.0f, 5.0f, 0.0f);
	glm::vec3 p1 = glm::vec3(2.0f, 5.0f, 0.0f);
	glm::vec3 v0 = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 a = glm::vec3(0.0f, -9.8f, 0.0f);
	*/
	float currentFrames[particleNum];
	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{
		// Set frame time
		///GLfloat currentFrame = (GLfloat)  glfwGetTime() - firstFrame;
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
		/*
		**	SIMULATION
		*/
			/*
		force = mass * g;
		acc = force / mass;
		v = v + (dtime * acc);
		p = p + (dtime * v);
		*/
		//particle3.setPos(p1 + v0 * currentTime + 0.5f * a * currentTime * currentTime);
		
		//if (particle2.getTranslate()[3][1] < 0.0f) {
			//p0 = particle2.getTranslate()[3];
			//p0[1] = 0.0f;
			//v0 = v0 + a * currentTime;
			//v0[1] = -1.0f * v0[1];
			//firstFrame = glfwGetTime();
		//}
		//particle2.setPos(p0 + v0 * currentTime + 0.5f * a * currentTime * currentTime);
		//particle1.setPos(p);
		
		//particle1.setAcc(g);
		particle1.setAcc(particle1.applyForces(particle1.getPos(), particle1.getVel(), currentTime, dtime));
		particle1.setVel(particle1.getVel() + dtime*particle1.getAcc());
		glm::vec3 move = dtime*particle1.getVel();
		particle1.translate(move);
		
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

		Collide(corner, wall, particle1);
		//particle1.setPos();
		/*
		**	RENDER 
		*/		
		// clear buffer
		app.clear();
		// draw groud plane
		app.draw(plane);
		// draw particles
		app.draw(particle1.getMesh());				
		app.draw(particle2.getMesh());
		app.draw(particle3.getMesh());
		app.draw(particle4.getMesh());
		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}


#pragma once
#include "body.h"

class ridgidbody2 :
	public Body
{
public:
	ridgidbody2();
	~ridgidbody2();

	// set and get methods
	void setAngVel(const glm::vec3 &omega) { m_angVel = omega; }
	void setAngAccl(const glm::vec3 &alpha) { m_angAcc = alpha; }
	void setInvInertia(const glm::mat3 &invInertia) { m_invInertia = invInertia; }
	void setMass(float mass) { m_mass = mass; updateInvInertia(); }

	glm::vec3 getAngVel() { return m_angVel; }
	glm::vec3 getAngAcc() { return m_angAcc; }
	glm::mat3 getInvInertia() { return m_invInertia; }
	void scale(glm::vec3 vect);
	void updateInvInertia();

private:
	float m_density;
	glm::mat3 m_invInertia; // Inverse Inertia
	glm::vec3 m_angVel; // angular velocity
	glm::vec3 m_angAcc; // angular acceleration
};


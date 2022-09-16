
#pragma once

// glm
#include <glm/glm.hpp>

// project
#include "scene.hpp"


class Boid {
private:

	glm::vec3 m_position;
	glm::vec3 m_velocity;
	glm::vec3 m_acceleration;
	
	//color
	glm::vec3 m_color;
	//float radius
	float m_radius;

public:
	//normal boid
	Boid(glm::vec3 pos, glm::vec3 dir) : m_position(pos), m_velocity(dir) { }
	//preditor boid
	Boid(glm::vec3 pos, glm::vec3 dir, bool pred, glm::vec3 c, bool s) : m_position(pos), m_velocity(dir), preditor(pred), m_color(c),sphear(s) {}
	
	//get radius
	float radius() { return m_radius * 4; }


	glm::vec3 position() const { return m_position; }
	glm::vec3 velocity() const { return m_velocity; }
	glm::vec3 acceleration() const { return m_acceleration; }

	glm::vec3 color() const;

	void calculateForces(Scene *scene);
	void update(float timestep, Scene *scene);
	
	bool preditor;
	bool sphear;
		

};
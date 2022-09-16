
// glm
#include <glm/gtc/random.hpp>

// project
#include "boid.hpp"
#include "scene.hpp"
#include "cgra/cgra_mesh.hpp"

#include <glm/gtc/matrix_transform.hpp>

using namespace glm;
using namespace std;


vec3 Boid::color() const {
	return m_color;
}



void Boid::calculateForces(Scene* scene) {
	//-------------------------------------------------------------
	// [Assignment 3] :
	// Calculate the forces affecting the boid and update the
	// acceleration (assuming mass = 1).
	// Do NOT update velocity or position in this function.
	// Core : 
	//  - Cohesion
	//  - Alignment
	//  - Avoidance
	//  - Soft Bound (optional)
	// Completion : 
	//  - Cohesion and Alignment with only boids in the same flock
	//  - Predator Avoidance (boids only)
	//  - Predator Chase (predator only)
	// Challenge : 
	//  - Obstacle avoidance
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...

	//this boids local bo

	//calculate the m_valocity by calculingting the forces
	//cohesion

	std::vector<Boid> boids = scene->boids();
	//average position and average velocity
	vec3 averagePosition = { 0,0,0 };
	vec3 averageVelocity = { 0,0,0 };

	glm::vec3 cohesion = { 0,0, 0 };
	glm::vec3 alignment = { 0,0, 0 };
	glm::vec3 avoidance = { 0,0, 0 };

	
	glm::vec3 predAvoid = { 0,0, 0 };
	glm::vec3 predChase = { 0,0, 0 };
	
	glm::vec3 sphereAvoid = { 0,0, 0 };
	//closest sphere
	float closestSphere = -1;
	
	


	int cohesionNeighours = 0;
	int alignmentNeighours = 0;
	int predNeighours = 0;
	int closetPray = 999;



	

	//calculate cohesion
	for (int i = 0; i < boids.size(); i++) {
		glm::vec3 vecDis = m_position - boids[i].m_position;
		float dis = length(vecDis);
		//if the boid is not this boid
		if (boids[i].m_position != m_position) {
			//if the boid is in the same flock
			if (boids[i].color() == color()) {
				//if the boid is in the radius
				if (distance(boids[i].position(), m_position) < scene->local) {
					//add the position to the average position
					averagePosition += boids[i].position();
					//add the velocity to the average velocity
					averageVelocity += boids[i].velocity();
					//increase the number of neighbours
					cohesionNeighours++;
					alignmentNeighours++;
					if (boids[i].preditor) { predNeighours++; }
				}
				//if the boid is in the alignment radius
				if (distance(boids[i].position(), m_position) < scene->local) {
					//add the velocity to the average velocity
					averageVelocity += boids[i].velocity();
					//increase the number of neighbours
					alignmentNeighours++;
				}
				//avoidence radius

				if (boids.at(i).position() != m_position && glm::length(m_position - boids.at(i).position()) < scene->local && boids.at(i).color() != glm::vec3{ 0.5,0.5,0.5 }) {
					avoidance += (m_position - boids.at(i).position()) / (glm::length(m_position - boids.at(i).position()) * glm::length(m_position - boids.at(i).position()));
				}

			}

			//avoid
			if (boids.at(i).position() != m_position && dis < scene->localPred && boids.at(i).preditor) {
				predAvoid += (m_position - boids.at(i).position()) / (glm::length(m_position - boids.at(i).position()) * glm::length(m_position - boids.at(i).position()));
			}

			//chase
			if (boids.at(i).position() != m_position && dis < scene->localPred && dis < closetPray) {
				predChase = vecDis / (dis * dis);
				closetPray = dis;
			}
			//boids avoid sphears
			if (boids.at(i).position() != m_position && dis < scene->localPred && boids.at(i).sphear) {
				sphereAvoid += (m_position - boids.at(i).position()) / (glm::length(m_position - boids.at(i).position()) * glm::length(m_position - boids.at(i).position()));
				sphereAvoid *= 4;
			}
		}
	}

	//avoid sphears
	if (closestSphere != -1) {
		sphereAvoid = (m_position - scene->boids()[closestSphere].position()) / (glm::length(m_position - scene->boids()[closestSphere].position()) * glm::length(m_position - scene->boids()[closestSphere].position()));
		sphereAvoid *= 2;
	}

	
	
	//calculate the average position and velocity
	if (cohesionNeighours != 0) {
		averagePosition /= cohesionNeighours;
		cohesion = (averagePosition - m_position);
	}
	if (alignmentNeighours != 0) {
		averageVelocity /= alignmentNeighours;
		alignment = (averageVelocity - m_velocity);
	}
	
	

	

	//calculate new acc
	if (this->preditor == false) {
		// cap accelaration

		

		
		m_acceleration = (avoidance * scene->avoid) + (cohesion * scene->coher) + (alignment * scene->align) + (predAvoid * scene->predAvoid) + (sphereAvoid * scene->boidSohereAvoid);

		if (glm::length(cohesion) > scene->maxAcc) {
			cohesion = glm::normalize(cohesion) * scene->maxAcc;
		}
		if (glm::length(alignment) < scene->maxAcc) {
			alignment = glm::normalize(alignment) * scene->minAcc;
		}

		if (glm::length(m_velocity) > scene->maxS) {
			m_velocity = glm::normalize(m_velocity) * scene->maxS;
		}
		//if the boid is moving slower than the min speed set the velocity to the min speed
		if (glm::length(m_velocity) < scene->minS) {
			m_velocity = glm::normalize(m_velocity) * scene->minS;
		}

		

	}
	else if(preditor) {
		m_acceleration = vec3(0) - (predChase * scene->predChace) + (sphereAvoid * scene->boidSohereAvoid);

			if (glm::length(cohesion) > scene->predMaxAcc) {
				cohesion = glm::normalize(cohesion) * scene->predMaxAcc;
			}
			if (glm::length(alignment) < scene->predMinAcc) {
				alignment = glm::normalize(alignment) * scene->predMinAcc;
			}
			
			if (glm::length(m_velocity) > scene->predMaxS) {
				m_velocity = glm::normalize(m_velocity) * scene->predMaxS;
			}
			//if the boid is moving slower than the min speed set the velocity to the min speed
			if (glm::length(m_velocity) < scene->predMinS) {
				m_velocity = glm::normalize(m_velocity) * scene->predMinS;
			}
	}
	//if the boid is moving faster than the max speed set the velocity to the max speed




	//calculate predd acc


	




	
	
	
}


void Boid::update(float timestep, Scene* scene) {
	//-------------------------------------------------------------
	// [Assignment 3] :
	// Integrate the velocity of the boid using the timestep.
	// Update the position of the boid using the new velocity.
	// Take into account the bounds of the scene which may
	// require you to change the velocity (if bouncing) or
	// change the position (if wrapping).
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...

	//update the position of the boid using the new velocity

	glm::vec3 m_boundingbox = scene->bound();

	//hard bound just outside the bounding box
	if (m_position.x > m_boundingbox.x + 50) {
		m_position.x = m_boundingbox.x + 50;
		m_velocity.x = -m_velocity.x;
	}

	if (m_position.x < -m_boundingbox.x - 50) {
		m_position.x = -m_boundingbox.x - 50;
		m_velocity.x = -m_velocity.x;
	}

	if (m_position.y > m_boundingbox.y + 50) {
		m_position.y = m_boundingbox.y + 50;
		m_velocity.y = -m_velocity.y;
	}

	if (m_position.y < -m_boundingbox.y - 50) {
		m_position.y = -m_boundingbox.y - 50;
		m_velocity.y = -m_velocity.y;
	}

	if (m_position.z > m_boundingbox.z + 50) {
		m_position.z = m_boundingbox.z + 50;
		m_velocity.z = -m_velocity.z;
	}

	if (m_position.z < -m_boundingbox.z - 50) {
		m_position.z = -m_boundingbox.z - 50;
		m_velocity.z = -m_velocity.z;
	}

	



	//soft bound
	if (m_position.y < -m_boundingbox.y + 1) {
		m_velocity.y += 1;
	}
	
	if (m_position.y > m_boundingbox.y - 1) {
		m_velocity.y -= 1;
	}
	
	if (m_position.x < -m_boundingbox.x + 1) {
		m_velocity.x += 1;
	}
	
	if (m_position.x > m_boundingbox.x - 1) {
		m_velocity.x -= 1;
	}

	if (m_position.z < -m_boundingbox.z + 1) {
		m_velocity.z += 1;
	}

	if (m_position.z > m_boundingbox.z - 1) {
		m_velocity.z -= 1;
	}

	//update the position of the boid using the new velocity
	m_velocity += m_acceleration * timestep;
	m_position += m_velocity * timestep;
	
}



#ifndef _RIGIDBODY_STRUCT_H
#define _RIGIDBODY_STRUCT_H

#include "GLIncludes.h"


//Struct for rigidbody kinematics
struct RigidBody
{
	float mass;						//We will need to use both mass and inverse mass extensively here.
	float inverseMass;				//I tend to use inverse mass as opposed to mass itself. It saves lots of divides when forces are involved.

	glm::vec3 position;				//Position of the rigidbody
	glm::vec3 velocity;				//The velocity of the rigidbody
	glm::vec3 acceleration;			//The acceleration of the rigidbody

	glm::vec3 netForce;				//Forces over time
	glm::vec3 netImpulse;			//Instantaneous forces

									///
									//Default constructor, created rigidbody with all properties set to zero
	RigidBody::RigidBody()
	{
		mass = inverseMass = 1.0f;

		position = glm::vec3(0.0f, 0.0f, 0.0f);
		velocity = glm::vec3(0.0f, 0.0f, 0.0f);
		acceleration = glm::vec3(0.0f, 0.0f, 0.0f);

		netForce = glm::vec3(0.0f);
		netImpulse = glm::vec3(0.0f);
	}

	///
	//Parameterized constructor, creates rigidbody with specified initial values
	//
	//Parameters:
	//	pos: Initial position
	//	vel: Initial velocity
	//	acc: Initial acceleration
	//	mass: The mass of the rigidbody (0.0f for infinite mass)
	RigidBody::RigidBody(glm::vec3 pos, glm::vec3 vel, glm::vec3 acc, float m)
	{
		mass = m;
		inverseMass = m == 0.0f ? 0.0f : 1.0f / m;

		position = pos;
		velocity = vel;
		acceleration = acc;

		netForce = glm::vec3(0.0f);
		netImpulse = glm::vec3(0.0f);
	}
};


#endif _RIGIDBODY_STRUCT_H
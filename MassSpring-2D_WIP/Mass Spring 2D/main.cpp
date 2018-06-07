/*
Title: Mass Spring Softbody (2D)
File Name: main.cpp
Copyright © 2018
Original authors: Nicholas Gallagher
Refactored by: Benjamin Evans
Written under the supervision of David I. Schwartz, Ph.D., and
supported by a professional development seed grant from the B. Thomas
Golisano College of Computing & Information Sciences
(https://www.rit.edu/gccis) at the Rochester Institute of Technology.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Description:
This is a demonstration of using mass spring systems to simulate soft body physics.
The demo contains a blue cloth made of a 10x10 grid of masses with springs connecting them.

Each physics timestep the mass spring system is solved to determine the force on each
individual point mass in the system. This is done using Hooke's law. The springs also contain 
dampening forces to help relax the system upon purterbation.

The user can apply forces to the bottom edge of the cloth.
Hold the left mouse button to apply a force along the positive X axis.
Hold the right mouse button to apply a force along the negative X axis.
Hold Left shift to switch the axis to the Y axis.

It should be noted that this is a straightforward example without attempts to optimize.
It is best if an algorithm like this is done on the GPU, however even on the CPU this
is a very inefficient way to implement the algorithm. Please see the "Mass Spring Softbody (2D Fast)"
example for some tips on speeding this up even when running on the CPU.

References:
Game Physics by David Eberly
NGenVS by Nicholas Gallagher
PhysicsTimestep by Brockton Roth
Base by Srinivasan Thiagarajan
*/







struct Mesh* lattice;

struct SoftBody* body;

//glm::vec3 gravity(0.0f, -0.98f, 0.0f);

double time = 0.0;
double timebase = 0.0;
double accumulator = 0.0;
double physicsStep = 0.012; // This is the number of milliseconds we intend for the physics to update.

#pragma endregion Base_data								  

// Functions called only once every time the program is executed.
#pragma region Helper_functions

//Read shader source
std::string readShader(std::string fileName)
{
	std::string shaderCode;
	std::string line;

	std::ifstream file(fileName, std::ios::in);
	if (!file.good())
	{
		std::cout << "Can't read file: " << fileName.data() << std::endl;
		return "";
	}

	file.seekg(0, std::ios::end);
	shaderCode.resize((unsigned int)file.tellg());
	file.seekg(0, std::ios::beg);

	file.read(&shaderCode[0], shaderCode.size());

	file.close();

	return shaderCode;
}

//Creates shader program
GLuint createShader(std::string sourceCode, GLenum shaderType)
{
	GLuint shader = glCreateShader(shaderType);
	const char *shader_code_ptr = sourceCode.c_str(); 
	const int shader_code_size = sourceCode.size();

	glShaderSource(shader, 1, &shader_code_ptr, &shader_code_size);
	glCompileShader(shader);

	GLint isCompiled = 0;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &isCompiled);

	if (isCompiled == GL_FALSE)
	{
		char infolog[1024];
		glGetShaderInfoLog(shader, 1024, NULL, infolog);
		std::cout << "The shader failed to compile with the error:" << std::endl << infolog << std::endl;

		glDeleteShader(shader);
	}

	return shader;
}

// Initialization code
void init()
{
	// Initializes the glew library
	glewInit();
	glEnable(GL_DEPTH_TEST);

	// Create shaders
	std::string vertShader = readShader("../VertexShader.glsl");
	std::string fragShader = readShader("../FragmentShader.glsl");

	vertex_shader = createShader(vertShader, GL_VERTEX_SHADER);
	fragment_shader = createShader(fragShader, GL_FRAGMENT_SHADER);

	program = glCreateProgram();
	glAttachShader(program, vertex_shader);
	glAttachShader(program, fragment_shader);
	glLinkProgram(program);


	//Generate the View Projection matrix
	glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 2.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::mat4 proj = glm::ortho(-1.0f,1.0f, -1.0f, 1.0f, 0.1f, 100.0f);
	VP = proj * view;

	//Create uniforms
	uniMVP = glGetUniformLocation(program, "MVP");
	uniHue = glGetUniformLocation(program, "hue");

	// Set options
	glFrontFace(GL_CCW);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
}


#pragma endregion Helper_functions

///
//Performs second order euler integration for linear motion
//
//Parameters:
//	dt: The timestep
//	body: The rigidbody being integrated
void IntegrateLinear(float dt, RigidBody &body)
{
	//Calculate the current acceleration
	body.acceleration = body.inverseMass * body.netForce;

	//Calculate new position with
	//	X = X0 + V0*dt + (1/2) * A * dt^2
	glm::vec3 v0dT = dt * body.velocity;						//Solve for the first degree term
	glm::vec3 aT2 = (0.5f) * body.acceleration * powf(dt, 2);	//Solve for the second degree term
	body.position += v0dT + aT2;								//Take sum

	//determine the new velocity
	body.velocity += dt * body.acceleration + body.inverseMass * body.netImpulse;

	//Zero the net impulse and net force!
	body.netForce = body.netImpulse = glm::vec3(0.0f);
}

// This runs once every physics timestep.
void update(float dt)
{	

	//This is the external force we will apply based on which keys are pressed.
	glm::vec3 externalForce = glm::vec3(0.0f);


	if(glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
	{
		if(glfwGetMouseButton(window, 0) == GLFW_PRESS)
		{
			externalForce.y = 2.0f;
		}
		if(glfwGetMouseButton(window, 1) == GLFW_PRESS)
		{
			externalForce.y = -2.0f;
		}
	}
	else
	{
		if(glfwGetMouseButton(window, 0) == GLFW_PRESS)
		{
			externalForce.x = 2.0f;
		}
		if(glfwGetMouseButton(window, 1) == GLFW_PRESS)
		{
			externalForce.x = -2.0f;
		}
	}

	glm::vec3 displacement;	//The displacement between nodes
	glm::vec3 direction;	//The direction of the displacement
	float mag;				//The magnitude of the dispplacement

	//Apply forces to each rigidbody making up the softbody
	for(int i = 0; i < body->subdivisionsY; i++)
	{

		for(int j = 0; j < body->subdivisionsX; ++j)
		{

			//If there is a rigidbody above this one, calculate the spring force between this rigidbody and the rigidbody above
			if(i > 0)
			{
				//Get displacement from rigidBody[i-1] to rigidBody[i]
				displacement = body->bodies[i - 1][j].position - body->bodies[i][j].position;
				//Extract the direction and the magnitude from this displacement
				direction = glm::normalize(displacement);
				mag = glm::length(displacement);

				//Calculate and Add the applied force according the Hooke's law
				//Fspring = -k(dX)
				//And from that we must add the dampening force:
				//Fdamp = -V * C 
				//Where C is the dampening constant
				body->bodies[i][j].netForce += body->coefficient * (mag - body->restHeight) * direction - body->bodies[i][j].velocity * body->dampening;
			}

			//If there is a rigidbody below this one, calculate the spring force between this rigidbody and the one below
			if(i < body->subdivisionsY - 1)
			{
				displacement = body->bodies[i + 1][j].position - body->bodies[i][j].position;
				direction = glm::normalize(displacement);
				mag = glm::length(displacement);
				body->bodies[i][j].netForce += body->coefficient * (mag - body->restHeight) * direction - body->bodies[i][j].velocity * body->dampening;
			}

			//If there is a rigidbody left of this one, calculate the spring force between this rigidbody and the one below
			if(j > 0)
			{
				displacement = body->bodies[i][j - 1].position - body->bodies[i][j].position;
				direction = glm::normalize(displacement);
				mag = glm::length(displacement);
				body->bodies[i][j].netForce += body->coefficient * (mag - body->restWidth) * direction - body->bodies[i][j].velocity * body->dampening;
			}

			//If there is a rigidbody right of this one, calculate the spring force between this rigidbody and the one below
			if(j < body->subdivisionsX - 1)
			{
				displacement = body->bodies[i][j + 1].position - body->bodies[i][j].position;
				direction = glm::normalize(displacement);
				mag = glm::length(displacement);
				body->bodies[i][j].netForce += body->coefficient * (mag - body->restWidth) * direction - body->bodies[i][j].velocity * body->dampening;
			}

			//If the vertex is on the bottom row, apply the external force
			if(i == 0)
				body->bodies[i][j].netForce += externalForce;
		}
	}


	//Update the rigidbodies & mesh
	//For each rigidbody
	for(int i = 0; i < body->subdivisionsY; ++i)
	{
		for(int j = 0; j < body->subdivisionsX;++j)
		{
			int numVertex = i * body->subdivisionsY + j;
			//Integrate kinematics
			IntegrateLinear(dt, body->bodies[i][j]);
			//And change the mesh's vertices to match this rigidbodies position
			lattice->vertices[numVertex].x = body->bodies[i][j].position.x;
			lattice->vertices[numVertex].y = body->bodies[i][j].position.y;
			lattice->vertices[numVertex].z = body->bodies[i][j].position.z;
		}
	}

}

// This runs once every frame to determine the FPS and how often to call update based on the physics step.
void checkTime()
{
	// Get the current time.
	time = glfwGetTime();

	// Get the time since we last ran an update.
	double dt = time - timebase;

	// If more time has passed than our physics timestep.
	if (dt > physicsStep)
	{

		timebase = time; // set new last updated time

		// Limit dt
		if (dt > 0.25)
		{
			dt = 0.25;
		}
		accumulator += dt;

		// Update physics necessary amount
		while (accumulator >= physicsStep)
		{
			update(physicsStep);
			accumulator -= physicsStep;
		}
	}
}



// This function runs every frame
void renderScene()
{
	// Clear the color buffer and the depth buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Clear the screen to white
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glLineWidth(1.0f);

	// Tell OpenGL to use the shader program you've created.
	glUseProgram(program);

	//Set hue uniform
	glUniformMatrix4fv(uniHue, 1, GL_FALSE, glm::value_ptr(hue));

	//Refresh the rope vertices
	lattice->RefreshData();
	// Draw the Gameobjects
	lattice->Draw();
}

#pragma endregion util_Functions



void main()
{
	glfwInit();

	// Create a window
	window = glfwCreateWindow(800, 800, "Mass Spring Softbody (2D)", nullptr, nullptr);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(0);

	// Initializes most things needed before the main loop
	init();

	//Generate the rope mesh
	float latticeArr[10 * 10 * (sizeof(struct Vertex) / sizeof(float))];
	for(int i = 0; i < 10; i++)
	{
		for(int j = 0; j < 10; j++)
		{
			int offset = sizeof(struct Vertex) / sizeof(float);
			int index = (i * 10 + j) * offset;
			Vertex* v = (Vertex*)(latticeArr + index);
			v->x = (1.0f / 10.0f) * (float)j;
			v->y = (1.0f / 10.0f) * (float)i;
			v->z = 0.0f;

			v->r = 0.0f;
			v->g = 1.0f;
			v->b = 1.0f;
			v->a = 1.0f;
		}
	}

	Vertex *latticeVerts = (Vertex*)latticeArr;

	const int numIndices = 9 * 9 * 4;
	GLuint latticeElems[numIndices];
	for(int i = 0; i < 9; i++)
	{
		for(int j = 0; j < 9; j++)
		{
			int element = (i * 10 + j) * 4 - 4 * i;
			latticeElems[element] = (i * 10) + j;
			latticeElems[element + 1] = (i * 10) + (j + 1);
			latticeElems[element + 2] = ((i + 1) * 10) + (j + 1);
			latticeElems[element + 3] = ((i + 1) * 10) + j;
		}
	}

	//lattice creation
	lattice = new struct Mesh(10 * 10, latticeVerts, numIndices, latticeElems, GL_QUADS);

	//Scale the lattice
	lattice->scale = glm::scale(lattice->scale, glm::vec3(1.0f));

	//Set spring constant, rest length, and dampening constant
	float coeff = 25.0f;
	float damp = 0.5f;

	//Generate the softbody
	body = new SoftBody(1.0f, 1.0f, 10, 10, coeff, damp);

	//Print controls
	printf("Controls:\nPress and hold the left mouse button to cause a positive constant force\n along the selected axis.\n");
	printf("Press and hold the right mouse button to cause a negative constant force\n along the selected axis.\n");
	printf("The selected axis by default is the X axis\n");
	printf("Hold Left Shift to change the selected axis to the Y axis\n");
	

	// Enter the main loop.
	while (!glfwWindowShouldClose(window))
	{
		//Check time will update the programs clock and determine if & how many times the physics must be updated
		checkTime();

		// Call the render function.
		renderScene();

		// Swaps the back buffer to the front buffer
		// Remember, you're rendering to the back buffer, then once rendering is complete, you're moving the back buffer to the front so it can be displayed.
		glfwSwapBuffers(window);

		// Checks to see if any events are pending and then processes them.
		glfwPollEvents();
	}

	// After the program is over, cleanup your data!
	glDeleteShader(vertex_shader);
	glDeleteShader(fragment_shader);
	glDeleteProgram(program);
	// Note: If at any point you stop using a "program" or shaders, you should free the data up then and there.

	delete lattice;
	delete body;


	// Frees up GLFW memory
	glfwTerminate();
}
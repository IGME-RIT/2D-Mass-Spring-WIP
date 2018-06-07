#ifndef _MESH_STRUCT_H
#define _MESH_STRUCT_H


#include "GLRender.h"


//Struct for rendering
struct Mesh
{
	GLuint VBO;
	GLuint EBO;
	GLuint VAO;
	glm::mat4 translation;
	glm::mat4 rotation;
	glm::mat4 scale;
	int numVertices;
	int numIndices;
	struct Vertex* vertices;
	GLuint* indices;
	GLenum primitive;

	Mesh::Mesh(int numVert, struct Vertex* vert, int numInd, GLuint* inds, GLenum primType)
	{

		glm::mat4 translation = glm::mat4(1.0f);
		glm::mat4 rotation = glm::mat4(1.0f);
		glm::mat4 scale = glm::mat4(1.0f);

		this->numVertices = numVert;
		this->vertices = new struct Vertex[this->numVertices];
		memcpy(this->vertices, vert, this->numVertices * sizeof(struct Vertex));

		this->numIndices = numInd;
		this->indices = new GLuint[this->numIndices];
		memcpy(this->indices, inds, this->numIndices * sizeof(GLuint));

		this->primitive = primType;

		//Generate VAO
		glGenVertexArrays(1, &this->VAO);
		//bind VAO
		glBindVertexArray(VAO);

		//Generate VBO & EBO
		//We must use an element buffer here so that we do not need to worry about duplicate vertices
		//while we are repositioning the vertices of the mesh
		glGenBuffers(1, &this->VBO);
		glGenBuffers(1, &this->EBO);

		//Configure VBO & EBO
		glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->EBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(struct Vertex) * this->numVertices, this->vertices, GL_DYNAMIC_DRAW);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * this->numIndices, this->indices, GL_STATIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(struct Vertex), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(struct Vertex), (void*)12);
	}

	Mesh::~Mesh(void)
	{
		delete[] this->vertices;
		glDeleteVertexArrays(1, &this->VAO);
		glDeleteBuffers(1, &this->VBO);
	}

	glm::mat4 Mesh::GetModelMatrix()
	{
		return translation * rotation * scale;
	}

	void Mesh::RefreshData(void)
	{
		//BEcause we are changing the vertices themselves and not transforming them
		//We must write the new vertices over the old on the GPU.
		glBindVertexArray(VAO);

		GLvoid* memory = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		memcpy(memory, vertices, numVertices * sizeof(Vertex));
		glUnmapBuffer(GL_ARRAY_BUFFER);
	}

	void Mesh::Draw(void)
	{
		//GEnerate the MVP for this model
		glm::mat4 MVP = VP * this->GetModelMatrix();

		//Bind the VAO being drawn
		glBindVertexArray(this->VAO);

		// Set the uniform matrix in our shader to our MVP matrix for this mesh.
		glUniformMatrix4fv(uniMVP, 1, GL_FALSE, glm::value_ptr(MVP));
		//Draw the mesh
		//glDrawArrays(this->primitive, 0, this->numVertices);
		glDrawElements(this->primitive, this->numIndices, GL_UNSIGNED_INT, 0);
	}
};

#endif _MESH_STRUCT_H
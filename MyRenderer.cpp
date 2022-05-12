#pragma once
#include "MyRenderer.hpp"
#include "MyRendererUtils.hpp"

#include <QDebug>
#include <QImage>
#include <QMouseEvent>
#include <QWheelEvent>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <unordered_map>
#include <utility>
#include <type_traits>

#include <cmath>

static const bool RENDER_PARTICLES = false;
static const bool RENDER_SLICES = true;
static const bool RENDER_DEBUG = false;
static const bool RENDER_OBJECT_SHADOWS = true;

static const std::string defaultFileName = "models/testscene.obj";
static const std::string smokePath = "models/smoke.bin";


void MyRenderer::openScene(const std::string& fileName) {
	uint currIndexCount;
	gl::Buffer* currVertexBuffer = new gl::Buffer();
	gl::Buffer* currIndexBuffer = new gl::Buffer();
	gl::VertexArray* currVAO = new gl::VertexArray();
	gl::Program* currProgram = new gl::Program();
	bool currHasTexture;

	std::string currentFileName = fileName;
	int currentMesh = 0;
	numObjectsInScene = 0;

	bool successful = loadMesh(currentFileName, currentMesh, *currVAO, *currVertexBuffer, *currIndexBuffer, currIndexCount, *currProgram, currHasTexture);

	while (!successful) {
		qDebug() << "Could not open file or file did not contain an Object!";
		currentFileName = QFileDialog::getOpenFileName(Q_NULLPTR, "Open Scene File", "", "Wavefront OBJ (*.obj)").toStdString();
		successful = loadMesh(currentFileName, currentMesh, *currVAO, *currVertexBuffer, *currIndexBuffer, currIndexCount, *currProgram, currHasTexture);
	}

	while (successful) {
		sceneIndexCounts.push_back(currIndexCount);
		sceneVertexBuffers.push_back(currVertexBuffer);
		sceneIndexBuffers.push_back(currIndexBuffer);
		sceneVAOs.push_back(currVAO);
		scenePrograms.push_back(currProgram);
		sceneHasTexture.push_back(currHasTexture);
		numObjectsInScene++;

		currIndexCount = 0;
		currVertexBuffer = new gl::Buffer();
		currIndexBuffer = new gl::Buffer();
		currVAO = new gl::VertexArray();
		currProgram = new gl::Program();

		currentMesh++;
		successful = loadMesh(currentFileName, currentMesh, *currVAO, *currVertexBuffer, *currIndexBuffer, currIndexCount, *currProgram, currHasTexture);
	}
}

//Compute the Frustum planes of the Smoke bounding Box from the camera's view
//Putting all Smoke Rendering Slices within these Bounds will reduce Artifacts and unnecessary rendering of "empty" Slices
void MyRenderer::computeSmokePlanes(Eigen::Matrix4d view) {
	float maxX = 0.0f;
	float minX = 100.0f;
	float maxY = 0.0f;
	float minY = 100.0f;
	float maxZ = 0.0f;
	float minZ = 100.0f;
	for (int i = 0; i < 8; i++) {
		Eigen::Vector4d point(smokeBoundingBox[3 * i + 0], smokeBoundingBox[3 * i + 1], smokeBoundingBox[3 * i + 2], 1.0);
		point = view * point;
		maxX = fmax(maxX, point.x());
		minX = fmin(minX, point.x());
		maxY = fmax(maxY, point.y());
		minY = fmin(minY, point.y());
		maxZ = fmax(maxZ, point.z());
		minZ = fmin(minZ, point.z());
	}
	maxZ = fmin(maxZ, 0.0);
	smokeNearPlane = maxZ;
	smokeFarPlane = -minZ;
	smokeLeftPlane = minX;
	smokeRightPlane = maxX;
	smokeBottomPlane = minY;
	smokeTopPlane = maxY;
}

MyRenderer::MyRenderer(QObject* parent)
	: OpenGLRenderer{ parent }
{
	{
		//Load Scene Meshes
		openScene(defaultFileName);

		//Load Smoke Data
		loadSmokeData(smokePath, smokeData, smokeDims, smokeBoundingBox);

		//Swap x and z axis of test smoke data
		{
			std::vector<float> newSmokeData;
			for (int x = 0; x < smokeDims[0]; x++) {
				for (int y = 0; y < smokeDims[1]; y++) {
					for (int z = 0; z < smokeDims[2]; z++) {
						int pos = x + y * (int)smokeDims[0] + z * (int)smokeDims[0] * (int)smokeDims[1];
						newSmokeData.push_back(smokeData[pos]);
					}
				}
			}
			smokeData.swap(newSmokeData);
			size_t temp = smokeDims[0];
			smokeDims[0] = smokeDims[2];
			smokeDims[2] = temp;

			smokeBoundingBox = createSmokeBoundingBox(smokeDims);
		}

		//Setup Smoke Particle Rendering
		if (RENDER_PARTICLES) {

			//Initialize Smoke Particle VAO
			{
				glBindVertexArray(smokePartVAO.id());
				//glBindBuffer(GL_ARRAY_BUFFER, smokePartVertexBuffer.bufferId());
				//glBufferData(GL_ARRAY_BUFFER, smokePartVertices.size() * sizeof(float), smokePartVertices.data(), GL_STATIC_DRAW);
				glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
				glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (GLvoid*)(3 * sizeof(float)));
				glEnableVertexAttribArray(0);
				glEnableVertexAttribArray(1);
				glBindVertexArray(0);
				glCheckError();
			}

			//Initialize Smoke Particle Shader Program
			{
				gl::Shader vertexShader{ GL_VERTEX_SHADER };
				gl::Shader fragmentShader{ GL_FRAGMENT_SHADER };

				std::vector<char> vsText;
				std::vector<char> fsText;

				vsText = loadResource("shaders/smokeParticle.vert");
				fsText = loadResource("shaders/smokeParticle.frag");

				vertexShader.compile(vsText.data(), static_cast<GLint>(vsText.size()));
				fragmentShader.compile(fsText.data(), static_cast<GLint>(fsText.size()));

				if (!smokePartProgram.link(vertexShader, fragmentShader))
				{
					qDebug() << "Shader compilation failed:\n" << smokePartProgram.infoLog().get();
					std::abort();
				}

				GLuint pid = smokePartProgram.id();
				glBindAttribLocation(pid, 0, "aPos");
				glBindAttribLocation(pid, 1, "aDensity");
				glCheckError();
			}
		}

		//Setup Smoke Slice Rendering
		if (RENDER_SLICES) {

			//Create Slice Planes
			createSmokeRenderingPlanes(smokeSliceVertices, smokeSliceIndices);

			//Initialize Smoke Slice VAO
			{
				glBindVertexArray(smokeSliceVAO.id());

				glBindBuffer(GL_ARRAY_BUFFER, smokeSliceVertexBuffer.id());
				glBufferData(GL_ARRAY_BUFFER, smokeSliceVertices.size() * sizeof(float), smokeSliceVertices.data(), GL_STATIC_DRAW);
				glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
				glEnableVertexAttribArray(0);

				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, smokeSliceIndexBuffer.id());
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, smokeSliceIndices.size() * sizeof(uint), smokeSliceIndices.data(), GL_STATIC_DRAW);

				glBindVertexArray(0);
				glCheckError();
				qDebug() << "Smoke Slices have" << smokeSliceVertices.size() << "Vertex Entries and" << smokeSliceIndices.size() << "Index Entries!";
			}

			//Initialize Smoke Slice Shader Program
			{
				gl::Shader vertexShader{ GL_VERTEX_SHADER };
				gl::Shader fragmentShader{ GL_FRAGMENT_SHADER };

				std::vector<char> vsText;
				std::vector<char> fsText;

				vsText = loadResource("shaders/smokeSlice.vert");
				fsText = loadResource("shaders/smokeSlice.frag");

				vertexShader.compile(vsText.data(), static_cast<GLint>(vsText.size()));
				fragmentShader.compile(fsText.data(), static_cast<GLint>(fsText.size()));

				if (!smokeSliceProgram.link(vertexShader, fragmentShader))
				{
					qDebug() << "Shader compilation failed:\n" << smokeSliceProgram.infoLog().get();
					std::abort();
				}
				glCheckError();
			}
		}

		//Setup Debug Quad
		if (RENDER_DEBUG) {
			//Initialize VAO
			{
				glBindVertexArray(debugVAO.id());
				glBindBuffer(GL_ARRAY_BUFFER, debugVertexBuffer.id());
				glBufferData(GL_ARRAY_BUFFER, 4 * 5 * sizeof(float), planeVertices, GL_STATIC_DRAW);
				glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), nullptr);
				glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (GLvoid*)(3 * sizeof(float)));
				glEnableVertexAttribArray(0);
				glEnableVertexAttribArray(1);

				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, debugIndexBuffer.id());
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(uint), planeIndices, GL_STATIC_DRAW);

				glBindVertexArray(0);
				glCheckError();
			}

			//Initialize Shader Program
			{
				gl::Shader vertexShader{ GL_VERTEX_SHADER };
				gl::Shader fragmentShader{ GL_FRAGMENT_SHADER };

				std::vector<char> vsText;
				std::vector<char> fsText;

				vsText = loadResource("shaders/debug.vert");
				fsText = loadResource("shaders/debug.frag");

				vertexShader.compile(vsText.data(), static_cast<GLint>(vsText.size()));
				fragmentShader.compile(fsText.data(), static_cast<GLint>(fsText.size()));

				if (!debugQuadProgram.link(vertexShader, fragmentShader))
				{
					qDebug() << "Shader compilation failed:\n" << debugQuadProgram.infoLog().get();
					std::abort();
				}
				glCheckError();
			}
		}

		//Initialize Smoke Data 3D Texture
		{
			glBindTexture(GL_TEXTURE_3D, smokeDataTexture.id());
			glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, (int)smokeDims[0], (int)smokeDims[1], (int)smokeDims[2], 0, GL_RED, GL_FLOAT, smokeData.data());

			float borderColor[] = { 0.0f, 0.0f, 0.0f, 0.0f };
			glTexParameterfv(GL_TEXTURE_3D, GL_TEXTURE_BORDER_COLOR, borderColor);
			glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

			glCheckError();
		}

		//Initialize Object Texture
		{
			auto img = QImage(":/textures/test.png").convertToFormat(QImage::Format_RGBA8888).mirrored();
			glBindTexture(GL_TEXTURE_2D, testTexture.id());
			glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, img.width(), img.height(), 0, GL_RGBA, GL_UNSIGNED_INT_8_8_8_8_REV, img.constBits());

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

			glCheckError();
		}

		//Initialize Deep Shadow Map Texture
		{
			glBindTexture(GL_TEXTURE_2D_ARRAY, deepShadowTexture.id());
			glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RG16F, DEEPSHADOWMAP_SIZE, DEEPSHADOWMAP_SIZE, 8, 0, GL_RG, GL_HALF_FLOAT, NULL);

			float borderColor[] = { -10.0f, 1.0f, 1.0f, 0.0f };
			glTexParameterfv(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_BORDER_COLOR, borderColor);
			glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

			glBindImageTexture(2, deepShadowTexture.id(), 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RG16F);
			glCheckError();
		}

		//Initialize Depth Map Texture and FBO
		{
			glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO.id());

			glBindTexture(GL_TEXTURE_2D, depthTexture.id());

			glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, SHADOWMAP_SIZE, SHADOWMAP_SIZE, 0, GL_RED, GL_FLOAT, NULL);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glBindTexture(GL_TEXTURE_2D, 0);

			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, depthTexture.id(), 0);

			unsigned int rbo;
			glGenRenderbuffers(1, &rbo);
			glBindRenderbuffer(GL_RENDERBUFFER, rbo);

			glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, SHADOWMAP_SIZE, SHADOWMAP_SIZE);
			glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
			glBindRenderbuffer(GL_RENDERBUFFER, 0);

			glCheckError();
			glBindFramebuffer(GL_FRAMEBUFFER, 0);

		}

		//Initialize Depth Shader Program
		{
			gl::Shader vertexShader{ GL_VERTEX_SHADER };
			gl::Shader fragmentShader{ GL_FRAGMENT_SHADER };

			std::vector<char> vsText;
			std::vector<char> fsText;

			vsText = loadResource("shaders/depth.vert");
			fsText = loadResource("shaders/depth.frag");

			vertexShader.compile(vsText.data(), static_cast<GLint>(vsText.size()));
			fragmentShader.compile(fsText.data(), static_cast<GLint>(fsText.size()));

			if (!depthProgram.link(vertexShader, fragmentShader))
			{
				qDebug() << "Shader compilation failed:\n" << depthProgram.infoLog().get();
				std::abort();
			}
			glCheckError();
		}

		//Initialize Deep Shadow Map Shader Program
		{
			gl::Shader computeShader{ GL_COMPUTE_SHADER };

			std::vector<char> csText;
			csText = loadResource("shaders/deepShadowMap.comp");
			computeShader.compile(csText.data(), static_cast<GLint>(csText.size()));

			if (!deepShadowProgram.link(computeShader))
			{
				qDebug() << "Shader compilation failed:\n" << deepShadowProgram.infoLog().get();
				std::abort();
			}
			glCheckError();

		}

		//Initialize Smoke Particle Creation Shader Program and Buffer
		{
			int bufferSize = smokeDims[0] * smokeDims[1] * smokeDims[2];
			glBindBuffer(GL_SHADER_STORAGE_BUFFER, smokePartCompBuffer.id());
			glBufferData(GL_SHADER_STORAGE_BUFFER, bufferSize * 4 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
			glCheckError();

			gl::Shader computeShader{ GL_COMPUTE_SHADER };

			std::vector<char> csText;
			csText = loadResource("shaders/particleCreation.comp");
			computeShader.compile(csText.data(), static_cast<GLint>(csText.size()));

			if (!particleCreationProgram.link(computeShader))
			{
				qDebug() << "Shader compilation failed:\n" << particleCreationProgram.infoLog().get();
				std::abort();
			}
			glCheckError();

			glBindAttribLocation(particleCreationProgram.id(), 1, "outBuffer");
		}


	}
	this->timer.start();
}

void MyRenderer::resize(int w, int h)
{
	this->width = w;
	this->height = h;
	// update projection matrix to account for (potentially) changed aspect ratio
	this->projectionMatrix = calculateInfinitePerspective(
		0.78539816339744831, // 45 degrees in radians
		static_cast<double>(w) / h,
		0.01 // near plane (chosen "at random")
	);
}

void MyRenderer::render()
{
	auto currentTimeNS = this->timer.nsecsElapsed();
	auto deltaTimeNS = currentTimeNS - this->lastTimeNS;
	this->lastTimeNS = currentTimeNS;

	int loc = -1;

	//Move the Light
	{
		auto sa = std::sin(lightAzimuth);
		auto ca = std::cos(lightAzimuth);
		auto se = std::sin(lightElevation);
		auto ce = std::cos(lightElevation);
		auto distance = 5.0;
		lightPos[0] = distance * se * ca;
		lightPos[1] = distance * se * sa;
		lightPos[2] = distance * ce;
	}

	//Compute Matrices
	{
		// recompute view matrix (camera position and direction) from azimuth/elevation
		{
			auto sa = std::sin(this->cameraAzimuth);
			auto ca = std::cos(this->cameraAzimuth);
			auto se = std::sin(this->cameraElevation);
			auto ce = std::cos(this->cameraElevation);
			auto distance = this->zoomFactor;

			cameraPos[0] = distance * se * ca;
			cameraPos[1] = distance * se * sa;
			cameraPos[2] = distance * ce;

			this->viewMatrix = calculateLookAtMatrix(
				distance * Eigen::Vector3d{ se * ca, se * sa, ce },
				{ 0, 0, 0 },
				{ 0, 0, 1 }
			);
			inverseViewMatrix = viewMatrix.inverse();
		}

		//Calculate Orthographic Projection Matrix for Light
		lightProjectionMatrix = calculateOrthograficPerspective(2.5, -2.5, 2.5, -2.5, SHADOW_NEAR_FRUST, SHADOW_FAR_FRUST);

		//Calculate View Matrix from Light's perspective
		lightViewMatrix = calculateLookAtMatrix(
			{ lightPos[0], lightPos[1], lightPos[2] },
			{ 0, 0, 0 },
			{ 0.0, 1.0, 0.0 }
		);

		inverseLightViewMatrix = lightViewMatrix.inverse();

	}

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBlendEquation(GL_FUNC_ADD);

	//Clear Screen
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Run Compute Shader to create Deep Shadow Map
	{
		Eigen::Vector3f light = Eigen::Vector3f(lightPos);
		computeSmokePlanes(lightViewMatrix);

		//Calculate Smaller Orthographic Projection for Deep Shadow Map to increase precision
		dsmProjectionMatrix = calculateOrthograficPerspective(smokeRightPlane, smokeLeftPlane, smokeTopPlane, smokeBottomPlane, SHADOW_NEAR_FRUST, SHADOW_FAR_FRUST);

		auto pid = deepShadowProgram.id();
		glUseProgram(pid);
		loc = glGetUniformLocation(pid, "lightPos");
		glUniform3fv(loc, 1, lightPos);
		loc = glGetUniformLocation(pid, "smokeDims");
		glUniform3f(loc, smokeDims[0], smokeDims[1], smokeDims[2]);
		loc = glGetUniformLocation(pid, "shadowFarFrust");
		glUniform1f(loc, SHADOW_FAR_FRUST);
		loc = glGetUniformLocation(pid, "smokeNearPlane");
		glUniform1f(loc, smokeNearPlane);
		loc = glGetUniformLocation(pid, "smokeFarPlane");
		glUniform1f(loc, smokeFarPlane);
		loc = glGetUniformLocation(pid, "inverseLightViewMatrix");
		glUniformMatrix4fv(loc, 1, GL_FALSE, inverseLightViewMatrix.cast<float>().eval().data());
		loc = glGetUniformLocation(pid, "lightViewMatrix");
		glUniformMatrix4fv(loc, 1, GL_FALSE, lightViewMatrix.cast<float>().eval().data());
		loc = glGetUniformLocation(pid, "lightProjectionMatrix");
		glUniformMatrix4fv(loc, 1, GL_FALSE, dsmProjectionMatrix.cast<float>().eval().data());
		loc = glGetUniformLocation(pid, "inverseLightViewProjectionMatrix");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (dsmProjectionMatrix * lightViewMatrix).inverse().cast<float>().eval().data());

		loc = glGetUniformLocation(pid, "smokeData");
		glUniform1i(loc, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_3D, smokeDataTexture.id());
		glCheckError();

		glDispatchCompute(DEEPSHADOWMAP_SIZE / 16, DEEPSHADOWMAP_SIZE / 16, 1);
		glCheckError();
	}

	//Run Compute Shader for Creating Smoke Particles
	{
		//TEST
		//qDebug() << "Cam Pos:" << cameraPos[0] << cameraPos[1] << cameraPos[2];

		auto pid = particleCreationProgram.id();
		glUseProgram(pid);

		loc = glGetUniformLocation(pid, "camPos");
		glUniform3fv(loc, 1, cameraPos);
		loc = glGetUniformLocation(pid, "smokeDims");
		glUniform3f(loc, smokeDims[0], smokeDims[1], smokeDims[2]);

		loc = glGetUniformLocation(pid, "smokeData");
		glUniform1i(loc, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_3D, smokeDataTexture.id());

		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, smokePartCompBuffer.id());
		glCheckError();

		glDispatchCompute(smokeDims[0] / 8, smokeDims[1] / 8, smokeDims[2] / 8);
		glCheckError();
	}

	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);
	glGetIntegerv(GL_VIEWPORT, viewportSize);

	//Render to Depth Map
	{
		//Use the program
		auto pid = this->depthProgram.id();
		glUseProgram(pid);

		//Insert Light Space Conversion matrix into program
		loc = glGetUniformLocation(pid, "lightSpaceMatrix");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (lightProjectionMatrix * lightViewMatrix).cast<float>().eval().data());

		//Resize Viewport to Shadow Map Size
		glViewport(0, 0, SHADOWMAP_SIZE, SHADOWMAP_SIZE);
		glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO.id());
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//Render Scene
		if (RENDER_OBJECT_SHADOWS) {
			for (int i = 0; i < numObjectsInScene; i++) {
				glBindVertexArray(sceneVAOs[i]->id());
				glDrawElements(GL_TRIANGLES, sceneIndexCounts[i], GL_UNSIGNED_INT, nullptr);
				glBindVertexArray(0);
			}
		}
		glCheckError();
		//Cleanup
		glViewport(viewportSize[0], viewportSize[1], viewportSize[2], viewportSize[3]);
		//Instead of the default (0) Framebuffer, we go back to Framebuffer 2 because Qt doesn't use the default one
		glBindFramebuffer(GL_FRAMEBUFFER, 2);
	}


	//Reset the Viewport
	glClearColor(0.21f, 0.74f, 0.95f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Render the Scene
	{
		for (int i = 0; i < numObjectsInScene; i++) {
			glBindVertexArray(sceneVAOs[i]->id());

			auto pid = scenePrograms[i]->id();
			glUseProgram(pid);

			//Insert View/Projection Matrix into Program
			loc = glGetUniformLocation(pid, "modelViewProjection");
			glUniformMatrix4fv(loc, 1, GL_FALSE, (projectionMatrix * viewMatrix).cast<float>().eval().data());
			//Insert Light Space conversion Matrix into Program
			loc = glGetUniformLocation(pid, "lightViewMatrix");
			glUniformMatrix4fv(loc, 1, GL_FALSE, (lightViewMatrix).cast<float>().eval().data());
			loc = glGetUniformLocation(pid, "lightProjectionMatrix");
			glUniformMatrix4fv(loc, 1, GL_FALSE, (lightProjectionMatrix).cast<float>().eval().data());
			loc = glGetUniformLocation(pid, "dsmProjectionMatrix");
			glUniformMatrix4fv(loc, 1, GL_FALSE, (dsmProjectionMatrix).cast<float>().eval().data());

			//Insert Light Position and Color into Program
			loc = glGetUniformLocation(pid, "lightColor");
			glUniform3fv(loc, 1, lightCol);
			loc = glGetUniformLocation(pid, "lightPos");
			glUniform3fv(loc, 1, lightPos);
			//Insert Camera Position into Program
			loc = glGetUniformLocation(pid, "cameraPos");
			glUniform3fv(loc, 1, cameraPos);

			//Insert Textures into Program
			if (sceneHasTexture[i]) {
				//Insert Color Texture
				loc = glGetUniformLocation(pid, "colorTexture");
				glUniform1i(loc, 0);
				glActiveTexture(GL_TEXTURE0);
				glBindTexture(GL_TEXTURE_2D, testTexture.id());

				//Insert Shadow Map
				loc = glGetUniformLocation(pid, "shadowMap");
				glUniform1i(loc, 1);
				glActiveTexture(GL_TEXTURE1);
				glBindTexture(GL_TEXTURE_2D, depthTexture.id());

				//Insert Deep Shadow Map
				loc = glGetUniformLocation(pid, "deepShadowMap");
				glUniform1i(loc, 2);
				glActiveTexture(GL_TEXTURE2);
				glBindTexture(GL_TEXTURE_2D_ARRAY, deepShadowTexture.id());
			}
			else {
				//Insert Shadow Map
				loc = glGetUniformLocation(pid, "shadowMap");
				glUniform1i(loc, 0);
				glActiveTexture(GL_TEXTURE0);
				glBindTexture(GL_TEXTURE_2D, depthTexture.id());

				//Insert Deep Shadow Map
				loc = glGetUniformLocation(pid, "deepShadowMap");
				glUniform1i(loc, 1);
				glActiveTexture(GL_TEXTURE1);
				glBindTexture(GL_TEXTURE_2D_ARRAY, deepShadowTexture.id());
			}

			//Render the Object
			glDrawElements(GL_TRIANGLES, sceneIndexCounts[i], GL_UNSIGNED_INT, nullptr);
			glCheckError();
		}
	}

	//Render the Debug Quad
	if (RENDER_DEBUG) {
		//Use the program
		auto pid = debugQuadProgram.id();
		glUseProgram(pid);

		//Insert the parameters
		loc = glGetUniformLocation(pid, "projection");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (this->projectionMatrix).cast<float>().eval().data());

		//insert the textures
		loc = glGetUniformLocation(pid, "debugTexture");
		glUniform1i(loc, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D_ARRAY, deepShadowTexture.id());

		//Bind VAO
		glBindVertexArray(debugVAO.id());
		//draw the Quad
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

		glBindVertexArray(0);
	}

	//Render the Smoke Slices
	if (RENDER_SLICES) {
		//Compute Near and Far Planes of the Smoke Volume
		computeSmokePlanes(viewMatrix);

		//Use the program
		auto pid = smokeSliceProgram.id();
		glUseProgram(pid);

		//Insert the Parameters
		loc = glGetUniformLocation(pid, "inverseView");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (this->inverseViewMatrix).cast<float>().eval().data());
		loc = glGetUniformLocation(pid, "projection");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (this->projectionMatrix).cast<float>().eval().data());
		loc = glGetUniformLocation(pid, "smokeDims");
		glUniform3f(loc, smokeDims[0], smokeDims[1], smokeDims[2]);
		loc = glGetUniformLocation(pid, "modelViewProjection");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (this->projectionMatrix * this->viewMatrix).cast<float>().eval().data());
		loc = glGetUniformLocation(pid, "lightSpaceMatrix");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (lightProjectionMatrix * lightViewMatrix).cast<float>().eval().data());
		loc = glGetUniformLocation(pid, "dsmLightSpaceMatrix");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (dsmProjectionMatrix * lightViewMatrix).cast<float>().eval().data());
		loc = glGetUniformLocation(pid, "smokeNear");
		glUniform1f(loc, smokeNearPlane);
		loc = glGetUniformLocation(pid, "smokeFar");
		glUniform1f(loc, smokeFarPlane);
		loc = glGetUniformLocation(pid, "numSlices");
		glUniform1i(loc, NUM_SMOKE_SLICES);
		//Insert Light Position and Color into Program
		loc = glGetUniformLocation(pid, "lightColor");
		glUniform3fv(loc, 1, lightCol);
		loc = glGetUniformLocation(pid, "lightPos");
		glUniform3fv(loc, 1, lightPos);


		//insert the textures
		//Smoke Data Texture
		loc = glGetUniformLocation(pid, "smokeData");
		glUniform1i(loc, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_3D, smokeDataTexture.id());
		//glBindTexture(GL_TEXTURE_3D, shadowVolumeTexture.textureId());

		//Insert Shadow Map
		loc = glGetUniformLocation(pid, "shadowMap");
		glUniform1i(loc, 1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, depthTexture.id());

		//Insert Deep Shadow Map
		loc = glGetUniformLocation(pid, "deepShadowMap");
		glUniform1i(loc, 2);
		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_2D_ARRAY, deepShadowTexture.id());

		//Bind VAO
		glBindVertexArray(smokeSliceVAO.id());
		//Draw
		glDrawElements(GL_TRIANGLES, (int)smokeSliceIndices.size(), GL_UNSIGNED_INT, nullptr);

		glBindVertexArray(0);
	}

	//Render the Smoke Particles
	if (RENDER_PARTICLES) {
		//Setup Render Mode
		//glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		glPointSize(3.0f);
		glDepthMask(GL_FALSE);
		glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

		//Use the program
		auto pid = smokePartProgram.id();
		glUseProgram(pid);

		//Insert Uniforms
		loc = glGetUniformLocation(pid, "lightColor");
		glUniform3fv(loc, 1, lightCol);
		//Insert View/Projection Matrix into Program
		loc = glGetUniformLocation(pid, "modelViewProjection");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (projectionMatrix * viewMatrix).cast<float>().eval().data());
		//Insert Light Space conversion Matrix into Program
		loc = glGetUniformLocation(pid, "lightViewMatrix");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (lightViewMatrix).cast<float>().eval().data());
		loc = glGetUniformLocation(pid, "lightProjectionMatrix");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (lightProjectionMatrix).cast<float>().eval().data());
		loc = glGetUniformLocation(pid, "dsmProjectionMatrix");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (dsmProjectionMatrix).cast<float>().eval().data());

		//Insert Textures
		//Insert Shadow Map
		loc = glGetUniformLocation(pid, "shadowMap");
		glUniform1i(loc, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, depthTexture.id());

		//Insert Deep Shadow Map
		loc = glGetUniformLocation(pid, "deepShadowMap");
		glUniform1i(loc, 1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D_ARRAY, deepShadowTexture.id());


		//Render
		glBindVertexArray(smokePartVAO.id());

		int count = smokeDims[0] * smokeDims[1] * smokeDims[2];
		glBindBuffer(GL_ARRAY_BUFFER, smokePartCompBuffer.id());
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
		glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (GLvoid*)(3 * sizeof(float)));
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);

		//glDrawArrays(GL_POINTS, 0, smokePartCount);
		glDrawArrays(GL_POINTS, 0, count);

		//Cleanup
		glBindVertexArray(0);
		//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDepthMask(GL_TRUE);
	}

}

void MyRenderer::mouseEvent(QMouseEvent* e)
{
	auto type = e->type();
	auto pos = e->localPos();

	// begin rotation interaction
	if (type == QEvent::MouseButtonPress && e->button() == Qt::LeftButton)
	{
		this->lastPos = pos;
		this->rotateInteraction = true;
		return;
	}

	// end rotation interaction
	if (type == QEvent::MouseButtonRelease && e->button() == Qt::LeftButton)
	{
		this->rotateInteraction = false;
		return;
	}

	// begin light rotation interaction
	if (type == QEvent::MouseButtonPress && e->button() == Qt::RightButton)
	{
		this->lastPos = pos;
		this->lightRotateInteraction = true;
		return;
	}

	// end rotation interaction
	if (type == QEvent::MouseButtonRelease && e->button() == Qt::RightButton)
	{
		this->lightRotateInteraction = false;
		return;
	}

	// perform rotation interaction
	if (this->rotateInteraction)
	{
		auto delta = pos - this->lastPos;
		this->lastPos = pos;

		// scale rotation depending on window diagonal
		auto scale = constants::two_pi<double> / Eigen::Vector2d{ this->width, this->height }.norm();

		// modify azimuth and elevation by change in mouse position
		cameraAzimuth -= scale * delta.x();
		cameraAzimuth = std::fmod(cameraAzimuth, constants::two_pi<double>);
		cameraElevation -= scale * delta.y();

		// limit elevation so up is never collinear with camera view direction
		cameraElevation = std::fmax(std::fmin(cameraElevation, constants::pi<double> -0.01), 0.01);

		// tell widget to update itself to account for changed position
		this->update();
	}

	if (this->lightRotateInteraction) {
		auto delta = pos - lastPos;
		lastPos = pos;

		// scale rotation depending on window diagonal
		auto scale = constants::two_pi<double> / Eigen::Vector2d{ this->width, this->height }.norm();

		// modify azimuth and elevation by change in mouse position
		lightAzimuth += scale * delta.x();
		lightAzimuth = std::fmod(lightAzimuth, constants::two_pi<double>);
		lightElevation += scale * delta.y();

		lightElevation = std::fmax(std::fmin(lightElevation, constants::pi<double> -0.01), 0.01);

		// tell widget to update itself to account for changed position
		this->update();
	}
}

//Zoom Camera based on Mouse Wheel Movement
void MyRenderer::wheelEvent(QWheelEvent* e) {
	auto scrollAmount = e->angleDelta();
	zoomFactor *= 1 - (0.001 * scrollAmount.y());
	this->update();
}

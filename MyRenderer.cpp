#include "MyRenderer.hpp"

#include "MyRendererUtils.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>

static const bool RENDER_PARTICLES = true;
static const bool RENDER_SLICES = false;
static const bool RENDER_DEBUG = false;
static const bool RENDER_OBJECT_SHADOWS = true;

static const std::string defaultFileName = "C:\\Users\\Jan\\Desktop\\testscene.obj";
static const std::string smokePath = "C:\\Users\\Jan\\Desktop\\smoke.bin";

//static int lightCircleDegrees = 0;

void MyRenderer::openScene(const std::string& fileName) {
	uint currIndexCount;
	QOpenGLBuffer currVertexBuffer = QOpenGLBuffer();
	QOpenGLBuffer currIndexBuffer = QOpenGLBuffer();
	QOpenGLVertexArrayObject *currVAO = new QOpenGLVertexArrayObject();
	QOpenGLShaderProgram *currProgram = new QOpenGLShaderProgram();
	bool currHasTexture;

	std::string currentFileName = fileName;
	int currentMesh = 0;
	numObjectsInScene = 0;

	bool successful = loadMesh(currentFileName, currentMesh, *currVAO, currVertexBuffer, currIndexBuffer, currIndexCount, *currProgram, currHasTexture);

	while (!successful) {
		qDebug() << "Could not open file or file did not contain an Object!";
		currentFileName = QFileDialog::getOpenFileName(Q_NULLPTR, "Open Scene File", "", "Wavefront OBJ (*.obj)").toStdString();
		successful = loadMesh(currentFileName, currentMesh, *currVAO, currVertexBuffer, currIndexBuffer, currIndexCount, *currProgram, currHasTexture);
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
		currVertexBuffer = QOpenGLBuffer();
		currIndexBuffer = QOpenGLBuffer();
		currVAO = new QOpenGLVertexArrayObject();
		currProgram = new QOpenGLShaderProgram();

		currentMesh++;
		successful = loadMesh(currentFileName, currentMesh, *currVAO, currVertexBuffer, currIndexBuffer, currIndexCount, *currProgram, currHasTexture);
	}
}

//Compute the near and far Frustum of the Smoke bounding Box from the camera's view
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
		maxX = max(maxX, point.x());
		minX = min(minX, point.x());
		maxY = max(maxY, point.y());
		minY = min(minY, point.y());
		maxZ = max(maxZ, point.z());
		minZ = min(minZ, point.z());
	}
	maxZ = min(maxZ, 0.0);
	smokeNearPlane = maxZ;
	smokeFarPlane = -minZ;
	smokeLeftPlane = minX;
	smokeRightPlane = maxX;
	smokeBottomPlane = minY;
	smokeTopPlane = maxY;
}

MyRenderer::MyRenderer(QObject * parent)
	: OpenGLRenderer{ parent }
	, cameraAzimuth{ 3.14159265 }
	, cameraElevation{ 1.5707963267948966192313216916398 }
	, zoomFactor{ 4.0 }
	, testTexture{ QOpenGLTexture::Target2D }
	, smokeDataTexture{ QOpenGLTexture::Target3D }
	, deepShadowTexture{ QOpenGLTexture::Target2DArray }
	, depthMapFBO{ SHADOWMAP_SIZE, SHADOWMAP_SIZE, QOpenGLFramebufferObject::Depth}
{
	//Query some Data about OpenGL's Limits
	int workGroupCount[3];
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &workGroupCount[0]);
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 1, &workGroupCount[1]);
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 2, &workGroupCount[2]);
	qDebug() << "Maximum Work Group Count:" << workGroupCount[0] << workGroupCount[1] << workGroupCount[2];
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &workGroupCount[0]);
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 1, &workGroupCount[1]);
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 2, &workGroupCount[2]);
	qDebug() << "Maximum Work Group Size:" << workGroupCount[0] << workGroupCount[1] << workGroupCount[2];
	glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &workGroupCount[2]);
	qDebug() << "Maximum local Work Group Invocations:" << workGroupCount[2];
	glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE, &workGroupCount[1]);
	qDebug() << "Maximum 3D Texture Size:" << workGroupCount[1];

	//Setup Render Timer
	timer.setSampleCount(6);
	timer.create();	

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
	}


	//Process Smoke Data into Particle Vertices
	//TESTING
	//if (RENDER_PARTICLES)	processSmokeData(smokeData, smokeDims, smokePartVertices, smokePartCount);
	//Create Slices for Smoke Rendering
	std::vector<float> smokeBoundingBox = createSmokeBoundingBox(smokeDims);
	if (RENDER_SLICES) createSmokeRenderingPlanes(smokeSliceVertices, smokeSliceIndices);

	//Setup Smoke Particle Rendering
	if (RENDER_PARTICLES) {

		//Initialize Smoke Particle VAO
		{
			smokePartVAO.create();
			smokePartVAO.bind();
			smokePartVertexBuffer.create();
			//glBindBuffer(GL_ARRAY_BUFFER, smokePartVertexBuffer.bufferId());
			//glBufferData(GL_ARRAY_BUFFER, smokePartVertices.size() * sizeof(float), smokePartVertices.data(), GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
			glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (GLvoid*)(3 * sizeof(float)));
			glEnableVertexAttribArray(0);
			glEnableVertexAttribArray(1);
			smokePartVAO.release();
		}

		//Initialize Smoke Particle Shader Program
		{
			smokePartProgram.create();
			smokePartProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/smokeParticle.vert");
			smokePartProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/smokeParticle.frag");
			smokePartProgram.link();

			GLuint pid = smokePartProgram.programId();
			glBindAttribLocation(pid, 0, "aPos");
			glBindAttribLocation(pid, 1, "aDensity");
		}
	}

	//Setup Smoke Slice Rendering
	if (RENDER_SLICES) {

		//Initialize Smoke Slice VAO
		{
			smokeSliceVAO.create();
			smokeSliceVAO.bind();
			smokeSliceVertexBuffer.create();
			glBindBuffer(GL_ARRAY_BUFFER, smokeSliceVertexBuffer.bufferId());
			glBufferData(GL_ARRAY_BUFFER, smokeSliceVertices.size() * sizeof(float), smokeSliceVertices.data(), GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
			glEnableVertexAttribArray(0);
			smokeSliceIndexBuffer.create();
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, smokeSliceIndexBuffer.bufferId());
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, smokeSliceIndices.size() * sizeof(uint), smokeSliceIndices.data(), GL_STATIC_DRAW);
			smokeSliceVAO.release();

			qDebug() << "Smoke Slices have" << smokeSliceVertices.size() << "Vertex Entries and" << smokeSliceIndices.size() << "Index Entries!";
		}

		//Initialize Smoke Slice Shader Program
		{
			smokeSliceProgram.create();
			smokeSliceProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/smokeSlice.vert");
			smokeSliceProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/smokeSlice.frag");
			smokeSliceProgram.link();
		}
	}

	//Initialize Smoke Data 3D Texture
	{
		this->smokeDataTexture.create();
		this->smokeDataTexture.bind();
		this->smokeDataTexture.setFormat(QOpenGLTexture::R32F);
		this->smokeDataTexture.setSize((int)smokeDims[0], (int)smokeDims[1], (int)smokeDims[2]);
		this->smokeDataTexture.setBorderColor(0.0f, 0.0f, 0.0f, 0.0f);
		this->smokeDataTexture.setWrapMode(QOpenGLTexture::ClampToBorder);
		this->smokeDataTexture.setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);
		this->smokeDataTexture.allocateStorage();
		this->smokeDataTexture.setData(QOpenGLTexture::Red, QOpenGLTexture::Float32, smokeData.data());
		this->smokeDataTexture.release();
	}

	//Initialize Object Texture
	{
		this->testTexture.create();
		this->testTexture.bind();
		this->testTexture.setData(QImage(":/textures/testTex.png"));
		this->testTexture.setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
		this->testTexture.setMagnificationFilter(QOpenGLTexture::Linear);
		this->testTexture.setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
		this->testTexture.setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::Repeat);
		this->testTexture.release();
	}
	
	//Initialize Depth Shader Program
	{
		depthProgram.create();
		depthProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/depth.vert");
		depthProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/depth.frag");
		depthProgram.link();
	}

	//Debug Quad Initialization
	if (RENDER_DEBUG) {
		//Initialize VAO
		{
			debugVAO.create();
			debugVAO.bind();
			debugVertexBuffer.create();
			glBindBuffer(GL_ARRAY_BUFFER, debugVertexBuffer.bufferId());
			glBufferData(GL_ARRAY_BUFFER, 4 * 5 * sizeof(float), planeVertices, GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), nullptr);
			glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (GLvoid*)(3 * sizeof(float)));
			glEnableVertexAttribArray(0);
			glEnableVertexAttribArray(1);
			debugIndexBuffer.create();
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, debugIndexBuffer.bufferId());
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(uint), planeIndices, GL_STATIC_DRAW);
			debugVAO.release();
		}

		//Initialize Shader Program
		{
			debugQuadProgram.create();
			debugQuadProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/debug.vert");
			debugQuadProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/debug.frag");
			debugQuadProgram.link();
		}
	}

	//Initialize Deep Shadow Map Texture
	{
		deepShadowTexture.create();
		deepShadowTexture.bind();
		deepShadowTexture.setFormat(QOpenGLTexture::RG16F);
		deepShadowTexture.setSize(DEEPSHADOWMAP_SIZE, DEEPSHADOWMAP_SIZE);
		deepShadowTexture.setLayers(8);
		deepShadowTexture.setBorderColor(-10.0f, 1.0f, 1.0f, 0.0f);
		deepShadowTexture.setWrapMode(QOpenGLTexture::ClampToBorder);
		deepShadowTexture.setMinMagFilters(QOpenGLTexture::Nearest, QOpenGLTexture::Nearest);
		deepShadowTexture.allocateStorage(QOpenGLTexture::RG, QOpenGLTexture::Float16);
		glBindImageTexture(2, deepShadowTexture.textureId(), 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RG16F);
		deepShadowTexture.release();
	}

	//Initialize Deep Shadow Map Shader Program
	{
		deepShadowProgram.create();
		deepShadowProgram.addShaderFromSourceFile(QOpenGLShader::Compute, ":/shaders/deepShadowMap.comp");
		deepShadowProgram.link();
	}

	//Initialize Smoke Particle Creation Shader Program and Buffer
	{
		int bufferSize = smokeDims[0] * smokeDims[1] * smokeDims[2];
		smokePartCompBuffer.create();
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, smokePartCompBuffer.bufferId());
		glBufferData(GL_SHADER_STORAGE_BUFFER, bufferSize * 4 * sizeof(float), NULL, GL_DYNAMIC_DRAW);

		particleCreationProgram.create();
		particleCreationProgram.addShaderFromSourceFile(QOpenGLShader::Compute, ":/shaders/particleCreation.comp");
		particleCreationProgram.link();

		particleCreationProgram.bindAttributeLocation("outBuffer", 1);
	}
}

void MyRenderer::render() {

	//Reset render Timer
	timer.reset();
	
	//Move the Light
	{
		lightCircleDegrees = (lightCircleDegrees + 1) % 360;
		float x = cos(M_PI * lightCircleDegrees / 180);
		float y = sin(M_PI * lightCircleDegrees / 180);
		lightPos[0] = x * 5.0;
		lightPos[1] = y * 5.0;
	}

	//Compute Matrices
	{
		//Calculate View Matrix from Camera Position
		{
			auto sa = std::sin(this->cameraAzimuth);
			auto ca = std::cos(this->cameraAzimuth);
			auto se = std::sin(this->cameraElevation);
			auto ce = std::cos(this->cameraElevation);
			cameraPos[0] = zoomFactor * se * ca;
			cameraPos[1] = zoomFactor * se * sa;
			cameraPos[2] = zoomFactor * ce;
			viewMatrix = calculateLookAtMatrix(
			{ cameraPos[0], cameraPos[1], cameraPos[2] },
			{ 0, 0, 0 },
			{ -ce * ca, -ce * sa, se }
			);
			inverseViewMatrix = viewMatrix.inverse();
		}

		//Calculate Orthografic Projection Matrix for Light
		lightProjectionMatrix = calculateOrthograficPerspective(2.5, -2.5, 2.5, -2.5, SHADOW_NEAR_FRUST, SHADOW_FAR_FRUST);

		

		//Calculate View Matrix from Light's perspective
		lightViewMatrix = calculateLookAtMatrix(
		{ lightPos[0], lightPos[1], lightPos[2] },
		{ 0, 0, 0 },
		{ 0.0, 1.0, 0.0 }
		);

		inverseLightViewMatrix = lightViewMatrix.inverse();

	}

	//Use this for Wireframe Mode
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

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

	GLint loc;

	//Run Compute Shader to create Deep Shadow Map
	timer.recordSample();
	{
		Eigen::Vector3f light = Eigen::Vector3f(lightPos);
		computeSmokePlanes(lightViewMatrix);

		//Calculate Smaller Orthografic Projection for Deep Shadow Map to increase precision
		dsmProjectionMatrix = calculateOrthograficPerspective(smokeRightPlane, smokeLeftPlane, smokeTopPlane, smokeBottomPlane, SHADOW_NEAR_FRUST, SHADOW_FAR_FRUST);

		auto pid = deepShadowProgram.programId();
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
		glBindTexture(GL_TEXTURE_3D, smokeDataTexture.textureId());

		glDispatchCompute(DEEPSHADOWMAP_SIZE/16, DEEPSHADOWMAP_SIZE/16, 1);
	}

	//Run Compute Shader for Creating Smoke Particles
	timer.recordSample();
	{
		//TEST
		qDebug() << "Cam Pos:" << cameraPos[0] << cameraPos[1] << cameraPos[2];

		auto pid = particleCreationProgram.programId();
		glUseProgram(pid);

		loc = glGetUniformLocation(pid, "camPos");
		glUniform3fv(loc, 1, cameraPos);
		loc = glGetUniformLocation(pid, "smokeDims");
		glUniform3f(loc, smokeDims[0], smokeDims[1], smokeDims[2]);

		loc = glGetUniformLocation(pid, "smokeData");
		glUniform1i(loc, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_3D, smokeDataTexture.textureId());

		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, smokePartCompBuffer.bufferId());
		glDispatchCompute(smokeDims[0] / 8, smokeDims[1] / 8, smokeDims[2] / 8);
	}

	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);

	//Render to Depth Map
	timer.recordSample();
	{
		//Use the program
		auto pid = this->depthProgram.programId();
		glUseProgram(pid);

		//Insert Light Space Conversion matrix into program
		loc = glGetUniformLocation(pid, "lightSpaceMatrix");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (lightProjectionMatrix * lightViewMatrix).cast<float>().eval().data());

		//Resize Viewport to Shadow Map Size
		glGetIntegerv(GL_VIEWPORT, viewportSize);
		glViewport(0, 0, SHADOWMAP_SIZE, SHADOWMAP_SIZE);
		depthMapFBO.bind();
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//Render Scene
		if(RENDER_OBJECT_SHADOWS){
			for (int i = 0; i < numObjectsInScene; i++) {
				sceneVAOs[i]->bind();
				glDrawElements(GL_TRIANGLES, sceneIndexCounts[i], GL_UNSIGNED_INT, nullptr);
				sceneVAOs[i]->release();
			}
		}

		//Cleanup
		depthMapFBO.release();
	}

	//Reset the Viewport
	glViewport(viewportSize[0], viewportSize[1], viewportSize[2], viewportSize[3]);
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	//Render the Scene
	timer.recordSample();
	{
		for (int i = 0; i < numObjectsInScene; i++) {
			sceneVAOs[i]->bind();

			auto pid = scenePrograms[i]->programId();
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
				glBindTexture(GL_TEXTURE_2D, testTexture.textureId());

				//Insert Shadow Map
				loc = glGetUniformLocation(pid, "shadowMap");
				glUniform1i(loc, 1);
				glActiveTexture(GL_TEXTURE1);
				glBindTexture(GL_TEXTURE_2D, depthMapFBO.texture());

				//Insert Deep Shadow Map
				loc = glGetUniformLocation(pid, "deepShadowMap");
				glUniform1i(loc, 2);
				glActiveTexture(GL_TEXTURE2);
				glBindTexture(GL_TEXTURE_2D_ARRAY, deepShadowTexture.textureId());
			}
			else {
				//Insert Shadow Map
				loc = glGetUniformLocation(pid, "shadowMap");
				glUniform1i(loc, 0);
				glActiveTexture(GL_TEXTURE0);
				glBindTexture(GL_TEXTURE_2D, depthMapFBO.texture());

				//Insert Deep Shadow Map
				loc = glGetUniformLocation(pid, "deepShadowMap");
				glUniform1i(loc, 1);
				glActiveTexture(GL_TEXTURE1);
				glBindTexture(GL_TEXTURE_2D_ARRAY, deepShadowTexture.textureId());
			}

			//Render the Object
			glDrawElements(GL_TRIANGLES, sceneIndexCounts[i], GL_UNSIGNED_INT, nullptr);
		}
	}

	//Render the Debug Quad
	if (RENDER_DEBUG) {
		//Use the program
		auto pid = debugQuadProgram.programId();
		glUseProgram(pid);

		//Insert the parameters
		loc = glGetUniformLocation(pid, "projection");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (this->projectionMatrix).cast<float>().eval().data());

		//insert the textures
		loc = glGetUniformLocation(pid, "debugTexture");
		glUniform1i(loc, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D_ARRAY, deepShadowTexture.textureId());

		//Bind VAO
		this->debugVAO.bind();
		//draw the Quad
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

		debugVAO.release();
	}

	//Render the Smoke Slices
	timer.recordSample();
	if (RENDER_SLICES) {
		//Compute Near and Far Planes of the Smoke Volume
		computeSmokePlanes(viewMatrix);

		//Use the program
		auto pid = smokeSliceProgram.programId();
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
		glBindTexture(GL_TEXTURE_3D, smokeDataTexture.textureId());
		//glBindTexture(GL_TEXTURE_3D, shadowVolumeTexture.textureId());

		//Insert Shadow Map
		loc = glGetUniformLocation(pid, "shadowMap");
		glUniform1i(loc, 1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, depthMapFBO.texture());

		//Insert Deep Shadow Map
		loc = glGetUniformLocation(pid, "deepShadowMap");
		glUniform1i(loc, 2);
		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_2D_ARRAY, deepShadowTexture.textureId());

		//Bind VAO
		this->smokeSliceVAO.bind();
		//Draw
		glDrawElements(GL_TRIANGLES, (int)smokeSliceIndices.size(), GL_UNSIGNED_INT, nullptr);

		smokeSliceVAO.release();
	}

	//Render the Smoke Particles
	if (RENDER_PARTICLES) {
		//Setup Render Mode
		//glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		glPointSize(3.0f);
		glDepthMask(GL_FALSE);
		glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

		//Use the program
		auto pid = smokePartProgram.programId();
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
		glBindTexture(GL_TEXTURE_2D, depthMapFBO.texture());

		//Insert Deep Shadow Map
		loc = glGetUniformLocation(pid, "deepShadowMap");
		glUniform1i(loc, 1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D_ARRAY, deepShadowTexture.textureId());


		//Render
		smokePartVAO.bind();

		int count = smokeDims[0] * smokeDims[1] * smokeDims[2];
		glBindBuffer(GL_ARRAY_BUFFER, smokePartCompBuffer.bufferId());
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
		glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (GLvoid*)(3 * sizeof(float)));
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		

		//glDrawArrays(GL_POINTS, 0, smokePartCount);
		glDrawArrays(GL_POINTS, 0, count);

		//Cleanup
		smokePartVAO.release();
		//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDepthMask(GL_TRUE);
	}

	timer.recordSample();
	QVector<GLuint64> intervals = timer.waitForIntervals();
	//qDebug() << "Deep Shadow Map took" << intervals[0] * 0.000001 << "ms, Particle Creation took" << intervals[1] * 0.000001 << "ms, Shadow Map took" << intervals[2] * 0.000001 << "ms, Scene took" << intervals[3] * 0.000001 << "ms, Smoke took" << intervals[4] * 0.000001 << "ms, Overall Frame Time:" << (intervals[0] + intervals[1] + intervals[2] + intervals[3] + intervals[4]) / 1000000 << "ms";
}

//Move Camera based on Mouse Movement
void MyRenderer::mouseEvent(QMouseEvent * e)
{
	auto type = e->type();
	auto pos = e->localPos();

	if (type == QEvent::MouseButtonPress && e->button() == Qt::LeftButton)
	{
		this->lastPos = pos;
		this->rotateInteraction = true;
		return;
	}
	/*if (type == QEvent::MouseButtonPress && e->button() == Qt::RightButton)
	{
		this->lastPosRMB = pos;
		this->rotateLight = true;
		return;
	}*/

	if (type == QEvent::MouseButtonRelease && e->button() == Qt::LeftButton)
	{
		this->rotateInteraction = false;
		return;
	}
	/*if (type == QEvent::MouseButtonRelease && e->button() == Qt::RightButton)
	{
		this->rotateLight = false;
		return;
	}*/

	if (this->rotateInteraction)
	{
		auto delta = pos - this->lastPos;
		cameraAzimuth -= 0.01 * delta.x();
		cameraAzimuth = std::fmod(cameraAzimuth, 6.283185307179586476925286766559);
		cameraElevation -= 0.01 * delta.y();
		cameraElevation = std::fmax(std::fmin(cameraElevation, 3.1415926535897932384626433832795), 0);

		this->lastPos = pos;
	}
	/*if (this->rotateLight) {
		auto delta = pos - this->lastPosRMB;
		this->lightCircleDegrees = (lightCircleDegrees + (int)round(delta.x() / 5.0)) % 360;

		this->lastPosRMB = pos;
	}*/
}

//Zoom Camera based on Mouse Wheel Movement
void MyRenderer::wheelEvent(QWheelEvent * e)
{
	auto scrollAmount = e->angleDelta();

	zoomFactor *= 1 - (0.001 * scrollAmount.y());
}

//Copied from given Project
void MyRenderer::resize(int w, int h)
{
	this->projectionMatrix = calculateInfinitePerspective(
		0.78539816339744831, // 45 degrees in radians
		static_cast<double>(w) / h,
		0.01 // near plane (chosen "at random")
		);
	this->inverseProjectionMatrix = this->projectionMatrix.inverse();
}

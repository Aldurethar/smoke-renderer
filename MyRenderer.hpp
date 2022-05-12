#pragma once

#include "OpenGLRenderer.hpp"
#include "constants.hpp"

#include <OpenGLObjects.h>

#include <QElapsedTimer>
#include <QPoint>

#include <Eigen/Core>

class MyRenderer : public OpenGLRenderer
{
	Q_OBJECT

public:
	MyRenderer(QObject* parent);

	void resize(int w, int h) override;
	void render() override;

	void mouseEvent(QMouseEvent* e) override;
	void wheelEvent(QWheelEvent* e) override;

private:
	//Camera and Controls
	double
		cameraAzimuth = constants::pi<double>,
		cameraElevation = constants::half_pi<double>;
	double zoomFactor = 4.0;
	bool rotateInteraction = false;
	float cameraPos[3];
	GLint viewportSize[4];
	double lightAzimuth = constants::half_pi<double>;
	double lightElevation = constants::half_pi<double>;
	bool lightRotateInteraction = false;

	int width = 0, height = 0;

	QPointF lastPos;
	QElapsedTimer timer;
	quint64 lastTimeNS = 0;

	//Smoke Data from File
	std::vector<float> smokeData;
	std::vector<size_t> smokeDims;
	std::vector<float> smokeBoundingBox;

	//Smoke Particle Rendering
	std::vector<float> smokePartVertices;
	uint smokePartCount;

	//Smoke Slice Rendering
	std::vector<float> smokeSliceVertices;
	std::vector<GLuint> smokeSliceIndices;
	float smokeNearPlane, smokeFarPlane, smokeRightPlane, smokeLeftPlane, smokeTopPlane, smokeBottomPlane;

	//Scene to be rendered
	std::vector<uint> sceneIndexCounts;
	std::vector<gl::Buffer*> sceneVertexBuffers;
	std::vector<gl::Buffer*> sceneIndexBuffers;
	std::vector<gl::VertexArray*> sceneVAOs;
	std::vector<gl::Program*> scenePrograms;
	std::vector<bool> sceneHasTexture;
	int numObjectsInScene;



	Eigen::Matrix4d
		projectionMatrix, inverseProjectionMatrix,
		viewMatrix, inverseViewMatrix,
		lightViewMatrix, lightProjectionMatrix,
		inverseLightViewMatrix,
		dsmProjectionMatrix;

	gl::Buffer
		icosphereVertexBuffer, icosphereIndexBuffer,
		debugVertexBuffer, debugIndexBuffer,
		smokePartVertexBuffer,
		smokePartCompBuffer,
		smokeSliceVertexBuffer, smokeSliceIndexBuffer;

	gl::VertexArray
		icosphereVAO,
		skyboxVAO,
		debugVAO,
		smokePartVAO,
		smokeSliceVAO;

	gl::Program
		icosphereProgram,
		skyboxProgram,
		depthProgram, debugQuadProgram,
		smokePartProgram,
		smokeSliceProgram,
		deepShadowProgram,
		particleCreationProgram;

	gl::Texture
		earthTexture,
		moonTexture,
		starsCubeMap,
		testTexture,
		smokeDataTexture,
		depthTexture,
		deepShadowTexture;

	gl::Framebuffer
		depthMapFBO;

	GLsizei numIcosphereIndices = 0;

	void openScene(const std::string& fileName);
	void computeSmokePlanes(Eigen::Matrix4d view);
};

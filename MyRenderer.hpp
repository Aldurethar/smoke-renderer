#pragma once

#include "OpenGLRenderer.hpp"

#include <glad/glad.h>

#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFramebufferObject>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QOpenGLTimeMonitor>

#include <assimp/Importer.hpp>
#include <assimp/DefaultLogger.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <Eigen/Core>

class MyRenderer : public OpenGLRenderer
{
	Q_OBJECT

public:
	MyRenderer(QObject * parent);
	
	void resize(int w, int h) override;
	void render() override;

	void mouseEvent(QMouseEvent * e) override;
	void wheelEvent(QWheelEvent * e) override;

private:
	//Camera and camera Movement
	double cameraAzimuth, cameraElevation, zoomFactor;
	bool rotateInteraction;
	float cameraPos[3];
	GLint viewportSize[4];
	QPointF lastPos;
	bool rotateLight;
	QPointF lastPosRMB;
	int lightCircleDegrees = 0;

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
	std::vector<QOpenGLBuffer> sceneVertexBuffers;
	std::vector<QOpenGLBuffer> sceneIndexBuffers;
	std::vector<QOpenGLVertexArrayObject*> sceneVAOs;
	std::vector<QOpenGLShaderProgram*> scenePrograms;
	std::vector<bool> sceneHasTexture;
	int numObjectsInScene;

	uint testIndexCount, groundIndexCount;

	QOpenGLTimeMonitor timer;

	Eigen::Matrix4d
		projectionMatrix, inverseProjectionMatrix,
		viewMatrix, inverseViewMatrix,
		lightViewMatrix, lightProjectionMatrix,
		inverseLightViewMatrix,
		dsmProjectionMatrix;

	QOpenGLBuffer
		debugVertexBuffer, debugIndexBuffer,
		smokePartVertexBuffer,
		smokePartCompBuffer,
		smokeSliceVertexBuffer, smokeSliceIndexBuffer;

	QOpenGLVertexArrayObject
		debugVAO,
		smokePartVAO,
		smokeSliceVAO;

	QOpenGLShaderProgram
		depthProgram, debugQuadProgram,
		smokePartProgram,
		smokeSliceProgram,
		deepShadowProgram,
		particleCreationProgram;
	
	QOpenGLTexture
		testTexture,
		smokeDataTexture,
		deepShadowTexture;

	QOpenGLFramebufferObject
		depthMapFBO;

	void openScene(const std::string& fileName);
	void computeSmokePlanes(Eigen::Matrix4d view);
};
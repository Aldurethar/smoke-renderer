#pragma once
#include "MyRenderer.hpp"

#include <unordered_map>

#include "FileIO.hpp"

#include <QDebug>
#include <QFileDialog>
#include <QElapsedTimer>
#include <iostream>

#include <Eigen/Core>
#include <assimp/Importer.hpp>
#include <assimp/DefaultLogger.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

//CONSTANTS
static const int NUM_SMOKE_SLICES = 1024;
static const int SHADOWMAP_SIZE = 2048;
static const int DEEPSHADOWMAP_SIZE = 512;
static const float SHADOW_NEAR_FRUST = 1.0;
static const float SHADOW_FAR_FRUST = 10.0;

static GLenum glCheckError_(const char* file, int line)
{
	GLenum errorCode;
	while ((errorCode = glGetError()) != GL_NO_ERROR)
	{
		std::string error;
		switch (errorCode)
		{
		case GL_INVALID_ENUM:                  error = "INVALID_ENUM"; break;
		case GL_INVALID_VALUE:                 error = "INVALID_VALUE"; break;
		case GL_INVALID_OPERATION:             error = "INVALID_OPERATION"; break;
		case GL_STACK_OVERFLOW:                error = "STACK_OVERFLOW"; break;
		case GL_STACK_UNDERFLOW:               error = "STACK_UNDERFLOW"; break;
		case GL_OUT_OF_MEMORY:                 error = "OUT_OF_MEMORY"; break;
		case GL_INVALID_FRAMEBUFFER_OPERATION: error = "INVALID_FRAMEBUFFER_OPERATION"; break;
		}
		std::cout << "OpenGL Error: " << error << " | " << file << " (" << line << ")" << std::endl;
	}
	return errorCode;
}
#define glCheckError() glCheckError_(__FILE__, __LINE__)

//Debug Screen Quad
static float planeVertices[] = {
	// positions           // texcoords
	-1.0f,  1.0f,  0.1f,   0.0f, 0.0f,
	-1.0f, -1.0f,  0.1f,   0.0f, 1.0f,
	1.0f,  1.0f,  0.1f,   1.0f, 0.0f,
	1.0f, -1.0f,  0.1f,   1.0f, 1.0f,
};
static GLuint planeIndices[] = {
	0, 1, 2,
	2, 1, 3
};

static float defaultColor[] = {
	0.8f, 0.8f, 0.8f
};

static float lightPos[] = {
	3.0, 0.0, 3.0
};
static float lightCol[] = {
	1.0, 1.0, 1.0
};

// helper array with the corner vertices of an icosahedron
static float icosahedronVertices[] = {
	0.000000f, -1.000000f, 0.000000f,
	0.723600f, -0.447214f, 0.525720f,
	-0.276386f, -0.447214f, 0.850640f,
	-0.894424f, -0.447214f, 0.000000f,
	-0.276386f, -0.447214f, -0.850640f,
	0.723600f, -0.447214f, -0.525720f,
	0.276386f, 0.447214f, 0.850640f,
	-0.723600f, 0.447214f, 0.525720f,
	-0.723600f, 0.447214f, -0.525720f,
	0.276386f, 0.447214f, -0.850640f,
	0.894424f, 0.447214f, 0.000000f,
	0.000000f, 1.000000f, 0.000000f
};

// helper array of indices which form the triangular faces of an icosahedron
static GLubyte icosahedronIndices[] = {
	0, 1, 2,
	1, 0, 5,
	0, 2, 3,
	0, 3, 4,
	0, 4, 5,
	1, 5, 10,
	2, 1, 6,
	3, 2, 7,
	4, 3, 8,
	5, 4, 9,
	1, 10, 6,
	2, 6, 7,
	3, 7, 8,
	4, 8, 9,
	5, 9, 10,
	6, 10, 11,
	7, 6, 11,
	8, 7, 11,
	9, 8, 11,
	10, 9, 11
};

namespace std
{
	// specialization of std::hash for std::pair so that std::unordered_map<std::pair<A,B>, T> can be used
	template<typename A, typename B>
	struct hash<pair<A, B>>
	{
		using argument_type = pair<A, B>;
		using result_type = size_t;

		result_type operator()(argument_type const& p) const noexcept
		{
			return hash<decay_t<A>>{}(p.first) ^ (hash<decay_t<B>>{}(p.second) << 1);
		}
	};
}

// helper function to subdivide a triangular mesh and project it onto the unit sphere
static void subdivideIcosphere(std::vector<float>& vertices, std::vector<unsigned>& indices)
{
	std::unordered_map<std::pair<unsigned, unsigned>, unsigned> newVertexLookup;
	// reserve number of vertices (= vertices.size() / 3) times 3, as total is multiplied by ~4 per subdivision
	newVertexLookup.reserve(vertices.size());

	auto midpointForEdge = [&](unsigned first, unsigned second) {
		if (first > second)
			std::swap(first, second);
		auto inserted = newVertexLookup.insert({ {first, second}, static_cast<unsigned>(vertices.size() / 3) });
		if (inserted.second)
		{
			Eigen::Map<Eigen::Vector3f> e0{ vertices.data() + 3 * first };
			Eigen::Map<Eigen::Vector3f> e1{ vertices.data() + 3 * second };
			auto newVertex = (e0 + e1).normalized();
			vertices.insert(std::end(vertices), newVertex.data(), newVertex.data() + 3);
		}
		return inserted.first->second;
	};

	std::vector<unsigned> newIndices;
	newIndices.reserve(4 * indices.size());

	for (std::size_t i = 0; i < indices.size(); i += 3)
	{
		unsigned midpoints[3];
		for (int e = 0; e < 3; ++e)
			midpoints[e] = midpointForEdge(indices[i + e], indices[i + (e + 1) % 3]);
		for (int e = 0; e < 3; ++e)
		{
			newIndices.emplace_back(indices[i + e]);
			newIndices.emplace_back(midpoints[e]);
			newIndices.emplace_back(midpoints[(e + 2) % 3]);
		}
		newIndices.insert(std::end(newIndices), std::begin(midpoints), std::end(midpoints));
	}

	indices.swap(newIndices);
}

// helper function to compute a perspective projection matrix with an infinite far range
static Eigen::Matrix4d calculateInfinitePerspective(double minimumFieldOfView, double aspectRatio, double zNear)
{
	// linear field of view factor
	auto range = std::tan(minimumFieldOfView / 2);

	// scale factor for left/right depending on field of view and aspect ratio
	auto right = aspectRatio >= 1.0 ? range * aspectRatio : range;

	// scale factor for top/bottom depending on field of view and aspect ratio
	auto top = aspectRatio >= 1.0 ? range : range / aspectRatio;

	Eigen::Matrix4d P;
	P <<
		1 / right, 0, 0, 0,
		0, 1 / top, 0, 0,
		0, 0, 0, -2 * zNear,
		0, 0, -1, 0;
	return P;
}

// helper function to compute a lookat-matrix (camera at eye, target at center, global up direction; up must be linearly independent of (center - eye))
static Eigen::Matrix4d calculateLookAtMatrix(Eigen::Vector3d eye, Eigen::Vector3d center, Eigen::Vector3d up)
{
	// compute forward direction
	Eigen::RowVector3d f = (eye - center).normalized();
	// compute orthogonal sideways/right direction
	Eigen::RowVector3d s = up.cross(f).normalized();
	// compute orthogonal final up direction
	Eigen::RowVector3d u = f.cross(s);

	// s/u/f define a rotation matrix, inverse translation by rotated eye position, unit row in w for affine transformation
	Eigen::Matrix4d M;
	M <<
		s, -s.dot(eye),
		u, -u.dot(eye),
		f, -f.dot(eye),
		Eigen::RowVector4d::UnitW();
	return M;
}

// helper function to compute an orthografic projection matrix
static Eigen::Matrix4d calculateOrthograficPerspective(double right, double left, double top, double bottom, double nearFrust, double farFrust) {
	Eigen::Matrix4d P;
	P <<
		2 / (right - left), 0, 0, 0,
		0, 2 / (top - bottom), 0, 0,
		0, 0, -2 / (farFrust - nearFrust), -((farFrust + nearFrust) / (farFrust - nearFrust)),
		0, 0, 0, 1;
	return P;
}

// helper function to load a Qt resource as an array of char (bytes)
static std::vector<char> loadResource(char const* path)
{
	QFile f(QString(":/") + QString::fromUtf8(path));
	auto opened = f.open(QIODevice::ReadOnly);
	assert(opened); (void)opened;
	auto size = f.size();
	std::vector<char> buf(size);
	auto read = f.read(buf.data(), size);
	assert(read == size); (void)read;
	return buf;
}

//Import Mesh from given File
static bool importMesh(const std::string& fileName, int numberOfObject, std::vector<float>& verts, uint& numVerts, std::vector<uint>& indices, uint& numIndices, bool& hasTexCoords, std::string& name) {
	//Create Importer
	Assimp::Importer importer;
	//Logger for Assimp
	Assimp::DefaultLogger::create("", Assimp::Logger::NORMAL);


	//Read file
	const aiScene* scene = importer.ReadFile(fileName,
		aiProcess_Triangulate |
		aiProcess_PreTransformVertices |
		aiProcess_JoinIdenticalVertices |
		aiProcess_FlipUVs
	);

	//Report Errors
	if (!scene) {
		Assimp::DefaultLogger::get()->error("Could not import mesh");
		Assimp::DefaultLogger::get()->info(importer.GetErrorString());
		Assimp::DefaultLogger::kill();
		return false;
	}

	//extract mesh data
	if (numberOfObject >= (int)scene->mNumMeshes) {
		return false;
	}

	const aiMesh* firstMesh = scene->mMeshes[numberOfObject];
	//Copy Vertices
	std::vector<float> newVerts;
	if (firstMesh->HasTextureCoords(0)) {
		//Case with Texture Coordinates
		hasTexCoords = true;
		newVerts.resize(firstMesh->mNumVertices * 8);
		for (uint i = 0; i < firstMesh->mNumVertices; ++i) {
			newVerts[8 * i] = firstMesh->mVertices[i].x;
			newVerts[8 * i + 1] = firstMesh->mVertices[i].y;
			newVerts[8 * i + 2] = firstMesh->mVertices[i].z;
			newVerts[8 * i + 3] = firstMesh->mNormals[i].x;
			newVerts[8 * i + 4] = firstMesh->mNormals[i].y;
			newVerts[8 * i + 5] = firstMesh->mNormals[i].z;
			newVerts[8 * i + 6] = firstMesh->mTextureCoords[0][i].x;
			newVerts[8 * i + 7] = firstMesh->mTextureCoords[0][i].y;
		}
	}
	else {
		//Case without Texture Coordinates
		hasTexCoords = false;
		newVerts.resize(firstMesh->mNumVertices * 6);
		for (uint i = 0; i < firstMesh->mNumVertices; ++i) {
			newVerts[6 * i] = firstMesh->mVertices[i].x;
			newVerts[6 * i + 1] = firstMesh->mVertices[i].y;
			newVerts[6 * i + 2] = firstMesh->mVertices[i].z;
			newVerts[6 * i + 3] = firstMesh->mNormals[i].x;
			newVerts[6 * i + 4] = firstMesh->mNormals[i].y;
			newVerts[6 * i + 5] = firstMesh->mNormals[i].z;
		}
	}

	verts = newVerts;
	numVerts = firstMesh->mNumVertices;

	//Copy Indices
	std::vector<uint> newIndices(firstMesh->mNumFaces * 3);
	for (uint i = 0; i < firstMesh->mNumFaces; ++i) {
		newIndices[3 * i] = firstMesh->mFaces[i].mIndices[0];
		newIndices[3 * i + 1] = firstMesh->mFaces[i].mIndices[1];
		newIndices[3 * i + 2] = firstMesh->mFaces[i].mIndices[2];
	}
	indices = newIndices;
	numIndices = firstMesh->mNumFaces * 3;

	Assimp::DefaultLogger::kill();
	return true;

}

//Load a Mesh from the file and set up all its parts
static bool loadMesh(const std::string& fileName, int number, gl::VertexArray& vao, gl::Buffer& vertBuffer, gl::Buffer& indBuffer, uint& indexCount, gl::Program& program, bool& hasTexCoords) {

	//Buffers for the imported data
	std::vector<float> vertexData;
	uint vertexCount;
	std::vector<GLuint> indexData;
	//bool hasTexCoords;
	std::string objName;

	//import from file
	bool successful = importMesh(fileName, number, vertexData, vertexCount, indexData, indexCount, hasTexCoords, objName);
	if (successful) {
		qDebug() << "Sucessfully imported Object " << objName.data() << " with number " << number << ", numVerts " << vertexCount << ", numInds " << indexCount << ", vertSize " << vertexData.size() << ", indSize " << indexData.size() << ", TexCoords " << hasTexCoords;
	}
	else {
		qDebug() << "Import of Object " << objName.data() << " with number " << number << " was unsuccessful!";
		return false;
	}

	//Create and Bind Vertex Array Object that will hold all the Data for the Object to render
	glBindVertexArray(vao.id());
	{
		//Create and fill Vertex Data Buffer
		glBindBuffer(GL_ARRAY_BUFFER, vertBuffer.id());
		if (hasTexCoords) {

			glBufferData(GL_ARRAY_BUFFER, vertexCount * 8 * sizeof(float), vertexData.data(), GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), nullptr);
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (GLvoid*)(3 * sizeof(float)));
			glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (GLvoid*)(6 * sizeof(float)));
			glEnableVertexAttribArray(0);
			glEnableVertexAttribArray(1);
			glEnableVertexAttribArray(2);
		}
		else {
			glBufferData(GL_ARRAY_BUFFER, vertexCount * 6 * sizeof(float), vertexData.data(), GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (GLvoid*)(3 * sizeof(float)));
			glEnableVertexAttribArray(0);
			glEnableVertexAttribArray(1);
		}


		//Create and fill Index Data Buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indBuffer.id());
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexCount * sizeof(uint), &indexData[0], GL_STATIC_DRAW);

		glBindVertexArray(0);
	}

	GLuint pid;
	GLint loc;

	//Create Shader Program
	{
		//Add Shaders from file
		gl::Shader vertexShader{ GL_VERTEX_SHADER };
		gl::Shader fragmentShader{ GL_FRAGMENT_SHADER };

		std::vector<char> vsText;
		std::vector<char> fsText;

		if (hasTexCoords) {
			vsText = loadResource("shaders/phong_textured.vert");
			fsText = loadResource("shaders/phong_textured.frag");
		}
		else {
			vsText = loadResource("shaders/phong_color.vert");
			fsText = loadResource("shaders/phong_color.frag");
		}

		vertexShader.compile(vsText.data(), static_cast<GLint>(vsText.size()));
		fragmentShader.compile(fsText.data(), static_cast<GLint>(fsText.size()));

		// link shader program and check for errors. shader objects are no longer required
		if (!program.link(vertexShader, fragmentShader))
		{
			qDebug() << "Shader compilation failed:\n" << program.infoLog().get();
			std::abort();
		}

		//Bind Vertex Data Location
		pid = program.id();
		glBindAttribLocation(pid, 0, "aPos");
		glBindAttribLocation(pid, 1, "aNormal");
		if (hasTexCoords) {
			glBindAttribLocation(pid, 2, "uvCoord");
		}

		glUseProgram(pid);

		//Secure colorTexture uniform from being optimized away
		if (hasTexCoords) {
			loc = glGetUniformLocation(pid, "colorTexture");
			glUniform1i(loc, 0);
		}
		//Insert default color
		else {
			loc = glGetUniformLocation(pid, "objColor");
			glUniform3fv(loc, 1, defaultColor);
		}

	}

	return true;
}

//Create Bounding Box Vertices for the Smoke Data
//Assumes a grid size of 1cm and Box centered on (0, 0, 0)
static std::vector<float> createSmokeBoundingBox(std::vector<size_t>& dims) {
	std::vector<float> bb;
	//+++
	bb.push_back(dims[2] * 0.005);
	bb.push_back(dims[1] * 0.005);
	bb.push_back(dims[0] * 0.005);
	//++-
	bb.push_back(dims[2] * 0.005);
	bb.push_back(dims[1] * 0.005);
	bb.push_back(dims[0] * -0.005);
	//+-+
	bb.push_back(dims[2] * 0.005);
	bb.push_back(dims[1] * -0.005);
	bb.push_back(dims[0] * 0.005);
	//+--
	bb.push_back(dims[2] * 0.005);
	bb.push_back(dims[1] * -0.005);
	bb.push_back(dims[0] * -0.005);
	//-++
	bb.push_back(dims[2] * -0.005);
	bb.push_back(dims[1] * 0.005);
	bb.push_back(dims[0] * 0.005);
	//-+-
	bb.push_back(dims[2] * -0.005);
	bb.push_back(dims[1] * 0.005);
	bb.push_back(dims[0] * -0.005);
	//--+
	bb.push_back(dims[2] * -0.005);
	bb.push_back(dims[1] * -0.005);
	bb.push_back(dims[0] * 0.005);
	//---
	bb.push_back(dims[2] * -0.005);
	bb.push_back(dims[1] * -0.005);
	bb.push_back(dims[0] * -0.005);

	return bb;
}

//Load the Smoke Data from a File
static void loadSmokeData(const std::string& fileName, std::vector<float>& data, std::vector<size_t>& dims, std::vector<float>& boundingBox)
{
	bool succ = readField(fileName, data, dims);
	while (!succ) {
		std::cout << "Could not read Smoke Data, please select another File!";
		std::string newFileName = QFileDialog::getOpenFileName(Q_NULLPTR, "Open Smoke Data File", "", "Smoke Data (*.bin)").toStdString();
		succ = readField(newFileName, data, dims);
	}
	boundingBox = createSmokeBoundingBox(dims);
	std::cout << "Successfully read Smoke Data!";


	// Check Smoke Data statistics
	{
		float maximum = 0.0f;
		float minimum = 1000.0f;
		float avg = 0.0f;
		int numZero = 0;
		int numNonInteger = 0;
		for (int i = 0; i < data.size(); i++) {
			float val = data[i];
			maximum = fmax(val, maximum);
			minimum = fmin(val, minimum);
			avg += val;
			if (val == 0) { numZero++; }
			if (val != round(val)) { numNonInteger++; }
		}
		float avg2 = avg / (data.size() - numZero);
		avg /= data.size();

		std::ostringstream output;
		output << "Smoke Data Size: " << data.size() << ", " << dims.size() << " Dimensions: ";
		for (int i = 0; i < dims.size(); i++) {
			output << dims[i] << ", ";
		}
		output << "Maximum: " << maximum << ", Minimum: " << minimum << ", Average: " << avg << ", Zero Entries: " << numZero << ", Average of nonzero entries: " << avg2 << ", Non-Integer Entries: " << numNonInteger;
		qDebug(output.str().data());
	}
}

//Create the Planes for Smoke Slice Rendering
static void createSmokeRenderingPlanes(std::vector<float>& planesVerts, std::vector<GLuint>& planesInds) {
	//Measure Time
	QElapsedTimer timer;
	timer.start();

	//Maximum Depth to which the Smoke is Visible
	float maxDepth = 1.0;
	//Number of Slices
	int numSteps = NUM_SMOKE_SLICES;
	float stepSize = maxDepth / numSteps;

	std::cout << "Stepsize:" << stepSize;

	planesVerts.clear();
	planesInds.clear();
	for (int i = 0; i < numSteps; i++) {
		float depth = -(maxDepth - i * stepSize);
		int offset = i * 4;
		float x = 2.5f;
		float y = 2.0f;
		planesVerts.push_back(-x);	planesVerts.push_back(y);	planesVerts.push_back(depth);
		planesVerts.push_back(-x);	planesVerts.push_back(-y);	planesVerts.push_back(depth);
		planesVerts.push_back(x);	planesVerts.push_back(y);	planesVerts.push_back(depth);
		planesVerts.push_back(x);	planesVerts.push_back(-y);	planesVerts.push_back(depth);
		planesInds.push_back(offset + 0);
		planesInds.push_back(offset + 1);
		planesInds.push_back(offset + 2);
		planesInds.push_back(offset + 2);
		planesInds.push_back(offset + 1);
		planesInds.push_back(offset + 3);
	}
	std::cout << "Creating Smoke Planes took" << timer.nsecsElapsed() * 0.000001 << "ms";
}

//Transform Smoke Data into array of Vertices for Particle Rendering
static void processSmokeData(std::vector<float> data, std::vector<size_t> dims, std::vector<float>& smokeVerts, uint& numVerts)
{
	QElapsedTimer timer;
	timer.start();
	numVerts = 0;
	for (int z = 0; z < dims[2]; z++) {
		for (int y = 0; y < dims[1]; y++) {
			for (int x = 0; x < dims[0]; x++) {
				float dataPoint = data[x + y * dims[0] + z * dims[0] * dims[1]];
				if (dataPoint > 0) {
					numVerts++;
					smokeVerts.push_back((x - (float)dims[0] * 0.5) * 0.01);
					smokeVerts.push_back((y - (float)dims[1] * 0.5) * 0.01);
					smokeVerts.push_back((z - (float)dims[2] * 0.5) * 0.01);
					smokeVerts.push_back(dataPoint * 1.0);
				}
			}
		}
	}
	std::cout << "Processed Smoke Data into " << numVerts << " Vertices, making Array of size " << smokeVerts.size() << ", took " << timer.elapsed() << "ms";

}
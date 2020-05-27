#include "MyRenderer.hpp"
#include "FileIO.hpp"

#include <QFileDialog>
#include <QElapsedTimer>

#include <Eigen/Core>

//CONSTANTS
static const int NUM_SMOKE_SLICES = 1024;
static const int SHADOWMAP_SIZE = 2048;
static const int DEEPSHADOWMAP_SIZE = 512;
static const float SHADOW_NEAR_FRUST = 1.0;
static const float SHADOW_FAR_FRUST = 10.0;

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

Eigen::Matrix4d calculateInfinitePerspective(double verticalFieldOfView, double aspectRatio, double zNear)
{
	auto range = std::tan(verticalFieldOfView / 2);
	auto right = range * aspectRatio;
	auto top = range;

	Eigen::Matrix4d P;
	P <<
		1 / right, 0, 0, 0,
		0, 1 / top, 0, 0,
		0, 0, -1, -2 * zNear,
		0, 0, -1, 0;
	return P;
}

Eigen::Matrix4d calculateOrthograficPerspective(double right, double left, double top, double bottom, double nearFrust, double farFrust) {
	Eigen::Matrix4d P;
	P <<
		2 / (right - left), 0, 0, 0,
		0, 2 / (top - bottom), 0, 0,
		0, 0, -2 / (farFrust - nearFrust), -((farFrust + nearFrust) / (farFrust - nearFrust)),
		0, 0, 0, 1;
	return P;
}

Eigen::Matrix4d calculateLookAtMatrix(Eigen::Vector3d eye, Eigen::Vector3d center, Eigen::Vector3d up)
{
	Eigen::RowVector3d f = (eye - center).normalized();
	Eigen::RowVector3d s = up.cross(f).normalized();
	Eigen::RowVector3d u = f.cross(s);

	Eigen::Matrix4d M;
	M <<
		s, -s.dot(eye),
		u, -u.dot(eye),
		f, -f.dot(eye),
		Eigen::RowVector4d::UnitW();
	return M;
}

//Import Mesh from given File
bool importMesh(const std::string& fileName, int numberOfObject, std::vector<float>& verts, uint& numVerts, std::vector<uint>& indices, uint& numIndices, bool& hasTexCoords, std::string& name) {
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
bool loadMesh(const std::string& fileName, int number, QOpenGLVertexArrayObject& vao, QOpenGLBuffer& vertBuffer, QOpenGLBuffer& indBuffer, uint& indexCount, QOpenGLShaderProgram& program, bool& hasTexCoords) {

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

	//Create Vertex Array Object that will hold all the Data for the Object to render
	vao.create();
	{
		//Bind VAO
		QOpenGLVertexArrayObject::Binder boundVAO{ &vao };

		//Create and fill Vertex Data Buffer
		vertBuffer.create();
		glBindBuffer(GL_ARRAY_BUFFER, vertBuffer.bufferId());
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
		indBuffer.create();
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indBuffer.bufferId());
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexCount * sizeof(uint), &indexData[0], GL_STATIC_DRAW);

		vao.release();
	}

	GLuint pid;
	GLint loc;

	//Create Shader Program
	program.create();
	{
		//Add Shaders from file
		if (hasTexCoords) {
			program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/phong_textured.vert");
			program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/phong_textured.frag");
		}
		else {
			program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/phong_color.vert");
			program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/phong_color.frag");
		}
		program.link();

		//Bind Vertex Data Location
		pid = program.programId();
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
std::vector<float> createSmokeBoundingBox(std::vector<size_t>& dims) {
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
void loadSmokeData(const std::string& fileName, std::vector<float>& data, std::vector<size_t>& dims, std::vector<float>& boundingBox)
{
	bool succ = readField(fileName, data, dims);
	while (!succ) {
		qDebug() << "Could not read Smoke Data, please select another File!";
		std::string newFileName = QFileDialog::getOpenFileName(Q_NULLPTR, "Open Smoke Data File", "", "Smoke Data (*.bin)").toStdString();
		succ = readField(newFileName, data, dims);
	}
	boundingBox = createSmokeBoundingBox(dims);
	qDebug() << "Successfully read Smoke Data!";


	// Check Smoke Data statistics
	{
		float maximum = 0.0f;
		float minimum = 1000.0f;
		float avg = 0.0f;
		int numZero = 0;
		int numNonInteger = 0;
		for (int i = 0; i < data.size(); i++) {
			float val = data[i];
			maximum = max(val, maximum);
			minimum = min(val, minimum);
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
void createSmokeRenderingPlanes(std::vector<float>& planesVerts, std::vector<GLuint>& planesInds) {
	//Measure Time
	QElapsedTimer timer;
	timer.start();

	//Maximum Depth to which the Smoke is Visible
	float maxDepth = 1.0;
	//Number of Slices
	int numSteps = NUM_SMOKE_SLICES;
	float stepSize = maxDepth / numSteps;

	qDebug() << "Stepsize:" << stepSize;

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
	qDebug() << "Creating Smoke Planes took" << timer.nsecsElapsed() * 0.000001 << "ms";
}

//Transform Smoke Data into array of Vertices for Particle Rendering
void processSmokeData(std::vector<float> data, std::vector<size_t> dims, std::vector<float>& smokeVerts, uint& numVerts)
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
	qDebug() << "Processed Smoke Data into " << numVerts << " Vertices, making Array of size " << smokeVerts.size() << ", took " << timer.elapsed() << "ms";

}
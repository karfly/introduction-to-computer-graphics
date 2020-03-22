#include <cassert>
#include "mesh.h"

#include "vertexrecorder.h"

using namespace std;

void Mesh::load( const char* filename )
{
	// 4.1. load() should populate bindVertices, currentVertices, and faces

    // draw obj mesh here
    // read vertices and face indices from vecv, vecn, vecf
    
	std::cout << "Reading mesh..." << std::endl;
	std::string line;

	std::ifstream infile(filename);
	while(std::getline(infile, line)) {
		std::stringstream lineStream(line);

		std::string lineType;
		lineStream >> lineType;

		if (lineType == "v") {
			Vector3f v;
			lineStream >> v[0] >> v[1] >> v[2];
			bindVertices.push_back(v);
		} else if (lineType == "f") {
			Tuple3u fTmp;
			lineStream >> fTmp[0] >> fTmp[1] >> fTmp[2];
			faces.push_back(Tuple3u(fTmp[0] - 1, fTmp[1] - 1, fTmp[2] - 1));
		} else {
			assert(false);
		}
	}

	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	// 4.2 Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".

	VertexRecorder rec;
	// for (size_t i = 0; i < faces.size(); ++i) {
	for (auto& face : faces) {
		Vector3f v_0 = currentVertices[face[0]];
		Vector3f v_1 = currentVertices[face[1]];
		Vector3f v_2 = currentVertices[face[2]];

		Vector3f n = Vector3f::cross(v_1 - v_0, v_2 - v_0);

		rec.record(v_0, n);
		rec.record(v_1, n);
		rec.record(v_2, n);
	}

	rec.draw();
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 4.3. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
	std::cout << "Reading attachments..." << std::endl;
	std::string line;

	std::ifstream infile(filename);
	while(std::getline(infile, line)) {
		std::stringstream lineStream(line);

		vector<float> weights;
		weights.push_back(0.0f);
		for (size_t i = 1; i < numJoints; ++i) {
			float weight;
			lineStream >> weight;

			weights.push_back(weight);
		}

		attachments.push_back(weights);
	}
}

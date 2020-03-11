#include <math.h>

#include "surf.h"
#include "vecmath.h"
#include "vertexrecorder.h"
using namespace std;

const float c_pi = 3.14159265358979323846f;

namespace
{
    
    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i=0; i<profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0) {
                    std::cout << "V: " << profile[i].V[2] << std::endl;
                    std::cout << "T: " << profile[i].T[2] << std::endl;
                    std::cout << "N: " << profile[i].N[2] << std::endl;
                    return false;
                }

    
        return true;
    }
}

// DEBUG HELPER
Surface quad() { 
	Surface ret;
	ret.VV.push_back(Vector3f(-1, -1, 0));
	ret.VV.push_back(Vector3f(+1, -1, 0));
	ret.VV.push_back(Vector3f(+1, +1, 0));
	ret.VV.push_back(Vector3f(-1, +1, 0));

	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));

	ret.VF.push_back(Tup3u(0, 1, 2));
	ret.VF.push_back(Tup3u(0, 2, 3));
	return ret;
}

Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
	// surface = quad();
    
    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    int n_points = profile.size();
    for (size_t i = 0; i < steps; ++i) {
        float t = 2.0f * c_pi * (float(i) / steps);

        for (size_t point_i = 0; point_i < n_points; ++point_i) {
            Matrix3f rotation_matrix(
                cos(t),  0.0f, sin(t),
                0.0f,    1.0f, 0.0f,
                -sin(t), 0.0,  cos(t)
            );

            Vector3f new_point = rotation_matrix * profile[point_i].V;
            Vector3f new_normal = -(rotation_matrix * profile[point_i].N);
        
            surface.VV.push_back(new_point);
            surface.VN.push_back(new_normal);
        }
    }

    for (size_t i = 0; i < steps; ++i) {
        for (size_t j = 0; j < n_points - 1; ++j) {
            int tl = j + i * n_points;
            int bl = (j + 1) + i * n_points;
            int tr = j + ((i + 1) % steps) * n_points;
            int br = (j + 1) + ((i + 1) % steps) * n_points;

            surface.VF.push_back(Tup3u(bl, tr, tl));
            surface.VF.push_back(Tup3u(bl, br, tr));
        }
    }
 
    return surface;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;
	// surface = quad();

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // TODO: Here you should build the surface.  See surf.h for details.
    for (size_t sweep_i = 0; sweep_i < sweep.size(); ++sweep_i) {
        for (size_t profile_i = 0; profile_i < profile.size(); ++profile_i) {
            Matrix4f M_sweep(
                sweep[sweep_i].N[0], sweep[sweep_i].B[0], sweep[sweep_i].T[0], sweep[sweep_i].V[0],
                sweep[sweep_i].N[1], sweep[sweep_i].B[1], sweep[sweep_i].T[1], sweep[sweep_i].V[1],
                sweep[sweep_i].N[2], sweep[sweep_i].B[2], sweep[sweep_i].T[2], sweep[sweep_i].V[2],
                0.0,  0.0,  0.0,  1.0
            );

            Matrix4f M_profile_point(
                profile[profile_i].N[0], profile[profile_i].B[0], profile[profile_i].T[0], profile[profile_i].V[0],
                profile[profile_i].N[1], profile[profile_i].B[1], profile[profile_i].T[1], profile[profile_i].V[1],
                profile[profile_i].N[2], profile[profile_i].B[2], profile[profile_i].T[2], profile[profile_i].V[2],
                0.0,  0.0,  0.0,  1.0
            );

            Matrix4f new_point = M_sweep * M_profile_point;

            surface.VV.push_back(Vector3f(new_point(0, 3), new_point(1, 3), new_point(2, 3)));
            surface.VN.push_back(-Vector3f(new_point(0, 0), new_point(1, 0), new_point(2, 0)));
        }
    }
    
    int sweep_size = sweep.size();
    int profile_size = profile.size();
    for (size_t i = 0; i < sweep.size(); ++i) {
        for (size_t j = 0; j < profile.size(); ++j) {
            int tl = j + i * profile_size;
            int bl = ((j + 1) % profile_size) + i * profile_size;
            int tr = j + ((i + 1) % sweep_size) * profile_size;
            int br = ((j + 1) % profile_size) + ((i + 1) % sweep_size) * profile_size;

            surface.VF.push_back(Tup3u(bl, tr, tl));
            surface.VF.push_back(Tup3u(bl, br, tr));
        }
    }

    cerr << "\t>>> makeGenCyl called (but not implemented).\n\t>>> Returning empty surface." <<endl;

    return surface;
}

void recordSurface(const Surface &surface, VertexRecorder* recorder) {
	const Vector3f WIRECOLOR(0.4f, 0.4f, 0.4f);
    for (int i=0; i<(int)surface.VF.size(); i++)
    {
		recorder->record(surface.VV[surface.VF[i][0]], surface.VN[surface.VF[i][0]], WIRECOLOR);
		recorder->record(surface.VV[surface.VF[i][1]], surface.VN[surface.VF[i][1]], WIRECOLOR);
		recorder->record(surface.VV[surface.VF[i][2]], surface.VN[surface.VF[i][2]], WIRECOLOR);
    }
}

void recordNormals(const Surface &surface, VertexRecorder* recorder, float len)
{
	const Vector3f NORMALCOLOR(0, 1, 1);
    for (int i=0; i<(int)surface.VV.size(); i++)
    {
		recorder->record_poscolor(surface.VV[i], NORMALCOLOR);
		recorder->record_poscolor(surface.VV[i] + surface.VN[i] * len, NORMALCOLOR);
    }
}

void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (int i=0; i<(int)surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (int i=0; i<(int)surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;
    
    for (int i=0; i<(int)surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}

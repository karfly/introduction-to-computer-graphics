#include "curve.h"
#include "vertexrecorder.h"
#include "vecmath.h"
using namespace std;

const float c_pi = 3.14159265358979323846f;

namespace
{
// Approximately equal to.  We don't want to use == because of
// precision issues with floating point.
inline bool approx(const Vector3f& lhs, const Vector3f& rhs)
{
	const float eps = 1e-8f;
	return (lhs - rhs).absSquared() < eps;
}


}

Vector3f evalBezierValues(const std::vector< Vector3f >& points, float t, int order = 0) {
	assert(points.size() == 4);
	assert(t >= 0.0 && t <= 1.0);
	assert(order == 0 || order == 1 || order == 2);

	Vector4f coeffs;
	switch(order)
	{
		case 0:
			coeffs[0] = (1 - t) * (1 - t) * (1 - t);
			coeffs[1] = 3 * t * (1 - t) * (1 - t);
			coeffs[2] = 3 * t * t * (1 - t);
			coeffs[3] = t * t * t;

			break;
		case 1:
			coeffs[0] = -3 * (1 - t) * (1 - t);
			coeffs[1] = 3 * (1 - t) * (1 - 3 * t);
			coeffs[2] = 3 * t * (2 - 3 * t);
			coeffs[3] = 2 * t * t;

			break;
		case 2:
			coeffs[0] = 6 * (1 - t);
			coeffs[1] = -6 * (2 - 3 * t);
			coeffs[2] = 6 * (1 - 3 * t);
			coeffs[3] = 6 * t;
	}


	Vector3f positionVector;
	for (size_t i = 0; i < 3; ++i) {
		float position = 0.0;
		for (size_t j = 0; j < 4; ++j) {
			position += coeffs[j] * points[j][i];
		}
		positionVector[i] = position;
	}

	return positionVector;
}


Vector3f evalBsplineValues(const std::vector< Vector3f >& points, float t, int order = 0) {
	assert(points.size() == 4);
	assert(t >= 0.0 && t <= 1.0);
	assert(order == 0 || order == 1 || order == 2);

	Vector4f coeffs;
	float t2 = t * t;
	float t3 = t * t * t;
	switch(order)
	{
		case 0:
			coeffs[0] = 1.0f/6 * (1 * (1) + t * (-3) + t2 * (3) + t3 * (-1));
			coeffs[1] = 1.0f/6 * (1 * (4) + t * (0) + t2 * (-6) + t3 * (3));
			coeffs[2] = 1.0f/6 * (1 * (1) + t * (3) + t2 * (3) + t3 * (-3));
			coeffs[3] = 1.0f/6 * (1 * (0) + t * (0) + t2 * (0) + t3 * (1));

			break;
		case 1:
			coeffs[0] = 1.0f/6 * (1 * (-3) + t * (6)   + t2 * (-3) + t3 * (0));
			coeffs[1] = 1.0f/6 * (1 * (0)  + t * (-12) + t2 * (9)  + t3 * (0));
			coeffs[2] = 1.0f/6 * (1 * (3)  + t * (6)   + t2 * (-9) + t3 * (0));
			coeffs[3] = 1.0f/6 * (1 * (0)  + t * (0)   + t2 * (3)  + t3 * (0));

			break;
		case 2:
			coeffs[0] = 1.0f/6 * (1 * (6)   + t * (-6)  + t2 * (0) + t3 * (0));
			coeffs[1] = 1.0f/6 * (1 * (-12) + t * (18)  + t2 * (0) + t3 * (0));
			coeffs[2] = 1.0f/6 * (1 * (6)   + t * (-18) + t2 * (0) + t3 * (0));
			coeffs[3] = 1.0f/6 * (1 * (0)   + t * (6)   + t2 * (0) + t3 * (0));
	}


	Vector3f positionVector;
	for (size_t i = 0; i < 3; ++i) {
		float position = 0.0;
		for (size_t j = 0; j < 4; ++j) {
			position += coeffs[j] * points[j][i];
		}
		positionVector[i] = position;
	}

	return positionVector;
}



Curve evalBezier(const vector< Vector3f >& P, unsigned steps)
{
	// Check
	if (P.size() < 4 || P.size() % 3 != 1)
	{
		cerr << "evalBezier must be called with 3n+1 control points." << endl;
		exit(0);
	}

	// TODO:
	// You should implement this function so that it returns a Curve
	// (e.g., a vector< CurvePoint >).  The variable "steps" tells you
	// the number of points to generate on each piece of the spline.
	// At least, that's how the sample solution is implemented and how
	// the SWP files are written.  But you are free to interpret this
	// variable however you want, so long as you can control the
	// "resolution" of the discretized spline curve with it.

	// Make sure that this function computes all the appropriate
	// Vector3fs for each CurvePoint: V,T,N,B.
	// [NBT] should be unit and orthogonal.

	// Also note that you may assume that all Bezier curves that you
	// receive have G1 continuity.  Otherwise, the TNB will not be
	// be defined at points where this does not hold.

	cerr << "\t>>> evalBezier has been called with the following input:" << endl;

	cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
	// for (int i = 0; i < (int)P.size(); ++i)
	// {
	// 	std::cout << "[ " << P[i][0] << " " << P[i][1] << " " << P[i][2] << " ]" << std::endl;
	// }

	Curve curve(steps + 1);

	size_t curve_i = 0;
	for (size_t start_i = 0; start_i < P.size(); start_i += 4) {
		Vector3f binormal_prev = Vector3f(0.0, 0.0, 1.0);

		for (size_t i = 0; i < steps; ++i)
		{
			// step from 0 to 1
			float t = float(i) / steps;

			// extract points of current piece
			std::vector< Vector3f > currentPoints;
			for (size_t point_i = start_i; point_i < start_i + 4; ++point_i) {
				Vector3f point(P[point_i][0], P[point_i][1], P[point_i][2]);
				currentPoints.push_back(point);
			}
			
			// Initialize position
			curve[curve_i].V = evalBezierValues(currentPoints, t, 0);

			// Tangent vector is first derivative
			auto tangent = evalBezierValues(currentPoints, t, 1);
			tangent.normalize();
			curve[curve_i].T = tangent;

			// Normal vector is second derivative
			auto normal = Vector3f::cross(binormal_prev, tangent);
			normal.normalize();
			curve[curve_i].N = normal;

			// Finally, binormal
			auto binormal = Vector3f::cross(tangent, normal);
			binormal.normalize();
			curve[curve_i].B = binormal;

			binormal_prev = binormal;

			curve_i++;
		}
	}

	return curve;
}

Curve evalBspline(const vector< Vector3f >& P, unsigned steps)
{
	// Check
	if (P.size() < 4)
	{
		cerr << "evalBspline must be called with 4 or more control points." << endl;
		exit(0);
	}

	// TODO:
	// It is suggested that you implement this function by changing
	// basis from B-spline to Bezier.  That way, you can just call
	// your evalBezier function.

	cerr << "\t>>> evalBSpline has been called with the following input:" << endl;

	cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
	for (int i = 0; i < (int)P.size(); ++i)
	{
		cerr << "\t>>> " << P[i] << endl;
	}

	cerr << "\t>>> Steps (type steps): " << steps << endl;

	Curve curve(steps * (P.size() - 3));
	size_t curve_i = 0;
	for (size_t start_i = 0; start_i < P.size() - 3; ++start_i) {
		Vector3f binormal_prev = Vector3f(0.0, 0.0, 1.0);

		for (size_t i = 0; i < steps; ++i)
		{
			// step from 0 to 1
			float t = float(i) / steps;

			// extract points of current piece
			std::vector< Vector3f > currentPoints;
			for (size_t point_i = start_i; point_i < start_i + 4; ++point_i) {
				Vector3f point(P[point_i][0], P[point_i][1], P[point_i][2]);
				currentPoints.push_back(point);
			}
			
			// Initialize position
			curve[curve_i].V = evalBsplineValues(currentPoints, t, 0);

			// Tangent vector is first derivative
			auto tangent = evalBsplineValues(currentPoints, t, 1);
			tangent.normalize();
			curve[curve_i].T = tangent;

			// Normal vector is second derivative
			auto normal = Vector3f::cross(binormal_prev, tangent);
			normal.normalize();
			curve[curve_i].N = normal;

			// Finally, binormal
			auto binormal = Vector3f::cross(tangent, normal);
			binormal.normalize();
			curve[curve_i].B = binormal;

			binormal_prev = binormal;

			curve_i++;
		}
	}

	return curve;
}

Curve evalCircle(float radius, unsigned steps)
{
	// This is a sample function on how to properly initialize a Curve
	// (which is a vector< CurvePoint >).

	// Preallocate a curve with steps+1 CurvePoints
	Curve R(steps + 1);

	// Fill it in counterclockwise
	for (unsigned i = 0; i <= steps; ++i)
	{
		// step from 0 to 2pi
		float t = 2.0f * c_pi * float(i) / steps;

		// Initialize position
		// We're pivoting counterclockwise around the y-axis
		R[i].V = radius * Vector3f(cos(t), sin(t), 0);

		// Tangent vector is first derivative
		R[i].T = Vector3f(-sin(t), cos(t), 0);

		// Normal vector is second derivative
		R[i].N = Vector3f(-cos(t), -sin(t), 0);

		// Finally, binormal is facing up.
		R[i].B = Vector3f(0, 0, 1);
	}

	return R;
}

void recordCurve(const Curve& curve, VertexRecorder* recorder)
{
	const Vector3f WHITE(1, 1, 1);
	for (int i = 0; i < (int)curve.size() - 1; ++i)
	{
		recorder->record_poscolor(curve[i].V, WHITE);
		recorder->record_poscolor(curve[i + 1].V, WHITE);
	}
}
void recordCurveFrames(const Curve& curve, VertexRecorder* recorder, float framesize)
{
	Matrix4f T;
	const Vector3f RED(1, 0, 0);
	const Vector3f GREEN(0, 1, 0);
	const Vector3f BLUE(0, 0, 1);
	
	const Vector4f ORGN(0, 0, 0, 1);
	const Vector4f AXISX(framesize, 0, 0, 1);
	const Vector4f AXISY(0, framesize, 0, 1);
	const Vector4f AXISZ(0, 0, framesize, 1);

	for (int i = 0; i < (int)curve.size(); ++i)
	{
		T.setCol(0, Vector4f(curve[i].N, 0));
		T.setCol(1, Vector4f(curve[i].B, 0));
		T.setCol(2, Vector4f(curve[i].T, 0));
		T.setCol(3, Vector4f(curve[i].V, 1));
 
		// Transform orthogonal frames into model space
		Vector4f MORGN  = T * ORGN;
		Vector4f MAXISX = T * AXISX;
		Vector4f MAXISY = T * AXISY;
		Vector4f MAXISZ = T * AXISZ;

		// Record in model space
		recorder->record_poscolor(MORGN.xyz(), RED);
		recorder->record_poscolor(MAXISX.xyz(), RED);

		recorder->record_poscolor(MORGN.xyz(), GREEN);
		recorder->record_poscolor(MAXISY.xyz(), GREEN);

		recorder->record_poscolor(MORGN.xyz(), BLUE);
		recorder->record_poscolor(MAXISZ.xyz(), BLUE);
	}
}


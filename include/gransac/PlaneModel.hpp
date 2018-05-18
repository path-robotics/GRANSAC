#pragma once

#include "gransac/AbstractModel.hpp"

typedef std::array<GRANSAC::VPFloat, 3> Vector3VP;

class Point3D
    : public GRANSAC::AbstractParameter
{
public:
  Point3D(GRANSAC::VPFloat x, GRANSAC::VPFloat y, GRANSAC::VPFloat z)
    {
	m_Point3D[0] = x;
	m_Point3D[1] = y;
	m_Point3D[2] = z;
    };

    Vector3VP m_Point3D;
};


inline GRANSAC::VPFloat Dot(const Point3D& Left, const Point3D& Right)
{
    return (Left.m_Point3D[0] * Right.m_Point3D[0] + Left.m_Point3D[1] * Right.m_Point3D[1] + Left.m_Point3D[2] * Right.m_Point3D[2]);
};

inline Point3D Cross(const Point3D& Left, const Point3D& Right)
{
    Point3D Result(0,0,0);
    Result.m_Point3D[0] = Left.m_Point3D[1] * Right.m_Point3D[2] - Left.m_Point3D[2] * Right.m_Point3D[1];
    Result.m_Point3D[1] = Left.m_Point3D[2] * Right.m_Point3D[0] - Left.m_Point3D[0] * Right.m_Point3D[2];
    Result.m_Point3D[2] = Left.m_Point3D[0] * Right.m_Point3D[1] - Left.m_Point3D[1] * Right.m_Point3D[0];
    return Result;
};

inline Point3D Normalize(const Point3D& V)
{
    GRANSAC::VPFloat Len = sqrt(Dot(V,V));
    if(Len == 0.0)
    {
        return V;
    }
    else
    {
        GRANSAC::VPFloat Factor = 1.0 / Len;
        return Point3D(V.m_Point3D[0] * Factor, V.m_Point3D[1] * Factor, V.m_Point3D[2] * Factor);
    }
};

struct PlaneParams {
    GRANSAC::VPFloat a;
    GRANSAC::VPFloat b;
    GRANSAC::VPFloat c;
    GRANSAC::VPFloat d;
};

inline PlaneParams ConstructFromPointNormal(const Point3D& Pt, const Point3D& Normal)
{
  PlaneParams Result;
  Point3D NormalizedNormal = Normalize(Normal);
  Result.a = NormalizedNormal.m_Point3D[0];
  Result.b = NormalizedNormal.m_Point3D[1];
  Result.c = NormalizedNormal.m_Point3D[2];
  Result.d = -Dot(Pt, NormalizedNormal);
  return Result;
};

inline Point3D Subtract(const Point3D& Left, const Point3D& Right)
{
  Point3D Result(0, 0, 0);
  Result.m_Point3D[0] = Left.m_Point3D[0] - Right.m_Point3D[0];
  Result.m_Point3D[1] = Left.m_Point3D[1] - Right.m_Point3D[1];
  Result.m_Point3D[2] = Left.m_Point3D[2] - Right.m_Point3D[2];
  return Result;
};

inline Point3D Add(const Point3D& Left, const Point3D& Right)
{
  Point3D Result(0, 0, 0);
  Result.m_Point3D[0] = Left.m_Point3D[0] + Right.m_Point3D[0];
  Result.m_Point3D[1] = Left.m_Point3D[1] + Right.m_Point3D[1];
  Result.m_Point3D[2] = Left.m_Point3D[2] + Right.m_Point3D[2];
  return Result;
};

inline PlaneParams ConstructFromPoints(const Point3D& V0, const Point3D& V1, const Point3D& V2)
{
  Point3D Normal = Normalize(Cross(Subtract(V1, V0), Subtract(V2, V0)));
  return ConstructFromPointNormal(V0, Normal);
};

class Plane3DModel
    : public GRANSAC::AbstractModel<3>
{
protected:
    // Parametric form
    GRANSAC::VPFloat m_a, m_b, m_c, m_d; // ax + by + cz + d = 0
    GRANSAC::VPFloat m_DistDenominator; // = sqrt(a^2 + b^2 + c^2). Stored for efficiency reasons

    // Hessian normal form - http://mathworld.wolfram.com/Plane.html
    GRANSAC::VPFloat m_n_x;
    GRANSAC::VPFloat m_n_y;
    GRANSAC::VPFloat m_n_z;
    GRANSAC::VPFloat m_p;

    virtual GRANSAC::VPFloat ComputeDistanceMeasure(std::shared_ptr<GRANSAC::AbstractParameter> Param) override
    {
	auto ExtPoint3D = std::dynamic_pointer_cast<Point3D>(Param);
	if(ExtPoint3D == nullptr)
	    throw std::runtime_error("Plane3DModel::ComputeDistanceMeasure() - Passed parameter are not of type Point3D.");

	// Return distance between passed "point" and this plane
	// http://mathworld.wolfram.com/Plane.html
    GRANSAC::VPFloat Dist = m_n_x * ExtPoint3D->m_Point3D[0] + m_n_y * ExtPoint3D->m_Point3D[1] + m_n_z * ExtPoint3D->m_Point3D[2] + m_p;

	return Dist;
    };

public:
    Plane3DModel(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams)
    {
	Initialize(InputParams);
    };

    virtual void Initialize(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams) override
    {
	if(InputParams.size() != 3)
	    throw std::runtime_error("Plane3DModel - Number of input parameters does not match minimum number required for this model.");

	// Check for AbstractParamter types
	auto Point1 = std::dynamic_pointer_cast<Point3D>(InputParams[0]);
	auto Point2 = std::dynamic_pointer_cast<Point3D>(InputParams[1]);
	auto Point3 = std::dynamic_pointer_cast<Point3D>(InputParams[2]);
	if(Point1 == nullptr || Point2 == nullptr || Point3 == nullptr)
	    throw std::runtime_error("Plane3DModel - InputParams type mismatch. It is not a Point3D.");

	std::copy(InputParams.begin(), InputParams.end(), m_MinModelParams.begin());

	// Compute the line parameters
    PlaneParams params = ConstructFromPoints(*Point1, *Point2, *Point3);
    m_a = params.a;
    m_b = params.b;
    m_c = params.c;
    m_d = params.d;

	m_DistDenominator = sqrt(m_a * m_a + m_b * m_b + m_c * m_c); // Cache square root for efficiency

	m_n_x = m_a / m_DistDenominator;
	m_n_y = m_b / m_DistDenominator;
	m_n_z = m_c / m_DistDenominator;
    m_p = m_d / m_DistDenominator;

    };

    virtual std::pair<GRANSAC::VPFloat, std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>> Evaluate(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &EvaluateParams, GRANSAC::VPFloat Threshold)
    {
	std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> Inliers;
	int nTotalParams = EvaluateParams.size();
	int nInliers = 0;

	for(auto& Param : EvaluateParams)
	{
	    if(ComputeDistanceMeasure(Param) < Threshold)
	    {
		Inliers.push_back(Param);
		nInliers++;
	    }
	}

	GRANSAC::VPFloat InlierFraction = GRANSAC::VPFloat(nInliers) / GRANSAC::VPFloat(nTotalParams); // This is the inlier fraction

	return std::make_pair(InlierFraction, Inliers);
    };
};

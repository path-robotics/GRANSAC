#include "gransac/GRANSAC.hpp"
#include "gransac/PlaneModel.hpp"
#include <gtest/gtest.h>

TEST(DotTest, Zero) {
  Point3D Left(0, 0, 0);
  Point3D Right(0, 0, 0);
  ASSERT_EQ(0, Dot(Left, Right));
}

TEST(DotTest, Positive) {
  Point3D Left(1, 1, 1);
  Point3D Right(1, 1, 1);
  ASSERT_EQ(3, Dot(Left, Right));
  Left = Point3D(2, 2, 2);
  ASSERT_EQ(6, Dot(Left, Right));
  Right = Point3D(2, 2, 2);
  ASSERT_EQ(12, Dot(Left, Right));
  Left = Point3D(sqrt(2), sqrt(2), sqrt(2));
  ASSERT_DOUBLE_EQ(6, Dot(Left, Left));
}

TEST(DotTest, PositiveAndNegative) {
  Point3D Left(-1, 0, -1);
  Point3D Right(1, 1, -1);
  ASSERT_EQ(0, Dot(Left, Right));
  Left = Point3D(-2, 2, 2);
  ASSERT_EQ(-2, Dot(Left, Right));
  Right = Point3D(2, 2, -2);
  ASSERT_EQ(-4, Dot(Left, Right));
  Left = Point3D(-sqrt(2), sqrt(2), -sqrt(2));
  ASSERT_DOUBLE_EQ(6, Dot(Left, Left));
}

TEST(CrossTest, Zero) {
  Point3D Left(0, 0, 0);
  Point3D Right(0, 0, 0);
  ASSERT_EQ(Left, Cross(Left, Right));
}

TEST(CrossTest, UnitVectors) {
  Point3D x_hat(1, 0, 0);
  Point3D y_hat(0, 1, 0);
  Point3D z_hat(0, 0, 1);
  Point3D neg_x_hat(-1, 0, 0);
  Point3D neg_y_hat(0, -1, 0);
  Point3D neg_z_hat(0, 0, -1);
  ASSERT_EQ(z_hat, Cross(x_hat, y_hat));
  ASSERT_EQ(neg_z_hat, Cross(y_hat, x_hat));
  ASSERT_EQ(y_hat, Cross(z_hat, x_hat));
  ASSERT_EQ(neg_y_hat, Cross(x_hat, z_hat));
  ASSERT_EQ(x_hat, Cross(y_hat, z_hat));
  ASSERT_EQ(neg_x_hat, Cross(z_hat, y_hat));

  ASSERT_EQ(z_hat, Cross(neg_x_hat, neg_y_hat));
  ASSERT_EQ(neg_z_hat, Cross(neg_y_hat, neg_x_hat));
  ASSERT_EQ(y_hat, Cross(neg_z_hat, neg_x_hat));
  ASSERT_EQ(neg_y_hat, Cross(neg_x_hat, neg_z_hat));
  ASSERT_EQ(x_hat, Cross(neg_y_hat, neg_z_hat));
  ASSERT_EQ(neg_x_hat, Cross(neg_z_hat, neg_y_hat));
}

TEST(NormalizeTest, Zero) {
  Point3D zero_vec(0, 0, 0);
  ASSERT_EQ(zero_vec, Normalize(zero_vec));
}

TEST(NormalizeTest, UnitVectors) {
  Point3D x_hat(1, 0, 0);
  Point3D y_hat(0, 1, 0);
  Point3D z_hat(0, 0, 1);
  Point3D neg_x_hat(-1, 0, 0);
  Point3D neg_y_hat(0, -1, 0);
  Point3D neg_z_hat(0, 0, -1);
  ASSERT_EQ(x_hat, Normalize(x_hat));
  ASSERT_EQ(y_hat, Normalize(y_hat));
  ASSERT_EQ(z_hat, Normalize(z_hat));
  ASSERT_EQ(neg_x_hat, Normalize(neg_x_hat));
  ASSERT_EQ(neg_y_hat, Normalize(neg_y_hat));
  ASSERT_EQ(neg_z_hat, Normalize(neg_z_hat));
}

TEST(NormalizeTest, Positive) {
  Point3D val1(1/sqrt(3), 1/sqrt(3), 1/sqrt(3));
  Point3D val2(1, 1, 1);
  ASSERT_EQ(val1, Normalize(val2));
}

TEST(NormalizeTest, PositiveAndNegative) {
  Point3D val1(-1/sqrt(3), 1/sqrt(3), -1/sqrt(3));
  Point3D val2(-1, 1, -1);
  ASSERT_EQ(val1, Normalize(val2));
}

TEST(ConstructFromPointNormalTest, UnitPlanes) {
  Point3D origin(0, 0, 0);
  Point3D x_hat(1, 0, 0);
  Point3D y_hat(0, 1, 0);
  Point3D z_hat(0, 0, 1);
  Point3D neg_x_hat(-1, 0, 0);
  Point3D neg_y_hat(0, -1, 0);
  Point3D neg_z_hat(0, 0, -1);

  PlaneParams x_y_plane(0, 0, 1, 0);
  PlaneParams x_z_plane(0, 1, 0, 0);
  PlaneParams y_z_plane(1, 0, 0, 0);

  ASSERT_EQ(x_y_plane, ConstructFromPointNormal(origin, z_hat));
  ASSERT_EQ(x_y_plane, ConstructFromPointNormal(origin, neg_z_hat));
  ASSERT_EQ(x_z_plane, ConstructFromPointNormal(origin, y_hat));
  ASSERT_EQ(x_z_plane, ConstructFromPointNormal(origin, neg_y_hat));
  ASSERT_EQ(y_z_plane, ConstructFromPointNormal(origin, x_hat));
  ASSERT_EQ(y_z_plane, ConstructFromPointNormal(origin, neg_x_hat));
}

TEST(SubtractTest, Zero) {
  Point3D zero_vec(0, 0, 0);
  ASSERT_EQ(zero_vec, Subtract(zero_vec, zero_vec));
}

TEST(SubtractTest, Positive) {
  Point3D Left(1, 1, 1);
  Point3D Right(1, 1, 1);
  Point3D Result(0, 0, 0);
  ASSERT_EQ(Result, Subtract(Left, Right));
  Left = Point3D(2, 2, 2);
  Result = Point3D(1, 1, 1);
  ASSERT_EQ(Result, Subtract(Left, Right));
  Right = Point3D(3, 2, 1);
  Result = Point3D(-1, 0, 1);
  ASSERT_EQ(Result, Subtract(Left, Right));
  Left = Point3D(sqrt(2), sqrt(2), sqrt(2));
  Right = Point3D(1, 2, 3);
  Result = Point3D(sqrt(2)-1, sqrt(2)-2, sqrt(2)-3);
  ASSERT_EQ(Result, Subtract(Left, Right));
}

TEST(SubtractTest, PositiveAndNegative) {
  Point3D Left(1, 1, 1);
  Point3D Right(-2, 1, 2);
  Point3D Result(3, 0, -1);
  ASSERT_EQ(Result, Subtract(Left, Right));
  Left = Point3D(2, 2, 2);
  Result = Point3D(4, 1, 0);
  ASSERT_EQ(Result, Subtract(Left, Right));
  Right = Point3D(-10, -20, 0);
  Result = Point3D(12, 22, 2);
  ASSERT_EQ(Result, Subtract(Left, Right));
  Left = Point3D(sqrt(2), sqrt(2), sqrt(2));
  Right = Point3D(-1, -2, -3);
  Result = Point3D(sqrt(2)+1, sqrt(2)+2, sqrt(2)+3);
  ASSERT_EQ(Result, Subtract(Left, Right));
}

TEST(ConstructFromPointsTest, UnitPlanes) {
  Point3D origin(0, 0, 0);
  Point3D x_hat(1, 0, 0);
  Point3D y_hat(0, 1, 0);
  Point3D z_hat(0, 0, 1);
  Point3D neg_x_hat(-1, 0, 0);
  Point3D neg_y_hat(0, -1, 0);
  Point3D neg_z_hat(0, 0, -1);

  PlaneParams x_y_plane(0, 0, 1, 0);
  PlaneParams x_z_plane(0, 1, 0, 0);
  PlaneParams y_z_plane(1, 0, 0, 0);

  ASSERT_EQ(x_y_plane, ConstructFromPoints(origin, x_hat, y_hat));
  ASSERT_EQ(x_y_plane, ConstructFromPoints(origin, neg_x_hat, neg_y_hat));
  ASSERT_EQ(x_y_plane, ConstructFromPoints(origin, neg_x_hat, y_hat));
  ASSERT_EQ(x_y_plane, ConstructFromPoints(origin, x_hat, neg_y_hat));

  ASSERT_EQ(x_z_plane, ConstructFromPoints(origin, x_hat, z_hat));
  ASSERT_EQ(x_z_plane, ConstructFromPoints(origin, neg_x_hat, neg_z_hat));
  ASSERT_EQ(x_z_plane, ConstructFromPoints(origin, neg_x_hat, z_hat));
  ASSERT_EQ(x_z_plane, ConstructFromPoints(origin, x_hat, neg_z_hat));

  ASSERT_EQ(y_z_plane, ConstructFromPoints(origin, y_hat, z_hat));
  ASSERT_EQ(y_z_plane, ConstructFromPoints(origin, neg_y_hat, neg_z_hat));
  ASSERT_EQ(y_z_plane, ConstructFromPoints(origin, neg_y_hat, z_hat));
  ASSERT_EQ(y_z_plane, ConstructFromPoints(origin, y_hat, neg_z_hat));
}

TEST(Plane3DModelInitializeTest, UnitPlanes) {
  Point3D origin(0, 0, 0);
  Point3D x_hat(1, 0, 0);
  Point3D y_hat(0, 1, 0);
  Point3D z_hat(0, 0, 1);
  Point3D neg_x_hat(-1, 0, 0);
  Point3D neg_y_hat(0, -1, 0);
  Point3D neg_z_hat(0, 0, -1);

  PlaneParams x_y_plane(0, 0, 1, 0);
  PlaneParams x_z_plane(0, 1, 0, 0);
  PlaneParams y_z_plane(1, 0, 0, 0);

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > data_points;

  std::shared_ptr<GRANSAC::AbstractParameter> origin_ap = std::make_shared<Point3D>(origin.m_Point3D[0], origin.m_Point3D[1], origin.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> x_hat_ap = std::make_shared<Point3D>(x_hat.m_Point3D[0], x_hat.m_Point3D[1], x_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> y_hat_ap = std::make_shared<Point3D>(y_hat.m_Point3D[0], y_hat.m_Point3D[1], y_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> z_hat_ap = std::make_shared<Point3D>(z_hat.m_Point3D[0], z_hat.m_Point3D[1], z_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> neg_x_hat_ap = std::make_shared<Point3D>(neg_x_hat.m_Point3D[0], neg_x_hat.m_Point3D[1], neg_x_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> neg_y_hat_ap = std::make_shared<Point3D>(neg_y_hat.m_Point3D[0], neg_y_hat.m_Point3D[1], neg_y_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> neg_z_hat_ap = std::make_shared<Point3D>(neg_z_hat.m_Point3D[0], neg_z_hat.m_Point3D[1], neg_z_hat.m_Point3D[2]);

  // x-y plane
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_y_plane_points_0 = {origin_ap, x_hat_ap, y_hat_ap};
  Plane3DModel x_y_plane_0(x_y_plane_points_0);
  ASSERT_EQ(x_y_plane, x_y_plane_0.GetPlaneParams());

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_y_plane_points_1 = {origin_ap, neg_x_hat_ap, neg_y_hat_ap};
  Plane3DModel x_y_plane_1(x_y_plane_points_1);
  ASSERT_EQ(x_y_plane, x_y_plane_1.GetPlaneParams());

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_y_plane_points_2 = {origin_ap, neg_x_hat_ap, y_hat_ap};
  Plane3DModel x_y_plane_2(x_y_plane_points_2);
  ASSERT_EQ(x_y_plane, x_y_plane_2.GetPlaneParams());

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_y_plane_points_3 = {origin_ap, x_hat_ap, neg_y_hat_ap};
  Plane3DModel x_y_plane_3(x_y_plane_points_3);
  ASSERT_EQ(x_y_plane, x_y_plane_3.GetPlaneParams());

  // x-z plane
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_z_plane_points_0 = {origin_ap, x_hat_ap, z_hat_ap};
  Plane3DModel x_z_plane_0(x_z_plane_points_0);
  ASSERT_EQ(x_z_plane, x_z_plane_0.GetPlaneParams());

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_z_plane_points_1 = {origin_ap, neg_x_hat_ap, neg_z_hat_ap};
  Plane3DModel x_z_plane_1(x_z_plane_points_1);
  ASSERT_EQ(x_z_plane, x_z_plane_1.GetPlaneParams());

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_z_plane_points_2 = {origin_ap, neg_x_hat_ap, z_hat_ap};
  Plane3DModel x_z_plane_2(x_z_plane_points_2);
  ASSERT_EQ(x_z_plane, x_z_plane_2.GetPlaneParams());

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_z_plane_points_3 = {origin_ap, x_hat_ap, neg_z_hat_ap};
  Plane3DModel x_z_plane_3(x_z_plane_points_3);
  ASSERT_EQ(x_z_plane, x_z_plane_3.GetPlaneParams());

  // y-z plane
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > y_z_plane_points_0 = {origin_ap, y_hat_ap, z_hat_ap};
  Plane3DModel y_z_plane_0(y_z_plane_points_0);
  ASSERT_EQ(y_z_plane, y_z_plane_0.GetPlaneParams());

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > y_z_plane_points_1 = {origin_ap, neg_y_hat_ap, neg_z_hat_ap};
  Plane3DModel y_z_plane_1(y_z_plane_points_1);
  ASSERT_EQ(y_z_plane, y_z_plane_1.GetPlaneParams());

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > y_z_plane_points_2 = {origin_ap, neg_y_hat_ap, z_hat_ap};
  Plane3DModel y_z_plane_2(y_z_plane_points_2);
  ASSERT_EQ(y_z_plane, y_z_plane_2.GetPlaneParams());

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > y_z_plane_points_3 = {origin_ap, y_hat_ap, neg_z_hat_ap};
  Plane3DModel y_z_plane_3(y_z_plane_points_3);
  ASSERT_EQ(y_z_plane, y_z_plane_3.GetPlaneParams());
}

TEST(ComputeDistanceMeasureTest, OnPlane) {
  Point3D origin(0, 0, 0);
  Point3D x_hat(1, 0, 0);
  Point3D y_hat(0, 1, 0);
  Point3D z_hat(0, 0, 1);
  Point3D neg_x_hat(-1, 0, 0);
  Point3D neg_y_hat(0, -1, 0);
  Point3D neg_z_hat(0, 0, -1);

  PlaneParams x_y_plane(0, 0, 1, 0);
  PlaneParams x_z_plane(0, 1, 0, 0);
  PlaneParams y_z_plane(1, 0, 0, 0);

  std::shared_ptr<GRANSAC::AbstractParameter> origin_ap = std::make_shared<Point3D>(origin.m_Point3D[0], origin.m_Point3D[1], origin.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> x_hat_ap = std::make_shared<Point3D>(x_hat.m_Point3D[0], x_hat.m_Point3D[1], x_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> y_hat_ap = std::make_shared<Point3D>(y_hat.m_Point3D[0], y_hat.m_Point3D[1], y_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> z_hat_ap = std::make_shared<Point3D>(z_hat.m_Point3D[0], z_hat.m_Point3D[1], z_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> neg_x_hat_ap = std::make_shared<Point3D>(neg_x_hat.m_Point3D[0], neg_x_hat.m_Point3D[1], neg_x_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> neg_y_hat_ap = std::make_shared<Point3D>(neg_y_hat.m_Point3D[0], neg_y_hat.m_Point3D[1], neg_y_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> neg_z_hat_ap = std::make_shared<Point3D>(neg_z_hat.m_Point3D[0], neg_z_hat.m_Point3D[1], neg_z_hat.m_Point3D[2]);

  // x-y plane
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_y_plane_points_0 = {origin_ap, x_hat_ap, y_hat_ap};
  Plane3DModel x_y_plane_0(x_y_plane_points_0);
  ASSERT_EQ(0.0, x_y_plane_0.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, x_y_plane_0.ComputeDistanceMeasure(x_hat_ap));
  ASSERT_EQ(0.0, x_y_plane_0.ComputeDistanceMeasure(y_hat_ap));

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_y_plane_points_1 = {origin_ap, neg_x_hat_ap, neg_y_hat_ap};
  Plane3DModel x_y_plane_1(x_y_plane_points_1);
  ASSERT_EQ(0.0, x_y_plane_1.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, x_y_plane_1.ComputeDistanceMeasure(neg_x_hat_ap));
  ASSERT_EQ(0.0, x_y_plane_1.ComputeDistanceMeasure(neg_y_hat_ap));

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_y_plane_points_2 = {origin_ap, neg_x_hat_ap, y_hat_ap};
  Plane3DModel x_y_plane_2(x_y_plane_points_2);
  ASSERT_EQ(0.0, x_y_plane_2.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, x_y_plane_2.ComputeDistanceMeasure(neg_x_hat_ap));
  ASSERT_EQ(0.0, x_y_plane_2.ComputeDistanceMeasure(y_hat_ap));

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_y_plane_points_3 = {origin_ap, x_hat_ap, neg_y_hat_ap};
  Plane3DModel x_y_plane_3(x_y_plane_points_3);
  ASSERT_EQ(0.0, x_y_plane_3.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, x_y_plane_3.ComputeDistanceMeasure(x_hat_ap));
  ASSERT_EQ(0.0, x_y_plane_3.ComputeDistanceMeasure(neg_y_hat_ap));

  // x-z plane
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_z_plane_points_0 = {origin_ap, x_hat_ap, z_hat_ap};
  Plane3DModel x_z_plane_0(x_z_plane_points_0);
  ASSERT_EQ(0.0, x_z_plane_0.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, x_z_plane_0.ComputeDistanceMeasure(x_hat_ap));
  ASSERT_EQ(0.0, x_z_plane_0.ComputeDistanceMeasure(z_hat_ap));

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_z_plane_points_1 = {origin_ap, neg_x_hat_ap, neg_z_hat_ap};
  Plane3DModel x_z_plane_1(x_z_plane_points_1);
  ASSERT_EQ(0.0, x_z_plane_1.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, x_z_plane_1.ComputeDistanceMeasure(neg_x_hat_ap));
  ASSERT_EQ(0.0, x_z_plane_1.ComputeDistanceMeasure(neg_z_hat_ap));

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_z_plane_points_2 = {origin_ap, neg_x_hat_ap, z_hat_ap};
  Plane3DModel x_z_plane_2(x_z_plane_points_2);
  ASSERT_EQ(0.0, x_z_plane_2.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, x_z_plane_2.ComputeDistanceMeasure(neg_x_hat_ap));
  ASSERT_EQ(0.0, x_z_plane_2.ComputeDistanceMeasure(z_hat_ap));

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > x_z_plane_points_3 = {origin_ap, x_hat_ap, neg_z_hat_ap};
  Plane3DModel x_z_plane_3(x_z_plane_points_3);
  ASSERT_EQ(0.0, x_z_plane_3.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, x_z_plane_3.ComputeDistanceMeasure(x_hat_ap));
  ASSERT_EQ(0.0, x_z_plane_3.ComputeDistanceMeasure(neg_z_hat_ap));

  // y-z plane
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > y_z_plane_points_0 = {origin_ap, y_hat_ap, z_hat_ap};
  Plane3DModel y_z_plane_0(y_z_plane_points_0);
  ASSERT_EQ(0.0, y_z_plane_0.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, y_z_plane_0.ComputeDistanceMeasure(y_hat_ap));
  ASSERT_EQ(0.0, y_z_plane_0.ComputeDistanceMeasure(z_hat_ap));

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > y_z_plane_points_1 = {origin_ap, neg_y_hat_ap, neg_z_hat_ap};
  Plane3DModel y_z_plane_1(y_z_plane_points_1);
  ASSERT_EQ(0.0, y_z_plane_1.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, y_z_plane_1.ComputeDistanceMeasure(neg_y_hat_ap));
  ASSERT_EQ(0.0, y_z_plane_1.ComputeDistanceMeasure(neg_z_hat_ap));

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > y_z_plane_points_2 = {origin_ap, neg_y_hat_ap, z_hat_ap};
  Plane3DModel y_z_plane_2(y_z_plane_points_2);
  ASSERT_EQ(0.0, y_z_plane_2.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, y_z_plane_2.ComputeDistanceMeasure(neg_y_hat_ap));
  ASSERT_EQ(0.0, y_z_plane_2.ComputeDistanceMeasure(z_hat_ap));

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > y_z_plane_points_3 = {origin_ap, y_hat_ap, neg_z_hat_ap};
  Plane3DModel y_z_plane_3(y_z_plane_points_3);
  ASSERT_EQ(0.0, y_z_plane_3.ComputeDistanceMeasure(origin_ap));
  ASSERT_EQ(0.0, y_z_plane_3.ComputeDistanceMeasure(y_hat_ap));
  ASSERT_EQ(0.0, y_z_plane_3.ComputeDistanceMeasure(neg_z_hat_ap));
}

TEST(ComputeDistanceMeasureTest, OffPlane) {
  Point3D origin(0, 0, 0);
  Point3D ones(1, 1, 1);
  Point3D twos(2, 2, 2);
  Point3D x_hat(1, 0, 0);
  Point3D y_hat(0, 1, 0);
  Point3D z_hat(0, 0, 1);
  Point3D neg_x_hat(-1, 0, 0);
  Point3D neg_y_hat(0, -1, 0);
  Point3D neg_z_hat(0, 0, -1);

  PlaneParams x_y_plane(0, 0, 1, 0);
  PlaneParams x_z_plane(0, 1, 0, 0);
  PlaneParams y_z_plane(1, 0, 0, 0);

  std::shared_ptr<GRANSAC::AbstractParameter> origin_ap = std::make_shared<Point3D>(origin.m_Point3D[0], origin.m_Point3D[1], origin.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> ones_ap = std::make_shared<Point3D>(ones.m_Point3D[0], ones.m_Point3D[1], ones.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> twos_ap = std::make_shared<Point3D>(twos.m_Point3D[0], twos.m_Point3D[1], twos.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> x_hat_ap = std::make_shared<Point3D>(x_hat.m_Point3D[0], x_hat.m_Point3D[1], x_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> y_hat_ap = std::make_shared<Point3D>(y_hat.m_Point3D[0], y_hat.m_Point3D[1], y_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> z_hat_ap = std::make_shared<Point3D>(z_hat.m_Point3D[0], z_hat.m_Point3D[1], z_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> neg_x_hat_ap = std::make_shared<Point3D>(neg_x_hat.m_Point3D[0], neg_x_hat.m_Point3D[1], neg_x_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> neg_y_hat_ap = std::make_shared<Point3D>(neg_y_hat.m_Point3D[0], neg_y_hat.m_Point3D[1], neg_y_hat.m_Point3D[2]);
  std::shared_ptr<GRANSAC::AbstractParameter> neg_z_hat_ap = std::make_shared<Point3D>(neg_z_hat.m_Point3D[0], neg_z_hat.m_Point3D[1], neg_z_hat.m_Point3D[2]);

  // ones plane
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > ones_plane_points_0 = {x_hat_ap, y_hat_ap, z_hat_ap};
  Plane3DModel ones_plane_0(ones_plane_points_0);
  ASSERT_DOUBLE_EQ(1/sqrt(3), ones_plane_0.ComputeDistanceMeasure(origin_ap));
  ASSERT_DOUBLE_EQ(2/sqrt(3), ones_plane_0.ComputeDistanceMeasure(ones_ap));
  ASSERT_DOUBLE_EQ(5/sqrt(3), ones_plane_0.ComputeDistanceMeasure(twos_ap));

  // ones plane reversed
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > ones_plane_points_1 = {z_hat_ap, y_hat_ap, x_hat_ap};
  Plane3DModel ones_plane_1(ones_plane_points_1);
  ASSERT_DOUBLE_EQ(1/sqrt(3), ones_plane_1.ComputeDistanceMeasure(origin_ap));
  ASSERT_DOUBLE_EQ(2/sqrt(3), ones_plane_1.ComputeDistanceMeasure(ones_ap));
  ASSERT_DOUBLE_EQ(5/sqrt(3), ones_plane_1.ComputeDistanceMeasure(twos_ap));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

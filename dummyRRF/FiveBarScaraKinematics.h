/*
 * FiveBarScaraKinematics.h
 *
 *  Created on: 11 Nov 2018
 *      Author: Joerg
 *
 *	documentation: https://duet3d.dozuki.com/Guide/Five+Bar+Parallel+SCARA/24?lang=en
 */

#ifndef SRC_MOVEMENT_KINEMATICS_FIVEBARSCARAKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_FIVEBARSCARAKINEMATICS_H_

#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <cmath>

// Standard setup for 5 Bar SCARA (parallel SCARA) machines assumed by this firmware

enum class Arm : uint8_t
{
	left,
	right
};


inline float fsquare(float a) { return a*a; }

inline constexpr float min(float _a, float _b)
{
  return (std::isnan(_a) || _a < _b) ? _a : _b;
}

inline constexpr float max(float _a, float _b)
{
  return (std::isnan(_a) || _a > _b) ? _a : _b;
}

constexpr size_t XYZ_AXES = 3;// The number of Cartesian axes
constexpr size_t X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2, E0_AXIS = 3;// The indices of the Cartesian axes in drive arrays


class FiveBarScaraKinematics
{
public:
	// Constructors
	FiveBarScaraKinematics();


	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const ;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const ;
	bool IsReachable(float x, float y, bool isCoordinated) const ;

	static constexpr float DefaultSegmentsPerSecond = 100.0;
	static constexpr float DefaultMinSegmentSize = 0.2;

	static constexpr const char *Home5BarScaraFileName = "home5barscara.g";

	void Recalc();
	int getQuadrant(float x, float y) const;
	bool isCantilevered(int mode) const;
	float getAbsoluteAngle(float xOrig, float yOrig, float xDest, float yDest) const;
	void getIntersec(float result12[], float firstRadius, float secondRadius, float firstX, float firstY, float secondX, float secondY) const;
	void getTheta(float result[], float proximal, float distal, float proxX, float proxY, float destX, float destY, Arm arm) const;
	void getXYFromAngle(float resultcoords[], float angle, float length, float origX, float origY) const;
	void getForward(float resultcoords[], float thetaL, float thetaR) const;
	void getInverse(const float coords[]) const;
	float getAngle(float x1, float y1, float xAngle, float yAngle, float x2, float y2) const;
	float getTurn(float x1, float y1, float xAngle, float yAngle, float x2, float y2) const;
	bool isPointInsideDefinedPrintableArea(float x0, float y0) const;
	bool constraintsOk(const float coords[]) const;

	// Primary parameters
	float xOrigL;
	float yOrigL;
	float xOrigR;
	float yOrigR;
	float proximalL;
	float proximalR;
	float distalL;
	float distalR;
	float cantL;
	float cantR;
	int workmode;
	float homingAngleL;
	float homingAngleR;

	bool printAreaDefined;
	float printArea[4];	// x1, y1, x2, y2

	float constrMin;
	float constrMax;
	float proxDistLAngleMin;
	float proxDistLAngleMax;
	float proxDistRAngleMin;
	float proxDistRAngleMax;
	float actuatorAngleLMin;
	float actuatorAngleLMax;
	float actuatorAngleRMin;
	float actuatorAngleRMax;

	// Derived parameters

	// State variables
	mutable float cachedX0, cachedY0, cachedThetaL, cachedThetaR, cachedXL, cachedXR, cachedYL, cachedYR,
		cachedX1, cachedY1;
	mutable bool cachedInvalid;
};

#endif /* SRC_MOVEMENT_KINEMATICS_FIVEBARSCARAKINEMATICS_H_ */

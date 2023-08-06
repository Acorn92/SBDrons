#ifndef MATHMODELQUADROTOR_HPP
#define MATHMODELQUADROTOR_HPP

#include <typesData.hpp>
#include <math.hpp>
#include <message.hpp>



class MathModelQuadrotor
{
	public:
		MathModelQuadrotor(const ParamsQuadrotor *paramsQuadrotor, const ParamsSimulator *paramsSimulator);
		StateVector		calculateStateVector(StateVector &lastStateVector, VectorXd_t rotorsAngularVelocity);
	private:
		const ParamsQuadrotor	*paramsQuadrotor;
		const ParamsSimulator	*paramsSimulator;
		Eigen::Vector3d			acceleration;
		Eigen::Vector3d			acceleration2;
		Eigen::Vector3d			velocity;
		Eigen::Vector3d			position;
		Eigen::Vector3d			angularAcceleration;
		Eigen::Vector3d			angularAcceleration2;
		Eigen::Vector3d			angularVelocity;
		Eigen::Vector3d			orientation;
	public:	
		StateVector		functionRight(StateVector &lastStateVector, VectorXd_t rotorsAngularVelocity);
		Eigen::Vector3d TestMatrRotation(StateVector &lastStateVector);

		StateVector TestMathModel(StateVector &lastStateVector, VectorXd_t testRotorsAngularVelocity);
};

// размер вектора значений правой части
const int SIZE_FUNCTION_RIGHT = 6;

const int NUMBER_OF_AXIS = 3;

#endif
#ifndef UAVCONTROLSYSTEM_HPP
#define UAVCONTROLSYSTEM_HPP

#include "typesData.hpp"
#include "math.hpp"
#include "motionPlanner.hpp"
#include "PID.hpp"
#include <cmath>
#include <memory>

class PID_Circuit
{
	public:
		PID_Circuit (const Eigen::Vector3d Kp, const Eigen::Vector3d Ki, const Eigen::Vector3d Kd, double min, double max);
		Eigen::Vector3d	 output(Eigen::Vector3d	 &inputValue, Eigen::Vector3d	 &targetValue, double dt);
		
	private:
		std::unique_ptr<PID[]> circuit;
		int countCircuit;	
};



class UAVControlSystem
{
	public:
		UAVControlSystem(const ParamsControlSystem *paramsControlSystem, const ParamsSimulator *paramsSimulator,
						 const ParamsQuadrotor *paramsQuadrotor, MotionPlanner* motionPlanner);
		VectorXd_t					calculateMotorVelocity(StateVector stateVector, MatrixXd_t targetPoints, double time);
		
		bool 				checkRadius(const VectorXd_t& waypoint);
	private:
		const ParamsSimulator		*paramsSimulator;
		const ParamsQuadrotor		*paramsQuadrotor;
		const ParamsControlSystem	*parContrlSyst;
		
		std::unique_ptr<PID_Circuit> position; //ПИД по позиции(вход - положение в системе координат, выход углы)
		std::unique_ptr<PID_Circuit> velocity; //ПИД по скорости(вход - скорость, выход ускорение)
		std::unique_ptr<PID_Circuit> angle; //ПИД по углу(вход - наклон по оси, выход скорость по оси)
		std::unique_ptr<PID> thrust;//Пид по тяге

		//текущие значения
		Eigen::Vector3d	 			currentPosition;//текущая позиция
		Eigen::Vector3d				currentVelocity;//текущая скорость
		Eigen::Vector3d				currentAcceleration;//текущий наклон

		// Ошибки
		Eigen::Vector3d				angularRateError;
		Eigen::Vector3d				angularPositionError;
		Eigen::Vector3d				positionError;
		Eigen::Vector3d				velocityError;
		Eigen::Vector3d				accelerationError;
	
		Eigen::Vector3d				angularRateErrorPast;
		Eigen::Vector3d				angularPositionErrorPast;
		Eigen::Vector3d				positionErrorPast;
		Eigen::Vector3d				velocityErrorPast;
		Eigen::Vector3d				accelerationErrorPast;

		// Интеграл от ошибки
		Eigen::Vector3d				integralAngularRateError;
		Eigen::Vector3d				integralAngleError;
		Eigen::Vector3d				integralPoseError;
		Eigen::Vector3d				integralVelocityError;
		Eigen::Vector3d				integralAccelerationError;

		// Производная от ошибки
		Eigen::Vector3d				derivativeAngularRateError;
		Eigen::Vector3d				derivativeAngleError;
		Eigen::Vector3d				derivativePoseError;
		Eigen::Vector3d				derivativeVelocityError;
		Eigen::Vector3d				derivativeAccelerationError;

		// Целевые параметры управления
		double						desTang;// целевая тяга(координата Z)
		double 						desYaw;
		Eigen::Vector3d				desiredPosition;
		Eigen::Vector3d				desiredPositionP;
		Eigen::Vector3d				desiredVelocity;
		Eigen::Vector3d				desiredAcceleration;
		Eigen::Vector3d				desiredTorque;
		Eigen::Vector3d				desiredAngle;
		Eigen::Vector2d				desiredAngleHorizontal;
		Eigen::Vector3d				desiredAngularRate;
		Eigen::Vector3d				desiredAngularAcceleration;

		

		VectorXd_t					mixerCommands;
		StateVector					stateVector;
		double						time;
		double						timeTrajectory; // время, за которое БЛА пролетает траекторию
		double						timeStopTrajectory; // время, для остановки движения дрона, после достижения конечной точки
		int							indexPoint; // текущая точка, к которой летит БЛА
		bool						stopTime; // отключает расчёт траектории от времени

		MotionPlanner*				motionPlanner;

		VectorXd_t			mixer();
		
		void				PIDThrust();
		void				PIDPosition();
		void				PIDAngles();
		void				PIDAngularRate();
		void				fillDesiredPosition(VectorXd_t targetPoints);
		void				fillDesiredPositionForOPt();
		void				saturation(double &arg, double min, double max);
		double				commandThrustToOmegaRotors(double commandThrust);
};

#endif
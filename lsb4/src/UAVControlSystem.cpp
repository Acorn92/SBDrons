#include "UAVControlSystem.hpp"

#define DEBUG
#ifdef DEBUG
	#include <iostream>
#endif

PID_Circuit::PID_Circuit (const Eigen::Vector3d Kp, const Eigen::Vector3d Ki, const Eigen::Vector3d Kd)
{		
	circuit = new PID[3];
	for (int i = 0; i < 3; i++)
		circuit[i] = PID(Kp[i], 
						 Ki[i], 
					 	 Kd[i]);
}

UAVControlSystem::UAVControlSystem(const ParamsControlSystem *paramsControlSystem, const ParamsSimulator *paramsSimulator,
								   const ParamsQuadrotor *paramsQuadrotor, MotionPlanner* pathPlaner)
{	
	PID_Circuit velocity(paramsControlSystem->KpAngularRate, paramsControlSystem->KiAngularRate, paramsControlSystem->KdAngularRate);	
	PID_Circuit angle(paramsControlSystem->KpAngularRate, paramsControlSystem->KiAngularRate, paramsControlSystem->KdAngularRate);
	PID_Circuit position(paramsControlSystem->KpAngularRate, paramsControlSystem->KiAngularRate, paramsControlSystem->KdAngularRate);
	//TODO: продолжить разработку системы управления. таймкод вебинара: 31:40
}

/**
 * @brief расчёт угловых скоростей роторов
 * 
 * @param stateVector вектор состояния БЛА
 * @param targetPoints целевая точка(также могут входить данные о скорости и ускорении)
 * @param time 
 * @return VectorXd_t 
 */
VectorXd_t	UAVControlSystem::calculateMotorVelocity(VectorXd_t stateVector, MatrixXd_t targetPoints, double time)
{
}

/**
 * @brief Алгоритм смешивания комманд
 * 
 * @return угловая скорость вращения роторов 
 */
VectorXd_t	UAVControlSystem::mixer()
{

}

/**
 * @brief функция, которая заполняет вектор желаемой позиции
 * 
 * @param targetPoints массив точек
 */
void		UAVControlSystem::fillDesiredPosition(MatrixXd_t targetPoints)
{
}

/**
 * @brief ПИД по тяги
 * 
 */
void		UAVControlSystem::PIDThrust()
{
}


/**
 * @brief ПИД по позиции
 * 
 */
void		UAVControlSystem::PIDPosition()
{
}

/**
 * @brief ПИД по углу
 * 
 */
void		UAVControlSystem::PIDAngles()
{
}

/**
 * @brief ПИД по угловой скорости
 * 
 */
void UAVControlSystem::PIDAngularRate(const ParamsControlSystem *paramsControlSystem)
{
}

/**
 * @brief проверка попадания в радиус целевой позиции
 * 
 * @param targetPoints целевая позиция
 * @return true - принадлежим сфере 
 * @return false - не принадлежим сфере
 */
bool		UAVControlSystem::checkRadius(VectorXd_t targetPoints)
{
}

/**
 * @brief ограничивает аргумент
 * 
 * @param arg аргумент, который требуется ограничить
 * @param min минимальное значение ограничения
 * @param max максимальное значение ограничения
 */
void		UAVControlSystem::saturation(double &arg, double min, double max)
{
}

/**
 * @brief перевод из команды по тяги в угловую скорость
 * 
 * @param commandThrust команда по тяги
 * @return угловая скорость ротора
 */
double		UAVControlSystem::commandThrustToOmegaRotors(double commandThrust)
{
}

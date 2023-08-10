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

Eigen::Vector3d	 PID_Circuit::output(Eigen::Vector3d	 inputValue, Eigen::Vector3d	 targetValue, double dt)
{
	Eigen::Vector3d	 res;
	for (int i = 0; i < 3; i++)
		res[i] = circuit[i].update(inputValue[i], targetValue[i], dt);
	return res;

}

UAVControlSystem::UAVControlSystem(const ParamsControlSystem *paramsControlSystem, const ParamsSimulator *paramsSimulator,
								   const ParamsQuadrotor *paramsQuadrotor, MotionPlanner* pathPlaner)
{	
	PID_Circuit velocity(paramsControlSystem->KpAngularRate, paramsControlSystem->KiAngularRate, paramsControlSystem->KdAngularRate);	
	PID_Circuit angle(paramsControlSystem->KpAngle, paramsControlSystem->KiAngle, paramsControlSystem->KdAngle);
	position = new PID_Circuit(paramsControlSystem->KpPosition, paramsControlSystem->KiPosition, paramsControlSystem->KdPosition);
	//TODO: продолжить разработку системы управления. таймкод вебинара: 31:40
	motionPlanner = pathPlaner;
	this->indexPoint = 0;
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
	//в этой функции делаем управление
	this->indexPoint = time;
	//пока не пролетели все точки из траектории
	// //while (this->indexPoint < motionPlanner->getSizeTimeTrajectory())
	// while (time < 1/*количество точек*/)
	// {
		//назначем точку как целевую
		this->fillDesiredPosition(targetPoints);
	
		//пока не прилетели
		while (checkRadius(targetPoints))
		{
			//летим
			this->PIDPosition();
			this->PIDAngles();
			this->PIDAngularRate();
		}
		//миксер
		//летим в следующую
		
	// }
	return this->mixerCommands;
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
void UAVControlSystem::fillDesiredPosition(MatrixXd_t targetPoints)
{
	for (int i = 0; i < 3; i++)
		this->desiredPosition[i] = targetPoints(this->indexPoint,i);
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
	//TODO - получить тягу из позиции Z
	Eigen::Vector3d	 pos;
	for (int i = 0; i < 3; i++)
		pos[i] = this->stateVector[i];
	desiredVelocity = position->output(pos, this->desiredPosition, paramsSimulator->dt);
	//pos[2] - должнв быть команда по тяге
	
}

/**
 * @brief ПИД по углу
 * 
 */
void		UAVControlSystem::PIDAngles()
{
	desiredAngle = position->output(desiredVelocity, this->desiredPosition, paramsSimulator->dt);
}

/**
 * @brief ПИД по угловой скорости
 * 
 */
void UAVControlSystem::PIDAngularRate()
{
	desiredAngularRate = position->output(desiredAngle, this->desiredPosition, paramsSimulator->dt);
}

/**
 * @brief проверка попадания в радиус целевой позиции
 * 
 * @param targetPoints целевая позиция
 * @return true - принадлежим сфере 
 * @return false - не принадлежим сфере
 */
bool UAVControlSystem::checkRadius(MatrixXd_t targetPoints)
{
	bool res;
	for (int i = 0; i < 3; i++)
		if ((targetPoints(this->indexPoint, i) - stateVector[i]) <= 0.1) 
			res = true;
		else
			res = false;

	return res;
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
double UAVControlSystem::commandThrustToOmegaRotors(double commandThrust)
{
	//Pdes - тяга
}

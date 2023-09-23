#include "UAVControlSystem.hpp"

#define DEBUG
#ifdef DEBUG
	#include <iostream>
#endif

//TODO - добавить векторы минимума и максимума
PID_Circuit::PID_Circuit (const Eigen::Vector3d Kp, const Eigen::Vector3d Ki, const Eigen::Vector3d Kd, double min, double max)
{		
	this->circuit = std::unique_ptr<PID[]>(new PID[3]);
	for (int i = 0; i < 3; i++)
	{
		this->circuit[i] = PID(Kp[i], Ki[i], Kd[i], min, max);
		// this->circuit[i] = PID(1, 1, 1);
	}
}

Eigen::Vector3d	 PID_Circuit::output(Eigen::Vector3d &inputValue, Eigen::Vector3d &targetValue, double dt)
{
	//TODO - проверить работу каскада ПИДов
	Eigen::Vector3d	 res;
	for (int i = 0; i < 3; i++)
	{
		res[i] = this->circuit[i].update(inputValue[i], targetValue[i], dt);
	}
	return res;

}

//TODO: вынести переменные в поля класса velocity и angle
UAVControlSystem::UAVControlSystem(const ParamsControlSystem *paramsControlSystem, const ParamsSimulator *paramsSimulator,
								   const ParamsQuadrotor *paramsQuadrotor, MotionPlanner* pathPlaner)
{	
	this->velocity = new PID_Circuit(paramsControlSystem->KpAngularRate, paramsControlSystem->KiAngularRate, paramsControlSystem->KdAngularRate, -3000, 3000);	
	// PID_Circuit angle(paramsControlSystem->KpAngle, paramsControlSystem->KiAngle, paramsControlSystem->KdAngle);
	this->position = new PID_Circuit(paramsControlSystem->KpPosition, paramsControlSystem->KiPosition, paramsControlSystem->KdPosition, -3000, 3000);
	//TODO: продолжить разработку системы управления. таймкод вебинара: 31:40
	motionPlanner = pathPlaner;
	this->paramsSimulator = paramsSimulator;
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
VectorXd_t	UAVControlSystem::calculateMotorVelocity(StateVector stateVector, MatrixXd_t targetPoints, double time)
{
	for (int i = 0; i < 3; i++)
		currentPosition[i] = stateVector[i];
	for (int i = 0; i < 3; i++)
		desiredAngle[i] = stateVector[6 + i];
	this->mixerCommands = VectorXd_t(4);
	//в этой функции делаем управление
	this->indexPoint = time;
	this->stateVector = stateVector;
	//пока не пролетели все точки из траектории
	// //while (this->indexPoint < motionPlanner->getSizeTimeTrajectory())
	// while (time < 1/*количество точек*/)
	// {
		//назначем точку как целевую
	this->fillDesiredPosition(targetPoints);
	
		//пока не прилетели
		//while (checkRadius(targetPoints))
		//{
			//летим
	this->PIDPosition();
	// this->PIDAngles();
	this->PIDAngularRate();
		//}
		//миксер
		//летим в следующую
	mixer();
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
	VectorXd_t res;

	this->mixerCommands << this->desiredVelocity[2] + this->desiredAngularRate[0] - this->desiredAngularRate[2],
						   this->desiredVelocity[2] - this->desiredAngularRate[1] + this->desiredAngularRate[2],
						   this->desiredVelocity[2] - this->desiredAngularRate[0] - this->desiredAngularRate[2],
						   this->desiredVelocity[2] + this->desiredAngularRate[1] + this->desiredAngularRate[2];
	return res;
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
	desTang = targetPoints(this->indexPoint,3);//тангаж
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
	this->desiredVelocity = this->position->output(currentPosition, this->desiredPosition, this->paramsSimulator->dt);
	this->desiredVelocity[0] = 0; 
	this->desiredVelocity[1] = 0;
}

/**
 * @brief ПИД по углу
 * 
 */
void		UAVControlSystem::PIDAngles()
{
	// Eigen::Vector3d angel;
	// angel << this->desiredVelocity[1], this->desiredVelocity[2], this->desTang;
	// desiredAngle = position->output(angel, this->desiredPosition, paramsSimulator->dt);
}

/**
 * @brief ПИД по угловой скорости
 * 
 */
void UAVControlSystem::PIDAngularRate()
{
	Eigen::Vector3d desiredAR(0, 0, 0);
	this->desiredAngularRate = this->velocity->output(desiredAngle, desiredAR, paramsSimulator->dt);
}

/**
 * @brief проверка попадания в радиус целевой позиции
 * 
 * @param targetPoints целевая позиция
 * @return true - принадлежим сфере 
 * @return false - не принадлежим сфере
 */
bool UAVControlSystem::checkRadius(const Eigen::Vector3d& waypoint)
{
	if ((this->currentPosition - waypoint).norm() <= 0.01)
		return (true);
	else
		return (false);
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

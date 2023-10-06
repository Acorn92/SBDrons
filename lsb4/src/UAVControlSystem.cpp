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
	this->position = std::unique_ptr<PID_Circuit>(new PID_Circuit(paramsControlSystem->KpPosition, paramsControlSystem->KiPosition, paramsControlSystem->KdPosition, -0.5, 0.5));	
	//перенастроить
	this->angle = std::unique_ptr<PID_Circuit>(new PID_Circuit(paramsControlSystem->KpAngle, paramsControlSystem->KiAngle, paramsControlSystem->KdAngle, -5, 5));
	this->velocity = std::unique_ptr<PID_Circuit>(new PID_Circuit(paramsControlSystem->KpAngularRate, paramsControlSystem->KiAngularRate, paramsControlSystem->KdAngularRate, -15, 15));	
	

	this->thrust = std::unique_ptr<PID>(new PID(paramsControlSystem->KpPosition[2],paramsControlSystem->KiPosition[2], paramsControlSystem->KdPosition[2], 1500, 2631));
	//TODO: продолжить разработку системы управления. таймкод вебинара: 31:40
	motionPlanner = pathPlaner;
	this->paramsSimulator = paramsSimulator;
	this->indexPoint = 0;
	this->desiredAngularRate << 0, 0, 0;
	this->desiredAngularRate << 0, 0, 0;
	this->desTang = 0;
	this->desiredPosition << 0, 0, 0;
	this->desiredPositionP << 0, 0, 0;
}

/**
 * @brief расчёт угловых скоростей роторов
 * 
 * @param stateVector вектор состояния БЛА
 * @param targetPoints целевая точка(также могут входить данные о скорости и ускорении)
 * @param time 
 * @return VectorXd_t 
 */
VectorXd_t	UAVControlSystem::calculateMotorVelocity(StateVector stateVector, VectorXd_t targetPoints, double time)
{
	for (int i = 0; i < 3; i++)
	{
		this->currentPosition[i] = stateVector[i];
		// this->currentVelocity[i] = stateVector[INDEX_VELOCITY_STATE + i];
		this->currentVelocity[i] = stateVector[INDEX_RATE_STATE + i];
		// INDEX_RATE_STATE
		this->currentAcceleration[i] = stateVector[INDEX_ANGLE_POSITION_STATE + i];
	}
	this->mixerCommands = VectorXd_t(4);
	this->stateVector = stateVector;
	fillDesiredPosition(targetPoints);
	
	// for (int i = 0; i < 2; i++)
	// {
	// 	if ((abs(desiredPosition[i] - currentPosition[i]) <= 0.5) && (abs(currentPosition[i] - desiredPosition[i]) <= 0.1))

    // 		desiredPosition[i] = (desiredPosition[i] != desiredPositionP[i]) ? (currentPosition[i] + 1) : desiredPositionP[i];
	// }

	

	this->PIDThrust();
	this->PIDPosition();
	this->PIDAngles();
	this->PIDAngularRate();
	mixer();

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

	this->mixerCommands << this->desTang + this->desiredAngularRate[0] - this->desiredAngularRate[2],
						   this->desTang - this->desiredAngularRate[1] + this->desiredAngularRate[2],
						   this->desTang - this->desiredAngularRate[0] - this->desiredAngularRate[2],
						   this->desTang + this->desiredAngularRate[1] + this->desiredAngularRate[2];
	return res;
}

/**
 * @brief функция, которая заполняет вектор желаемой позиции
 * 
 * @param targetPoints массив точек
 */
void UAVControlSystem::fillDesiredPosition(VectorXd_t targetPoints)
{
	for (int i = 0; i < 3; i++)
		this->desiredPosition[i] = targetPoints(i);
	this->desiredPosition[2] = targetPoints(2);
	this->desYaw = targetPoints(3);//рыскание	
}

/**
 * @brief ПИД по тяги
 * 
 */
void		UAVControlSystem::PIDThrust()
{
	this->desTang = this->thrust->update(this->currentPosition[2], this->desiredPosition[2], this->paramsSimulator->dt);
}


/**
 * @brief ПИД по позиции
 * 
 */
void		UAVControlSystem::PIDPosition()
{
	this->desiredAcceleration = this->position->output(this->currentPosition, this->desiredPosition, this->paramsSimulator->dt);
	this->desiredAcceleration[2] = 0;
	this->desiredAcceleration = this->desiredAcceleration.transpose() * Math::rotationMatrix2d(this->currentAcceleration[2]);
	// this->desiredAcceleration[0] = 0;
	// this->desiredAcceleration[1] = 0;
	this->desiredAcceleration[1] = this->desiredAcceleration[1] * (-1);
	this->desiredAcceleration[2] = this->desYaw;
	// this->desiredAcceleration[2] = 0;
}

/**
 * @brief ПИД по углу
 * 
 */
void		UAVControlSystem::PIDAngles()
{
	this->desiredVelocity = angle->output(this->currentAcceleration, this->desiredAcceleration, this->paramsSimulator->dt);
}

/**
 * @brief ПИД по угловой скорости
 * 
 */
void UAVControlSystem::PIDAngularRate()
{
	// this->desiredVelocity << 0.1, 0.1, 0.1;
	this->desiredAngularRate = this->velocity->output(this->currentVelocity, this->desiredVelocity, this->paramsSimulator->dt);
}

/**
 * @brief проверка попадания в радиус целевой позиции
 * 
 * @param targetPoints целевая позиция
 * @return true - принадлежим сфере 
 * @return false - не принадлежим сфере
 */
bool UAVControlSystem::checkRadius(const VectorXd_t& waypoint)
{
	// if ((this->currentPosition - buf).norm() <= 0.01)
	Eigen::Vector3d	buf;
	buf[0] = waypoint[0];
	buf[1] = waypoint[1];
	buf[2] = waypoint[2];
	if ((pow((this->currentPosition[0] - buf[0]),2) + 
		pow((this->currentPosition[1] - buf[1]),2) +
		pow((this->currentPosition[2] - buf[2]),2)) <= pow(0.3, 2))	
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

#include "motionPlanner.hpp"

MotionPlanner::MotionPlanner()
{
	this->countPoints = 0;
	this->indexPoints = 0;
}

/**
 * @brief функция для расчёта траектории(заполняет коэффициенты полинома)
 * 
 * @param stateVector вектор состояния
 * @param targetPoints матрица координат точек (nx3), где n - кол-во точек
 * @param timeTrajectory массив времени, за которое требуется пролететь от точки до точки
 */
void MotionPlanner::calculateTrajectory(StateVector stateVector, MatrixXd_t targetPoints, VectorXd_t timeTrajectory)
{
	this->timeTrajectory = timeTrajectory;
	this->countPoints = timeTrajectory.size();
	VectorXd_t currentPoint(9);
	VectorXd_t targetPoint(9); 
	targetPoint << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	for (int i = 0; i < currentPoint.size(); i++)
		currentPoint[i] = stateVector[i];

	this->xCoeff = MatrixXd_t(this->countPoints ,6);
	this->yCoeff = MatrixXd_t(this->countPoints ,6);
	this->zCoeff = MatrixXd_t(this->countPoints ,6);
	for (;this->indexPoints < this->countPoints; this->indexPoints++ )
	{
		targetPoint[0] = targetPoints(this->indexPoints, 0);
		targetPoint[1] = targetPoints(this->indexPoints, 1);
		targetPoint[2] = targetPoints(this->indexPoints, 2);
		trajectoryGenerator(currentPoint, targetPoint, timeTrajectory(this->indexPoints));
		currentPoint[0]= targetPoint[0];
		currentPoint[1] = targetPoint[1];
		currentPoint[2] = targetPoint[2];
	}
}

VectorXd_t		MotionPlanner::getRowsCoeffX(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(this->xCoeff, indexRows));
}

VectorXd_t		MotionPlanner::getRowsCoeffY(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(this->yCoeff, indexRows));
}

VectorXd_t		MotionPlanner::getRowsCoeffZ(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(this->zCoeff, indexRows));
}

double			MotionPlanner::getTimeTrajectory(unsigned int indexPoint)
{
	return (this->timeTrajectory(indexPoint));
}

unsigned int	MotionPlanner::getSizeTimeTrajectory()
{
	return (timeTrajectory.size());
}

/**
 * @brief функция для расчёта минимума по рывку
 * 
 * @param c 
 * @param t 
 * @return double 
 */
double			MotionPlanner::calculateDesiredPosition(VectorXd_t c, double t)
{
	std::cout << "Коэффициенты примененные: " << std::endl;

	for (unsigned int i = 0; i < c.size(); i++)
	{
		std::cout << c(i) << " ";
	}
	std::cout << std::endl;
	return c[5]*pow(t,5) + c[4]*pow(t,4) + c[3]*pow(t,3) + c[2]*pow(t,2) + c[1]*t + c[0];
	
}

double			MotionPlanner::calculateDesiredVelocity(VectorXd_t c, double t)
{
}

double			MotionPlanner::calculateDesiredAcceleration(VectorXd_t c, double t)
{
}

void		MotionPlanner::trajectoryGenerator(VectorXd_t currentPoints, VectorXd_t targetPoints, double T)
{
	//тут рассчитать матрицу с точками для оптимальной траектории
	
	VectorXd_t b_x(6);
	VectorXd_t b_y(6);
	VectorXd_t b_z(6);
	MatrixXd_t A(6, 6);

	A << 0,			0,			0,			0,			0,		1,
		pow(T,5),	pow(T,4),	pow(T,3),	pow(T,2),	T,		1,
		0,			0,			0,			0,			1,		0,	
		5*pow(T,4), 4*pow(T,3),	3*pow(T,2),	2*T,		1,		0,
		0,			0,			0,			2,			0,		0,	
		20*pow(T,3),12*pow(T,2),6*T,		2,			0,		0;

	b_x << currentPoints(0), targetPoints(0), currentPoints(3), targetPoints(3), currentPoints(6), targetPoints(6); 
	b_y << currentPoints(1), targetPoints(1), currentPoints(4), targetPoints(4), currentPoints(7), targetPoints(7); 
	b_z << currentPoints(2), targetPoints(2), currentPoints(5), targetPoints(5), currentPoints(8), targetPoints(8); 
	
	VectorXd_t xC(6);
	VectorXd_t yC(6);
	VectorXd_t zC(6);
	xC = A.inverse() * b_x;
	yC = A.inverse() * b_y;
	zC = A.inverse() * b_z;

	
	for (unsigned int i = 0; i < xC.size(); i++)
	{
		this->xCoeff(this->indexPoints, i) = xC(i);
		this->yCoeff(this->indexPoints, i) = yC(i);
		this->zCoeff(this->indexPoints, i) = zC(i);
	}

	std::cout << "Коэффициенты рассчётные:" << std::endl;

	for (unsigned int i = 0; i < xC.size(); i++)
	{
		std::cout << this->xCoeff(this->indexPoints, i) << " " 
				  << this->yCoeff(this->indexPoints, i) << " "
				  << this->zCoeff(this->indexPoints, i) << std::endl;
	}

}
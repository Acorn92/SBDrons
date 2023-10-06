#include "motionPlanner.hpp"

MotionPlanner::MotionPlanner()
{
	this->countPoitns = 0;
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
	numberPoints = timeTrajectory.size();
	
	VectorXd_t currentTargetPoints;

	
}

VectorXd_t		MotionPlanner::getRowsCoeffX(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(xCoeff, indexRows));
}

VectorXd_t		MotionPlanner::getRowsCoeffY(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(yCoeff, indexRows));
}

VectorXd_t		MotionPlanner::getRowsCoeffZ(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(zCoeff, indexRows));
}

double			MotionPlanner::getTimeTrajectory(unsigned int indexPoint)
{
	return (timeTrajectory(indexPoint));
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
	c[0]*pow(t,5) + c[1]*pow(t,4) + c[2]*pow(t,3) + c[3]*pow(t,2) + c[4]*t + c[5];
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
	int countPoints = getSizeTimeTrajectory();
	// VectorXd_t x_coeff(6);
	// VectorXd_t y_coeff(6);
	// VectorXd_t z_coeff(6);
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
	
	// for (int i = 0; i < this->numberPoints; i++)
	// {
	// 	this->xCoeff(i) = A.inverse() * b_x;
	// 	this->yCoeff(i) = A.inverse() * b_y;
	// 	this->zCoeff(i) = A.inverse() * b_z;
	// }
}
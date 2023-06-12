#include "mathModelQuadrotor.hpp"

MathModelQuadrotor::MathModelQuadrotor(const ParamsQuadrotor *paramsQuadrotor, const ParamsSimulator *paramsSimulator)
{
    this->paramsQuadrotor = paramsQuadrotor;
	this->paramsSimulator = paramsSimulator;
	acceleration << 0, 0, 0;//начальные ускорения по каждой оси
	velocity  << 0, 0, 0;//начальные скорости по каждой оси
    position << 0, 0, 0;//начальное положение по каждой оси
    angularAcceleration << 0, 0, 0;//начальные угловые ускорения по каждой оси
    angularVelocity << 0, 0, 0;//начальные угловые скорости по каждой оси
    orientation << 0, 0, 0;//начальные угловые положения по каждой оси
}

/**
 * @brief метод рассчитывающий вектор состояния системы
 * 
 * @param rotorsAngularVelocity угловая скорость вращения роторов
 * @return вектор состояния системы состоящий из 9 компонент(позиция, ориентация, угловая скорость)
 */
StateVector MathModelQuadrotor::calculateStateVector(StateVector &lastStateVector, VectorXd_t rotorsAngularVelocity)
{
    StateVector res;
    return res;
}

/**
 * @brief Метод, содержащий в себе функцию правых частей
 * 
 * @param angularVelocityRotors Вектор угловых скоростей роторов
 * @return Вектор ускорений, вектор угловых ускорений
 */
StateVector	MathModelQuadrotor::functionRight(StateVector &lastStateVector, VectorXd_t angularVelocityRotors)
{
    /*
    * промежуточные значения для получения 
    * положения и скорости
    */
    StateVector res;

    double sumRotorAngularVelocity = 0;//суммированные угловые скорости двигателей
    Eigen::Vector3d momentsThrustRotors;
    Eigen::Vector3d normalizeVector(0, 0, 1);
    VectorXd_t squarAVR(paramsQuadrotor->numberOfRotors);

    //получаем квадраты угловых скоростей роторов
    for (uint i = 0; i < paramsQuadrotor->numberOfRotors; i++)  
        squarAVR[i] = Math::squaring(angularVelocityRotors[i]);
    Eigen::Matrix3d inertialTensor;
    inertialTensor << paramsQuadrotor->Ixx, 0,                    0,
                                      0,                    paramsQuadrotor->Iyy, 0,
                                      0,                    0,                    paramsQuadrotor->Izz;

    //получаем сумму квадратов угловых скоростей двигателей
    for (uint i = 0; i < paramsQuadrotor->numberOfRotors; i++)  
        sumRotorAngularVelocity += squarAVR[i];

    //верно для соостного квадракоптера
    momentsThrustRotors[0] = paramsQuadrotor->lengthOfFlyerArms * paramsQuadrotor->b * (squarAVR[0]- squarAVR[2]);
    momentsThrustRotors[1] = paramsQuadrotor->lengthOfFlyerArms * paramsQuadrotor->b * (squarAVR[3]- squarAVR[1]);
    momentsThrustRotors[2] = paramsQuadrotor->d * (squarAVR[3]+ squarAVR[1] - squarAVR[0] - squarAVR[2]);

    acceleration = ((paramsQuadrotor->b * sumRotorAngularVelocity)*normalizeVector.transpose() * 
                    Math::rotationMatrix(lastStateVector.Pitch, lastStateVector.Roll, lastStateVector.Yaw) +
                    paramsQuadrotor->mass * (-GRAVITY_ACCELERATION * normalizeVector.transpose()) ) / paramsQuadrotor->mass;

    angularAcceleration = inertialTensor.inverse() * 
                          (momentsThrustRotors - angularVelocity.cross(inertialTensor * angularVelocity));

    res.X = acceleration[0];
    res.Y = acceleration[1];
    res.Z = acceleration[2];
    
    res.VelX = angularAcceleration[0];
    res.VelY = angularAcceleration[1];
    res.VelZ = angularAcceleration[2];

    return res;
}

Eigen::Vector3d MathModelQuadrotor::TestMatrRotation(StateVector &lastStateVector)
{
    Eigen::Vector3d res;
    Eigen::Vector3d testVector(1, 0, 0);

    res = Math::rotationMatrix(lastStateVector.Roll, lastStateVector.Pitch, lastStateVector.Yaw)*
          testVector;  
    return res;
}

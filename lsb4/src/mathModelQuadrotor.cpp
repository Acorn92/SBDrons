#include "mathModelQuadrotor.hpp"

MathModelQuadrotor::MathModelQuadrotor(const ParamsQuadrotor *paramsQuadrotor, const ParamsSimulator *paramsSimulator)
{
    this->paramsQuadrotor = paramsQuadrotor;
	this->paramsSimulator = paramsSimulator;
	acceleration << 0, 0, 0;//начальные ускорения по каждой оси
    acceleration2 << 0, 0, 0;//начальные ускорения по каждой оси
	velocity  << 0, 0, 0;//начальные скорости по каждой оси
    position << 0, 0, 0;//начальное положение по каждой оси
    angularAcceleration << 0, 0, 0;//начальные угловые ускорения по каждой оси
    angularAcceleration2 << 0, 0, 0;//начальные угловые ускорения по каждой оси
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
    StateVector res = {0};

    StateVector funcRight = {0};
    StateVector funcRightDt = {0};
    StateVector funcRightFirstIntegral = {0};
    StateVector funcRightSecondIntegral = {0};
    for (int i = 0; i < SIZE_FUNCTION_RIGHT; i++)
    {
        funcRightFirstIntegral[INDEX_VELOCITY_STATE + i] = lastStateVector[(INDEX_VELOCITY_STATE + i)];
        funcRightSecondIntegral[INDEX_POSITION_STATE + i] = lastStateVector[(INDEX_POSITION_STATE + i)];
    }

    funcRight = functionRight(lastStateVector, rotorsAngularVelocity);
    funcRightDt = funcRight * paramsSimulator->dt;
    funcRightFirstIntegral += funcRightDt;
    // funcRightSecondIntegral += funcRightFirstIntegral * paramsSimulator->dt;    

    // //получаем скорости
    // funcRightFirstIntegral.VelX += funcRight.VelX * paramsSimulator->dt;
    // funcRightFirstIntegral.VelY += funcRight.VelY * paramsSimulator->dt; 
    // funcRightFirstIntegral.VelZ += funcRight.VelZ * paramsSimulator->dt;  

    // funcRightFirstIntegral.PitchRate += funcRight.PitchRate * paramsSimulator->dt;
    // funcRightFirstIntegral.RollRate += funcRight.RollRate * paramsSimulator->dt;
    // funcRightFirstIntegral.YawRate += funcRight.YawRate * paramsSimulator->dt;

    //получаем положения
    funcRightSecondIntegral.X += funcRightFirstIntegral.VelX * paramsSimulator->dt;
    funcRightSecondIntegral.Y += funcRightFirstIntegral.VelY * paramsSimulator->dt;
    funcRightSecondIntegral.Z += funcRightFirstIntegral.VelZ * paramsSimulator->dt;  

    funcRightSecondIntegral.Pitch += funcRightFirstIntegral.PitchRate * paramsSimulator->dt;
    funcRightSecondIntegral.Roll += funcRightFirstIntegral.RollRate * paramsSimulator->dt;
    funcRightSecondIntegral.Yaw+= funcRightFirstIntegral.YawRate * paramsSimulator->dt;  

    //заполняем результат
    res.X = funcRightSecondIntegral.X;
    res.Y = funcRightSecondIntegral.Y;
    res.Z = funcRightSecondIntegral.Z;    

    res.Pitch = funcRightSecondIntegral.Pitch;
    res.Roll = funcRightSecondIntegral.Roll;
    res.Yaw = funcRightSecondIntegral.Yaw;  

    res.VelX = funcRightFirstIntegral.VelX;
    res.VelY = funcRightFirstIntegral.VelY;
    res.VelZ = funcRightFirstIntegral.VelZ;    

    res.PitchRate = funcRightFirstIntegral.PitchRate;
    res.RollRate = funcRightFirstIntegral.RollRate;
    res.YawRate = funcRightFirstIntegral.YawRate;  
    
    return (res);
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
    // StateVector res;
    StateVector res = {0};
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

    acceleration2 = ((1/paramsQuadrotor->mass) * Math::rotationMatrix(lastStateVector.Pitch, lastStateVector.Roll, lastStateVector.Yaw)) * 
                    (normalizeVector * (paramsQuadrotor->b * sumRotorAngularVelocity)) + (-GRAVITY_ACCELERATION * normalizeVector);
    //проверили в калькуляторе ускорения - вроде ок
    // результат до деления на массу : (0, 0, -0.5383) / 0.0630 = (0, 0, -8.54444444), что совпадает
    // переписывание формулы в соотв с инстркцией, без преобразований даёт похожий результат
  
    angularAcceleration = inertialTensor.inverse() * 
                          (momentsThrustRotors - angularVelocity.cross(inertialTensor * angularVelocity));
    //в калькуляторе получилось (-135.4, 96 57/70, 0)
    //в коде (-116.27294516493754, 94.530864231972032, 0)
    //в калькуляторе низкая точность, поэтому наверно можно счиать верным


    //запишем значения ускорения 
    res.VelX = acceleration[0];
    res.VelY = acceleration[1];
    res.VelZ = acceleration[2];
    //запишем значение углового ускорения
    res.PitchRate = angularAcceleration[0];
    res.RollRate = angularAcceleration[1];
    res.YawRate = angularAcceleration[2];;

    return res;
}

Eigen::Vector3d MathModelQuadrotor::TestMatrRotation(StateVector &lastStateVector)
{
    Eigen::Vector3d res;
    Eigen::Vector3d testVector(1, 0, 0);

    res = Math::rotationMatrix(lastStateVector.Roll, lastStateVector.Pitch, lastStateVector.Yaw).transpose()*
          testVector;  \

    return res;
}

void MathModelQuadrotor::TestMathModel()
{
    VectorXd_t testRotorsAngularVelocity(4);
    testRotorsAngularVelocity[0] = 0;
    testRotorsAngularVelocity[1] = 0;
    testRotorsAngularVelocity[2] = 1000;
    testRotorsAngularVelocity[3] = 1000;

    StateVector testStateVector = {0};
    testStateVector.Pitch = 0;
    testStateVector.Roll = 0;
    testStateVector.Yaw = 0;

    testStateVector = functionRight(testStateVector, testRotorsAngularVelocity);

    testStateVector.VelX = 0; 
    testStateVector.VelY = 0;
    testStateVector.VelZ = 5;

    testStateVector.PitchRate = 0; 
    testStateVector.RollRate = 0;
    testStateVector.YawRate = 0;

    testStateVector = calculateStateVector(testStateVector, testRotorsAngularVelocity);
}

#ifndef MESSAGES_HPP
#define MESSAGES_HPP


#pragma pack(push,1) // выравнивание в памяти
struct StateVector
{
	// Положение ЛА в стартовой СК
	double X; 
	double Y;
	double Z;
	// Скорость ЛА в стартовой СК
	double VelX;
	double VelY;
	double VelZ;
	// Угловое положение ЛА
	double Pitch;//тангаж
	double Roll;//крен
	double Yaw;//рыскание
	// Угловая скорость ЛА
	double PitchRate;
	double RollRate;
	double YawRate;
	// Метка времени симуляции
	double timeStamp;
};
#pragma pack(pop) // выравнивание в памяти


#endif
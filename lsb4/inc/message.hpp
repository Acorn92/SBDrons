#ifndef MESSAGES_HPP
#define MESSAGES_HPP


#pragma pack(push,1) // выравнивание в памяти
struct StateVector
{
	// Положение ЛА в стартовой СК
	double X; 
	double Y;
	double Z;
	// Угловое положение ЛА
	double Pitch;//тангаж
	double Roll;//крен
	double Yaw;//рыскание
	// скорость ЛА в стартовой СК
	double VelX;
	double VelY;
	double VelZ;
	// Угловая скорость ЛА
	double PitchRate;
	double RollRate;
	double YawRate;
	// Метка времени симуляции
	double timeStamp;
	// //ускорение аппарата
	// double Ax;
	// double Ay;
	// double Az;
	// //угловое ускорение
	// double Aax;
	// double Aay;
	// double Aaz;

};
#pragma pack(pop) // выравнивание в памяти

#define INDEX_POSITION_STATE 0
#define INDEX_VELOCITY_STATE 6

#endif
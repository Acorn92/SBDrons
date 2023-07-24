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

	double &operator[](int i)
	{
		switch (i)
		{
			case 0: return this->X;
			case 1: return this->Y;
			case 2: return this->Z;
			case 3: return this->Pitch;
			case 4: return this->Roll;
			case 5: return this->Yaw;
			case 6: return this->VelX;
			case 7: return this->VelY;
			case 8: return this->VelZ;
			case 9: return this->PitchRate;
			case 10: return this->RollRate;
			case 11: return this->YawRate;
			default: return this->timeStamp;
		}
	}
	//TODO: перегрузка умножения

	StateVector &operator*(const double b)
	{
		StateVector res = *this;
		res.X *= b;
		res.Y *= b;
		res.Z *= b;
		res.Pitch *= b;
		res.Roll *= b;
		res.Yaw *= b;
		res.VelX *= b;
		res.VelY *= b;
		res.VelZ *= b;
		res.PitchRate *= b;
		res.RollRate *= b;
		res.YawRate *= b;
		return res;
	}

	StateVector &operator+= (StateVector &a)
	{
		this->X += a.X;
		this->X += a.Y;
		this->Z += a.Z;
		this->Pitch += a.Pitch;
		this->Roll += a.Roll;
		this->Yaw += a.Yaw;
		this->VelX += a.VelX;
		this->VelY += a.VelY;
		this->VelZ += a.VelZ;
		this->PitchRate += a.PitchRate;
		this->RollRate += a.RollRate;
		this->YawRate += a.YawRate;
		this->timeStamp = a.timeStamp;
		return *this;
	}


};
#pragma pack(pop) // выравнивание в памяти

#define INDEX_POSITION_STATE 0
#define INDEX_VELOCITY_STATE 6

#endif
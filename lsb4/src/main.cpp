#include <simulator.hpp>
#include <typesData.hpp>
#include <configLoader.hpp>


#include <iostream>
#include <thread>


int	main(int argc, char *argv[])
{
	// // создаем экземпляры структур под параметры симулятора
	ParamsQuadrotor		parmsQuadrotor;
	ParamsSimulator		paramsSimulator;
	ParamsControlSystem	paramsControlSystem;
	

	// std::cout <<  << std::endl;

	// // заполнение параметров квадрокоптера и симулятора
	loadModelConfig(pathQuadModelConfig, parmsQuadrotor, paramsSimulator);

	// // заполнение параметров
	loadControlSysConfig(pathQuadControlSystemConfig, paramsControlSystem);

	// // Создаем объект симулятора
	Simulator	uavSim(parmsQuadrotor, paramsSimulator, paramsControlSystem);
	

	// // Запускаем симуляцию
	uavSim.run();

	// gr.End();
	
	return (0);
}
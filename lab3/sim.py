#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from matplotlib import pyplot as plt

class AngleDinamic:
    def __init__(self, l, k_b, rotorCount, accInit, velInit, posInit):
        '''

        :param l: длина плеча
        :type l: float
        :param k_b: коэффициент тяги двигателя
        :type k_b: float
        :param rotorCount: количество двигателей в системе
        :type rotorCount: int        
        :param accInit: начальное значение углового ускорения ЛА
        :type accInit: float
        :param velInit: начальное значение угловой скорости ЛА
        :type velInit: float
        :param posInit: начальное значение углового положения ЛА
        :type posInit: float

        '''
        self.l = l
        self.k_b = k_b
        
        self.rotorCount = rotorCount
        self.acceleration = accInit
        self.velocity = velInit
        self.position = posInit
        # Величина ускорения свободного падения
        self.g = 9.81 

    # получение угловой ускорения ЛА
    def calcAcc(self, rotorsAngularAcc, cmdT, Iy):
        '''

        :param rotorsAngularVel: угловое ускорение двигателей
        :type rotorsAngularVel: float
        :param cmdT: управляющее воздействие по тяге
        :type cmdT: float
        '''
        # Для всех двигателей рассчитаем угловое ускорение
        # w1 = rotorsAngularAcc + cmdT
        # w2 = rotorsAngularAcc*(-1) + cmdT
        w = []
        # получаем квадраты угловых скоростей двигателя
        for i in range(self.rotorCount):
            if (i % 2 == 0):
                w.append((rotorsAngularAcc + cmdT) ** 2)
            else:
                w.append((rotorsAngularAcc*(-1) + cmdT) ** 2)

        raznW = w[0]
        for i in range(1, self.rotorCount):
            raznW -= w[i]
        # raznW = w[1] - w[0]
        M = self.k_b * self.l * raznW

        self.acceleration = M/Iy

    def integration(self, dt):
        '''

        :param dt: шаг моделирования
        :type dt: float

        '''
        #интегрируем угловое ускорение чтобы получить угловую скорость
        self.velocity = self.acceleration * dt
        #интегрируем угловую скорость чтобы получить угловое положение
        self.position = self.velocity * dt

    def calculatePos(self, u, cmdT, Iy, dt):
        '''

        :param u: управляющие воздействие
        :type u: float
        :param dt: шаг моделирования
        :type dt: float

        '''
        self.calcAcc(u, cmdT, Iy)
        self.integration(dt)

    def getPosition(self):
        '''

        :return: положение ЛА
        :rtype: float

        '''
        return self.position

    def getVelocity(self):
        '''

        :return: скорость ЛА
        :rtype: float

        '''
        return self.velocity

    def getAcceleration(self):
        '''

        :return: ускорение
        :rtype: float

        '''
        return self.acceleration    


class ControlSystem():
    def __init__(self, k_p, k_i, k_d, controlLimit):
        '''

        :param k_p: коэффициент П регулятора
        :type k_p: float
        :param k_i: коэффициент И регулятора
        :type k_i: float
        :param k_d: коэффициент Д регулятора
        :type k_d: float
        :param controlLimit: ограничение по управляющему воздействию
        :type controlLimit: float

        '''
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.desiredPosition = 0
        self.error = 0
        self.errorPast = 0
        self.integral = 0
        self.controlLimit = controlLimit

    def setDesiredPosition(self, desiredPosition):
        '''

        :param desiredPosition: целевое положение ЛА
        :type desiredPosition: float

        '''
        # данный метод устанавливает целевое положение ЛА,
        # к которому система с течением времени будет стремиться - угловое ускорение
        self.desiredPosition = desiredPosition

    def PID(self, currentPosition, dt):
        '''

        :param currentPosition: текущее положение ЛА
        :type currentPosition: float
        :param dt: шаг моделирования
        :type dt: float

        '''
        # Вычислим функцию ошибки
        self.error = self.desiredPosition - currentPosition
        # Вычисляем интеграл ошибки
        self.integral += self.error * dt
        # Получим рассчетную управляющую угловую скорость двигателей при помощи ПИД регулятора
        u = self.k_p * self.error + self.k_i * self.integral + \
            self.k_d * ((self.error - self.errorPast) / dt)
        # Установим предыдущую ошибку для использования в дальнейших итерациях
        self.errorPast = self.error
        # Вызовем звено насыщения для ограничения максимального управляющего воздействия
        u = self.saturation(u)
        return u
    

    def saturation(self, inputVal):
        '''

        :param inputVal: входное значение
        :type inputVal: float
        :return: выходное значение после прохождения проверки на ограничение
        :rtype: float

        '''
        # Звено насыщения ограничивает размер входного параметра
        # На выходе метода,абсолютное значение не может быть больше 
        # заданного предела controlLimit
        if inputVal > self.controlLimit:
            inputVal = self.controlLimit
        elif inputVal < -self.controlLimit:
            inputVal = - self.controlLimit

        return inputVal
    
class Simulator():

    def __init__(self, Tend, dt, controlSys1, controlSys2, dynamicModel):
        '''

        :param Tend: конечное время моделирования
        :type Tend: float
        :param dt: шаг моделирования
        :type dt: float
        :param controlSys: объект системы управления высотой ЛА
        :type controlSys: ControlSystem
        :param dynamicModel: объект модели ЛА
        :type dynamicModel: VehicleSimpleDynamic

        '''
        self.dt = dt
        self.Tend = Tend
        self.controlSys1 = controlSys1
        self.controlSys2 = controlSys2
        self.dynamicModel = dynamicModel
        # значения углового ускорения ЛА 
        self.accList = []
        # значения угловой скорости ЛА
        self.velList = []
        # значения углового положения ЛА
        self.posList = []
        self.timeList = []

    def runSimulation(self):
        '''

       метод запускает моделирование системы от 0 до конечного времени Tend 
       с шагом dt

        '''
        # Задаем 0 время и начинаем рассчет до тех пор пока 
        # время не достигнет конечного значения Tend
        time = 0
        while (time <= self.Tend):
            # получаем положение ЛА
            
            pose = self.dynamicModel.getPosition()
            # Получаем скорость ЛА
            vel = self.dynamicModel.getVelocity()
            # Получаем ускорение ЛА
            acc = self.dynamicModel.getAcceleration()
            # Записываем полученные значения в списки
            # для дальнейшего построения графиков
            self.posList.append(pose)
            self.velList.append(vel)
            self.accList.append(acc)
            # self.timeList.append(time)

            # рассчитываем новое управляющие воздействие
            # на основе текущего положения(pose) ЛА 
            u = self.controlSys1.PID(pose, self.dt) # тут получили скорость
            self.controlSys2.setDesiredPosition(u)
            u1 = self.controlSys2.PID(vel, self.dt) # тут получили ускорение
            # Рассчитываем положение ЛА с учетом полученного
            # управляющего воздействия 
            self.dynamicModel.calculatePos(u1, 10, 7.1694e-05, self.dt)
            
            # увеличиваем время на dt, то есть на шаг моделирования
            time += self.dt

    def showPlots(self):
        '''
        метод строит графики на основе измерений полученных в 
        ходе моделирования системы

        '''
        f = plt.figure(constrained_layout=True)
        gs = f.add_gridspec(3, 5)
        ax1 = f.add_subplot(gs[0, :-1])
        ax1.plot(self.posList)
        ax1.grid()
        ax1.set_title('position')

        ax2 = f.add_subplot(gs[1, :-1])
        ax2.plot(self.velList, "g")
        ax2.grid()
        ax2.set_title('velocity')

        ax3 = f.add_subplot(gs[2, :-1])
        ax3.plot(self.accList, "r")
        ax3.grid()
        ax3.set_title('acceleration')

        plt.show()

        
'''
 Объявим параметры для моделирования
'''
k_p = 300 #коэффициент Пропорционального регулирования
k_i = 35 #коэффициент Интегрального регулирования
k_d = 100 #коэффициент Дифференциального регулирования

k_p2 = 300 #коэффициент Пропорционального регулирования
k_i2 = 35 #коэффициент Интегрального регулирования
k_d2 = 180 #коэффициент Дифференциального регулирования

Tend = 10 # конечное время моделирования 
dt = 0.01 # шаг моделирования системы 

# длина плеча ЛА
l = 0.17
# Коэффициент тяги двигателя ЛА
k_b = 3.9865e-08
# Количество двигателей ЛА
rotorCount = 2
# Ограничение на угловую скорость двигателей рад/сек
motorSpeedLimit = 1000

uavSimpleDynamic = AngleDinamic(l, k_b, rotorCount, 0, 0, 0)
controller1 = ControlSystem(k_p, k_i, k_d, 100)
controller2 = ControlSystem(k_p2, k_i2, k_d2, 1500)
'''
Установим целевое положение для нашей системы
'''
controller1.setDesiredPosition(30)


"""
Создадим объект симулятора и передадим в него контроллер
 и математическую модель
"""
sim = Simulator(Tend, dt, controller1, controller2, uavSimpleDynamic)
sim.runSimulation()  # запуск симулятора
sim.showPlots()  # построение графиков
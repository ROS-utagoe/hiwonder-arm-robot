#!/usr/bin/python3
import pigpio
import time
import threading

pin = 12 #要控制的io口
position = 1500 #位置
positionSet = 1500 #设置的目标位置
positionSet_t = positionSet #positionSet的一个临时存储

stepTime = 20 #每一步的间隔时间是20ms
rTime = 0 #舵机从当前位置转动到目标位置的时间
rTime_t = rTime #rTime的一个临时存储
incTimes = 0 #舵机从当前位置转动到目标位置经过的步数
positionInc = 0 #舵机转到目标位置的每一步的递增量
posChanged = False #舵机的位置被重新设定了
isRunning = False #舵机是否正在运动

pi = pigpio.pi() #初始化pigpio库
pi.set_PWM_range(pin, 20000)#5是要输出PWM的IO口， 20000设定PWM的调节范围，
                          #我们的舵机的控制信号是50Hz，就是20ms为一个周期。就是20000us。
                          #设为20000,就是最小调节为1us
pi.set_PWM_frequency(pin, 50) #设定PWM的频率，5是要设定的IO口， 50 是频率
pi.set_PWM_dutycycle(pin, position)


#设定舵机目标位置，和从当前位置转到目标位置的时间
#这个函数在时间不为0时，并不直接改变舵机位置，而是设置一些变量和标志让一个一直循环的线程来改变

def setPosition(pos, time = 0):
    global position
    global positionSet
    global pi
    global rTime_t
    global positionSet_t
    global posChanged

    if time is 0:  #时间是0, 直接转到目标位置
        position = pos  #位置设为目标位置
        positionSet = position
        pi.set_PWM_dutycycle(pin, position)
    else:
        #限制时间的最大最小值，并设置转动时间
        if time < 20:
            rTime_t = 20
        elif time > 30000:
            rTime_t = 30000
        else:
            rTime_t = time
        #限制位置的最大最小值，并设置位置
        if pos > 2500:
            positionSet = 2500
        elif pos < 500:
            positionSet = 500
        else:
            positionSet = pos
        positionSet_t = pos
        posChanged = True #标志舵机位置已经被设置

##更新舵机位置，不断循环 每20ms 执行一次
count = 0
def updatePosition():
    global count
    global posChanged
    global rTime
    global rTime_t
    global positionSet
    global positionSet_t
    global incTimes
    global stepTime
    global position
    global positionInc
    global isRunning
    global pin

    while True:
        if posChanged is True: #舵机的位置被设置了
            rTime = rTime_t
            positionSet = positionSet_t
            posChanged = False

            incTimes = rTime / stepTime #计算走到目标位置要的步数

            #计算当前位置与目标位置的差距
            if positionSet > position:
                positionInc = positionSet - position
                positionInc = -positionInc
            else:
                positionInc = position - positionSet
            #计算每一步要递增的位置
            positionInc = positionInc / float(incTimes)
            isRunning = True #置真isRunning标志，然后程序就会根据设好的各个参数依步改变舵机位置

        if isRunning is True:
            incTimes -= 1; #到目标位置需要的步数减少1步
            if incTimes is 0:  #所有的步数都走完了，直接转到目标位置，
                               #运行完毕，正在运行标志置为False
                position = positionSet
                isRunning = False
            else:
                #将位置向目标位置靠近一步
                position = positionSet + positionInc * incTimes
            #改变信号
            pi.set_PWM_dutycycle(pin, position)
        count += 1
        time.sleep(0.02) #延时20ms

threading.Thread(target=updatePosition).start() #建立舵机转动速度控制的线程

while True:
    #循环以不同速度来回转动，体现速度变化
    setPosition(2500, 1000)
    time.sleep(1.1)
    print(count)
    setPosition(800, 1000)
    time.sleep(1.1)
    print(count)
    setPosition(500, 3000)
    time.sleep(3.1)
    print(count)



while True:
    for i in range(500, 2500+1,5):
        pi.set_PWM_dutycycle(pin, i)
        time.sleep(0.01)
    time.sleep(0.5)
    for i in range(2500, 500-1,-5):
        pi.set_PWM_dutycycle(pin, i)
        time.sleep(0.01)
    time.sleep(0.5)

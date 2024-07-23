import RPi.GPIO as GPIO
import time
import websockets.sync.client
import json

tmp = []  # 用来存放读取到的数据
DHT11_pin = 4  # 使用BCM编号
a, b = 0, 0

class DHT11Client:
    def __init__(self):
        self.uri_humidity = "ws://101.32.194.5:8000/ws/humidity/test"
        self.uri_temperature = "ws://101.32.194.5:8000/ws/temperature/test"
        self.websocket_humidity = websockets.sync.client.connect(self.uri_humidity)
        self.websocket_temperature = websockets.sync.client.connect(self.uri_temperature)

    def send_humidity(self, humidity):
        self.websocket_humidity.send(json.dumps({
            "role": "humidity",
            "humidity": humidity
        }, ensure_ascii=False))

    def send_temperature(self, temperature):
        self.websocket_temperature.send(json.dumps({
            "role": "temperature",
            "temperature": temperature
        }, ensure_ascii=False))

    def close(self):
        self.websocket_humidity.close()
        self.websocket_temperature.close()

client = DHT11Client()

def delay_us(t):  # 微秒级延时函数
    start = time.time()  # 记录开始时间
    t = (t - 3) / 1000000  # 将输入t的单位转换为秒，-3是时间补偿
    while time.time() - start < t:  # 循环至时间差值大于或等于设定值时
        pass

def DHT11():
    GPIO.setup(DHT11_pin, GPIO.OUT)  # 设置GPIO口为输出模式
    GPIO.output(DHT11_pin, GPIO.HIGH)  # 设置GPIO输出高电平
    delay_us(10 * 1000)  # 延时10毫秒
    GPIO.output(DHT11_pin, GPIO.LOW)  # 设置GPIO输出低电平
    delay_us(25 * 1000)  # 延时25毫秒
    GPIO.output(DHT11_pin, GPIO.HIGH)  # 设置GPIO输出高电平
    GPIO.setup(DHT11_pin, GPIO.IN)  # 设置GPIO口为输入模式

    a = time.time()  # 记录循环开始时间
    while GPIO.input(DHT11_pin):  # 一直循环至输入为低电平
        b = time.time()  # 记录结束时间
        if (b - a) > 0.1:  # 判断循环时间是否超过0.1秒，避免程序进入死循环卡死
            break  # 跳出循环

    a = time.time()
    while GPIO.input(DHT11_pin) == 0:  # 一直循环至输入为高电平
        b = time.time()
        if (b - a) > 0.1:
            break

    a = time.time()
    while GPIO.input(DHT11_pin):  # 一直循环至输入为低电平
        b = time.time()
        if (b - a) >= 0.1:
            break

    for i in range(40):  # 循环40次，接收温湿度数据
        a = time.time()
        while GPIO.input(DHT11_pin) == 0:  # 一直循环至输入为高电平
            b = time.time()
            if (b - a) > 0.1:
                break

        delay_us(28)  # 延时28微秒

        if GPIO.input(DHT11_pin):  # 超过28微秒后判断是否还处于高电平
            tmp.append(1)  # 记录接收到的bit为1

            a = time.time()
            while GPIO.input(DHT11_pin):  # 一直循环至输入为低电平
                b = time.time()
                if (b - a) > 0.1:
                    break
        else:
            tmp.append(0)  # 记录接收到的bit为0

try:
    while True:
        GPIO.setmode(GPIO.BCM)  # 设置为BCM编号模式
        GPIO.setwarnings(False)
        del tmp[:]  # 删除列表
        time.sleep(1)  # 延时1秒

        DHT11()

        humidity_bit = tmp[0:8]  # 分隔列表，第0到7位是湿度整数数据
        humidity_point_bit = tmp[8:16]  # 湿度小数
        temperature_bit = tmp[16:24]  # 温度整数
        temperature_point_bit = tmp[24:32]  # 温度小数
        check_bit = tmp[32:40]  # 校验数据

        humidity_int = 0
        humidity_point = 0
        temperature_int = 0
        temperature_point = 0
        check = 0

        for i in range(8):  # 二进制转换为十进制
            humidity_int += humidity_bit[i] * 2 ** (7 - i)
            humidity_point += humidity_point_bit[i] * 2 ** (7 - i)
            temperature_int += temperature_bit[i] * 2 ** (7 - i)
            temperature_point += temperature_point_bit[i] * 2 ** (7 - i)
            check += check_bit[i] * 2 ** (7 - i)

        humidity = humidity_int + humidity_point / 10
        temperature = temperature_int + temperature_point / 10

        check_tmp = humidity_int + humidity_point + temperature_int + temperature_point
        print("Temperature is ", temperature, "C\nHumidity is ", humidity, "%")  # 打印温湿度数据
    if check==check_tmp:
        print("Temperature is ", temperature, "C\nHumidity is ", humidity, "%")  # 打印温湿度数据
        # 发布温度湿度信息到服务器
        client.send_humidity(humidity)
        client.send_temperature(temperature)

        time.sleep(1)
    else:
        print("error")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    client.close()
    GPIO.cleanup()

from datetime import datetime
import requests

def get_response(cmd_word, api_key=None, city=None):
    if cmd_word == "取药":
        return "收到取药指令，马上执行"
    elif cmd_word == "救命":
        return "已呼叫医生和监护人"
    elif cmd_word == "跌倒警报":
        return "已呼叫医生和监护人"    
    elif cmd_word == "取餐":
        return "收到取餐指令，马上执行"
    elif cmd_word == "去卫生间":
        return "收到去卫生间指令，马上执行"
    elif cmd_word == "呼叫医生":
        return "收到呼叫医生指令，立即呼叫医生"
    elif cmd_word == "现在的时间":
        return get_current_time_in_chinese()
    elif cmd_word == "星期几":
        return get_current_weekday_in_chinese()
    elif cmd_word == "今天的天气":
        return get_weather_today('e1c4383047ca4196916e4056d1fed0c7', '121.472644,31.231706')
    elif cmd_word == "几月几号":
        return get_current_date_in_chinese()
    else:
        return "抱歉，暂不支持这个命令"

def get_current_time_in_chinese():
    now = datetime.now()
    hour = now.hour
    minute = now.minute
    return f"现在的时间是{hour}点{minute}分"

def get_current_weekday_in_chinese():
    weekdays = ["星期一", "星期二", "星期三", "星期四", "星期五", "星期六", "星期日"]
    return "今天是"+weekdays[datetime.today().weekday()]

def get_weather_today(api_key, city):
    base_url = f'https://devapi.qweather.com/v7/weather/now?location={city}&key={api_key}'
    response = requests.get(base_url)
    if response.status_code == 200:
        weather_data = response.json()
        if weather_data['code'] == '200':
            weather_description = weather_data['now']['text']
            temperature = weather_data['now']['temp']
            humidity = weather_data['now']['humidity']
            return f"今天天气{weather_description}，气温{temperature}摄氏度，湿度{humidity}%"
        else:
            return f"抱歉，无法获取天气信息，错误代码：{weather_data['code']}"
    else:
        return "抱歉，无法获取天气信息"

def get_current_date_in_chinese():
    now = datetime.now()
    year = now.year
    month = now.month
    day = now.day
    return f"今天是{year}年{month}月{day}日"

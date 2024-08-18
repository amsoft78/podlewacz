import time
from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.oled.device import ssd1331

import RPi.GPIO as GPIO
import threading
import socket
import select
import json

class Device:
    STATE_WAITING = 1
    STATE_WORKING = 2

    SHEDULED_HOUR = 21
    SHEDULED_MINUTE = 30

    NEXT_MIN_AFTER = 18 # next shedule after 18 hour from previous

    # BCM pin numbers
    CHANNELS = [ ("Zraszacze", 21), ("Bok", 20), ("Przod", 26), ("Warzywniak", 16) ]
    STATE_DIODE = 12
    STOP_BUTTON = 5
    START_BUTTON = 6

    CHANNEL_TIME_MIN = 20 # how many minutes for one channel
    

    def __init__(self) -> None:
        self.start_pressed = False
        self.stop_pressed = False
        self.last_run = 0 # timestamp of last run. Long ago!
        intf = spi(device=0, port=0)
        self.disp_device = ssd1331(intf)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup (self.STOP_BUTTON, GPIO.IN,  pull_up_down=GPIO.PUD_UP)
        GPIO.setup (self.START_BUTTON, GPIO.IN,  pull_up_down=GPIO.PUD_UP)
        #GPIO.add_event_detect(self.STOP_BUTTON, GPIO.RISING, callback=self.OnStopButton, bouncetime=300)
        #GPIO.add_event_detect(self.START_BUTTON, GPIO.RISING, callback=self.OnStartButton, bouncetime=300)
        GPIO.setup(self.STATE_DIODE, GPIO.OUT)
        self.StateDiode(GPIO.LOW)
        for chan in self.CHANNELS:
            GPIO.setup(chan[1], GPIO.OUT)
        self.AllOff()
        self.do_stop = False
	
    def Message(self, msg):
        print (msg)
        with canvas(self.disp_device) as draw:
            draw.rectangle(self.disp_device.bounding_box, outline="white", fill="black")
            draw.text((2, 2), msg, fill="white", font_size=13)

    def CheckUDP(self):
        # UDP commands for listening
        UDP_PORT = 5055
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', UDP_PORT))
        while not self.do_stop:
            try:
                r, _, _ = select.select([sock], [], [], 0.1)
                if r:
                    data = sock.recv(1024)
                    print (f'UDP RECEIVED: ####{data}###')
                    json_data = json.loads(data.decode('utf-8'))
                    print(json_data)
                    if json_data["command"] == "start":
                        self.start_pressed = True
                    elif json_data["command"] == "stop":
                        self.stop_pressed = True
                    elif json_data["command"] == "reshedule":
                        self.SHEDULED_HOUR = json_data["hour"]
                        self.SHEDULED_MINUTE = json_data["minute"]
                        print(f'Czas startu zmiana na {self.SHEDULED_HOUR}:{self.SHEDULED_MINUTE}')
                        self.last_run = 0 # bad value, but must be something
            except:
                pass

    def RinsingOn(self, channel):
        chan = self.CHANNELS[channel-1]
        self.Message (f"START {chan[0]} \n {time.ctime()}")
        GPIO.output(chan[1], GPIO.HIGH)

    def RinsingOff(self, channel):
        chan = self.CHANNELS[channel-1]
        self.Message (f"STOP {chan[0]} :\n {time.ctime()}")
        GPIO.output(chan[1], GPIO.LOW)

    def AllOff(self):
        for i in range (0, len(self.CHANNELS)):
            self.RinsingOff(i+1)

    def StateDiode(self, v):
        GPIO.output(self.STATE_DIODE, v)
    
    def OnStartButton(self, dummy):
        self.start_pressed = True
        
    def OnStopButton(self, dummy):
        self.Message ("Button STOP!!")
        self.stop_pressed = True        
        
    def StrTime(t, withhours=False):
        time_struct = time.gmtime(t)
        if time_struct.tm_hour != 0 or withhours:
            return time.strftime("%H:%M:%S", time_struct)
        else:
            return time.strftime("%M:%S", time_struct)

    def MainLoop(self):
    
        #last_checked = -1 # weekday when last sheduled checked
        running_channel = -1
        channel_started_ts = 0

        state = self.STATE_WAITING

        while not self.do_stop:
            sleep_time = 1 #1 second sleep
            if state == self.STATE_WAITING:
                curr_time = time.time()
                # check buttons
                if self.start_pressed:
                    state = self.STATE_WORKING
                # check time
                from_last = (curr_time - self.last_run) / 60.0 / 60.0
                from_last_hours = int(from_last)

                # if counting down would be enabled at this point, check the STOP button
                if from_last_hours >= self.NEXT_MIN_AFTER and self.stop_pressed:
                    self.last_run = curr_time
                    self.stop_pressed = False
                    self.Message ("ZATRZYMANE ODLICZANIE!")
                    state = self.STATE_WORKING
                    continue
                    
                from_last_minutes = int((from_last - from_last_hours) * 60.0)
                if from_last_hours >= self.NEXT_MIN_AFTER:
                    curr_local_time = time.localtime(curr_time)
                    hours_left = self.SHEDULED_HOUR - curr_local_time.tm_hour
                    minutes_left = self.SHEDULED_MINUTE - curr_local_time.tm_min
                    if minutes_left < 0:
                        minutes_left += 60
                        hours_left -= 1
                    if hours_left < 0:
                        hours_left += 24
                    time_left_sec = (hours_left * 60 + minutes_left) * 60 - curr_local_time.tm_sec
                    self.Message (f"Podlewanie za\n\n           {Device.StrTime(time_left_sec)}")
                    self.StateDiode(GPIO.HIGH)
                    # blink if very short time
                    if hours_left == 0 and minutes_left <= 10:
                        if minutes_left <= 2:
                            sleep_time = 0.2
                        else:
                            sleep_time = 0.5
                        time.sleep(sleep_time)
                        self.StateDiode(GPIO.LOW)
                    if hours_left * 60 + minutes_left <= 0:
                        state = self.STATE_WORKING
                else:
                    self.Message (f"Od podlewania\n {Device.StrTime(curr_time - self.last_run, withhours=True)}")
                    #ignore stop button in this state
                    self.stop_pressed = False
                    self.StateDiode(GPIO.LOW)

                if state == self.STATE_WORKING:
                    self.StateDiode(GPIO.LOW)
                    self.last_run = curr_time
                    channel_started_ts = curr_time
                    running_channel = 1
                    self.RinsingOn(running_channel)
                    self.start_pressed = False
            elif state == self.STATE_WORKING:
                if self.stop_pressed:
                    self.AllOff()
                    state = self.STATE_WAITING
                    self.stop_pressed = False
                    self.start_pressed = False
                    continue
                curr_time = time.time()
                rising_time = curr_time - channel_started_ts
                rising_time_min = rising_time / 60.0
                if rising_time_min >= self.CHANNEL_TIME_MIN:
                    self.RinsingOff(running_channel)
                    if running_channel == len(self.CHANNELS):
                        # ending
                        self.start_pressed = False
                        state = self.STATE_WAITING
                    else:
                        running_channel += 1
                        self.RinsingOn(running_channel)
                        channel_started_ts = curr_time
                else:
                    rising_remain = self.CHANNEL_TIME_MIN * 60 - rising_time
                    chan = self.CHANNELS[running_channel-1]
                    self.Message(f"*PODLEWANIE* \n   *{chan[0]}*\n         {Device.StrTime(rising_remain)}")
            time.sleep(sleep_time)
            
    def DoStop(self):
        self.do_stop = True

if __name__ == '__main__':
    dev = Device()
    try:
        listen_UDP = threading.Thread(target=dev.CheckUDP)
        listen_UDP.start()
        dev.MainLoop()
    except KeyboardInterrupt:
        print ( "ESC pressed - EXITING")
        dev.DoStop()
        dev.AllOff()
        dev.StateDiode(GPIO.LOW)
        listen_UDP.join()


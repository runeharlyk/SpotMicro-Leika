from gpiozero import LED, Button, DistanceSensor, CPUTemperature, LoadAverage, DiskUsage, pi_info, Device
from gpiozero.pins.native import NativeFactory
from threading import Thread
import board
import adafruit_dht
from time import sleep
import logging
import data

logger = logging.getLogger(__name__)
Device.pin_factory = NativeFactory()

class Gpio(Thread):
    """docstring for io."""

    def __init__(self, config):
        super(Gpio, self).__init__()
        self.isActive = True
        self.name = "Raspberry io"
        self.config = config
        if self.config["left_distance_sensor"]:
            l = self.config["left_distance_sensor"]
            self.LeftDistanceSensor = DistanceSensor(l["echo"], l["trig"], max_distance=l["max_distance"], threshold_distance=l["threshold_distance"])
        if self.config["right_distance_sensor"]:
            r = self.config["right_distance_sensor"]
            self.RightDistanceSensor = DistanceSensor(r["echo"], r["trig"], max_distance=r["max_distance"], threshold_distance=r["threshold_distance"])
        if self.config["dht"]:
            if self.config["dht"]["type"] == "dht22":
                self.dht = adafruit_dht.DHT22(self.config["dht"]["pin"])
            elif self.config["dht"]["type"] == "dht11":
                self.dht = adafruit_dht.DHT11(self.config["dht"]["pin"])
        if self.config["button_led"]:
            self.led = LED(self.config["button_led"])
            self.led.on()
        if self.config["button"]:
            self.button = Button(self.config["button"]["pin"], hold_time=self.config["button"]["hold_time"])
        #if self.config["battery"]:
            # ADC get battery data


        self.cpu = CPUTemperature()
        self.load = LoadAverage()
        self.disk = DiskUsage()

    def run(self):
        while self.isActive:
            try:
                if self.config["dht"]:
                    data.dht["t"] = self.dht.temperature
                    data.dht["h"] = self.dht.humidity
                if self.config["left_distance_sensor"]:
                    data.distance["l"] = self.LeftDistanceSensor.distance
                if self.config["right_distance_sensor"]:
                    data.distance["r"] = self.RightDistanceSensor.distance
                data.cpu["t"] = self.cpu.temperature
                data.cpu["u"] = self.disk.usage
                #load.is_active
                sleep(self.config["frequency"])
            except Exception as e:
                logging.debug("An exception occurred: {}".format(e))


    def terminate(self):
        self.led.off()
        self.isActive = False

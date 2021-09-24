# This tool simulates a solar and wind farm (well - it tries...)

import os
import math
from datetime import datetime
from datetime import timedelta
import time
import paho.mqtt.client as mqtt
import ssl
from scipy import stats
import configparser
import requests
import json
from json import JSONEncoder

# Helper functions
def sinD(x):
    return math.sin(math.radians(x))
def cosD(x):
    return math.cos(math.radians(x))
def tandD(x):
    return math.tan(math.radians(x))
def asinD(x):
    return math.degrees(math.asin(x))
def acosD(x):
    return math.degrees(math.acos(x))
def sinhD(x):
    return math.sinh(math.radians(x))
def coshD(x):
    return math.cosh(math.radians(x))

def endless_loop(msg):
    print(msg + " Entering endless loop. Check and redo deployment?")
    while True:
        pass

# To interpret configured templates of JSON
def fstr(template, v1 = "", v2 = "", v3 = "", v4 = "", v5 = "", v6 = "", v7 = "", v8 = "", v9 = "", v10 = ""):
    return eval(f"f'{template}'")

# -------------- OPENWEATHER PART ----------------->>>>

# To handle openWeather calls and data (including redis)
class OpenWeather:
    # Constructor method
    def __init__(self, url, apiKey, lat, lon, weatherTimeout, unitSystem, language, excludeInfo, smoothChange):
        super().__init__()
        self.baseurl = url
        self.apiKey = apiKey
        self.lat = lat
        self.lon = lon
        self.timeout = weatherTimeout
        self.units = unitSystem
        self.lang = language
        self.exclude = excludeInfo
        self.callUrl = "https://" + self.baseurl + "?lat=" + str(self.lat) + "&lon=" + str(self.lon) + "&exclude=" + self.exclude + "&units=" + self.units + "&lang=" \
                       + self.lang + "&appid=" + self.apiKey
        self.data = {} # Weather is cached here
        self.then = datetime.now() # Will be overwritten in first run, stores the expiry timestamp
        self.smooth = smoothChange # In n steps
        # Weather data values
        self.cloud = [-1.,-1.,-1.,0] # Target value, Old value, Current value,Counter
        self.temperature = [-1.,-1.,-1.,0] # Temperature in degrees Celsius
        self.windSpeed = [-1.,-1.,-1.,0] # Windspeed in m/s
        self.windGust = [-1.,-1.,-1.,0]  # Wind gust in m/s
        self.windDirection = [-1.,-1.,-1.,0]  # Direction on 360 degrees (0 = North, 180 = South)
        self.humidity = [-1.,-1.,-1.,0] # Humidty relative in %
        self.airpressure = [-1.,-1.,-1.,0] # Airpressure  in hPa
        self.rain = [-1.,-1.,-1.,0]  # Rain in mm
        self.snow = [-1.,-1.,-1.,0]  # Snow in mm
        self.dewPoint = [-1.,-1.,-1.,0] # Dew point in degrees Celsius
        self.uvIndex = [-1.,-1.,-1.,0] # UV Index 0 - 12 (none to extreme)        

    # Private method to get weather data (use only this)
    def __getData(self):
        # Caching the weather for n minutes
        now = datetime.now()
        durationMins = divmod((now-self.then).total_seconds(), 60)[0]
        if (durationMins > self.timeout) or self.data == {}:
            response = requests.get(self.callUrl)
            data = json.loads(response.text)
            print("No cached weather forecast available. Called openWeather with response " + str(response.status_code) + ".")
            if response.status_code == 200:
                # Cache the weather
                self.then = datetime.now()
                self.data = data
            else:
                endless_loop("Could not call openWeather - this is not good.")
        return self.data

    # Manages the change in a weather value e.g. wind, clouds.
    # The purpose is to smooth the change over time
    def __manageChange(self,input, v):
        if v[0] == -1: #Never set before, simulator just started
            v[0] = input
            v[1] = input
            v[2] = 0
        elif v[0] != input: # there was a value change
            v[1] = v[0] # Move target value to old value
            v[0] = input # Move changed value to target value
            v[2] = (v[0] - v[1]) / self.smooth
            v[3] = 0 # Reset counter
        if v[3] < self.smooth:
            v[1] = v[1] + v[2]
            v[3] = v[3] + 1
        return v[1]

    # Randomizes around a mean value considering standard deviation (+/-3)
    def __randomize(self, mean, scale, percent = False):
        if percent: # If we have no idea then we take percentages
            scale = mean * scale / 100         
        rndVal = stats.truncnorm.rvs(-3, 3, loc=mean, scale=scale, size=1)[0]
        return rndVal

    # Returns the timezone shift to UTC in seconds
    def getTimeZoneShift(self):
        data = self.__getData()
        timeZoneShift = int(data["timezone_offset"])
        return timeZoneShift

    # Get cloudiness (cloud coverage) - 0 = Sunny / 100 = Overcast
    def getClouds(self):
        data = self.__getData()
        currentClouds = float(data["current"]["clouds"])
        return self.__randomize(self.__manageChange(currentClouds, self.cloud),5,True) # 5% deviation 

    # Get temperature
    def getTemperature(self):
        data = self.__getData()
        currentTemp = float(data["current"]["temp"])
        temperature = self.__manageChange(currentTemp, self.temperature)
        return temperature

    # Get wind speed
    def getWindSpeed(self):
            data = self.__getData()
            currentWindSpeed = float(data["current"]["wind_speed"])
            windSpeed = self.__manageChange(currentWindSpeed, self.windSpeed)
            return self.__randomize(windSpeed, (self.getWindGust()-windSpeed)/2)
    
    # Get wind gust
    def getWindGust(self):
        data = self.__getData()
        currentWindGust = float(data["current"]["wind_gust"])
        windGust = self.__manageChange(currentWindGust, self.windGust)
        return windGust

    # Get wind direction
    def getWindDirection(self):
        data = self.__getData()
        currentWindDir = float(data["current"]["wind_deg"])
        windDirection = self.__manageChange(currentWindDir, self.windDirection)
        return int(self.__randomize(windDirection,3,True))

    # Get humidity
    def getHumidity(self):
        data = self.__getData()
        currentHumidity = float(data["current"]["humidity"])
        return int(self.__manageChange(currentHumidity, self.humidity))
    
    # Get airpressure
    def getAirpressure(self):
        data = self.__getData()
        currentPressure = float(data["current"]["pressure"])
        return int(self.__manageChange(currentPressure, self.airpressure))
    
    # Get rain
    def getRain(self):
        data = self.__getData()
        try:
            currentRain = float(data["current"]["rain"]["1h"])
        except Exception: #No rain
            return float(0)
        return int(self.__manageChange(currentRain, self.rain))

    # Get snow
    def getSnow(self):
        data = self.__getData()
        try:
            currentSnow = float(data["current"]["snow"]["1h"])
        except Exception: #No snow
            return float(0)
        return int(self.__manageChange(currentSnow, self.snow))

    # Get dew point
    def getDewPoint(self):
        data = self.__getData()
        try:
            currentDewPoint = float(data["current"]["dew_point"])
        except Exception: #No dew point
            return float(0)
        return self.__manageChange(currentDewPoint, self.dewPoint)
    
    # Get UV Index
    def getUVIndex(self):
        data = self.__getData()
        try:
            currentUVIndex = float(data["current"]["uvi"])
        except Exception: #No UV index
            return float(0)
        return self.__manageChange(currentUVIndex, self.uvIndex)
  
# <<<<-------------- OPENWEATHER PART -----------------

# # -------------- SOLAR PART ----------------->>>>

# Declination assuming a perfect circle of earth around the sun (360/365 fixes the position on that orbit)
def declination(dayOfYear):
    declinationC = 23.45 # +/- degrees of variation over the year
    return declinationC * sinD(360/365 * (dayOfYear - 81))
    
# Sunrise and sunset
def sunRiseSet(latitude, declination, d):
    rValue = -1 * sinD(latitude) * sinD(declination) / (cosD(latitude) * cosD(declination))
    try:
        rValue = acosD(rValue) / 15
    except Exception as e: # A complex number makes no sense in our model
        return [0,0]
    return [12 - rValue - d / 60, 12 + rValue - d / 60]

# Calculate elevation of the sun (0 when sunrise or sunset, max depends on declination)
def elevation(declination, latitude, d):
    return asinD(sinD(declination) * sinD(latitude) + cosD(declination) * cosD(latitude) * cosD(d))

# Convert time (local) to hour angle
def timeToHourAngle(time):    
    return 15 * ((time.hour + time.minute / 60 + time.second / 3600) - 12)

# Calculate air mass
def airMassSimple(angleV):
    return 1 / cosD(angleV)

# Calculate air mass intensity (kW/m²) - theoretical(!) maximum value (with empirical constancts)
def airMassIntensity(airMass):
    try:
        return 1.353 * math.pow(0.7, math.pow(airMass, 0.678))
    except Exception as e: # A negative airMass will lead to a complex value which makes no sense, set it to 0!
        return 0

# Calculate Solar Insolation (theoretical value)
def solarInsolation(latitude, day, hourAngle):
    decl = declination(day)
    elev = elevation(decl, latitude, hourAngle)
    aMas = airMassSimple(90 - elev)
    aMIt = airMassIntensity(aMas)
    return aMIt

# Get time - we might need to handle Zulu time to local time conversion
# Use only this function when you need the time and date!
def getLocalTime(weather):
    utc = datetime.utcnow()
    shiftInSeconds = weather.getTimeZoneShift()
    localTime = utc + timedelta(0, shiftInSeconds)
    return localTime

# Cloudiness: proposed model is f(x) = 1/(1+ax²) + b (for R0+)
# clouds from 0 to 100%
def cloudinessFactor(clouds, a = 10, b = 0):
    return 1 / (1 + a * math.pow((clouds/100),2)) + b

# Delivers the insolation considering current cloudiness
def insolationAddWeather(insolation, weather):
    clouds = weather.getClouds()
    return insolation * cloudinessFactor(clouds)

# Class definition
class SolarSim:

    # object constructor
    def __init__(self, deviceName, certFilename, pemCertFilePath, url, port, ack, measure, solarTemplate, weatherTemplate, lat, lon, weather):
        super().__init__()
        # Mqtt init
        self.msgTemplate = solarTemplate
        self.weatherTemplate = weatherTemplate
        self.mqtt = MqttClient(deviceName, certFilename, pemCertFilePath, url, port, ack, measure)
        self.mqtt.connect()
        # Solar init
        self.lat = lat # Latitude of simulated location
        self.lon = lon # Longitude
        self.weather = weather # Handle to weather object

    # Calculate day of year
    def __getDayOfYear(self):
        return getLocalTime(self.weather).timetuple().tm_yday # Day in a year (e.g. 1st Jan = 1)

    # Main method to simulate the solar farm
    def simulate(self):

        # 1. Solar Insolation - A theoretical value of kW/m² basis for many other values
        insolation = solarInsolation(self.lat, self.__getDayOfYear(), timeToHourAngle(getLocalTime(self.weather)))
        # 2. Solar Insolation under current, local cloudiness conditions
        insolationWithWeather = insolationAddWeather(insolation, self.weather)


        # FINALLY - Submit data to SAP IoT Platform
        self.mqtt.sendMessage(fstr(self.msgTemplate, insolation, insolationWithWeather))
        self.mqtt.sendMessage(fstr(self.weatherTemplate, self.weather.getWindSpeed(), self.weather.getWindGust(), self.weather.getWindDirection(), self.weather.getHumidity(), \
                                   self.weather.getAirpressure(), self.weather.getRain(), self.weather.getSnow(), self.weather.getDewPoint(), self.weather.getUVIndex(), \
                                   self.weather.getTemperature()))

    # End the simulation
    def endSimulation(self):
        self.mqtt.stop()

# <<<<-------------- SOLAR PART -----------------

# Class definition
class WindSim:

    # object constructor
    def __init__(self, deviceName, certFilename, pemCertFilePath, url, port, ack, measure, windTemplate, lat, lon, weather):
        super().__init__()
        # Mqtt init
        self.msgTemplate = windTemplate
        self.mqtt = MqttClient(deviceName, certFilename, pemCertFilePath, url, port, ack, measure)
        self.mqtt.connect()
        # Wind init
        self.lat = lat # Latitude of simulated location
        self.lon = lon # Longitude
        self.weather = weather # Handle to weather object

    # Main method to simulate the wind farm
    def simulate(self):
        pass

    # End the simulation
    def endSimulation(self):
        self.mqtt.stop()

# -------------- MQTT PART ----------------->>>>
class MqttClient:
    def __init__(self, deviceName, certFilename, pemCertFilePath, url, port, ack, measure):
        super().__init__()
        self.id = deviceName
        self.url = url
        self.port = port
        self.ack = ack
        self.ackId = self.ack+self.id
        self.measure = measure
        self.client = mqtt.Client(self.id) 
        self.client.on_connect = self.onConnect
        self.client.on_message = self.onMessage
        self.client.on_subscribe = self.onSubscribe
        self.client.tls_set(certfile=pemCertFilePath+certFilename, cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS, ciphers=None)

    # The callback for when a PUBLISH message is received from the server.
    def onMessage(self, client, userdata, msg):
        print(msg.topic + " " + str(msg.payload))

    # Subscription to topic confirmation
    def onSubscribe(self,client, userdata, mid, granted_qos):
        pass

    # This function gives a connection response from the server
    def onConnect(self, client, userdata, flags, rc):
        rcList = {
            0: "Connection successful",
            1: "Connection refused - incorrect protocol version",
            2: "Connection refused - invalid client identifier",
            3: "Connection refused - server unavailable",
            4: "Connection refused - bad username or password",
            5: "Connection refused",
        }
        print(rcList.get(rc, "Unknown server connection return code {}.".format(rc)))
        if rc == 0:
            (result, mid) = self.client.subscribe(self.ackId) #Subscribe to device ack topic (feedback given from SAP IoT MQTT Server)
            print("Subscribed to "+self.ackId+" with result: "+str(result)+" request #"+str(mid))

    # connect method for the device object
    def connect(self):
        self.client.connect(self.url, self.port)
        self.client.loop_start() #Listening loop start 

    # Send message to SAP MQTT Server
    def sendMessage(self, messageContentJson):
        messageInfo = self.client.publish(self.measure+self.id, messageContentJson)
        print(messageContentJson)
        print("Sent message for " + self.id + " with result " + str(messageInfo.rc) + " request #" + str(messageInfo.mid))

    # Stop the client
    def stop(self):
        self.client.loop_stop
        print("Shut down device "+self.id)

    # object destructor
    def __del__(self):
        pass   

# <<<<-------------- MQTT PART -----------------

# -------------- MAIN PROGRAM ---------------
def main():
    # Get configuration
    config = configparser.ConfigParser(inline_comment_prefixes="#")
    config.read(['./config/settings.cfg'])
    if not config.has_section("location"):
        endless_loop("Config: Location section missing.")
    if not config.has_section("server"):
        endless_loop("Config: Server section missing.")
    if not config.has_section("topics"):
        endless_loop("Config: Topics section missing.")
    if not config.has_section("devices"):
        endless_loop("Config: Devices section missing.")
    if not config.has_section("messages"):
        endless_loop("Config: Messages section missing.")
    if not config.has_section("timing"):
        endless_loop("Config: Timing section missing.")
    if not config.has_section("openweather"):
        endless_loop("Config: Openweather section missing.")   
    # -------------- Parameters ------------------>>>
    mqttServerUrl = config.get("server","mqttServerUrl")
    mqttServerPort = config.getint("server","mqttServerPort")
    pemCertFilePath = config.get("server","pemCertFilePath")
    ackTopicLevel = config.get("topics","ackTopicLevel")
    measuresTopicLevel = config.get("topics","measuresTopicLevel")
    solarDevName = config.get("devices","solarDevName")
    windDevName = config.get("devices","windDevName")
    solarTemplate = config.get("messages","solarTemplate")
    windTemplate = config.get("messages","windTemplate")
    weatherTemplate = config.get("messages","weatherTemplate")
    relOpenWeatherUrl = config.get("openweather","url")
    locLat = float(config.get("location","lat"))
    locLon = float(config.get("location","lon"))
    unitSystem = config.get("openweather","units")
    language = config.get("openweather","lang")
    apiKey =  config.get("openweather","apiid")
    excludeInfo = config.get("openweather","exclude")

    weatherTimeout = int(config.get("timing", "weatherTimeout"))
    smoothChange = int(config.get("timing", "smoothChange"))
    pauseTime = int(config.get("timing","pauseInSeconds"))
    runTime = int(config.get("timing","runtimeOfProgram"))
    # -------------- Parameters ------------------<<<

    loopCondition = True
    then = datetime.now()

    # Instantiate openWeather
    weather = OpenWeather(relOpenWeatherUrl, apiKey, locLat, locLon, weatherTimeout, unitSystem, language, excludeInfo, smoothChange)

    # Instantiate solar farm device
    solar = SolarSim(solarDevName, solarDevName+'.pem', pemCertFilePath, mqttServerUrl, mqttServerPort, ackTopicLevel, measuresTopicLevel, \
                     solarTemplate, weatherTemplate, locLat, locLon, weather)

    # Instantiate wind farm device
    #wind = WindSim(windDevName, windDevName+'.pem', pemCertFilePath, mqttServerUrl, mqttServerPort, ackTopicLevel, measuresTopicLevel, windTemplate, locLat, locLon, weather)
    
    # Start sending data to cloud
    while loopCondition:
        solar.simulate() #Do one round of simulation incl. MQTT transmission
        #wind.simulate() #TODO
        time.sleep(pauseTime)
        if runTime > 0:
            now = datetime.now()
            durationMins = divmod((now-then).total_seconds(), 60)[0]
            if durationMins > runTime:
                loopCondition = False

    time.sleep(2) # wait until we have all feedback messages from the server
    solar.endSimulation()
    #wind.endSimulation() # TODO
    print("Shut down all clients. Entering endless loop. Restart pod if needed.")
    while True:
        pass

# Main program start
if __name__ == "__main__":
    main()
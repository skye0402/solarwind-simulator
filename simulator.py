# This tool simulates a solar and wind farm (well - it tries...)

import os
import math
from datetime import datetime
import time
import paho.mqtt.client as mqtt
import ssl
import random
import configparser
import requests
from requests.auth import HTTPBasicAuth
import json
from json import JSONEncoder
import redis

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

# -------------- OPENWEATHER PART ----------------->>>>
# get weather (ideally cached)
def getCachedWeather(r, relOpenWeatherUrl, locLat, locLon, excludeInfo, unitSystem, language, apiKey):
    # Create request url
    openWeatherUrl = "https://" + relOpenWeatherUrl + "?lat=" + locLat + "&lon=" + locLon + "&exclude=" + excludeInfo + "&units=" + unitSystem + "&lang=" + language + "&appid=" + apiKey
    
    # Check if in Redis Cache
    rainfall = -1.0
    rainfallStr = r.get(rainfallKey)
    if rainfallStr == None:
        # Retrieve from openWeather
        response = requests.get(openWeatherUrl)
        data = json.loads(response.text)
        print("No cached weather forecast available. Called openWeather with response " + str(response.status_code) + ".")
        if response.status_code == 200:
            today = data['daily'][1]
            rainfall = today['rain']
            print("Forecasted rainfall: " + str(rainfall) + "mm.")
            r.setex(rainfallKey, rainfallMin * 60, rainfall)
    else: 
        rainfall = float(rainfallStr)
        print("Cached forecasted rainfall: " + str(rainfall) + "mm.")
    return rainfall      
# <<<<-------------- OPENWEATHER PART -----------------


# -------------- SOLAR PART ----------------->>>>

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
    return time.hour + time.minute / 60 + time.second / 3600

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

# Cloudiness: proposed model is f(x) = 1/(1+ax²) + b (for R0+)
# clouds from 0 to 100%
def cloudinessFactor(clouds, a = 10, b = 0):
    return 1 / (1 + a * math.pow((clouds/100),2)) + b

# <<<<-------------- SOLAR PART -----------------

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
    if not config.has_section("redis"):
        endless_loop("Config: Redis section missing.")    
    # -------------- Parameters ------------------>>>
    mqttServerUrl = config.get("server","mqttServerUrl")
    mqttServerPort = config.getint("server","mqttServerPort")
    pemCertFilePath = config.get("server","pemCertFilePath")
    ackTopicLevel = config.get("topics","ackTopicLevel")
    measuresTopicLevel = config.get("topics","measuresTopicLevel")
    commandsTopicLevel = config.get("topics","commandsTopicLevel")
    deviceNameTemplate = config.get("devices","deviceName")
    iotDevMessage = config.get("messages","messageTemplate")
    relOpenWeatherUrl = config.get("openweather","url")
    locLat = config.get("location","lat")
    locLon = config.get("location","lon")
    unitSystem = config.get("openweather","units")
    language = config.get("openweather","lang")
    apiKey =  config.get("openweather","apiid")
    excludeInfo = config.get("openweather","exclude")

    runtimeOfProgram = int(config.get("timing","runtimeOfProgram"))
    retrievalInterval = int(config.get("timing", "pauseInSeconds"))
    weatherTimeout = int(config.get("timing", "weatherTimeout"))

    redisHost = config.get("redis", "redisHost")
    redisPort = int(config.get("redis", "redisPort"))
    redisPassword = config.get("redis", "redisPassword")
    redisDb = int(config.get("redis", "redisDb"))
    pauseTime = int(config.get("timing","pauseInSeconds"))
    runTime = int(config.get("timing","runtimeOfProgram"))
    # -------------- Parameters ------------------<<<

    loopCondition = True
    then = datetime.now()

    # Connect to Redis DB
    r = redis.Redis(host=redisHost, port=redisPort, db=redisDb, password=redisPassword, socket_timeout=None, decode_responses=True)

    # Start sending data to cloud
    while loopCondition:
        #TODO
        time.sleep(pauseTime)
        if runTime > 0:
            now = datetime.now()
            durationMins = divmod((now-then).total_seconds(), 60)[0]
            if durationMins > runTime:
                loopCondition = False

    time.sleep(2) # wait until we have all feedback messages from the server


    print("Shut down all clients. Entering endless loop. Restart pod if needed.")
    while True:
        pass
if __name__ == "__main__":
    main()
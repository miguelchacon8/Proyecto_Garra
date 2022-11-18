#define IO_USERNAME  "mchacon21543"
#define IO_KEY       "aio_Spom63Y6dg1t15FCZdaeIwqMF8g0"
"""
'publish.py'
=========================================
Publishes an incrementing value to a feed
Author(s): Brent Rubell, Todd Treece for Adafruit Industries
"""
# Import standard python modules.
import sys
import time
import serial

from Adafruit_IO import MQTTClient
# Import Adafruit IO REST client.
#from Adafruit_IO import Client, Feed

# holds the count for the feed
contador = 0
modo = 0
ADAFRUIT_IO_KEY = 'aio_Spom63Y6dg1t15FCZdaeIwqMF8g0'

# Set to your Adafruit IO username.
# (go to https://accounts.adafruit.com to find your username)
ADAFRUIT_IO_USERNAME = 'mchacon21543'

# Set to the ID of the feed to subscribe to for updates.
FEED_ID = 'modo'
FEED_ID2 = 'position'
FEED_ID3 = 'writeread'
FEED_ID4 = 'abrir'
FEED_ID5 = 'cerrar'
FEED_ID6 = 'subir'
FEED_ID7 = 'bajar'
FEED_ID8 = 'acercar'
FEED_ID9 = 'alejar'
FEED_ID10 = 'rotar'

client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

ser = serial.Serial(port='COM7', baudrate=9600, timeout=.1)
print(ser.name)

# Define callback functions which will be called when certain events happen.
def connected(client):
    # Subscribe to changes on a feed named Counter.
    print('Subscribing to Feed {0}'.format(FEED_ID))
    #print('Subscribing to Feed {0}'.format(FEED_ID2))
    client.subscribe(FEED_ID)
    client.subscribe(FEED_ID2)
    client.subscribe(FEED_ID3)
    client.subscribe(FEED_ID4)
    client.subscribe(FEED_ID5)
    client.subscribe(FEED_ID6)
    client.subscribe(FEED_ID7)
    client.subscribe(FEED_ID8)
    client.subscribe(FEED_ID9)
    client.subscribe(FEED_ID10)
    print('Waiting for feed data...')

def disconnected(client):
    sys.exit(1)

def message(client, feed_id, payload):
    global modo
    print('Feed {0} received new value: {1}'.format(feed_id, payload))
    if (feed_id == 'modo'):
        if (payload == '1'):
            ser.write(bytes('1', 'utf-8'))
    if (feed_id == 'position'):
        if (payload == '1'):
            ser.write(bytes('2', 'utf-8'))
    if (feed_id == 'writeread'):
        if (payload == '1'):
            ser.write(bytes('3', 'utf-8'))
    if (feed_id == 'abrir'):
        if (payload == '1'):
            ser.write(bytes('4', 'utf-8'))
            print(payload)
    if (feed_id == 'cerrar'):
        if (payload == '1'):
            ser.write(bytes('5', 'utf-8'))
    if (feed_id == 'bajar'):
        if (payload == '1'):
            ser.write(bytes('7', 'utf-8'))
            print(payload)
    if (feed_id == 'subir'):
        if (payload == '1'):
            ser.write(bytes('6', 'utf-8'))
    if (feed_id == 'acercar'):
        if (payload == '1'):
            ser.write(bytes('8', 'utf-8'))
            print(payload)
    if (feed_id == 'alejar'):
        if (payload == '1'):
            ser.write(bytes('9', 'utf-8'))
    if (feed_id == 'rotar'):
        if (payload == '1'):
            ser.write(bytes('0', 'utf-8'))
    time.sleep(2)

client.on_connect = connected
client.on_disconnect = disconnected
client.on_message = message
client.connect()
client.loop_blocking()

#while True:
    #print('modo: ', contador)
    #modo = ser.readLine()
    #print(modo)
    #aio.send_data('modo', modo)
    # Adafruit IO is rate-limited for publishing
    # so we'll need a delay for calls to aio.send_data()

    #time.sleep(3)

#ser.close()
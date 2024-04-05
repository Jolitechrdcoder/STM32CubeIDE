import paho.mqtt.client as mqtt
import serial

# establecer conexion serial entre el micro y el script
ser = serial.Serial('COM5', 9600)

# configurar cliente
client = mqtt.Client()

# conectarnos al broker
client.connect("mqtt-dashboard.com", 1883, 60)


def serial_to_mqtt():
    while True:
        if ser.in_waiting > 0:  
            data = ser.readline().strip() 
            client.publish("temp1", data)  


serial_to_mqtt()

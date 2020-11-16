# Check 
# https://onedrive.live.com/?authkey=%21AOrO2lZdzyZJF6Y&cid=DE1F08C4DF653B00&id=DE1F08C4DF653B00%214061&parId=DE1F08C4DF653B00%213508&o=OneUp
# https://onedrive.live.com/?authkey=%21AJ7%5FutWs7TTd6tk&cid=DE1F08C4DF653B00&id=DE1F08C4DF653B00%213510&parId=DE1F08C4DF653B00%213508&o=OneUp
import time
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(7, GPIO.OUT)

p = GPIO.PWM(7, 50)

p.start(0)
print ("starting 0")
time.sleep(3)

p.ChangeDutyCycle(3)
print("start")
time.sleep(5)



while True:
	i = 4
	while i<10:
		
		print(i)
		p.ChangeDutyCycle(i)
		time.sleep(.05)
		i +=.02
	
	while i>4:
		print(i)
		p.ChangeDutyCycle(i)
		time.sleep(.05)
		i -=.05

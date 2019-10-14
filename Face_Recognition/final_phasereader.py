#!/usr/bin/env python
import cv2
import os
import RPi.GPIO as GPIO
import time
import datetime
from time import sleep

# Define GPIO to LCD mapping
LCD_RS = 26
LCD_E  = 19
LCD_D4 = 13
LCD_D5 = 6
LCD_D6 = 5
LCD_D7 = 11
 
# Define some device constants
LCD_WIDTH = 16 # Maximum characters per line
LCD_CHR   = True
LCD_CMD   = False
 
LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
 
# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

def getImages():
	cam = cv2.VideoCapture(0)
	cam.set(3, 640) # set video width
	cam.set(4, 480) # set video height
	face_detector = cv2.CascadeClassifier('/home/pi/Face_Recognition/haarcascade_frontalface_default.xml')
	
	# For each person, enter one numeric face id
	face_id = input('\n enter user id end press <return> ==>  ')
	print("\n [INFO] Initializing face capture. Look the camera and wait ...")
	
	# Initialize individual sampling face count
	count = 0
	while(True):
		ret, img = cam.read()
		img      = cv2.flip(img, -1) # flip video image vertically
		gray     = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		faces    = face_detector.detectMultiScale(gray, 1.3, 5)

	    for (x,y,w,h) in faces:
	        cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)     
	        count += 1
	        # Save the captured image into the datasets folder
	        cv2.imwrite("/home/pi/Face_Recognition/dataset/User." + str(face_id) + '.' + str(count) + ".jpg", gray[y:y+h,x:x+w])
	        cv2.imshow('image', img)
	    k = cv2.waitKey(100) & 0xff # Press 'ESC' for exiting video

	    if k == 27:
	        break
	    elif count >= 30: # Take 30 face sample and stop video
	         break
	
	# Do a bit of cleanup
	print("\n [INFO] Exiting Program and cleanup stuff")
	cam.release()
	cv2.destroyAllWindows()

def trainTrainer():
	# Path for face image database
	path       = '/home/pi/Face_Recognition/dataset/'
	recognizer = cv2.face.LBPHFaceRecognizer_create()
	detector   = cv2.CascadeClassifier("/home/pi/Face_Recognition/haarcascade_frontalface_default.xml");
	
	# function to get the images and label data
	def getImagesAndLabels(path):
		imagePaths = [os.path.join(path,f) for f in os.listdir(path)]     
		faceSamples= []
		ids        = []
	    
	    for imagePath in imagePaths:
			PIL_img   = Image.open(imagePath).convert('L') # convert it to grayscale
			img_numpy = np.array(PIL_img,'uint8')
			id        = int(os.path.split(imagePath)[-1].split(".")[1])
			faces     = detector.detectMultiScale(img_numpy)

	        for (x,y,w,h) in faces:
	            faceSamples.append(img_numpy[y:y+h,x:x+w])
	            ids.append(id)
	    return faceSamples,ids

	print ("\n [INFO] Training faces. It will take a few seconds. Wait ...")
	faces,ids = getImagesAndLabels(path)
	recognizer.train(faces, np.array(ids))
	
	# Save the model into trainer/trainer.yml
	recognizer.write('/home/pi/Face_Recognition/trainer.yml') # recognizer.save() worked on Mac, but not on Pi
	
	# Print the numer of faces trained and end program
	print("\n [INFO] {0} faces trained. Exiting Program".format(len(np.unique(ids)))) 

def detectFace():
	recognizer  = cv2.face.LBPHFaceRecognizer_create()
	recognizer.read('trainer/trainer.yml')
	cascadePath = "haarcascade_frontalface_default.xml"
	faceCascade = cv2.CascadeClassifier(cascadePath);
	font        = cv2.FONT_HERSHEY_SIMPLEX
	
	#iniciate id counter
	id = 0
	
	# names related to ids: example ==> Marcelo: id=1,  etc
	names = ['None', 'Marcelo', 'Paula', 'Ilza', 'Z', 'W'] 
	
	# Initialize and start realtime video capture
	cam = cv2.VideoCapture(0)
	cam.set(3, 640) # set video widht
	cam.set(4, 480) # set video height
	# Define min window size to be recognized as a face
	minW = 0.1*cam.get(3)
	minH = 0.1*cam.get(4)
	while True:
		ret, img = cam.read()
		img      = cv2.flip(img, -1) # Flip vertically
		gray     = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	    
	    faces = faceCascade.detectMultiScale( 
	        gray,
	        scaleFactor = 1.2,
	        minNeighbors = 5,
	        minSize = (int(minW), int(minH)),
	       )
	    for(x,y,w,h) in faces:
	        cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)
	        id, confidence = recognizer.predict(gray[y:y+h,x:x+w])

	        # Check if confidence is less them 100 ==> "0" is perfect match 
	        if (confidence < 100):
	            id = names[id]
	            confidence = "  {0}%".format(round(100 - confidence))
	        else:
	            id = "unknown"
	            confidence = "  {0}%".format(round(100 - confidence))
	        
	        cv2.putText(img, str(id), (x+5,y-5), font, 1, (255,255,255), 2)
	        cv2.putText(img, str(confidence), (x+5,y+h-5), font, 1, (255,255,0), 1)  
	    
	    cv2.imshow('camera',img) 
	    k = cv2.waitKey(10) & 0xff # Press 'ESC' for exiting video
	    if k == 27:
	        break
	# Do a bit of cleanup
	print("\n [INFO] Exiting Program and cleanup stuff")
	cam.release()
	cv2.destroyAllWindows()

def lcd_init():
     
	# Initialise display
    lcd_byte(0x33,LCD_CMD) # 110011 Initialise
    lcd_byte(0x32,LCD_CMD) # 110010 Initialise
    lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
    lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
    lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
    lcd_byte(0x01,LCD_CMD) # 000001 Clear display
    time.sleep(E_DELAY)

    sleep(E_DELAY)

def lcd_byte(bits, mode):
	# Send byte to data pins
    # bits = data
    # mode = True for character
    # False for command
     
    GPIO.output(LCD_RS, mode) # RS
     
    # High bits
    GPIO.output(LCD_D4, False)
    GPIO.output(LCD_D5, False)
    GPIO.output(LCD_D6, False)
    GPIO.output(LCD_D7, False)
    if bits&0x10==0x10:
        GPIO.output(LCD_D4, True)
    if bits&0x20==0x20:
        GPIO.output(LCD_D5, True)
    if bits&0x40==0x40:
        GPIO.output(LCD_D6, True)
    if bits&0x80==0x80:
        GPIO.output(LCD_D7, True)
     
    # Toggle 'Enable' pin
    lcd_toggle_enable()
     
    # Low bits
    GPIO.output(LCD_D4, False)
    GPIO.output(LCD_D5, False)
    GPIO.output(LCD_D6, False)
    GPIO.output(LCD_D7, False)
    if bits&0x01==0x01:
        GPIO.output(LCD_D4, True)
    if bits&0x02==0x02:
        GPIO.output(LCD_D5, True)
    if bits&0x04==0x04:
        GPIO.output(LCD_D6, True)
    if bits&0x08==0x08:
        GPIO.output(LCD_D7, True)
     
    # Toggle 'Enable' pin
    lcd_toggle_enable()
 
def lcd_toggle_enable():
    # Toggle enable
    time.sleep(E_DELAY)
    GPIO.output(LCD_E, True)
    time.sleep(E_PULSE)
    GPIO.output(LCD_E, False)
    time.sleep(E_DELAY)
 
def lcd_string(message,line):
    # Send string to display
     
    message = message.ljust(LCD_WIDTH," ")
     
    lcd_byte(line, LCD_CMD)
     
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]),LCD_CHR)

def lcd():
	# Main program block
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM) # Use BCM GPIO numbers
	GPIO.setup(LCD_E, GPIO.OUT) # E
	GPIO.setup(LCD_RS, GPIO.OUT) # RS
	GPIO.setup(LCD_D4, GPIO.OUT) # DB4
	GPIO.setup(LCD_D5, GPIO.OUT) # DB5
	GPIO.setup(LCD_D6, GPIO.OUT) # DB6
	GPIO.setup(LCD_D7, GPIO.OUT) # DB7
 
	# Initialise display
	lcd_init()
     
    today = datetime.datetime.now().strftime('%b %d  %H:%M:%S\n') 
	while True:
    	lcd_string("Face Detector",LCD_LINE_1)
    	lcd_string(today,LCD_LINE_2)
 
	sleep(3)

def main():
	lcd()
	inputNo = input('\n Enter 1 to to add')

	if inputNo == 1:
		getImages()
		trainTrainer()
	detectFace()

main()
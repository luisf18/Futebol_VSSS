# import the necessary packages
from threading import Thread
import cv2
import time

t = e1 = e2 = 0
def tic():
    global t, e1
    t = time.time()
    e1 = cv2.getTickCount()
def tac( n=1 ):
   global t, e1, e2
   dt = (time.time()-t)/n
   e2 = cv2.getTickCount()
   sec = int( dt )
   ms  = int( 1e3*dt ) - sec*1e3
   us  = int( 1e6*dt ) - ms*1e3 - sec*1e6
   print( f'time: {int(sec)}:{int(ms)}:{int(us)}',end='\n' )
   print('time:', (e2 - e1)/(n*cv2.getTickFrequency()) )
   print('f:', 1/dt, ' Hz' )


class video_thread:
	def __init__(self, src=0):
		self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
		self.cap.set(cv2.CAP_PROP_FPS, 120)
		if int((cv2.__version__).split('.')[0])  < 3 :
			fps = self.cap.get(cv2.cv.CV_CAP_PROP_FPS)
			print("Frames per second using video.get(cv2.cv.CV_CAP_PROP_FPS): {0}".format(fps))
		else :
			fps = self.cap.get(cv2.CAP_PROP_FPS)
			print("Frames per second using video.get(cv2.CAP_PROP_FPS) : {0}".format(fps))
		(self.grabbed, self.frame) = self.cap.read()
		self.Flag_on = False
		self.n = 0
	def start(self):
		self.Flag_on = True
		Thread(target=self.update, args=()).start()
	def update(self):
		while True:
			if not self.Flag_on: return
			# otherwise, read the next frame from the stream
			(self.ret, self.frame) = self.cap.read()
			self.n+=1
	def read(self):
		self.n = 0
		return self.frame
	def stop(self):
		self.Flag_on = False
	def available(self):
		return self.n

video = video_thread( 1 )

tic()
video.start()

n = 0
while( n < 240 ):
	if( video.available() ):
		video.read()
		n+=1
		print(n)
video.stop()
tac(240)
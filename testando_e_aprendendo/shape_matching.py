import os
import time
import cv2
import numpy as np
from cmath import polar

path = "C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS/"

# Rotation
def rotate(a, angle, rotPoint=None):
    (height,width) = a.shape[:2]
    if rotPoint is None:
        rotPoint = (width//2,height//2)
    rotMat = cv2.getRotationMatrix2D( rotPoint, angle, 1.0 )
    dimensions = (a.shape[1],a.shape[0])
    return cv2.warpAffine( a, rotMat, dimensions )

def find( mask, tela, a_min = 700, tag = "0" ):
    # Contornos
    contornos, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    a_max = 0
    cnt_list = []
    cnt_i = 0

    # Verificação dos contornos
    for i, cnt in enumerate(contornos):
        area = cv2.contourArea(cnt)
        if( area > a_min ):
            cnt_list += [ cnt ]
            if( area > a_max ):
                a_max = area
                cnt_i = len(cnt_list)-1

    if( tag == "ball" and ( len(cnt_list) > 0 ) ):
       cnt_list = [ cnt_list[cnt_i] ]

    cord = [ ]
    for cnt in cnt_list:

        if(tag == "ball"):
            center, radius = cv2.minEnclosingCircle(cnt)
            l = w = int(radius)
            v = [0,0]
            center = np.array([center[0],center[1]],np.int64)
            if( tela is not None ):
              cv2.circle(tela, center,l,(0,0,255),2)
        else:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            if( tela is not None ):
              cv2.drawContours(tela,[box],0,(0,0,255),2)

        if( tela is not None ):
            x,y,w,h = cv2.boundingRect(cnt)
            cv2.rectangle(tela,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.drawContours(tela, [cnt], -1, (0, 255, 0), 0)
            #cv2.circle( tela, (x,y), 3, (0,0,255), -1)
            #cv2.putText( tela, tag+f'({l},{w})', (center[0]+40,center[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,200,0), 2, cv2.LINE_AA )
    
    #return np.array( cord, np.float64 )



if __name__ == '__main__':

  img_file = path + "images/enfilerado_cap.png"

  while True:
    #break
    img = cv2.imread( img_file )
    cv2.imshow('img',img)

    hsv = cv2.cvtColor( img,cv2.COLOR_BGR2HSV)

    S_min = 90
    V_min = 90

    # total mask
    blur = cv2.GaussianBlur(hsv,(5,5),cv2.BORDER_DEFAULT)
    mask = cv2.inRange( blur, (0, S_min, V_min), (180,255,255) )
    #mask = cv2.bitwise_and( hsv, hsv, mask = mask )

    cv2.imshow('mask',mask)

    find( mask, img, a_min = 300, tag = "x" )

    img = rotate(img,-50,(0,0))

    cv2.imshow('tela',img)

    # key --------------------------------------------
    key = cv2.waitKey(300) & 0xFF
    if key == ord('q'): break
    # key --------------------------------------------
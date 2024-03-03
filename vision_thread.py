#import os
import cv2
import time
import numpy as np
from cmath import polar
from threading import Thread


def complex_to_xy( cp ):
   return np.array( [ cp.real, cp.imag ], np.int32 )

def xy_to_complex(points):
   if(len(points) <= 0):
      return np.array([])
   return points@np.array([[1],[1j]])[:,0]

def plot_arrow( img, center, v, hue = 0 ):
    cv2.arrowedLine(
        img,
        (center[0],center[1]),
        (v[0]+center[0],v[1]+center[1]),
        (hue,255,255), 3, tipLength = 0.2 )
    
def rect_coord(P):
  P = (P@np.array([[1],[1j]]))[:,0]
  center = np.mean(P)
  d = P - P[0]
  D = abs(d)
  sort = (-D).argsort()
  d = d[sort] # Dists ord.
  D = D[sort] # Dists ord.
  v = d[2]/D[2]
  return (
      np.array([center.real,center.imag],np.int64),
      np.array([v.real,v.imag],np.float64),
      np.array(D[1:3],np.int64))

def filtra_contornos( mask, A_min = 700, n = 0 ):
    # Contornos
    contornos, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt_list = []
    area_list = []
    # Verificação dos contornos
    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if( area > A_min ):
            area_list += [area]
            cnt_list += [cnt]
    
    area_list = np.array( area_list )
    sort = (-area_list).argsort()
    cnt_list = [ cnt_list[i] for i in sort ]
    area_list = area_list[sort]

    if( n != 0 ):
       cnt_list = cnt_list[:n]
       area_list = area_list[:n]

    return ( cnt_list, area_list )

def match_contours( contours, shape = 'rect', tela = None ):
    center_list = []
    dimension_list = []
    vector_list = []
    box_list = []
    for cnt in contours:
      if(shape == 'circle'):
          center, radius = cv2.minEnclosingCircle(cnt)
          dimension = np.array( [int(radius),int(radius)], np.uint32 )
          vector = np.array( [0,0], np.uint32 )
          rect = cv2.minAreaRect(cnt)
          box = np.int0(cv2.boxPoints(rect))
          center = np.array( center, np.uint32 )
          #cv2.circle(tela, center,l,(0,0,255),2)
      else:
          rect = cv2.minAreaRect(cnt)
          box = np.int0(cv2.boxPoints(rect))
          center, vector, dimension = rect_coord( box )
          if(tela is not None):
             plot_arrow( tela, center, np.array(50*vector,np.int32) )
      vector_list += [vector]
      dimension_list += [dimension]
      center_list += [center]
      box_list += [box]
    return (center_list, dimension_list, vector_list, box_list)

def link_0( R, points, colors, v, dimension, tag = ['black','black'], tela = None, delta = 10 ):

  L_ref = 32.5+17.5j
  k = (dimension.real/65 + dimension.imag/30)/2
  Z = k*L_ref

  if( tela is not None ):
     cv2.circle(tela, complex_to_xy( R ), 2 ,(55,255,255), 2 )
     cv2.circle(tela, complex_to_xy( R ), 3 ,(151,255,255), 2 )
     p1 = R+v*Z
     p2 = R+v*(Z.conj())
     cv2.circle(tela, complex_to_xy( p1 ), 2 ,(55,255,255), 2 )
     cv2.circle(tela, complex_to_xy( p1 ), 3 ,(151,255,255), 2 )
     cv2.circle(tela, complex_to_xy( p2 ), 2 ,(55,255,255), 2 )
     cv2.circle(tela, complex_to_xy( p2 ), 3 ,(151,255,255), 2 )
     p1 = R-v*Z
     p2 = R-v*(Z.conj())
     cv2.circle(tela, complex_to_xy( p1 ), 2 ,(55,255,255), 2 )
     cv2.circle(tela, complex_to_xy( p1 ), 3 ,(151,255,255), 2 )
     cv2.circle(tela, complex_to_xy( p2 ), 2 ,(55,255,255), 2 )
     cv2.circle(tela, complex_to_xy( p2 ), 3 ,(151,255,255), 2 )

  P = points
  T = (P-R)/v

  nota = ( abs(abs(T.real) - Z.real) + abs(abs(T.imag) - Z.imag) )

  #print( 'T',T )

  filtro = nota < delta
  nota = nota[filtro]
  colors = colors[filtro]
  T = T[filtro]

  filtro = nota.argsort()
  nota = nota[filtro]
  colors = colors[filtro]
  T = T[filtro]

  #print(f' K: {k} // ','v',v,'Dimension',dimension)
  #print('Nota[filtro]',nota)

  # decide quais os pontos escolhidos
  center = R
  for i, p in enumerate(T):
    A = T[i+1:] - p.conj()
    filtro = abs(A) < delta
    #print( f'tentativa {i} -> A', A )
    if( True in filtro ):
      c1 = colors[i]
      c2 = colors[i+1:][filtro][0]
      center = (p + T[i+1:][filtro][0])/4
      center = center*v+R
      v = (center - R)
      v = v/abs(v)
      if( (p.real > 0) == (p.imag > 0) ):
        tag = [ c1, c2 ]
      else:
        tag = [ c2, c1 ]
      break
  
  cv2.circle(tela, complex_to_xy( center ), 3 ,(0,255,255), 2 )
  cv2.circle(tela, complex_to_xy( center ), 30 ,(120,255,255), 2 )
  plot_arrow( tela, complex_to_xy( center ), complex_to_xy( 50*v ), hue = 60 )
  
  #print( ' FIM tentativas ', tag )
  
  return (center,tag,v)

def sort_bots( dict ):
   a = sorted(dict.items(), key=lambda x: x[0])
   return { b[0]: b[1] for b in a }


print('BEGIN')



def inrange( A, h_min, h_max, s_min, v_min ):
  C = np.logical_and( 
      np.logical_and( A[:,:,0] > h_min, A[:,:,0] < h_max ),
      np.logical_and( A[:,:,1] > s_min, A[:,:,2] > v_min )
    )
  return np.array(C,np.uint8)





def vision( img, COLOR, IN, conv ):

  hsv = cv2.cvtColor( img,cv2.COLOR_BGR2HSV)
  
  # -----------------------------------------------------------------------------------------
  # OBTENÇÃO DAS INFORMAÇÕES DE CADA COR ----------------------------------------------------
  # CENTRO / VETOR DIRETOR / DIMENSSÕES  ----------------------------------------------------
  # -----------------------------------------------------------------------------------------
  data_center = {}
  data_vector = {}
  data_dimension = {}
  # UMA COR DE CADA VEZ ---------------------------------------------------------------------
  
  tela = hsv.copy()
  color_img = np.zeros(img.shape,np.uint8)

  for i,color in enumerate(COLOR['MEAN']):

    # MASK ----------------------------------------------------------------------------------
    if( COLOR['MIN'][color] < COLOR['MAX'][color] ):
      mask = cv2.inRange( hsv, (COLOR['MIN'][color], IN['S min'], IN['V min']), (COLOR['MAX'][color],255,255) )
    else:
      mask = cv2.inRange( hsv, (COLOR['MIN'][color], IN['S min'], IN['V min']), (180,255,255) )
      mask |= cv2.inRange( hsv, (0, IN['S min'], IN['V min']), (COLOR['MAX'][color],255,255) )
    color_img[ np.array( mask, np.bool8 ) ] = [ COLOR['MEAN'][color], 255, 255 ]
    # MASK ----------------------------------------------------------------------------------

    # OBTENÇÃO DAS INFORMAÇÕES DE CADA COR --------------------------------------------------
    # CENTRO / VETOR DIRETOR / DIMENSSÕES  --------------------------------------------------
    n = 0
    shape = 'rect'
    if( color == 'darkblue' or color == 'yellow' ):
        n = 3
    elif( color == 'orange' ):
        n = 1
        shape = 'circle'
    
    contornos, area = filtra_contornos( mask, 5*IN['area min'], n=n )
    center, dimension, vector, box = match_contours( contornos, shape=shape ) #, tela=hsv_2 )

    data_center[color] = xy_to_complex(center)
    data_vector[color] = xy_to_complex(vector)
    data_dimension[color] = xy_to_complex(dimension)
    # ----------------------------------------------------------------------------------------
    
    # MARCAÇOES NA TELA --------------------------------------------------------
    #cv2.drawContours( hsv, contornos, -1, (color_hue_mean[i],255,255), 0 )
    if( color == 'orange' ):
       if( len( center ) > 0 ):
        cv2.circle(tela, center[0], 5 ,(60,255,255),2)
        cv2.circle(tela, center[0], dimension[0][0],(60,255,255),2)
        cv2.circle(color_img, center[0], dimension[0][0],(60,255,255),2)
    else:
      for p in center:
        cv2.circle(tela, p, 5 ,(COLOR['MEAN'][color],255,255),2)
      for b in box:
        cv2.drawContours(tela,[b],0,(COLOR['MEAN'][color],255,255),2)
  # ------------------------------------------------------------------------------------------------

  # -----------------------------------------------------------------------------------------
  # SEGMENTAÇÃO DOS DADOS DE COR ------------------------------------------------------------
  # ORIENTAÇÃO GLOBAL ROBÔS E BOLA  ---------------------------------------------------------
  # -----------------------------------------------------------------------------------------

  # robôs: time / id / angle / vector / pos / dimension

  #message SSL_DetectionBall {
  #  required float  confidence = 1;
  #  optional uint32 area       = 2;
  #  required float  x          = 3;
  #  required float  y          = 4;
  #  optional float  z          = 5;
  #  required float  pixel_x    = 6;
  #  required float  pixel_y    = 7;
  #}

  #message SSL_DetectionRobot {
  #  required float  confidence  =  1;
  #  optional uint32 robot_id    =  2;
  #  required float  x           =  3;
  #  required float  y           =  4;
  #  optional float  orientation =  5;
  #  required float  pixel_x     =  6;
  #  required float  pixel_y     =  7;
  #  optional float  height      =  8;
  #}

  ball_detection = {
     'ok': False,
     'pos': 0,
     'dimension': 0
  }

  bot_detection = {
     'id': 0,
     'pos': 0,
     'orientation': 0,
     'dimension': 0,
     'vector': 0,
     'colors': ['orange','orange']
  }

  game = {
     'ball': ball_detection,
     'team_blue': {},
     'team_yellow': {}
  }
  
  # Bola ------------------------------------------------------------------------------------
  if( len(data_center['orange']) ):
    game['ball']['ok'] = True
    game['ball']['pos'] = data_center['orange'][ 0 ]*conv
    game['ball']['dimension'] = data_dimension['orange'][0]*conv
  # -----------------------------------------------------------------------------------------

  # TRANSFORMA EM NUMPY ARRAY PARA FACILITAR O PROCESSAMENTO --------------------------------
  SEG_COLORS = []
  SEG_POINTS = []
  for color in ['red','blue','green','pink']:
    SEG_COLORS = np.append( SEG_COLORS, np.array(len(data_center[color])*[color]) )
    SEG_POINTS = np.append( SEG_POINTS, np.array(data_center[color]) )
  # -----------------------------------------------------------------------------------------

  tag2number = {
     'darkblue':0,
     'yellow': 0,
     'red': 1,
     'green': 2,
     'blue': 3,
     'pink': 4
  }

  # SEGUIMENTAÇÕES DOS ROBÔS UM DE CADA VEZ -------------------------------------------------
  for team in [ 'darkblue', 'yellow' ]:
    gen_id = 50
    if( team == 'yellow' ):
       team_key = 'team_yellow'
    else:
       team_key = 'team_blue'
    for i,ref in enumerate(data_center[team]):
      if(i < 3):
        center, tag, v = link_0( ref, SEG_POINTS, SEG_COLORS,
                                 data_vector[team][i],
                                 data_dimension[team][i],
                                 tag = [team,team],
                                 tela=tela,
                                 delta = IN['delta'] )
        id = 10*tag2number[tag[0]] + tag2number[tag[1]]
        if(id == 0):
           id = gen_id
           gen_id+=1
        game[team_key][id] = bot_detection.copy()
        game[team_key][id]['id'] = id
        game[team_key][id]['pos'] = center*conv
        game[team_key][id]['colors'] = tag
        game[team_key][id]['orientation'] = polar(v)[1]
        game[team_key][id]['vector'] = v
        game[team_key][id]['dimension'] = data_dimension[team][i]*conv
    game[team_key] = sort_bots( game[team_key] )
  # -----------------------------------------------------------------------------------------
  
  # ATUALIZA OS MONITORES -------------------------------------------------------------------
  img_data = {}
  img_data['vision'] = tela
  img_data['colors'] = color_img
  #OUT[ 'monitor_color' ].update_hsv( color_img )
  #OUT[ 'monitor_mask' ].update_hsv(tela)
  # -----------------------------------------------------------------------------------------

  return (game, img_data)









COLOR = {}
IN = {
   'S min': 70,
   'V min': 70,
   'delta': 20,
   'area min': 20
}

COLOR['MIN'] = {
    'orange': 12,
    'yellow': 28,
    'green': 55,
    'blue': 85,
    'darkblue': 116,
    'pink': 151,
    'red': 178
}

COLOR['MAX'] = {
    'orange': 28,
    'yellow': 55,
    'green': 85,
    'blue': 116,
    'darkblue': 151,
    'pink': 178,
    'red': 12
}

COLOR['MEAN'] = { color: (COLOR['MAX'][color]+COLOR['MIN'][color])//2 for color in COLOR['MIN'] }


if __name__ == '__main__':
  
  print( '-- MAIN --' )

  #time.sleep(2)
  
  #import main_gui as gui

  #gui.camera.cap.cap.set(cv2.CAP_PROP_FPS, 120)

  video = cv2.VideoCapture(0, cv2.CAP_DSHOW)
  video.set(cv2.CAP_PROP_FPS, 120)
  video.set(cv2.CAP_PROP_FRAME_WIDTH, 3840/6)
  video.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160/6)

  #gui.camera.cap.cap.set(cv2.CAP_PROP_FRAME_WIDTH , 3840/6)
  #gui.camera.cap.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160/6)

  start = time.time()
  
  FILE = __file__
  path = FILE[:FILE.rfind('\\')+1]

  frame_1 = cv2.imread( path + '\images\\vsss_teste_cap.png' )

  while(True):
      start = time.time()
      #for i in range(1):

      #frame = frame_1.copy()
      
      ret, frame = video.read()
      if not ret:
          print("Can't receive frame (stream end?). Exiting ...")
          break
      ''''''
      game, img_data = vision( frame, COLOR, IN, 1 )

      color = cv2.cvtColor( img_data['colors'], cv2.COLOR_HSV2BGR)
      tela  = cv2.cvtColor( img_data['vision'], cv2.COLOR_HSV2BGR)
      cv2.imshow('vision',tela)
      cv2.imshow('colors',color)
      ''''''
      end = time.time()
      seconds = end - start
      fps  = int(1/seconds)
      print(f"Estimated frames per second : {fps} - {int(1000*seconds)}ms")

      cv2.putText( frame, f'FPS: {int(fps)}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2, cv2.LINE_AA )
      cv2.imshow('img',frame)
      if cv2.waitKey(1) == ord('q'):
          break

import os
import time
import cv2
import numpy as np
from cmath import polar

FILE = __file__
path = FILE[:FILE.rfind('\\')+1]



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

def relative_dist( refs, points ):
  R = np.array([refs]).transpose()
  P = np.kron( points, np.ones((len(refs),1))  )
  return P-R

def dist(a,b): return sum((b - a)**2)**0.5

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
      np.array(D[1:3],np.int64)
    )


# NOVAS FUNÇÕES ======================================================================================
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


# relaciona os pontos ====================================================================
def link_points( ref, points, min_dist = 15 ):
  center = []
  angle = []
  vector = []
  RD = abs(relative_dist( ref, points ))
  for i,r in enumerate(RD<min_dist):
    I = np.where(r == True)[0]
    if( len(I) ):
        center += [ [points[I[0]].real,points[I[0]].imag] ]
        vector += [ points[I[0]] - ref[i] ]
        angle += [ np.rad2deg(polar(vector[-1])[1]) ]
    else:
        center += [ [0,0] ]
        angle += [ 0 ]
        vector += [ 0 ]
  return (center, angle, vector)

# relaciona os pontos das tags ====================================================================

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



#####################################################################################
#################################   [__main__]   ####################################
#####################################################################################

colors = [
    ['orange',   7 ],
    ['yellow',   18 ],
    ['green',    38 ],
    ['blue',     72 ],
    ['darkblue', 99 ], #106
    ['pink',     134 ],
    ['red',      169 ],
]


def simple_vision( img, hue, conf ):
  hue_min, hue_max, hue_mean = hue

  S_min = conf['S min'].get()
  V_min = conf['V min'].get()
  A_min = conf['area min'].get()
  
  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  hsv_2 = hsv.copy()
  #blur = cv2.GaussianBlur(img,(1+2*Blur_ajuste,1+2*Blur_ajuste),cv2.BORDER_DEFAULT)
  #blur = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
  mask = cv2.inRange( hsv, (0, S_min, V_min), (180,255,255) )
  mask = cv2.inRange( hsv, (hue_min['blue'], S_min, V_min), (hue_max['blue'],255,255) )

  contornos, area = filtra_contornos( mask, A_min*2 )
  center, dimension, vector, box = match_contours( contornos, tela=hsv_2 )
  
  contornos, area = filtra_contornos( mask, A_min )
  center_b, dimension_b, vector_b, box_b = match_contours( contornos, tela=hsv_2 )

  center = xy_to_complex( center )
  center_b = xy_to_complex( center_b )

  for i,c in enumerate(center):
     L = dimension[i][1]



def tic(): return time.time()

def tac(t,tag):
   dt = time.time()-t
   print( f'{tag}: {int(1000*1000*(dt))}us',end='\t' )
   return dt



# MAIN VISION ================================================================================================

def sort_bots( dict ):
   a = sorted(dict.items(), key=lambda x: x[0])
   return { b[0]: b[1] for b in a }

T = 0

def vision( img, COLOR, IN, OUT, conv ):

  # teste contornos =====================================================================
  hsv = cv2.cvtColor( img,cv2.COLOR_BGR2HSV)
  tela = hsv.copy()
  if( 'Blur' in IN ):
     hsv = cv2.GaussianBlur(img,(1+2*IN['Blur'],1+2*IN['Blur']),cv2.BORDER_DEFAULT)
     hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)
  
  # -----------------------------------------------------------------------------------------
  # OBTENÇÃO DAS INFORMAÇÕES DE CADA COR ----------------------------------------------------
  # CENTRO / VETOR DIRETOR / DIMENSSÕES  ----------------------------------------------------
  # -----------------------------------------------------------------------------------------
  #data_color = {}
  data_center = {}
  data_vector = {}
  data_dimension = {}
  color_img = np.zeros(img.shape,np.uint8)
  # UMA COR DE CADA VEZ ---------------------------------------------------------------------
  
  #t = tic()
  blank2 = np.zeros(img.shape,np.uint8)
  blank2[:,:] = [0,255,255]
  for i,color in enumerate(COLOR['MEAN']):

    # MASK ----------------------------------------------------------------------------------
    if( COLOR['MIN'][color] < COLOR['MAX'][color] ):
      mask = cv2.inRange( hsv, (COLOR['MIN'][color], IN['S min'], IN['V min']), (COLOR['MAX'][color],255,255) )
    else:
      mask = cv2.inRange( hsv, (COLOR['MIN'][color], IN['S min'], IN['V min']), (180,255,255) )
      mask |= cv2.inRange( hsv, (0, IN['S min'], IN['V min']), (COLOR['MAX'][color],255,255) )
    blank = blank2.copy()
    blank[:,:,0] = COLOR['MEAN'][color]
    color_img |= cv2.bitwise_and(blank,blank,mask=mask)
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
    # --------------------------------------------------------------------------

  #t = tac(t,'masking time:')
  #print( '' )

    #print( f'[{i}] t -> {int(1000*1000*(time.time()-t))}us' )
  
  global T
  T = time.time()-T
  tag_HZ = f't:{int(1000*T)} ms  F: {1/T:0.1f}Hz'
  OUT['tagHZ'].set( tag_HZ )
  #print( tag_HZ )
  #cv2.rectangle( tela, (10,5), (280, 40), (0,0,0), -1)
  #cv2.putText( tela, tag_HZ, (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (60,255,255), 2, cv2.LINE_AA )
  T = time.time()
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





def test_gui():
  import main_gui as gui
  #while True:
  def display():
    
    img = gui.camera.read()

    # INPUT IMG =====================================================================================
    size = gui.painel.sliders['H'].get()
    if( size > 0 ):
       H, W, _ = img.shape
       w = (W*size)//200
       h = (H*size)//200
       x = W//2
       y = H//2
       #print( f'({x},{y}) -> [x: {x-w},{x+w}] [y: {y-h},{y+h}] ({W},{H}) -> ({w},{h})' )
       img = img[ y-h:y+h, x-w:x+w,:]
    gui.monitor_camera.update_BGR(img)
    # ==============================================================================================

    
    # VISION ==============================================================================
    VS_COLORS, VS_IN, VS_OUT = gui.loop()
    vs = vision(img, VS_COLORS, VS_IN, VS_OUT, 0.2 )[0]
    gui.update_tags( vs )
    # VISION ==============================================================================



    #gui.tag0.set( f'File: {img_file}' )

    # chamada
    gui.win.after(100,display)
  
  display()
  gui.win.mainloop()


if __name__ == '__main__':

   test_gui()
from time import time
from math import pi, sin, cos
from cmath import polar, phase, rect
import numpy as np


def constrain(x, MIN, MAX):
  if (x >= MAX): return MAX
  if (x <= MIN): return MIN
  return x

## NOVO ==================================================================

class new_pid:

  def __init__( self, kp=1.0, ki=0.0, kd=0.0, I_max = 100 ) -> None:
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.t = 0
    self.P = 0
    self.I = 0
    self.D = 0
    self.t_last = time()
    self.I_max = I_max

  def update( self, erro ):

    t = time()
    dt = max( t - self.t_last, 0.0001)
    self.t_last = t

    self.D = (self.P - erro)/dt
    self.I = min( self.I + erro*dt, self.I_max )
    self.P = erro
    
    return self.P*self.kp + self.I*self.ki + self.D*self.kd

# diferential drive robot
def DDR_wheels_speed( lin, angular, lin_th = 400 ):
  if( lin < 400 ):
    vl = lin - angular
    vr = lin + angular
  else:
    if( angular >= 0 ):
      vl = lin - angular
      vr = lin
    else:
      vl = lin
      vr = lin + angular
  return ( constrain( vl ), constrain( vr ) )

## NOVO ==================================================================



class pid:

  def __init__(self, kp=1.0, ki=0.0, kd=0.0, max = 1000) -> None:
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.t = 0
    self.P = 0
    self.I = 0
    self.D = 0
    self.theta_erro_last = 0
    self.max = max

  def update(self, speed, pos_act, theta_act, pos_set):

    t = time()
    self.t_last = time()
    dt = 1 #min( t - self.t_last, 100)

    v_set = pos_set - pos_act
    theta_set = polar(v_set)[1]
    theta_erro = theta_set - theta_act
    theta_erro = -theta_erro
    d = polar(v_set)[0]
    if d < 20:
      print( f"[ dist {polar(v_set)[0]} {polar(v_set)[1]} ][ erro {theta_erro} ]" )
      return (0,0)

    if( theta_erro > 0 ):
      theta_erro = theta_erro % pi
    elif (theta_erro < 0):
      theta_erro = -(abs(theta_erro) % pi)

    self.P = theta_erro * self.kp
    self.I += theta_erro * self.ki * dt
    self.I = constrain(self.I, -self.max, self.max)
    self.D = (theta_erro - self.theta_erro_last) * self.kd * dt
    self.theta_erro_last = theta_erro

    dif = self.P + self.I + self.D
    vl = constrain(speed - dif, -self.max, self.max)
    #vr = constrain(speed + dif, -self.max, self.max)
    vr = constrain(vl + 2*dif, -self.max, self.max)
    
    return (vl,vr,d,theta_erro)


class pid_linha:

  def __init__(self, kp=1.0, ki=0.0, kd=0.0, max = 1000, max_I = None) -> None:
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.t = 0
    self.P = 0
    self.I = 0
    self.D = 0
    self.erro_last = 0
    self.t_last = 0
    self.max = max
    self.k = 0.99
    if( max_I is None ):
      self.max_I = max
    else:
      self.max_I = max

  def update(self, y, set_y ):

    t = time()
    dt = constrain( t - self.t_last, 0.001, 0.05)

    erro = set_y - y

    #if erro < 10:
    #  print( f"[ dist {polar(v_set)[0]} {polar(v_set)[1]} ][ erro {theta_erro} ]" )
    #  return (0,0)

    self.P = erro * self.kp

    #self.I = (1.0-self.k)*erro*dt + self.I*self.k
    
    self.I += erro * dt
    self.I = constrain(self.I, -self.max_I, self.max_I)
    #if(self.ki > 0):
    #  self.I = constrain(self.ki*self.I, -self.max_I, self.max_I)/self.ki
    
    self.D = (erro - self.erro_last) * self.kd * dt
    self.erro_last = erro

    v = self.P + self.ki*self.I + self.D

    v = constrain(v,-self.max,self.max)

    self.t_last = t
    
    return (v,erro)
  


class pd:

  def __init__(self, kp=1.0, kd=0.0, max = 1000 ) -> None:
    self.kp = kp
    self.kd = kd
    
    self.t = 0
    self.t_last = 0
    
    self.erro_last = 0
    
    self.max = max

  def update(self, set_y, y=0 ):

    t = time()
    dt = constrain( t - self.t_last, 0.001, 0.05)

    erro = set_y - y

    OUT = self.kp * ( erro  + self.kd * ((erro - self.erro_last)/dt) )
    
    self.erro_last = erro

    OUT = constrain(OUT,-self.max,self.max)

    self.t_last = t
    
    return (OUT, erro)



from math import floor, e
def constrain_angle(x, Min=-np.pi, Max=np.pi ):
  return x-(Max-Min)*floor( (x-Min)/(Max-Min) )

def s_function( x, dx, min, max ):
  return ( min + (max-min)*(1/(1+e**(5-10*(x/dx)))) )



from cmath import rect

# a força que q1 faz em q2
def get_force( q1, q2, r1, r2 ):
  r = r2 - r1
  return q1*q2*r/(abs(r)**3)

def field1(z, Q=6, R=50, k=6):
  # carga no centro
  F1 = get_force( -k*Q, 1, 0, z )
  #F1 += get_force( -1, 1, 0.5j, z )
  # carga 2
  F1 += get_force( Q, 1, rect( R, np.pi/3 ), z )
  F1 += get_force( Q, 1, rect( R, np.pi/4 ), z )
  F1 += get_force( Q, 1, rect( R, 3*np.pi/4 ), z )
  F1 += get_force( Q, 1, rect( R, 0 ), z )
  F1 += get_force( Q, 1, rect( R, np.pi ), z )
  v = F1
  v = v/abs(v)
  return v


# a força que q1 faz em q2
def go2point( p, theta, ps, p2, kl, kth ):
  r1 = ps - p
  erro_l = abs( r1 )
  r2 = p2 - ps
  a1 = constrain_angle( polar(r1)[1] )
  a2 = constrain_angle( polar(r2)[1] )
  erro_g = a1
  if( erro_l < abs(r2) ):
    erro_l = 0
    erro_g = a2
  elif( erro_l < 2*abs(r2) ):
    kl/=2.0
    erro_g = (a1*erro_l+2*a2*abs(r2))/3.0

  erro_g = constrain_angle( erro_g - theta )

  linear_speed  = round( kl*erro_l, 3)
  angular_speed = round( kth*erro_g, 3)
  return ( linear_speed, angular_speed, erro_l, erro_g )

class dif_driver_control:

  def __init__(self, kpl, kdl, kpg, kdg, Max = 1000, kpgl = 0 ) -> None:
    self.pd_l = pd(kpl,kdl,Max)
    self.pd_g = pd(kpg,kdg,Max)
    self.Max = Max
    self.kpgl = kpgl
    self.n = 0
    self.erro_dist  = 0
    self.erro_theta = 0

  def update( self, erro_l, erro_g, k=0 ):
    self.erro_dist  = erro_l
    self.erro_theta = erro_g
    #kpg = self.pd_g.kp
    #self.pd_g.kp += self.kpgl*abs(erro_l)
    if( k==0 ):
      k = 1.0 + ( self.kpgl*abs(erro_l)/max(self.pd_g.kp,0.1) )
    dv, _ = self.pd_g.update( constrain_angle(erro_g) )
    v,  _ = self.pd_l.update( erro_l )
    vl = int( constrain( v+dv*k, -self.Max, self.Max ) )
    vr = int( constrain( v-dv*k, -self.Max, self.Max ) )
    #self.pd_g.kp = kpg
    return (vl,vr)
  
  def track( self, set_point, real_point, real_theta, trig_dist = 20 ):
    
    d = set_point - real_point
    
    dist, dist_theta = polar( d )
    vl, vr = self.update( dist, dist_theta-real_theta )
    
    return ( vl, vr, ( dist < trig_dist ) )
  
  def track2( self, set_point, set_theta, real_point, real_theta, trig_dist = 20, a = 400 ):

    data = {}

    d = (set_point - real_point)

    erro_l = abs( (d/rect(1,real_theta)).real )
    
    dist, dist_theta = polar( d )

    dist_theta = constrain_angle( dist_theta )

    #k1 = 1
    #if( dist < 4*trig_dist ):
    k1 = s_function( abs(dist), a, 0, 1 ) #dist/(4*trig_dist)
    
    set_theta = (1-k1)*set_theta + dist_theta*k1 #s_function( dist, 200, set_theta, dist_theta )

    erro_th = constrain_angle( set_theta - real_theta )

    trig = ( dist < trig_dist )

    if( trig ):
      erro_l = 0

    vl, vr = self.update( erro_l, erro_th )

    data['set_theta'] = set_theta
    data['k1'] = k1
    data['erro_th'] = set_theta-real_theta
    
    return ( vl, vr, trig, data )
  
  def track3( self, set_point, set_theta, real_point, real_theta, trig_dist = 20, a = 400 ):
    d = set_point - real_point
    dist, dist_theta = polar( d )
    vl, vr = self.update( dist, dist_theta-real_theta )
    return ( vl, vr, ( dist < trig_dist ) )
  
  
  def field( self, set_point, set_theta, real_point, real_theta, trig_dist = 20 ):

    data = {}

    d = (set_point - real_point)
    D, _ = polar( d )
    set_theta = polar( field1( -d ) )[1]
    erro_th = set_theta - real_theta

    erro_l = D
    
    trig = ( D < trig_dist )
    if( trig ):
      erro_l = 0
    
    vl, vr = self.update( erro_l, erro_th )
    data['set_theta'] = set_theta
    data['erro_th'] = set_theta-real_theta

    return ( vl, vr, trig, data )
  
  def track_path( self, path, real_point, real_theta, trig_dist = 20 ):

    if( self.n >= len(path) ):
      self.n = 0
      
    
    vl, vr, ok = self.track( path[self.n], real_point, real_theta, trig_dist )

    self.n += int(ok)

    return (vl, vr)




class dif_driver_control_2:

  def __init__(self, kpl, kdl, kpg, kdg, Max = 1000, kpgl = 0 ) -> None:
    self.pd_l = pd(kpl,kdl,Max)
    self.pd_g = pd(kpg,kdg,Max)
    self.Max = Max
    self.kpgl = kpgl
    self.n = 0
    self.erro_dist  = 0
    self.erro_theta = 0
  
  def update( self, erro_l, erro_g, k=1 ):
    dv = self.pd_g.update( erro_g )
    v  = self.pd_l.update( erro_l )
    vl = int( constrain( v+dv*k, -self.Max, self.Max ) )
    vr = int( constrain( v-dv*k, -self.Max, self.Max ) )
    return (vl,vr)

  # controlador de distancia e angulo
  def update_distance_angle( self, erro_l, erro_g ):
    k = 1.0 + ( self.kpgl*abs(erro_l)/max(self.pd_g.kp,0.1) )
    return self.update( erro_l, erro_g, k )
  
  # controlador de distancia e angulo
  def update_speed_angle( self, erro_l, erro_g ):
    return self.update( erro_l, erro_g, 1 )



def simular(x_set, y_set):
  import matplotlib.pyplot as plt

  tempo = 2.0
  N = 1000
  dt = tempo / N

  x = [0.0]
  y = [0.0]
  theta = [0.0]

  speed = 200
  l = 10

  bot = pid(500.0, 0, 0)

  for i in range(N):
    vl, vr, _, _ = bot.update(speed, x[i]+1j*y[i], theta[i], x_set+1j*y_set)
    w = (vr - vl) / l
    theta += [theta[i] + w * dt]
    x += [x[i] + speed * cos(theta[i + 1]) * dt]
    y += [y[i] + speed * sin(theta[i + 1]) * dt]

    if (polar(complex(x_set - x[i + 1], y_set - y[i + 1]))[0] < 1):
      break

  plt.title( f'simulation - setpoint: ({x_set},{y_set})' )
  plt.plot( x, y, '--' )
  plt.plot( 0, 0, 'or' )
  plt.plot( x_set, y_set, 'xr' )
  plt.show()



if __name__ == '__main__':
   simular( 10, 50 )
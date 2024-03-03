
import os
import time
from cmath import polar, phase
#from vision import vision
from vision_thread import vision, complex_to_xy
import cv2
import numpy as np
from math import floor
import main_gui as gui
from controle import new_pid as pid



##########################################################

PX2CM = 0.1 # conversão
img = np.zeros((10,10,3),np.uint8) # imagem
# pasta
FILE = __file__
path = FILE[:FILE.rfind('\\')+1]
os.chdir(path)


#### PID ##################################################

bot_control_lin = pid()
bot_control_ang = pid()

LAST_MODE = ''
vs_flag_new_data = False
vs = {}
VS_COLORS = VS_IN = VS_OUT = 0

def constrain_angle(x, Min=-np.pi, Max=np.pi ):
  return x-(Max-Min)*floor( (x-Min)/(Max-Min) )


#### DISPLAY #############################################

def plot_text( tela, p, text, color = (30,255,255) ):
    p = p/(10*PX2CM)
    cv2.putText(
        tela,
        text,
        (int(p.real),int(p.imag)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0,0,0),
        4,
        cv2.LINE_AA
    )
    cv2.putText(
        tela,
        text,
        (int(p.real),int(p.imag)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        color,
        2,
        cv2.LINE_AA
    )


def plot_x( tela, p, color = (110,255,255), l = 10 ):
    p = p/(10*PX2CM)
    cv2.line(
        tela,
        (int(p.real-l),int(p.imag)),
        (int(p.real+l),int(p.imag)),
        color,
        2
    )
    cv2.line(
        tela,
        (int(p.real),int(p.imag-l)),
        (int(p.real),int(p.imag+l)),
        color,
        2
    )

def plot_arrow( img, center, v, hue = 0 ):
    center = center/(10*PX2CM)
    #v = v/(10*PX2CM)
    v = 30*v/abs(v)
    cv2.arrowedLine(
        img,
        complex_to_xy(center),
        complex_to_xy(center+v),
        (hue,255,255), 3, tipLength = 0.2 )


#### GET INFO #############################################
    
def get_ball():
    return ( vs['ball']['pos'], vs['ball']['ok'] )

def get_bot_ref():
    
    ID = 24
    TEAM = 'team_yellow'
    MARK = 'team_blue'
    ok = False

    p = theta = 0

    #if( ID in vs[TEAM] ):
    if( len(vs[TEAM]) > 0 ):
        ID = list( vs[TEAM].keys() )[0]
        #if( vs['ball']['ok'] ):
        #    y_set = int(vs['ball']['pos'].imag)
        if( len(vs[MARK]) > 0 ):
            ID_M = list( vs[MARK].keys() )[0]
        p = vs[TEAM][ID]['pos']
        theta = vs[TEAM][ID]['orientation']
        ok = True
    
    return (p,theta,ok)

def get_bot_blue():
    
    ID = 24
    TEAM = 'team_blue'

    ok = False
    p = theta = 0

    if( len(vs[TEAM]) > 0 ):
        ID = list( vs[TEAM].keys() )[0]
        p = vs[TEAM][ID]['pos']
        theta = vs[TEAM][ID]['orientation']
        ok = True
    
    return (p,theta,ok)
        


# STEP RESPONSE ===========================================================
rec_data = []

def step_callback():
    global rec_data
    if(gui.rec_step.can_write(True)):

        global vs, vs_flag_new_data
        vs_data = vs
        
        if(vs_flag_new_data):
            vs_flag_new_data = False
            if( gui.rec_step.n == 0 ):
                gui.rec_step.save_lable( [ 'x', 'y' ] )
                v = int(10*gui.rec_step.voltage.get())
                #gui.serial.tx.send_ch_333( [ v,v, v,v, v,v ] )

            if( len(vs_data['team_yellow']) > 0 ):
                ID = list( vs_data['team_yellow'].keys() )[0]
                x = int(vs_data['team_yellow'][ID]['pos'].real)
                y = int(vs_data['team_yellow'][ID]['pos'].imag)
                rec_data = [x,y]
            
            gui.rec_step.save_line(rec_data)
            if( not gui.rec_step.Flag_save ):
                print('-- END --')
                #gui.serial.tx.send_ch_333( [] )

gui.rec_step.call = step_callback
# =========================================================================


#### BENCH #############################################
start = end = 0
def bench(frame):
    global end, start
    end = time.time()
    seconds = end - start
    fps  = int(1/max(seconds,1e-5))
    print(f"Estimated frames per second : {fps} - {int(1000*seconds)}ms")
    cv2.putText( frame, f'FPS: {int(fps)}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2, cv2.LINE_AA )
    cv2.imshow( 'camera', frame )
    start = end




#### PID #############################################
state = 0
kick_step = 0

timeout1 = 0
N = True

from cmath import rect
import datalogger as saver


def loop():

    # RESIZE CAMERA IMAGE ================================================================
    global img, state, kick_step
    global N, timeout1


    if(gui.camera.cap.available() > 0):
        
        img = gui.camera.read()
        h, w, _ = img.shape
        MIN = int( (h-1)*gui.painel.sliders['Y0'].get()/100.0)
        MAX = int((h-1)*gui.painel.sliders['Y fim'].get()/100.0 )
        if( MAX <= MIN ):
            MIN = MAX - 1
        k = (MAX - MIN)/h
        img = img[ MIN:MAX, int((0.5-k/2.0)*w) : int((0.5+k/2.0)*w) ]

        global PX2CM
        PX2CM = gui.painel.sliders['H'].get()/img.shape[0]
        campo_mm_x = int(10.0*PX2CM*img.shape[1])
        campo_mm_y = int(10.0*PX2CM*img.shape[0])
        gui.tag0.set(f"campo: ({campo_mm_x},{campo_mm_y})mm / {10*PX2CM:0.2f}mm/px")

        # VISION ==============================================================================
        global VS_COLORS, VS_IN, VS_OUT
        global vs, vs_flag_new_data
        vs_flag_new_data = True
        tela = img.copy()
        #vs, monitors = vision(tela, VS_COLORS, VS_IN, VS_OUT, 10*PX2CM )
        vs, monitors = vision(tela, VS_COLORS, VS_IN, 10*PX2CM )
        gui.update_tags( vs )
        gui.monitor_colors.update_hsv( monitors['colors'] )
        #OUT[ 'monitor_mask' ].update_hsv(tela)
        # ====================================================================================

        # SAVE ===============================================================================
        gui.save.loop(img)
        # ====================================================================================
        
        bench(img)

        # MONITORES ==========================================================================
        #gui.monitor_camera.update_BGR( img )

        #gui.camera.update( data['images']['tela'] )
        #gui.monitor_colors.update_hsv( data['images']['mask'] ) # mudar para update apenas
        #gui.monitor_colors_2.update( data['images']['colors'] )
        # ====================================================================================
    
    # ====================================================================================

        ## [1] ##
        ## Verifica o modo atual e se houve alteração ==============
        global LAST_MODE
        MODE = gui.mode.read()
        MODE_CHANGE = ( MODE != LAST_MODE )
        if( MODE_CHANGE ):
            if( LAST_MODE == 'VECT' ):
                saver.end()
            state = 0
            gui.serial.tx.stop()
            gui.serial.tx.println( "pid_w.I 0.0" )
            gui.serial.tx.println( "pid_w.I_MAX 1.0" )
        LAST_MODE = MODE
        ## =========================================================

        ## [2] ##
        ## atuaiza os parametro de controle ============================
        #gui.painel_pid.sliders['Pg'].set()
        
        #bot_control.kp = gui.painel_pid.sliders['Pl'].get()
        #bot_control.kd = gui.painel_pid.sliders['Dl'].get()
        #bot_control.kp = gui.painel_pid.sliders['Pg'].get()
        #bot_control.kd = gui.painel_pid.sliders['Dg'].get()
        #bot_control.kpgl = 0.01*gui.painel_pid.sliders['Kpgl'].get()
        bot_linear_speed  = 10*gui.painel_pid.sliders['vel'].get()
        bot_angular_speed = round( (20/100)*gui.painel_pid.sliders['w'].get(), 3 )
        ## ==============================================================


        ## [3] ##
        ## Input vision data ================================================
        p, theta, bot_ok = get_bot_ref()
        ball, ball_ok = get_ball()
        centro = int(campo_mm_x/2) + 1j*int(campo_mm_y/2)

        x = y = vl = vr = dist = erro = d_theta = 0
        erro_l = erro_g = 0

        erro_l = abs( centro - p )
        vector_d = centro - p
        if( erro_l < 50 ):
            erro_g = constrain_angle(0 - theta)
        else:
            theta_set = polar(vector_d)[1]
            erro_g = constrain_angle(theta_set - theta)
        
        if( not ball_ok ):
            ball = centro
        else:
            plot_x( monitors['vision'], ball ) # marca um x na posição da bola
        plot_x( monitors['vision'], centro ) # marca um x na posição do centro

        ## Input vision data ================================================
        

        ## [4] ##
        x = int(p.real) # ??
        

        ## Controle do robô ===================================================
        if(bot_ok):
            
            plot_x( monitors['vision'], p, color=(110,255,255) )
            kl  = 0.1*gui.painel_pid.sliders['kl'].get()
            kth = 5*0.01*gui.painel_pid.sliders['kth'].get()
            Q_ball = -0.2*gui.painel_pid.sliders['Q_ball'].get()
            Q_obs  = 0.2*gui.painel_pid.sliders['Q_obs'].get()
            Dmin   = gui.painel_pid.sliders['Dmin'].get()
            #bot_control_lin.ki = 0.1*gui.painel_pid.sliders['kd'].get()
            bot_control_lin.I_max = 100
            bot_control_lin.kp = kl

            # MODO vai para o centro
            if( MODE == 'CENTRO' ):
                #from controle import go2point
                r1 = centro - p
                erro_l = abs(r1)
                a1 = constrain_angle( polar(r1)[1] )
                if( erro_l < (40+state*40) ):
                    state = 1
                    erro_g = -theta
                    erro_l = 0
                else:
                    state = 0
                    erro_g = a1-theta
                
                bot_linear_speed  = round( kl*erro_l, 3 )    
                bot_angular_speed = round( kth*constrain_angle(erro_g), 3)
            
            elif( MODE == 'kick' ):

                if( state == 0 ): # vai até antes da bola
                    vector_d = (ball - 180) - p
                    erro_l = abs( vector_d )
                    if(erro_l < 40):
                        state = 1
                    else:
                        plot_x( monitors['vision'], (ball - 180), color=(0,255,255) )
                        theta_set = polar(vector_d)[1]
                        erro_g = constrain_angle(theta_set - theta)
                        bot_linear_speed  = round( kl*erro_l, 3)
                        bot_angular_speed = round( kth*erro_g, 3)
                
                if( state == 1 ): # ajusta o angulo
                    vector_d = ball - p
                    erro_l = abs( vector_d )
                    theta_set = polar(vector_d)[1]
                    erro_g = constrain_angle(theta_set - theta)
                    
                    if( abs(erro_g) < 0.1):
                        state = 2
                    else:
                        bot_linear_speed  = 0
                        bot_angular_speed = round( kth*erro_g, 3)

                if( state == 2 ): # vai até a bola
                    vector_d = ball - p
                    erro_l = abs( vector_d )
                    if(erro_l < 90):
                        state = 3
                        timeout1 = time.time() + 0.4
                    else:
                        plot_x( monitors['vision'], ball, color=(0,255,255) )
                        theta_set = polar(vector_d)[1]
                        erro_g = constrain_angle(theta_set - theta)
                        bot_linear_speed  = round( kl*erro_l, 3)
                        bot_angular_speed = round( kth*erro_g, 3)
                
                if( state == 3 ): # chuta durante um pequeno tempo
                    if(timeout1 <= time.time()):
                        state = 4
                    else:
                        plot_x( monitors['vision'], ball, color=(80,255,255) )
                        bot_linear_speed  = 1000
                        bot_angular_speed = 0
                
                if( state == 4 ): # fica parado
                    bot_linear_speed  = 0
                    bot_angular_speed = 0.0
                    gui.mode.update( "STOP" )
            
            elif( MODE == 'v_kick' ):

                if( state == 0 ): # vai até antes da bola
                    vector_d = p - centro
                    erro_l = abs( vector_d )

                    from controle import field1, get_force
                    if(erro_l < 40):
                        erro_g = -theta
                        plot_x( monitors['vision'], centro, color=(80,255,255) )
                        bot_linear_speed  = 0
                        bot_angular_speed = round( kth*erro_g, 3)
                    else:
                        z = p - centro
                        # carga ao redor do centro
                        R = 50
                        F1 = get_force( -6*Q_ball, 1, 0, z )
                        F1 += get_force( Q_ball, 1, rect( R, np.pi/3 ), z )
                        F1 += get_force( Q_ball, 1, rect( R, np.pi/4 ), z )
                        F1 += get_force( Q_ball, 1, rect( R, 3*np.pi/4 ), z )
                        F1 += get_force( Q_ball, 1, rect( R, 0 ), z )
                        F1 += get_force( Q_ball, 1, rect( R, np.pi ), z )
                        
                        # bola representando um obstaculo
                        if( ball_ok ):
                            plot_x( monitors['vision'], centro, color=(120,255,255) )
                            F1 += get_force( Q_ball, 1, ball-centro, z )

                        v = F1
                        v = v/abs(v)
                        
                        erro_g = constrain_angle( constrain_angle( polar(v)[1] ) - theta )
                        plot_x( monitors['vision'], centro, color=(0,255,255) )
                        bot_linear_speed  = round( kl*erro_l, 3)
                        bot_angular_speed = round( kth*erro_g, 3)
                
                if( state == 1 ): # ajusta o angulo
                    bot_linear_speed  = 0
                    bot_angular_speed = 0.0
                    gui.mode.update( "STOP" )
            
            elif( MODE == 'REPELE' ):

                if( state == 0 ): # vai até antes da bola
                    vector_d = p - ball
                    erro_l = abs( vector_d )

                    from controle import field1, get_force
                    if(erro_l > 200):
                        erro_g = polar(vector_d)[1] - theta
                        plot_x( monitors['vision'], centro, color=(80,255,255) )
                        bot_linear_speed  = 0
                        bot_angular_speed = round( kth*erro_g, 3)
                    else:

                        # bola representando um obstaculo
                        if( ball_ok ):
                            F1 = 0
                            plot_x( monitors['vision'], centro, color=(120,255,255) )
                            F1 += get_force( Q_ball, 1, 0, p - ball )
                            v = F1
                            v = v/abs(v)
                            erro_g = constrain_angle( constrain_angle( polar(v)[1] ) - theta )
                            plot_x( monitors['vision'], centro, color=(0,255,255) )
                            bot_linear_speed  = round( kl*erro_l, 3)
                            bot_angular_speed = round( kth*erro_g, 3)
                        else:
                            bot_linear_speed  = 0
                            bot_angular_speed = 0.0
                
                if( state == 1 ): # ajusta o angulo
                    bot_linear_speed  = 0
                    bot_angular_speed = 0.0
                    gui.mode.update( "STOP" )
            
            elif( MODE == 'VECT' ):

                if( state == 0 ):
                    saver.init( "vector_path.txt", header="t pos_x pos_y th bola_x bola_y obstaculo_x obstaculo_y", dtn=0.1 )
                    state = 1

                # Q_ball Q_obs Dmin
                TEAM = 'team_blue'
                pb = 0
                
                from controle import get_force
                plot_x( monitors['vision'], ball, color=(120,255,255) )
                F = get_force( -Q_ball, 1, 0, ball-p )
                plot_arrow( monitors['vision'], ball, F, 30 )
                plot_arrow( monitors['vision'], p, F, 30 )
                erro_g = constrain_angle(polar(F)[1]-theta)
                plot_text( monitors['vision'], p+40+40j, f"{ int((erro_g)*180.0/np.pi) }", (0,255,255) )
                if( len(vs[TEAM]) > 0 ):
                    for ID in vs[TEAM]:
                        pb = vs[TEAM][ID]['pos']
                        print( f'vs[TEAM][{ID}][pos] = {pb}' )
                        if( abs( p - pb ) < 500 ):
                            f = get_force( -Q_obs, 1, 0, pb-p )
                            F += f
                            plot_arrow( monitors['vision'], pb, f, 30 )
                            plot_arrow( monitors['vision'], p, f, 30 )
                print( f"ball{ball} -> F {F}" )
                erro_l = abs(ball-p)
                if( erro_l < 80 ):
                    bot_linear_speed  = 0
                    bot_angular_speed = 0.0
                    gui.mode.update( "STOP" )
                if(F==0): F=1
                erro_g = constrain_angle(polar(F)[1]-theta)
                #erro_g = constrain_angle( constrain_angle( polar(F)[1] ) - theta )
                #bot_linear_speed  = int( constrain( 300+kl*erro_l ) )
                from controle import constrain
                bot_linear_speed  = int( constrain( 300+kl*F, 0, 1023) )
                bot_angular_speed = round( kth*erro_g, 3)
                #bot_angular_speed = round( erro_g, 3)
                if( abs(erro_g) > np.pi/4.0 ):
                    if( bot_angular_speed > 0 ):
                        bot_angular_speed += 0.5
                    else:
                        bot_angular_speed -= 0.5
                    bot_linear_speed = 0
                

                saver.loop( f"{round(p.real,3)} {round(p.imag,3)} { round(np.rad2deg(theta),3) } {round(ball.real,3)} {round(ball.imag,3)} {round(pb.real,3)} {round(pb.imag,3)}" )
            
            #elif( MODE == 'PID 2' ): # ajuste angulo apenas
            #    # kick
            #elif( MODE == 'PID 3' or MODE == 'Ball' ): # seguir bola ou zero
            #    # pid 3 e Ball


        #gui.tag_pid.set( f'({vl},{vr}) dist: {int(bot_control.P)}mm  d_theta: {int(constrain_angle(bot_control.erro_theta)*180/np.pi)}°' )
        #gui.tag_pid.set( f'({vl},{vr}) dist: {int(bot_control.P)}mm' )
        gui.tag_pid.set( f" el:{round(erro_l,3)} eg:{round(erro_g,3)} pid.auto 1 pid.lin {int(bot_linear_speed)} pid.w {bot_angular_speed}" )


        # verifica se o robô esta no intervalo permitido
        abertura = 0.70
        x_min = campo_mm_x*0.5*(1-abertura)
        x_max = campo_mm_x*0.5*(1+abertura)
        bot_in_range = x < x_max and x > x_min

        # envia para o robô
        #if( MODE == 'VECT' and bot_in_range  ):
        #    color = (60,255,255)
        #    if( abs(bot_angular_speed) < 0.1 ):
        #        bot_angular_speed = 0
        #    gui.serial.tx.println( f" pid_w.I_MAX 1 pid_w.kd 1.0 pid_w.ki 1500 pid.auto 1 pid.auto_speed {bot_linear_speed} pid.auto_angle {0} pid_w.I {bot_angular_speed}" )
        #el
        if( MODE != 'STOP' and bot_in_range  ):
            color = (60,255,255)
            gui.serial.tx.println( f" pid_w.I_MAX 0.05 pid_w.kd 0.7 pid_w.ki 1500 pid.auto 1 pid.auto_speed {bot_linear_speed} pid.auto_angle {bot_angular_speed}" )
        else:
            color = (0,255,255)
            #gui.serial.tx.println( f" pid_w.I_MAX 0.05 pid_w.kd 0.7 pid_w.ki 1500 pid.auto 1 pid.auto_speed {1000} pid.auto_angle {3}" )
            gui.serial.tx.stop()

        # indicações na tela
        if( MODE != 'STOP'  ):
            cv2.line( monitors['vision'], (0,int(centro.imag/(10*PX2CM))), (img.shape[1],int(centro.imag/(10*PX2CM))), (150,255,255), 2)
            cv2.rectangle( monitors['vision'], (int(x_min/(10*PX2CM)),0), (int(x_max/(10*PX2CM)), img.shape[0]), color, 2)
            cv2.circle( monitors['vision'], (int(centro.real/(10*PX2CM)),int(centro.imag/(10*PX2CM))) , int( 5/PX2CM ), (110,255,255), 2 )
        gui.monitor_camera.update_BGR( img )
        gui.monitor_mask.update_hsv( monitors['vision'] )
    
    gui.win.after(1,loop)


def gui_loop():

    # LOOP ===============================================================================
    gui.win.after(300,gui_loop)
    # ====================================================================================

    global VS_COLORS, VS_IN, VS_OUT
    # update inputs of IHM ===============================================================
    VS_COLORS, VS_IN, VS_OUT = gui.loop()
    # ====================================================================================

gui_loop()
loop()

gui.win.mainloop()

gui.camera.close()

# ========================================================================================


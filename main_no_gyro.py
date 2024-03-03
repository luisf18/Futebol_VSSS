import os
import time
from cmath import polar, phase
#from vision import vision
from vision_thread import vision
import numpy as np
import main_gui as gui
from controle import pd, constrain, dif_driver_control
import cv2
from math import floor

# código anterior sem o uso do controlador de velocidade angular dos robôs

PX2CM = 0.1
img = np.zeros((10,10,3),np.uint8)

FILE = __file__
path = FILE[:FILE.rfind('\\')+1]
os.chdir(path)

bot_control = dif_driver_control( 5,0.168, 200,0.068, kpgl = 800/500 )

#gui.painel_pid.sliders['Pg'].set( 0.168 )
#gui.painel_pid.sliders['Pg'].set( 0.168 )
#gui.painel_pid.sliders['Pg'].set( 0.168 )

LAST_MODE = ''
vs_flag_new_data = False
vs = {}
VS_COLORS = VS_IN = VS_OUT = 0

def constrain_angle(x, Min=-np.pi, Max=np.pi ):
  return x-(Max-Min)*floor( (x-Min)/(Max-Min) )

def plot_text( tela, p, text, color = (110,255,255) ):
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
                gui.serial.tx.send_ch_333( [ v,v, v,v, v,v ] )

            if( len(vs_data['team_yellow']) > 0 ):
                ID = list( vs_data['team_yellow'].keys() )[0]
                x = int(vs_data['team_yellow'][ID]['pos'].real)
                y = int(vs_data['team_yellow'][ID]['pos'].imag)
                rec_data = [x,y]
            
            gui.rec_step.save_line(rec_data)
            if( not gui.rec_step.Flag_save ):
                print('-- END --')
                gui.serial.tx.send_ch_333( [] )

gui.rec_step.call = step_callback
# =========================================================================

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


# controle
state = 0
kick_step = 0
th_last = 0
t_now = time.time()
t_last = 0
bot_last_p = 0
bot_last_theta = 0
bot_last_w_sig = 0
bot_integral = 0

timeout1 = 0
N = True

from cmath import rect

def move_line( center, th, p, th_real, d_trig = 200 ):
    u = rect( 1, th )
    d = ( p - center )/u
    
    set_th = th+(np.pi/2)*constrain(-d.imag/d_trig,-1,1)
    return bot_control.update( 100, set_th - th_real, k=1 )



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
        ## MODE process ==========================================
        global LAST_MODE
        MODE = gui.mode.read()
        MODE_CHANGE = ( MODE != LAST_MODE )
        if( MODE_CHANGE ):
            gui.serial.tx.send_ch_333( [] )
            if( MODE == 'kick' ):
                kick_step = 0
                timeout1 = 0
        LAST_MODE = MODE
        ## MODE process ==========================================

        ## [2] ##
        ## update values ================================================
        # 5 = 5
        # 168 => 0.001*168 = 0.168
        # 29 => 10*29 = 290
        # 68 => 0.001*68 = 0.068
        # 1 =>  de 0 até 1000
        # 0.01 =>  de 0 até 10
        # 0.001 =>  de 0 até 1
        #gui.painel_pid.sliders['Pg'].set()
        bot_control.pd_l.kp = 0.01*gui.painel_pid.sliders['Pl'].get()
        bot_control.pd_l.kd = 0.001*gui.painel_pid.sliders['Dl'].get()
        bot_control.pd_g.kp = gui.painel_pid.sliders['Pg'].get()
        bot_control.pd_g.kd = 0.001*gui.painel_pid.sliders['Dg'].get()
        bot_control.kpgl    = 0.01*gui.painel_pid.sliders['Kpgl'].get()
        bot_control.Max     = 10*gui.painel_pid.sliders['vel'].get()
        ## update values ================================================


        ## [3] ##
        ## Input vision data ================================================
        p, theta, ok = get_bot_ref()

        # speed
        global t_last, bot_last_p, bot_last_theta, bot_last_w_sig
        t_now = time.time()
        dt = max( t_now - t_last, 0.00001 )
        speed_real = abs(p-bot_last_p)/dt
        w_real = (theta - bot_last_theta)/dt
        t_last = t_now
        bot_last_p = p
        bot_last_theta = theta

        # ajusta erro no sinal
        bot_w_sig = ( w_real > 0 )
        if( bot_last_w_sig != bot_w_sig ):
            w_real *= -1
        bot_last_w_sig = bot_w_sig

        x = y = vl = vr = dist = erro = d_theta = 0
        erro_l = erro_g = 0
        centro = int(campo_mm_x/2) + 1j*int(campo_mm_y/2)
        ball, ball_ok = get_ball()
        plot_x( monitors['vision'], centro )
        
        # bola
        if( not ball_ok ):
            ball = centro
        else:
            plot_x( monitors['vision'], ball )
        ## Input vision data ================================================


        ## [4] ##
        x = int(p.real) # ??
        

        ## Controle do robô ===================================================
        if(ok):
            
            plot_x( monitors['vision'], p, color=(110,255,255) )

            # MODO PID
            if( MODE == 'PID' ):
                
                #R = 300
                #alfa = 2*np.pi/8
                #rot = np.cos( alfa ) + 1j*np.sin( alfa )
                #path_line =  centro + R * np.array( [ 1, rot, rot**2, rot**3, rot**4, rot**5, rot**6, rot**7 ] )
                
                # ultimo usado ------
                #path_line =  centro + 300j*np.array( [-1,0,1] )
                #vl, vr = bot_control.track_path( path_line, p, theta, 50 )


                from cmath import rect
                R = gui.painel_pid.sliders['raio'].get()
                d, th = polar(-1j*(p-centro))
                erro_d = d - R
                set_th = th+(np.pi/2)*constrain(-erro_d/gui.painel_pid.sliders['dist'].get(),-1,1)
                vl, vr = bot_control.update( 100, set_th - theta, k=1 )
            
            # MODO PID 2
            elif( MODE == 'PID 2' ): # ajuste angulo apenas

                th = np.pi*(gui.painel_pid.sliders['theta'].get()-500.0)/500.0

                vl, vr = bot_control.update( 0, th-theta )
            
            # MODO Kick
            elif( MODE == 'kick' ):
                
                kick_ok = False
                if( kick_step == 0 ):
                    vl, vr, kick_ok = bot_control.track( centro-300j, p, theta, 30 )
                elif( kick_step == 1 ):
                    th = np.pi/2
                    vl, vr = bot_control.update( 0, th-theta, k=1 )
                    kick_ok = ( abs(th-theta) < 0.1 )
                elif( kick_step == 2 ):
                    th = np.pi/2 #np.pi*(gui.painel_pid.sliders['theta'].get()-500.0)/500.0
                    vl, vr = move_line( centro, th, p, theta, d_trig = gui.painel_pid.sliders['dist'].get() )
                    kick_ok = abs( centro + 200j - p ) < 80


                if( kick_ok ):
                    kick_step+=1
                    #time.sleep(0.5)
                

            elif( MODE == 'PID 3' or MODE == 'Ball' ): # seguir bola ou zero

                if( MODE == 'Ball' ):
                    sp = ball
                else:
                    sp = centro
                    #if( MODE == 'kick' ):
                    #    if( kick_step == 0 ):
                    #        sp = centro - 150j
                    #    else:
                    #        sp = centro
                
                dist = abs( sp - p )

                m = 30
                M = 60
                # schimit triger
                if( state == 0 ): # longe
                    if( dist < m ): state = 1
                else:
                    if( dist > M ): state = 0
                
                if( state == 1 ):
                    kick_step = 1
                    vl, vr = bot_control.update( 0, np.pi/2-theta )
                else:
                    vl, vr, _ = bot_control.track( sp, p, theta, 20 )


        gui.tag_pid.set( f'({vl},{vr}) dist: {int(bot_control.erro_dist)}mm  d_theta: {int(constrain_angle(bot_control.erro_theta)*180/np.pi)}°' )

        abertura = 0.70
        x_min = campo_mm_x*0.5*(1-abertura)
        x_max = campo_mm_x*0.5*(1+abertura)
        if( x < x_max and x > x_min ):
            color = (60,255,255)
            gui.serial.tx.send_ch_333( [ vl, vr, vl, vr, vl, vr ] )
        else:
            color = (0,255,255)
            gui.serial.tx.send_ch_333( [] )

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


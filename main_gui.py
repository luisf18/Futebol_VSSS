from tkinter import*
from tkinter import messagebox
from tkinter import ttk
from ttkthemes import ThemedTk
from tkinter.font import Font
from tkinter import filedialog as fd

import os
import time

import cv2
import numpy as np
from PIL import Image,ImageTk

import widgets as wg

FILE = __file__
path = FILE[:FILE.rfind('\\')+1]
os.chdir(path)

# INICIA A JANELA =========================================================
window_gui = wg.window("Futebol VSSS - visão trevinho",ico=path+'gui/icon.ico',height=720)
win = window_gui.win
# =========================================================================


# BEGIN ==============================================================

# camera
camera = wg.Camera( 10, 450, 'camera', win  )
save = wg.save_image( 10, 480, win )

# serial
serial = wg.serial_consol( 10, 510, win )

# monitores
monitor_camera = wg.monitor(10,10,500,400,"Camera",win)
monitor_mask   = wg.monitor(530,10,250,200,"Elementos",win)
tagHZ = wg.tag(530,10,win)
monitor_colors = wg.monitor(530,250,250,200,"Mascaras de Cores",win)

# tags
tag0 = wg.tag(10,10,win)

# ajustes gerais
painel = wg.panel( 'ajustes do sistema de visão', win, x=810, y=10 )
painel.add_slider( 0, 255, 'S min', 1, default=86 )
painel.add_slider( 0, 255, 'V min', 2, default=86 )
painel.add_slider( 0, 100, 'Y0', 3, default=21 )
painel.add_slider( 0, 100, 'Y fim', 4, default=94 )
painel.add_slider( 10, 200, 'H', 5, default=71, unit = 'cm' )
painel.add_slider( 1, 100, 'area min', 6, default=20 )
painel.add_slider( 0, 100, 'delta', 7, default=15 )

# ajuste PID
painel_pid = wg.panel( 'Controlador', win, x=810, y=480 )
painel_pid.add_slider( 0, 100,  'vel', 1, default=0, unit='%' )
painel_pid.add_slider( 0, 100,  'w',   2, default=0, unit='%' )
painel_pid.add_slider( 0, 100,  'kl', 3, default=18 ) # erro maximo lin
painel_pid.add_slider( 0, 100,  'kth', 4, default=70 ) # erro maximo lin
painel_pid.add_slider( 0, 100,  'Q_ball', 5, default=30  ) #68
painel_pid.add_slider( 0, 100,  'Q_obs', 6, default=9 )
painel_pid.add_slider( 1, 100,  'Dmin', 7, default=50, unit='%' )
painel_pid.add_slider( 1, 1000, 'theta', 8, default=50, unit='%' )
painel_pid.add_slider( 1, 1000, 'raio', 9, default=50, unit='%' )

# tag 1
tag_pid = wg.tag(810,480,win)

# ajustes de cor
colors_default = {
    'orange': 3, #7,
    'yellow': 14, #18
    'green': 38,
    'blue': 72,
    'darkblue': 99,
    'pink': 134,
    'red': 180 # 169
}

painel_cores = wg.panel( 'cores', win, 810, 245 )
for i,color in enumerate( colors_default ):
    painel_cores.add_slider( 0, 180, f"{color}", 1+i, default=colors_default[color]  )

# ball
ball = wg.ball_tag( win, 10, 625, 65 )

# tags dos bots
bot = { 
    'darkblue': [ wg.bot_tag( win, 235 + i*195, 550, 60, 'darkblue' ) for i in range(3) ],
    'yellow': [ wg.bot_tag( win, 235 + i*195, 630, 60, 'yellow' ) for i in range(3) ]
}

mode = wg.mode_selection(win,10,545,20,
                         'playing options',
                         ['STOP','CENTRO','REPELE','VECT','v_kick','kick'], # 'CONFIG'],
                         ['stop.png','caminho.png','raio.png','happy.png','ball.png','kick2.png']) # 'settings.png'])

def rec_step_call_default():
    if(rec_step.can_write(True)):
        rec_step.save_line([1,2,3,'A'])

rec_step = wg.record(10,710,win,rec_step_call_default)
#rec_step = wg.record(810,720,win,rec_step_call_default)

# LOOP ==================================================================
def loop():

    # atualiza sliders
    painel.update()
    painel_cores.update()
    painel_pid.update()
    rec_step.voltage.update()
    
    # UPDATE COLORS =======================================================================
    color_hue_min = [ painel_cores.sliders[c].get() for c in painel_cores.sliders ]
    color_hue_max = []
    color_hue_mean = []
    red_index = len(painel_cores.sliders)-1
    for i,color in enumerate( painel_cores.sliders ):
       if( i == red_index ):
          color_hue_max += [color_hue_min[0]]
          color_hue_mean += [((180+color_hue_min[i]+color_hue_max[i])//2)%180]
       else:
          color_hue_max += [color_hue_min[i+1]]
          color_hue_mean += [(color_hue_min[i]+color_hue_max[i])//2]
       painel_cores.sliders[color].color( wg.hue2ttkColor(color_hue_mean[-1]) )
    VS_COLORS = {}
    VS_COLORS['MEAN'] = { color: color_hue_mean[i] for i,color in enumerate(painel_cores.sliders) }
    VS_COLORS['MIN'] = { color: color_hue_min[i] for i,color in enumerate(painel_cores.sliders) }
    VS_COLORS['MAX'] = { color: color_hue_max[i] for i,color in enumerate(painel_cores.sliders) }
    # UPDATE COLORS =======================================================================
    
    for i,color in enumerate( painel_cores.sliders ):
        painel_cores.sliders[color].color( wg.hue2ttkColor( VS_COLORS['MEAN'][color] ) )
    
    VS_IN = {  ajuste: painel.sliders[ajuste].get()  for ajuste in painel.sliders }

    #print( "VS IN:", VS_IN )

    VS_OUT = { }
    VS_OUT['monitor_camera'] = monitor_camera
    VS_OUT['monitor_color'] = monitor_colors
    VS_OUT['monitor_mask'] = monitor_mask
    VS_OUT[ 'BOTS' ] = bot
    VS_OUT['tag0'] = tag0
    VS_OUT['tagHZ'] = tagHZ
    
    return (VS_COLORS,VS_IN,VS_OUT)

def update_tags( data ):
    # ATUALIZA AS TAGS ------------------------------------------------------------------------
    ball.update_pack( data['ball'] )
    for team in [ 'darkblue', 'yellow' ]:
        if( team == 'yellow' ):
            team_key = 'team_yellow'
        else:
            team_key = 'team_blue'
        keys = list(data[team_key].keys())
        bot_detection = {
            'id': 0,
            'pos': 0,
            'orientation': 0,
            'dimension': 0,
            'vector': 0,
            'colors': ['orange','orange']
        }
        for i in range(3):
            if( len(data[team_key]) > i ):
                bot[team][i].update_pack( data[team_key][keys[i]] )
            else:
                bot[team][i].update_pack( bot_detection )
    # -----------------------------------------------------------------------------------------
    

# FOR TESTS ==============================================================
if __name__ == '__main__':
    print( "FILE -> ", FILE, "\nPATH:", path )
    monitor_camera.update_BGR( camera.read() )
    loop( )
    win.mainloop()

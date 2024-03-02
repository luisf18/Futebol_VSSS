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

window_gui = wg.window("Amostragem",ico=path+'gui/icon.ico',height=150,width=500)
win = window_gui.win

def tk_icon( w, h, path ):
    img = Image.open( path )
    img = img.convert("RGBA").resize((w,h), Image.Resampling.LANCZOS )
    #icon = PhotoImage( file = path )
    icon = ImageTk.PhotoImage( img )
    return icon


class record:
    
    def __init__( self, x, y, root, call, path = path, dt = 50 ):
        self.dt = dt
        w = 20
        self.icon_play = tk_icon( w,w,'C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS/images/icons/play-big.png')
        self.icon_stop = tk_icon( w,w,'C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS/images/icons/stop-big.png') 

        self.path = path
        self.Flag_save = False
        self.Flag_reading = False

        self.call = call
        
        self.frame = ttk.Frame( root )
        self.frame1 = ttk.Frame( self.frame )
        self.frame2 = ttk.Frame( self.frame )
        self.btn = ttk.Button( self.frame1, width=0, padding=0, imag=self.icon_play, command=self.change )
        self.name = ttk.Entry( self.frame1, width=10 )
        self.time = ttk.Entry( self.frame1, width=7 )
        self.time.insert(0,'500')

        MenuBtn = ttk.Menubutton(self.frame1, text = "select elements") #, relief = RAISED)
        
        self.Flag_bot0_y = IntVar()
        self.Flag_bot1_y = IntVar()
        self.Flag_bot2_y = IntVar()
        self.Flag_bot0_b = IntVar()
        self.Flag_bot1_b = IntVar()
        self.Flag_bot2_b = IntVar()
        self.Flag_ball = IntVar()
    
        Menu1 = Menu(MenuBtn, tearoff = 0)
        
        Menu1.add_checkbutton(label = "team_yellow_bot0", variable = self.Flag_bot0_y)
        Menu1.add_checkbutton(label = "team_yellow_bot1", variable = self.Flag_bot1_y)
        Menu1.add_checkbutton(label = "team_yellow_bot2", variable = self.Flag_bot2_y)
        Menu1.add_checkbutton(label = "team_blue_bot0",   variable = self.Flag_bot0_b)
        Menu1.add_checkbutton(label = "team_blue_bot1",   variable = self.Flag_bot1_b)
        Menu1.add_checkbutton(label = "team_blue_bot2",   variable = self.Flag_bot2_b)
        Menu1.add_checkbutton(label = "ball",             variable = self.Flag_ball)

        MenuBtn["menu"] = Menu1

        
        self.voltage = wg.slider( 0, 100, 'voltage', self.frame2, length=50, default=50, border=2 )

        # progressbar
        s = ttk.Style(root)
        s.theme_use("black")
        s.configure("green.Horizontal.TProgressbar", foreground='#00FF00', background = '#00FF00' )
        self.bar = ttk.Progressbar( self.frame2, orient='horizontal', mode='determinate', length=150, style="green.Horizontal.TProgressbar" )
        self.bar.grid( column=1, row=0 )

        self.frame.place( x=x, y=y )
        self.frame1.grid( column=0, row=0 )
        self.btn.grid(    column=0, row=0, padx=2 )
        ttk.Label( self.frame1, text='name:', font=Font(size=9), anchor='center', justify='center'  ).grid( column=1, row=0, padx=3 )
        self.name.grid(   column=2, row=0, padx=2 )
        ttk.Label( self.frame1, text='time:', font=Font(size=9), anchor='center', justify='center'  ).grid( column=3, row=0, padx=3 )
        self.time.grid(   column=4, row=0, padx=2 )
        MenuBtn.grid(     column=5, row=0, padx=2 )
        self.frame2.grid( column=0, row=1, padx=2 )

        self.n = 0
        self.time_ini = 1000*time.time()

    def change(self):
        print( self.time.get() )
        self.Flag_save = not self.Flag_save
        if( self.Flag_save ):
            self.time_ini = time.time()
            self.btn.config( image = self.icon_stop )
            print('open')
            self.file_name = f'{self.name.get()}.txt'
            self.f = open( self.file_name, "a")
            self.f.write("n,time,data\n")
            self.total_time = max(int( self.time.get() ),5)
            print( f'time: {self.total_time}ms' )
            self.ID = self.frame.after( self.dt , self.call )
        else:
            if hasattr(self, 'f'):
                self.frame.after_cancel(self.ID)
                print('close')
                self.f.close()
                f = open( self.file_name , "r")
                print(f.read())
            self.btn.config( image = self.icon_play )
    
    def can_write(self,call=False):
        if self.f.closed:
            print( "[ x _ x ]" )
            return False
        else:
            if(call):
                self.call_loop()
            return True
    
    def save_list(self,list):
        if( self.can_write() ):
            data = ','.join( list )
            dt = int(1000*(time.time()-self.time_ini))
            self.f.write(f'{self.n},{dt},{data}\n')
            self.bar['value'] = int(100*(dt/self.total_time))
            if( dt > self.total_time ):
                self.change()


    def call_loop(self):
        print('.')
        self.n += 1
        self.ID = self.frame.after( self.dt, self.call )
    
    def call_default(self):
        if( self.can_write() ):

            list = []
            if( self.Flag_ball > 0 ):
                pass
            self.save_list(list)
            self.call_loop()

if __name__ == '__main__':
    #def loop():    
    #loop( )

    c = ['A','B','C','D']

    def b():
        if(A.can_write(True)):
            A.save_list(c)

    A = record(10,10, win,b,'')

    def a():
        #A.loop()
        win.after(500,a)



    #win.mainloop()
    
    a()
    win.mainloop()

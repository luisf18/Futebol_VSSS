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
from PIL import Image,ImageTk,ImageOps

FILE = __file__
path = FILE[:FILE.rfind('\\')+1]

def constrain( x, MIN, MAX ):
    if( x <= MIN ): return MIN
    if( x >= MAX ): return MAX
    return x
def hue2ttkColor(hue):
  color = cv2.cvtColor( np.array( [[[hue,255,255]]], np.uint8), cv2.COLOR_HSV2RGB)
  return f"#{color[0][0][0]:02X}{color[0][0][1]:02X}{color[0][0][2]:02X}"

def tk_icon( w, h, path ):
    img = Image.open( path )
    img = img.convert("RGBA").resize((w,h), Image.Resampling.LANCZOS )
    #icon = PhotoImage( file = path )
    icon = ImageTk.PhotoImage( img )
    return icon

# SLIDERS ============================================================================================================
class window:
    def __init__(self, title, width=1200, height=690, ico = None, theme = 'black', background='#222222' ) -> None:
        # INICIA A JANELA =========================================================
        self.win = ThemedTk()
        self.win.geometry(f"{width}x{height}")
        self.win.title( title )
        self.win.configure(background=background)
        self.win.minsize( width=width, height=height )

        if(ico is not None):
            self.win.iconbitmap( ico )
        # =========================================================================
        # style ==================================================================
        style = ttk.Style(self.win)
        style.theme_use("black")
        #style.theme_use("equilux")
        #style.theme_use("scidpurple")

        def changer(theme):
            global style
            print("tema:",theme)
            style.theme_use(theme)

        themes_options = ttk.Style().theme_names()
        themes_options2 = self.win.get_themes()
        my_menu = Menu(self.win)
        self.win.config(menu = my_menu)
        theme_menu = Menu(my_menu)
        my_menu.add_cascade(label="themes",menu=theme_menu)

        for t in themes_options2:
            theme_menu.add_command( label=t, command=lambda t=t: changer(t) )
        # ========================================================================     



# SLIDERS ============================================================================================================
class slider:
    def __init__(self, MIN, MAX, name, main_frame, row = 0, length = 200, default = None, unit = '', border = 3, lable_width = 13  ) -> None:
        self.unit = unit
        self.MIN = MIN
        self.MAX = MAX
        self.name = name
        self.length = length
        self.frame = ttk.Frame( main_frame, border=border )
        self.str_var = StringVar()
        if( default is None ):
            self.str_var.set(MIN)
        else:
            self.str_var.set(default)
        self.scale = ttk.Scale(  self.frame, from_=MIN, to=MAX, orient=HORIZONTAL, length=self.length, variable=self.str_var, command=lambda s:self.str_var.set(f'{int(float(s))}') )
        self.btn_L = ttk.Button( self.frame, text= "<", width=0, command=lambda : self.str_var.set( constrain( int(float(self.str_var.get()))-1, MIN, MAX) ) )
        self.btn_R = ttk.Button( self.frame, text= ">", width=0, command=lambda : self.str_var.set( constrain( int(float(self.str_var.get()))+1, MIN, MAX) ) )
        self.label = ttk.Label(  self.frame, text=f" {name}: {self.scale.get()}", foreground="white", background="#333333", width = lable_width )
        self.frame.grid( column=0, row=row, padx=0 )
        self.btn_L.grid( column=0, row=0, padx=0 )
        self.btn_R.grid( column=1, row=0, padx=0 )
        self.scale.grid( column=2, row=0, padx=6 )
        self.label.grid( column=3, row=0, padx=3 )

    def get(self):
        return int(self.scale.get())
    
    def update(self):
        self.label.configure(text=f" {self.name}: {int(self.scale.get())}{self.unit}")
    
    def color(self, txt_color):
        self.label.configure( foreground=txt_color)
    
    def set(self, x):
        self.str_var.set( constrain( x, self.MIN, self.MAX) )


# PAINEL ============================================================================================================
class panel:
    def __init__(self, name, main_frame, x = 0, y = 0 ) -> None:
        self.name = name
        self.frame = ttk.Frame( main_frame, border=4 )
        self.str_var = StringVar()
        self.frame.place( x=x, y=y )
        ttk.Label( self.frame, text=name, foreground="white", justify=LEFT ).grid( row=0 )
        self.sliders = {}
        self.buttons = {}
    def add_slider(self, MIN, MAX, name, row = 0, length = 200, default = None, unit = ''):
        self.sliders[name] = slider( MIN, MAX, name, self.frame, row = row, length = length, default = default, unit = unit)

    def update(self):
        for s in self.sliders:
            self.sliders[s].update()

# TAG ===========================================================================================================
class tag:
    def __init__(self, x, y, root ) -> None:
        self.x = x
        self.y = y
        self.f1 = Font(  size=12, weight="normal", slant="roman", underline=0, overstrike=0 )
        self.frame = ttk.Frame( root, padding=5 )
        self.frame.place(x=x,y=y)
        self.lable = ttk.Label( self.frame, text='' )
        self.lable.grid( column=0, row=0 )
    def set(self, txt ):
        self.lable.configure( text=txt )

# BUTTON ===========================================================================================================
class button:
    def __init__(self, x, y, name, root ) -> None:
        self.x = x
        self.y = y
        self.f1 = Font(  size=12, weight="normal", slant="roman", underline=0, overstrike=0 )
        self.button = ttk.Button( text=name )
        self.button.place(x=x,y=y)
    def set_function(self, f):
        self.button.configure( command=f )
    
# MONITOR ===========================================================================================================
class monitor:
    def __init__(self, x, y, w, h, title, root ) -> None:
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.title = title
        self.f1 = Font(  size=12, weight="normal", slant="roman", underline=0, overstrike=0 )
        self.frame_tela = ttk.Frame( root, padding=5 )
        self.frame_tela.place(x=x,y=y)
        self.tela_title = ttk.Label( self.frame_tela, text=title, foreground="white", justify=CENTER, font=self.f1 )
        self.tela_lable = ttk.Label( self.frame_tela, text=title )
        self.tela_title.grid( column=0, row=0 )
        self.tela_lable.grid( column=0, row=1 )
    
    def update(self, img ):
        img = cv2.resize(img,(self.w,self.h))
        image = Image.fromarray(img)
        pic = ImageTk.PhotoImage(image)
        self.tela_lable.configure(image=pic)
        self.tela_lable.image = pic

    def update_hsv(self, hsv ):
        self.update( cv2.cvtColor(hsv,cv2.COLOR_HSV2RGB) )
    
    def update_BGR(self, bgr ):
        self.update( cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB) )

    def update_mask(self, img ):
        blank = np.zeros((*img.shape,3),np.uint8)
        blank[:,:] = [255,255,255]
        img = cv2.bitwise_and(blank,blank,mask=img)
        self.update(img)



# IMAGE SOURCE ================================================================================
from threading import Thread
class video_thread:
    def __init__(self, src=0, api = cv2.CAP_ANY): #cv2.CAP_DSHOW
         self.Flag_running = False
         self.cap = cv2.VideoCapture(src, api)
         self.cap.set(cv2.CAP_PROP_FPS, 120)
         if int((cv2.__version__).split('.')[0])  < 3 :
             fps = self.cap.get(cv2.cv.CV_CAP_PROP_FPS)
             print("Frames per second using video.get(cv2.cv.CV_CAP_PROP_FPS): {0}".format(fps))
         else :
             fps = self.cap.get(cv2.CAP_PROP_FPS)
             print("Frames per second using video.get(cv2.CAP_PROP_FPS) : {0}".format(fps))
         (self.grabbed, self.frame) = self.cap.read()
         self.Flag_on = True
         self.n = 0
         if( self.grabbed is False ):
             self.Flag_on = False

    def start(self):
        self.Flag_on = True
        Thread(target=self.update, args=()).start()
    
    def update(self):
        self.Flag_running = True
        self.Flag_on = True
        while True:
             if not self.Flag_on:
                break
             # otherwise, read the next frame from the stream
             (self.ret, frame) = self.cap.read()
             if( self.ret is not False ):
                 self.frame = frame
                 self.n+=1
        print( "[ video thread ] end..." )
        self.Flag_running = False
    def read(self):
        self.n = 0
        return self.frame
    
    def stop(self):
        self.Flag_on = False
    
    def available(self):
        return self.n


class Camera:
    
    def __init__(self, x, y, title, root ) -> None:
        
        self.x = x
        self.y = y
        self.title = title
        self.changing = True
        self.Flag_flip = False
        
        self.frame = ttk.Frame( root, border=2)
        self.adds_ip = StringVar()
        self.camera_source = ttk.OptionMenu( self.frame, self.adds_ip, '0', *[0,1,2,'IP','image'], command=self.start )
        self.camera_ip = ttk.Entry( self.frame, width=30, font=Font(size=10) )
        self.camera_ip.insert(0,'http://192.168.1.11:8080/video')

        self.frame.place(x=x,y=y)
        ttk.Label( self.frame, text=self.title, font=Font(size=12)).grid( column=0, row=0,padx=3 )
        self.camera_source.grid( column=1, row=0,padx=3 )
        ttk.Label( self.frame, text="IP", font=Font(size=12) ).grid( column=2, row=0,padx=3 )
        self.camera_ip.grid( column=3, row=0, padx=3 )


        # control video / flip and stop
        self.Flag_play = True
        def flip():
            self.Flag_flip = not self.Flag_flip
            if( self.mode == 'image' ):
                self.img = cv2.flip(self.img,1)
        ttk.Button( self.frame, text='Flip', width=0, command=flip ).grid( column=4, row=0, padx=3 )

        self.playing = False
        def play_pause():
            print( "[ CAMERA - play_pause() ]" )
            self.playing = not self.playing
            if( self.playing ):
                self.play()
            else:
                self.stop()
        self.btn_play = ttk.Button( self.frame, text='Stop', width=0, command=play_pause )
        self.btn_play.grid( column=5, row=0 )

        # image or cap
        self.img = np.zeros((10,10,3),np.uint8)
        self.start(0)
    
    def play(self):
        print( "[ CAMERA - play() ]" )
        self.playing = True
    
    def stop(self):
        print( "[ CAMERA - stop() ]" )
        self.playing = False
    
    def close(self):
        print( "[ CAMERA - close ]" )
        if hasattr(self, 'cap'):
            print( "[ CAMERA - close ] ..." )
            self.cap.stop()
            while( self.cap.Flag_running ):
                pass
    
    def start(self, mode = 0 ):

        print( f"[ CAMERA - start ] { mode }" )

        # src: image or cap

        self.changing = True
        
        self.ok = False
        self.mode = mode
        self.src = 0
        self.playing = True

        self.close()

        if( self.mode == 'image' ):
            filetypes = (
                ('image files', '*.png'),
                ('image files', '*.jpg'),
                ('image files', '*.bmp'),
                ('All files', '*.*') )
            self.src = fd.askopenfilename(
                title='Open a file',
                initialdir='/',
                filetypes=filetypes)
            if(  len(self.src) > 0 ):
                self.img = cv2.imread( self.src )
                self.ok = True
        else:
            if( self.mode == 'IP' ):
                self.src = self.camera_ip.get()
                api = cv2.CAP_FFMPEG
            else:
                self.src = self.mode
                api = cv2.CAP_DSHOW
            
            self.cap = video_thread( self.src, api )
            if( self.cap.Flag_on ):
                self.cap.start()
                self.ok = True
        
        print(f'[ Video mode: {self.mode} ] src: {self.src} ----', end='\t')

        if( not self.ok ):
            print('Fail!')
            if( self.mode != 0 ):
                self.start(0)
        else:
            print('ok!')
        self.changing = False

    def read( self ):
        if( self.mode != 'image' ):
            if( not self.changing and self.playing ):
                self.img = self.cap.read()
                #print( "size: ", self.img.size )
                if( self.Flag_flip ):
                    self.img = cv2.flip(self.img,1)
        return self.img
#========================================================================================================



# SERIAL ================================================================================================

class serial_consol:

    def __init__( self, x, y, root ):
        
        import Serial.transmissor as tx
        self.tx = tx
        # Frames
        self.frame = ttk.Frame( root, border=2)#, relief='solid' )
        self.frame_com = ttk.Frame(  self.frame, border=2)#, relief='solid' )
        self.frame_send = ttk.Frame( self.frame, border=2)#, relief='solid' )

        # elements
        self.port_list = ['x']
        self.porta = StringVar()
        self.PORTAS = ttk.OptionMenu( self.frame_com, self.porta, self.port_list[0], *self.port_list,command=self.connect )
        self.BTN_REFRESH = ttk.Button( self.frame_com, text="REFRESH", command=self.refresh) #, fg = '#00FF00', bg = '#333333' )
        self.BTN_SEND = ttk.Button( self.frame_send, text="SEND", width=0, command=self.send)#, fg = '#00FF00', bg = '#333333' )
        self.MSG = ttk.Entry( self.frame_send, width=45 )
        self.MSG.insert(0,'send 333')

        self.refresh()

        self.frame.place( x=x, y=y )
        self.frame_com.grid( column=1,row=0 )
        self.frame_send.grid( column=0,row=0 )

        self.PORTAS.grid(row=0,column=0)
        self.BTN_REFRESH.grid(row=0,column=1)
        self.MSG.grid(row=0,column=2,padx=3)
        self.BTN_SEND.grid(row=0,column=3)

    # functions
    def send( self ):
        print( 'serial -> send' )
        self.tx.write(self.MSG.get())

    def connect( self, p ):
        self.tx.begin(p)
        self.refresh()

    def refresh( self ):
        self.port_list = self.tx.list_ports()
        if( len(self.port_list) == 0 ):
            self.port_list = ['x']
        print( f"[WIN-SERIAL] port list: {self.port_list}" )
        self.PORTAS.set_menu( None, *self.port_list )

#========================================================================================================


class save_image:
    def __init__( self, x, y, root, path = path ):
        #x=150,y=490
        self.path = path
        self.Flag_save_img = False
        def save_img():
            self.Flag_save_img = True
        self.frame = ttk.Frame( root )
        ttk.Button( self.frame, text='save images', command=save_img ).grid( column=0, row=0 )
        self.name = ttk.Entry( self.frame )
        self.frame.place( x=x, y=y )
        self.name.grid( column=1, row=0 )
    
    def save( self, img, mask = None ):
        os.chdir(self.path+'/images')
        print( "[saving...]", f'{self.name.get()}_mask.png' )
        self.Flag_save_img = False
        cv2.imwrite( f'{self.name.get()}_cap.png' , img )
        if( mask is not None ):
            cv2.imwrite( f'{self.name.get()}_mask.png' , mask )
        os.chdir(self.path)
        print( 'save:', self.path )
    
    def loop(self, img, mask = None):
        if( self.Flag_save_img ):
            self.save( img, mask )


class record:
    
    def __init__( self, x, y, root, call, path = path, dt = 50 ):
        self.dt = dt
        w = 20
        self.icon_play = tk_icon( w,w,'C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS_F18/images/icons/play-big.png')
        self.icon_stop = tk_icon( w,w,'C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS_F18/images/icons/stop-big.png') 

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

        
        self.voltage = slider( 0, 100, 'step', self.frame2, length=50, default=50, border=2, unit='%', lable_width=11 )

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
            self.n = 0
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
    
    def save_line(self,list):
        if( self.can_write() ):
            data = ','.join( map(str,list) )
            dt = int(1000*(time.time()-self.time_ini))
            self.f.write(f'{self.n},{dt},{data}\n')
            self.bar['value'] = int(100*(dt/self.total_time))
            self.n+=1
            if( dt > self.total_time ):
                self.change()
    
    def save_lable(self,list):
        if( self.can_write() ):
            data = ','.join( map(str,list) )
            self.f.write(f'N,time,{data}\n')

    def call_loop(self):
        print('.')
        self.ID = self.frame.after( self.dt, self.call )
    
    def call_default(self):
        if( self.can_write() ):

            list = []
            if( self.Flag_ball > 0 ):
                pass
            self.save_line(list)
            self.call_loop()


# BOT TAG ===============================================================================================
generic_color_hue = {
    'orange': 12,
    'yellow': 28,
    'green': 55,
    'blue': 85,
    'darkblue': 116,
    'pink': 151,
    'red': 178
}

class bot_tag:
    def __init__(self, root, x, y, w, time, colors_hue = generic_color_hue) -> None:
        self.x = x
        self.y = y
        self.w = w
        self.time = time

        self.colors_hue = colors_hue
        
        # frame
        self.frame = ttk.Frame( root, border=3, padding=2, relief='solid')
        self.frame.place( x=x,y=y )

        # img
        self.tag = ttk.Label( self.frame, text=time )
        self.tag.grid(column=0,row=0)

        # text
        self.text  = ttk.Frame( self.frame, padding=5 )
        self.text.grid(column=1,row=0)
        self.f1 = Font(  size=9, weight="normal", slant="roman", underline=0, overstrike=0 )
        self.label_pos = ttk.Label( self.text, justify='left', font=self.f1 )
        self.label_theta = ttk.Label( self.text, justify='left', font=self.f1 )
        self.label_LW = ttk.Label( self.text, justify='left', font=self.f1 )
        self.label_pos.grid(column=0,row=0)
        self.label_theta.grid(column=0,row=1)
        self.label_LW.grid(column=0,row=2)

        self.update(0, 0, 0, 0, 0, [time,time] )
    
    def update(self, x, y, theta, w, h, tag_colors, id = None, colors_hue = None ):
        if( colors_hue is not None ):
            self.colors_hue = colors_hue
        team_hue = self.colors_hue[self.time]
        img = np.zeros( (self.w,self.w,3), np.uint8 )
        img[0:self.w//2,0:] = [ team_hue, 255, 255 ]
        img[self.w//2:self.w,0:self.w//2] = [ self.colors_hue[tag_colors[0]], 255, 255 ]
        img[self.w//2:self.w,self.w//2:self.w] = [ self.colors_hue[tag_colors[1]], 255, 255 ]
        text_color = (60,255,255)
        if( self.time == 'yellow' ):
            text_color = (0,0,0)
        cv2.putText( img, f'ID:{id}', (3,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1, cv2.LINE_AA )
        img = cv2.cvtColor(img,cv2.COLOR_HSV2RGB)
        image = Image.fromarray( img )
        pic = ImageTk.PhotoImage(image)
        self.tag.configure(image=pic)
        self.tag.image = pic
        self.label_pos.configure( text = f"(x,y): ({int(x)},{int(y)})" )
        self.label_theta.configure( text = f"Theta: {int(theta)}°" )
        self.label_LW.configure(text = f"(W,H): ({int(w)},{int(h)})" )
    
    def update_pack(self, pack ):
        self.update(
            pack['pos'].real,
            pack['pos'].imag,
            pack['orientation']*(180/np.pi),
            pack['dimension'].real,
            pack['dimension'].imag,
            pack['colors'],
            pack['id']
        )


class ball_tag:
    def __init__(self, root, x, y, w, colors_hue = generic_color_hue, image_path = 'C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS_F18/images/vsss_ball.png') -> None:
        self.x = x
        self.y = y
        self.w = w
        self.colors_hue = colors_hue
        self.image_path = image_path
        
        # frame
        self.frame = ttk.Frame( root, border=3, padding=2, relief='solid')
        self.frame.place( x=x,y=y )

        # img
        self.tag = ttk.Label( self.frame, text=time )
        self.tag.grid(column=0,row=0)

        # text
        self.text  = ttk.Frame( self.frame, padding=5 )
        self.text.grid(column=1,row=0)
        self.f1 = Font(  size=9, weight="normal", slant="roman", underline=0, overstrike=0 )
        self.label_pos = ttk.Label( self.text, justify='left', font=self.f1 )
        self.label_R = ttk.Label( self.text, justify='left', font=self.f1 )
        self.label_pos.grid(column=0,row=0)
        self.label_R.grid(column=0,row=1)

        img = (Image.open( self.image_path ))
        img = img.resize((self.w,self.w), Image.ANTIALIAS)
        pic = ImageTk.PhotoImage( img )
        self.tag.configure(image=pic)
        self.tag.image = pic

        self.update( 0, 0, 0 )
    
    def update(self, x, y, R ):
        self.label_pos.configure( text = f"(x,y): ({int(x)},{int(y)})" )
        self.label_R.configure(text = f"(R): ({int(R)})" )
    
    def update_pack(self, pack ):
        self.update(
            pack['pos'].real,
            pack['pos'].imag,
            pack['dimension'].real
        )

class mode_selection:

    def __init__(self, root, x, y, w, title, options, images, text_size = 8, images_path = 'C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS_F18/images/icons/') -> None:
        
        self.x = x
        self.y = y
        self.w = w
        self.images_path = images_path
        self.images = images
        self.options = options
        
        # frame
        self.frame = ttk.Frame( root, border=1, padding=3, relief='solid')
        self.frame.place( x=x,y=y )
        
        self.lable_select_txt = ttk.Label( self.frame, padding=0, text=title, justify='center', anchor='center', font=Font(weight='bold',size=text_size) )
        self.lable_select_txt.grid( column=0, columnspan=len(options)+1, row=2 )

        self.opt_l_name = {}
        self.opt_l_img  = {}

        self.select = ''

        for i,name in enumerate(options):
            self.opt_l_name[name] = ttk.Label( self.frame, text=name, border=1, padding=1, font=Font(weight='bold',size=text_size), justify='center', anchor='center' )
            self.opt_l_img[name]  = ttk.Button( self.frame, command=lambda s=name: self.update(s), padding=3 )
            self.opt_l_name[name].grid( column=i+1, row=0 )
            self.opt_l_img[name].grid( column=i+1, row=1 )
        self.update( options[0] )
    
    def update( self, opt ):
        self.select = opt
        for i,name in enumerate(self.options):
            img = (Image.open( self.images_path + self.images[i] ))
            if( name != opt ): img = ImageOps.grayscale(img)
            img = img.convert("RGBA").resize((self.w,self.w), Image.ANTIALIAS)
            if( name != opt ):
                self.opt_l_name[name].config( foreground='white' )
            else:
                self.opt_l_name[name].config( foreground='#00FF00' )
            pic = ImageTk.PhotoImage( img )
            self.opt_l_img[name].configure(image=pic)
            self.opt_l_img[name].image = pic
    
    def read( self ):
        return self.select

# FOR TESTS ==============================================================
if __name__ == '__main__':
    
    print( "FILE -> ", FILE, "\nPATH:", path )

    G = window('widgets...',1200,500)

    painel = panel( "ajustes", G.win, 10, 10 )
    painel.add_slider( 0, 255, "S min", 1, default=86 )
    painel.add_slider( 10, 200, "H", 2, default=71, unit = 'cm' )

    painel_cores = panel( "cores", G.win, 10, 110 )
    painel_cores.add_slider( 0, 255, 'red', 1, default=55 )
    painel_cores.add_slider( 0, 255, 'green', 2, default=55 )
    painel_cores.add_slider( 0, 255, 'blue', 3, default=55 )

    img = np.zeros( (300,300,3), np.uint8 )
    tela = monitor( 390, 10, 200, 200, "teste", G.win )

    colors_hue = {
        'red': 0,
        'yellow': 30,
        'green': 50,
        'blue': 120
    }

    bot_b = [ bot_tag( G.win, 10 + i*220, 260, 65, 'darkblue' ) for i in range(3) ]
    bot_y = [ bot_tag( G.win, 10 + i*220, 350, 65, 'yellow'   ) for i in range(3) ]

    monitor_camera = monitor( 620, 10, 300, 200, "camera", G.win )
    camera = Camera(620, 260, 'CAM', G.win)
    serial = serial_consol( 620, 290, G.win )

    save = save_image( 620, 324, G.win )

    ball = ball_tag( G.win, 620, 350, 65 )

    tag0 = tag(620,10,G.win)


    mode = mode_selection(G.win,840,330,40,
                          'playing options',
                          ['STOP','PID','CONFIG'],
                          ['stop.png','caminho.png','settings.png'])

    def b():
        if(rec_step.can_write(True)):
            rec_step.save_line([1,2,3,'A'])

    rec_step = record(10,10,G.win,b,'')
    
    def display():

        monitor_camera.update_BGR( camera.read() )

        painel.update()
        
        c = [ 
            painel_cores.sliders['red'].get(),
            painel_cores.sliders['green'].get(),
            painel_cores.sliders['blue'].get()
        ]
        img[ 100:200, : ] = c
        painel_cores.sliders['red'  ].color( f"#{c[0]:02X}0000" )
        painel_cores.sliders['green'].color( f"#00{c[1]:02X}00" )
        painel_cores.sliders['blue' ].color( f"#0000{c[2]:02X}" )
        
        tela.update( img )

        colors_hue['red'  ] = constrain(painel_cores.sliders['red'  ].get(),0,180)
        colors_hue['green'] = constrain(painel_cores.sliders['green'].get(),0,180)
        colors_hue['blue' ] = constrain(painel_cores.sliders['blue' ].get(),0,180)

        #for b in bot_b:
        #    b.update(
        #        0,0,0,
        #        10,10,
        #        colors_hue=colors_hue
        #    )
        #for y in bot_y:
        #    y.update(
        #        0,0,0,
        #        10,10,
        #        colors_hue=colors_hue
        #    )
        
        tag0.set( f'tag: {painel.sliders["H"].get()}' )

        camera.read()

        G.win.after(100,display)
    
    display()
    G.win.mainloop()
    camera.close()

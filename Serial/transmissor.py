import sys
import glob
import serial
from time import sleep, time

host = None

def begin(porta):
    global host
    if( porta != 'x' ):
       if( host is not None ):
          if( host.port == porta ):
             print( '[SERIAL] port ->', host.port )
             return True
       host = serial.Serial( porta, 115200 , timeout = 0.1, writeTimeout = 0.1 )
       print( '[SERIAL] port ->', host.port )
       return True
    else:
       print( '[SERIAL] port -> None [not connected]' )
       host = None
       return False

def begin_host(porta):
    global host
    if( porta != 'x' ):
       if( host is not None ):
          if( host.port == porta ):
             print( '[SERIAL] port ->', host.port )
             return True
       host = serial.Serial( porta, 115200 , timeout = 0.1, writeTimeout = 0.1 )
       print( '[SERIAL] port ->', host.port )
       return (host, True)
    else:
       print( '[SERIAL] port -> None [not connected]' )
       host = None
       return (host, False)

def list_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def send_ch_333(data: list[int]) -> str:
  send( [333] + data )

def send(data: list[int]) -> str:
  msg = "Send "
  for d in data:
    msg += f"{d} "
  write(msg + " \n")

def write( msg ):
  global host
  if( host is not None ):
    try:
        print('[SERIAL] msg:',msg)
        #host.write(bytes(msg, "utf-8"))
        host.write(msg.encode())
        #host.flush()
        host.reset_output_buffer()
    except:
        host = None

def println( msg ):
  write( msg + "\n" )

def stop():
  println( "pid.auto 0 bot.stop" )

def close():
  if( host is not None ):
     send_ch_333([])
     print('[SERIAL] close')
     host.close()

if __name__ == '__main__':
    L = list_ports()
    print(L)
    if( len(L) > 0 ):
        begin(L[0])
        for i in range(10):
            write(f'ab azul {i}')
            sleep(0.02)
    
    while(True):
       if( host is not None ):
          if(host.isOpen() == False):
            print('.')
            host.open()
          else:
             if( host.in_waiting ):
                print( host.readline().decode('utf') )
             println('bot.bip') # command to robot make bip sound
             sleep(1)
       else:
          L = list_ports()
          if( len(L) > 0 ):
             begin(L[0])
          
    
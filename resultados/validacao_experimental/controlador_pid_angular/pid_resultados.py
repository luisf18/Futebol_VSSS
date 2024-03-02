#from Serial.transmissor import *
from time import sleep, time
import Serial.transmissor as s

if __name__ == '__main__':
    L = s.list_ports()
    print(L)
    if( len(L) > 0 ):
      s.begin(L[0])
      lin = 1000
      w = 0
      n = 100
      dt = 10
      s.println(f"pid_w.I 0")
      sleep(0.2)
      s.println(f"pid_w.I 0")
      sleep(0.2)
      s.println(f"pid_w.I_MAX 0.1")
      sleep(0.2)
      s.println(f"pid.test {lin} {w} {n} {dt}")
      sleep(0.001*n*dt*1.3)
   
    result = ""
    while( s.host.in_waiting ):
      line = s.host.readline().decode('utf')
      if( line[0].isdigit() or line[0] == 'n' ):
         result += line
      sleep(0.04)
    print( "--------------------------" )
    print( result )
    print( "--------------------------" )
    path = "C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS/resultados/"
    file = path + f"resultado_pid_w{int(w)}_lin{int(lin)}_new.txt"

    f = open(file, "w")
    f.write(result)
    f.close()
    #open and read the file after the overwriting:
    f = open(file, "r")
    print(f.read())
   
    s.close()

          
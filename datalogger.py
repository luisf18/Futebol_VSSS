from time import time as t

# Data logger
# Modulo registrador de dados
# permite salvar medições

FILE = __file__
Path = FILE[:FILE.rfind('\\')+1]+"/resultados/"
file = Path + "resultados.txt"

dt = 0.3
result = ""
t0 = t()
timeout = t0 + 0.3
N = 0

def init( f = "resultados.txt", path = Path, header = "", dtn = 0.3 ):
    global dt, timeout, t0, result, N, file
    file = path + f
    dt = dtn
    t0 =  t()
    timeout = t0 + dt
    result = f"N t {header}\n"
    N = 0

def loop( line ):
    global dt, timeout, t0, result, N
    T = t()
    if( T >= timeout ):
        result += f"{N} {T-t0} {line}\n"
        timeout = T + dt
        N+=1
    return N

def end():
    f = open(file, "w")
    f.write(result)
    f.close()
    #open and read the file after the overwriting:
    f = open(file, "r")
    print(f.read())
    return N

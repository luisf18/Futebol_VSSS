import numpy as np
import matplotlib.pyplot as plt


def graf( w, lin ):
    path = "C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS_F18/resultados/"
    file = path + f"resultado_pid_w{int(w)}_lin{int(lin)}_new.txt"
    # Leia o arquivo TXT
    with open(file, 'r') as file:
        data = file.read()

    # Processar os dados
    lines = data.strip().split('\n')
    columns = [line.split() for line in lines[1:]]
    columns = np.array(columns, dtype=float)

    print(lines[0])

    # Plotar os gráficos
    plt.figure(figsize=(18, 6))
    plt.rcParams.update({'font.size': 18})

    # Exemplo de gráfico de linha
    plt.subplot(2, 1, 1)
    plt.plot(columns[:, 1]-columns[0, 1], columns[:, 3], 'b--', label='velocidade angular desejada')
    plt.plot(columns[:, 1]-columns[0, 1], columns[:, 4], 'r', label='velocidade angular medida')
    #plt.xlabel('velocidade angular [rad/s]')
    plt.xlabel('tempo [ms]')
    plt.legend()
    #plt.grid()
    plt.grid( which='major', linestyle='-')
    plt.grid( which='minor', linestyle='--')
    plt.minorticks_on()

    # Exemplo de gráfico de dispersão
    plt.subplot(2, 1, 2)
    #plt.scatter(columns[:, 1]-columns[0, 1], columns[:, 4], label='Coluna 1 vs Coluna 3', color='red')
    plt.plot(columns[:, 1]-columns[0, 1], columns[:, 5], 'purple', label='resposta do controlador (sinal PWM)')
    plt.xlabel('tempo [ms]')
    #plt.ylabel('resposta do controlador')
    plt.legend()
    plt.grid( which='major', linestyle='-')
    plt.grid( which='minor', linestyle='--')
    plt.minorticks_on()


    plt.tight_layout()

    plt.savefig(path + f'pid_w_graficos/pid_w{int(columns[1,3])}_l{int(columns[1,2])}_16px.png')

    plt.show()

#graf( 4, 1000 )
#graf( -4, 1000 )
#graf( 5, 0 )
#graf( 10, 0 )
#graf( 20, 0 )
#graf( 10, 1000 )
#graf( 0, 1000 )
#graf( 5, 1000 )
#graf( 3, 0 )


def erro_med( w, lin ):
    path = "C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS_F18/resultados/"
    file = path + f"resultado_pid_w{int(w)}_lin{int(lin)}.txt"
    # Leia o arquivo TXT
    with open(file, 'r') as file:
        data = file.read()

    # Processar os dados
    lines = data.strip().split('\n')
    columns = [line.split() for line in lines[1:]]
    columns = np.array(columns, dtype=float)

    #print(lines[0])

    e = np.sum( np.abs(columns[:,4]-columns[:,3]) )/len( columns[:,3] )

    if( columns[0,3] == 0 ):
        print( f"erro medio L{int(lin)} {int(w)} rad/s: {round(e,2)} >> inf %" )
    else:
        print( f"erro medio L{int(lin)} {int(w)} rad/s: {round(e,2)} >> {round(100*e/columns[0,3],2)}%" )

    return e

def erro_med_tab( w, lin ):
    path = "C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS_F18/resultados/"
    file = path + f"resultado_pid_w{int(w)}_lin{int(lin)}.txt"
    # Leia o arquivo TXT
    with open(file, 'r') as file:
        data = file.read()

    # Processar os dados
    lines = data.strip().split('\n')
    columns = [line.split() for line in lines[1:]]
    columns = np.array(columns, dtype=float)

    #print(lines[0])

    e = np.sum( np.abs(columns[:,4]-columns[:,3]) )/len( columns[:,3] )

    if( columns[0,3] == 0 ):
        s = f"{int(lin)} & {int(w)} & {round(e,2)} & inf \\%\\\\"
    else:
        s = f"{int(lin)} & {int(w)} & {round(e,2)} & {round(100*e/columns[0,3],2)}\\% \\\\"
    print( s )
    return s


#erro_med_tab( 3, 0 )
#erro_med_tab( 5, 0 )
#erro_med_tab( 10, 0 )
#erro_med_tab( 20, 0 )
#erro_med_tab( 15, 500 )
#erro_med_tab( 3, 600 )
#erro_med_tab( 0, 1000 )
#erro_med_tab( -4, 1000 )
#erro_med_tab( 4, 1000 )
#erro_med_tab( 10, 1000 )
#erro_med_tab( 20, 1000 )

#for i in [0,4,-4,5,10]: graf( i, 1000 )

graf( 0, 1000 )
#for i in [3,5,10,20]: graf( i, 0 )
#graf( 3, 0 )


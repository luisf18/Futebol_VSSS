import numpy as np
import matplotlib.pyplot as plt


def graf( name ):
    # C:\Users\UERJBotz\Documents\LF18\(00) GITHUB\Futebol_VSSS\resultados\validacao_experimental\trajetoria_campo_vetorial
    path = "C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS/resultados/validacao_experimental/trajetoria_campo_vetorial/"
    file = path + name
    # Leia o arquivo TXT
    with open(file, 'r') as file:
        data = file.read()

    # Processar os dados
    lines = data.strip().split('\n')
    columns = [line.split() for line in lines[1:]]
    columns = np.array(columns, dtype=float)

    print(lines[0])

    # Plotar os gráficos
    plt.figure(figsize=(10, int(10*710/947)))

    # Exemplo de gráfico de linha
    #plt.subplot(2, 1, 1)
    plt.plot(columns[0, 5],710-columns[0, 6], 'rX',  color = 'purple', label='bola', markersize=15)
    #plt.plot(columns[0, 7],710-columns[0, 8], 'X', color = '#00FF00', label='obstaculo', markersize=15)
    plt.plot(columns[:, 2],710-columns[:, 3], 'bo-', label='trajetória')
    #plt.plot(columns[:, 0]-columns[0, 1], columns[:, 4], 'r', label='velocidade angular medida')
    #plt.xlabel('velocidade angular [rad/s]')
    
    # Definir limites dos eixos X e Y
    plt.xlim(0, 947)  # Limite X
    plt.ylim(0, 710)  # Limite Y

    plt.ylabel('y [mm]', fontsize=18)
    plt.xlabel('x [mm]', fontsize=18)
    plt.xticks(fontsize=18)  # Tamanho da fonte dos números no eixo X
    plt.yticks(fontsize=18)  # Tamanho da fonte dos números no eixo Y
    plt.legend(fontsize=18)
    plt.grid()

    #947 710

    ## Exemplo de gráfico de dispersão
    #plt.subplot(2, 1, 2)
    ##plt.scatter(columns[:, 1]-columns[0, 1], columns[:, 4], label='Coluna 1 vs Coluna 3', color='red')
    #plt.plot(columns[:, 1]-columns[0, 1], columns[:, 5], 'purple', label='resposta do controlador (sinal PWM)')
    #plt.xlabel('tempo [ms]')
    #plt.ylabel('resposta do controlador')
    #plt.legend()
    #plt.grid()

    #plt.tight_layout()

    plt.savefig(path + f'{name}.png', transparent=True )
    plt.show()

graf( "campo.txt" )





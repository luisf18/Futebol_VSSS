import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

def graf(name, x0, y0 ):
    # Caminho para a imagem de fundo

    # Leitura dos dados
    path = "C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS/resultados/validacao_experimental/trajetoria_campo_vetorial/"
    img_path = path + "cap_"+name+".png"
    file = path + name + ".txt"
    with open(file, 'r') as file:
        data = file.read()

    # Processamento dos dados
    lines = data.strip().split('\n')
    columns = [line.split() for line in lines[1:]]
    columns = np.array(columns, dtype=float)

    # Configuração do gráfico
    fig, ax = plt.subplots(figsize=(10, int(10*710/947)))
    ax.set_xlim(0, 947)
    ax.set_ylim(0, 710)

    # Adicionando a imagem de fundo
    #img = plt.imread(img_path)
    #ax.imshow(img, extent=[0, 947, 0, 710], aspect='auto', zorder=0)

    # Adicionando um fundo azul à imagem
    #img_with_bg = np.ones((h+400, w+400, 3)) * [1, 1, 1]  # Fundo azul
    
    img = plt.imread(img_path)
    img_rgb = img[:, :, :3]  # Convertendo para RGB
    h, w, _ = img.shape
    
    img_with_bg = np.ones((h+400, w+400, 3)) * [1, 1, 1]  # Fundo azul
    img_with_bg[200:h+200, 200:w+200, :] = img_rgb  # Imagem original no centro
    ax.imshow(img_with_bg, extent=[ 0, 947, 0, 710], aspect='auto', zorder=0)


    # Plotando os dados
    ax.plot(columns[0, 5], 710-columns[0, 6], 'ro', label='bola')
    ax.plot(columns[0, 7], 710-columns[0, 8], 'gx', label='obstaculo')
    ax.plot(columns[:, 2], 710-columns[:, 3], 'bo-', label='trajetória')

    # Posicionando a imagem de fundo
    #ax.set_position([x0, y0, 0.5, 0.5])  # Altere os valores conforme necessário

    # Adicionando legenda e grid
    ax.set_ylabel('y [mm]')
    ax.set_xlabel('x [mm]')
    ax.legend()
    ax.grid()

    # Salvando o gráfico
    plt.savefig(path + f'{name}.png', transparent=True)
    plt.show()

graf("campo", 0, 0 )

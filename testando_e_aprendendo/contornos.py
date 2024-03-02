import cv2
import numpy as np

def detectar_contornos_e_salvar(imagem_path, resultado_path, min_area=100):
    # Leitura da imagem
    imagem = cv2.imread(imagem_path)

    # Conversão para escala de cinza
    imagem_cinza = cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY)

    # Aplicação do filtro de Canny para detecção de bordas
    bordas = cv2.Canny(imagem_cinza, 50, 150)

    # Encontrar contornos na imagem
    contornos, _ = cv2.findContours(bordas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Desenhar contornos na imagem original
    imagem_contornos = imagem.copy()
    
    # Filtrar contornos por área
    for contorno in contornos:
        area = cv2.contourArea(contorno)
        if area > min_area:
            cv2.drawContours(imagem_contornos, [contorno], -1, (0, 255, 0), 1 )

    # Salvar a imagem resultante
    cv2.imwrite(resultado_path, imagem_contornos)

    # Exibir imagens
    cv2.imshow('Imagem Original', imagem)
    cv2.imshow('Detecção de Contornos', imagem_contornos)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Substitua 'caminho/para/sua/imagem.jpg' pelo caminho da sua imagem
    imagem_path    = "C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS/images/vsss_teste_mask.png"
    resultado_path = "C:/Users/UERJBotz/Documents/LF18/(00) GITHUB/Futebol_VSSS/images/vsss_teste_mask_resultado.png"
    
    detectar_contornos_e_salvar(imagem_path, resultado_path, min_area=0)
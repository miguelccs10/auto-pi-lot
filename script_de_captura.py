import cv2
import os
import time

# --- CONFIGURAÇÕES ---
# Caminho para a pasta onde as imagens brutas serão salvas
BASE_PATH = "data/raw"
# Índice da sua câmera (0 geralmente é a webcam integrada)
CAMERA_INDEX = 0
# Delay entre as capturas em segundos (1.0 = 1 foto por segundo)
CAPTURE_DELAY = 1.0 

# --- FIM DAS CONFIGURAÇÕES ---

def main():
    """Função principal para o script de coleta de dados."""
    
    # 1. Obter o nome da classe do usuário
    class_name = input("Digite o nome da classe para capturar (ex: frente) e pressione Enter: ")
    if not class_name:
        print("Nome da classe não pode ser vazio. Saindo.")
        return

    # NOVO: Obter o número de imagens a serem capturadas
    try:
        target_str = input(f"Digite o NÚMERO TOTAL de imagens a capturar para '{class_name}' (ex: 1000): ")
        TARGET_IMAGES = int(target_str)
        if TARGET_IMAGES <= 0:
            print("Erro: O número de imagens deve ser um inteiro positivo. Saindo.")
            return
    except ValueError:
        print("Erro: Entrada inválida. Por favor, digite um número inteiro. Saindo.")
        return

    # 2. Criar o diretório para a classe
    save_path = os.path.join(BASE_PATH, class_name)
    os.makedirs(save_path, exist_ok=True)
    print(f"\nDiretório para salvar as imagens: '{save_path}'")
    print(f"Meta de captura: {TARGET_IMAGES} imagens.")

    # 3. Inicializar a câmera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Erro: Não foi possível abrir a câmera com índice {CAMERA_INDEX}.")
        return

    # 4. Contagem regressiva para o usuário se preparar
    print("\nPrepare-se! A captura começará em 5 segundos...")
    for i in range(5, 0, -1):
        print(f"{i}...")
        time.sleep(1)
    print("Captura iniciada! Pressione 'q' na janela de vídeo para parar antes da meta.")

    # 5. Loop de captura
    img_counter = 0
    last_capture_time = time.time()

    # NOVO: O loop agora para quando a meta é atingida
    while img_counter < TARGET_IMAGES:
        ret, frame = cap.read()
        if not ret:
            print("Erro: Não foi possível capturar o frame.")
            break

        # Desenha informações na tela
        display_frame = frame.copy()
        cv2.putText(display_frame, f"Classe: {class_name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # NOVO: Mostra o progresso
        progress_text = f"Imagens Salvas: {img_counter} / {TARGET_IMAGES}"
        cv2.putText(display_frame, progress_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Lógica para tirar foto a cada N segundos
        current_time = time.time()
        if current_time - last_capture_time >= CAPTURE_DELAY:
            img_name = f"{class_name}_{int(current_time * 1000)}.png"
            full_path = os.path.join(save_path, img_name)
            
            cv2.imwrite(full_path, frame)
            
            print(f"({img_counter + 1}/{TARGET_IMAGES}) Imagem salva: {full_path}")
            img_counter += 1
            last_capture_time = current_time
            
            # Pisca um indicador de captura na tela
            cv2.putText(display_frame, "CAPTURED!", (frame.shape[1] // 2 - 50, frame.shape[0] // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        cv2.imshow("Coletor de Dados - Pressione 'q' para sair", display_frame)

        # Verifica se 'q' foi pressionado para sair
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\nCaptura interrompida pelo usuário.")
            break

    # 6. Libera os recursos
    if img_counter == TARGET_IMAGES:
        print(f"\nMeta atingida! Captura finalizada. Total de {img_counter} imagens salvas.")
    else:
        print(f"\nCaptura interrompida. Total de {img_counter} imagens salvas.")
        
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
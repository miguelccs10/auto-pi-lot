# app/main.py (Versão para Edge Impulse e 4WD)

import cv2
import configparser
import os
import sys
import time
from edge_impulse_linux.image import ImageImpulseRunner

# --- Lógica de Importação Condicional ---
try:
    from app.hardware.motor_control import MotorControl
    IS_RASPBERRY_PI = True
except (ImportError, RuntimeError) as e:
    # Se o erro for de RPi.GPIO, estamos em simulação. Se for outro, pode ser um erro real.
    if "RPi.GPIO" in str(e):
        IS_RASPBERRY_PI = False
        class MotorControl:
            """Classe de simulação para controle de motores em ambiente de desenvolvimento."""
            def __init__(self, config): print("[SIM] Motores inicializados.")
            def execute(self, command): print(f"[SIM] Comando: {command.upper()}")
            def parar(self): print("[SIM] Comando: PARAR")
    else:
        # Se for outro erro de importação, exibe e sai
        print(f"ERRO de importação inesperado: {e}")
        sys.exit(1)


def draw_detections(frame, results, threshold):
    """Desenha as caixas delimitadoras e os rótulos das detecções no frame."""
    if "bounding_boxes" not in results["result"].keys():
        return frame
        
    for bbox in results["result"]["bounding_boxes"]:
        if bbox['value'] > threshold:
            label = bbox['label']
            confidence = bbox['value']
            x, y, w, h = bbox['x'], bbox['y'], bbox['width'], bbox['height']
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return frame

def main():
    """Função principal que orquestra o sistema do robô."""
    print("--- Iniciando Auto-Pi-Lot (Edge Impulse) ---")

    # --- Carregamento de Configurações ---
    config = configparser.ConfigParser()
    if not os.path.exists('config.ini'):
        print("ERRO CRÍTICO: Arquivo 'config.ini' não encontrado!")
        sys.exit(1)
    config.read('config.ini', encoding='utf-8')

    try:
        model_path = 'models/seu-modelo.eim' # <-- LEMBRE-SE DE COLOCAR O NOME CORRETO AQUI
        confidence_threshold = config.getfloat('Comportamento', 'confidence_threshold')
        camera_index = config.getint('Simulacao', 'camera_index')
    except (configparser.NoSectionError, configparser.NoOptionError) as e:
        print(f"ERRO CRÍTICO: Chave de configuração faltando no 'config.ini': {e}")
        sys.exit(1)

    # --- Inicialização dos Módulos ---
    if not os.path.exists(model_path):
        print(f"ERRO CRÍTICO: Arquivo do modelo '{model_path}' não encontrado!")
        sys.exit(1)

    # O runner só pode ser iniciado no Linux/Raspberry Pi.
    runner = None
    if IS_RASPBERRY_PI:
        runner = ImageImpulseRunner(model_path)
        model_info = runner.init()
        print(f"Modelo '{model_info['project']['name']}' carregado.")
    else:
        print("AVISO: Rodando em modo de simulação. O runner do Edge Impulse não será iniciado.")

    motor_controller = MotorControl(config)
    
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"ERRO CRÍTICO: Não foi possível abrir a câmera com índice {camera_index}.")
        sys.exit(1)
    print("INFO: Câmera iniciada com sucesso.")

    print("\n--- Sistema em execução. Pressione Ctrl+C ou 'q' na janela para sair. ---\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            command_to_execute = "parar"
            
            # Só executa a detecção se estiver no Raspberry Pi com o runner ativo
            if runner:
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                res = runner.classify(img_rgb)
                
                if "bounding_boxes" in res["result"].keys():
                    detections = [bbox for bbox in res["result"]["bounding_boxes"] if bbox['value'] > confidence_threshold]
                    if detections:
                        # Pega a detecção com maior confiança
                        best_detection = max(detections, key=lambda x: x['value'])
                        command_to_execute = best_detection['label']
                
                # Desenha as detecções na tela
                frame = draw_detections(frame, res, confidence_threshold)

            motor_controller.execute(command_to_execute)
            cv2.imshow("Auto-Pi-Lot - Visao", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\nINFO: Programa interrompido pelo usuário (Ctrl+C).")
        
    finally:
        # --- Desligamento Seguro ---
        print("\n--- Desligando o sistema ---")
        if hasattr(motor_controller, 'cleanup'):
            motor_controller.cleanup()
        else:
            motor_controller.parar()

        if runner:
            runner.stop()
        cap.release()
        cv2.destroyAllWindows()
        print("INFO: Sistema desligado com segurança.")

if __name__ == '__main__':
    main()
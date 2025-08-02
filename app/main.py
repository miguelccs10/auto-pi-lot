# app/main.py (Versão com Subprocess para Bypassar o Bug)

import cv2
import configparser
import os
import sys
import time
import subprocess
import json

# Mude para True para ver informações detalhadas no console e na tela.
DEBUG_MODE = True

# --- Lógica de Importação Condicional ---
# Tenta importar o módulo de hardware. Se falhar, assume modo de simulação.
try:
    from app.hardware.motor_control import MotorControl
    IS_RASPBERRY_PI = True
except (ImportError, RuntimeError) as e:
    if "RPi" in str(e): # Verifica se o erro é específico do RPi.GPIO
        IS_RASPBERRY_PI = False
        class MotorControl:
            """Classe de simulação para controle de motores em ambiente de desenvolvimento."""
            def __init__(self, config): print("[SIM] Motores inicializados.")
            def execute(self, command): print(f"[SIM] Comando: {command.upper()}")
            def parar(self): print("[SIM] Comando: PARAR")
    else:
        # Se for outro erro de importação, é um problema real.
        print(f"ERRO de importação inesperado: {e}")
        sys.exit(1)

def draw_detections(frame, results, threshold):
    """Desenha as caixas delimitadoras e os rótulos das detecções válidas no frame."""
    if not results or "bounding_boxes" not in results.get("result", {}):
        return frame
        
    for bbox in results["result"]["bounding_boxes"]:
        if bbox['value'] >= threshold:
            label = bbox['label']
            confidence = bbox['value']
            x, y, w, h = bbox['x'], bbox['y'], bbox['width'], bbox['height']
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return frame

def draw_debug_info(frame, fps, results):
    """Desenha informações de FPS e detecções brutas na tela."""
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    if results and "bounding_boxes" in results.get("result", {}):
        all_detections = results["result"]["bounding_boxes"]
        cv2.putText(frame, f"Deteccoes Brutas: {len(all_detections)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return frame

def main():
    """Função principal que orquestra o sistema do robô."""
    print("--- Iniciando Auto-Pi-Lot (MODO SUBPROCESS) ---")

    config = configparser.ConfigParser()
    if not os.path.exists('config.ini'):
        print("ERRO CRÍTICO: Arquivo 'config.ini' não encontrado!")
        sys.exit(1)
    config.read('config.ini', encoding='utf-8')

    try:
        model_path = config.get('Modelo', 'model_path')
        confidence_threshold = config.getfloat('Comportamento', 'confidence_threshold')
        camera_index = config.getint('Simulacao', 'camera_index')
    except (configparser.NoSectionError, configparser.NoOptionError) as e:
        print(f"ERRO CRÍTICO: Chave de configuração faltando no 'config.ini': {e}")
        sys.exit(1)

    if not os.path.exists(model_path):
        print(f"ERRO CRÍTICO: Arquivo do modelo '{model_path}' não encontrado!")
        sys.exit(1)
    # Garante que o modelo tem permissão de execução, um passo crucial no Linux.
    os.chmod(model_path, 0o755)

    motor_controller = MotorControl(config)
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"ERRO CRÍTICO: Não foi possível abrir a câmera {camera_index}.")
        sys.exit(1)
    
    # Define um caminho para a imagem temporária. Usar /dev/shm no Linux é mais rápido (RAM disk).
    temp_image_path = "/dev/shm/frame.jpg" if IS_RASPBERRY_PI else "frame.jpg"

    fps_counter, start_time, current_fps = 0, time.time(), 0

    print(f"\n--- Sistema em execução. Usando modelo '{model_path}'. Pressione Ctrl+C para sair. ---\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            command_to_execute = "parar"
            results = None
            
            # --- Lógica de Detecção com Subprocess ---
            cv2.imwrite(temp_image_path, frame)
            
            try:
                process = subprocess.run([model_path, temp_image_path], 
                                         capture_output=True, text=True, check=True)
                results = json.loads(process.stdout)
                if DEBUG_MODE: print(results)
            except (subprocess.CalledProcessError, json.JSONDecodeError) as e:
                if DEBUG_MODE: print(f"AVISO: Falha na execução do modelo. Erro: {e}")
                results = {}
            
            if "bounding_boxes" in results.get("result", {}):
                detections = [b for b in results["result"]["bounding_boxes"] if b['value'] >= confidence_threshold]
                if detections:
                    best_detection = max(detections, key=lambda x: x['value'])
                    command_to_execute = best_detection['label']
            
            motor_controller.execute(command_to_execute)

            # --- Lógica de Exibição ---
            elapsed_time = time.time() - start_time
            fps_counter += 1
            if elapsed_time > 1:
                current_fps = fps_counter / elapsed_time
                start_time, fps_counter = time.time(), 0
            
            frame = draw_detections(frame, results, confidence_threshold)
            if DEBUG_MODE:
                frame = draw_debug_info(frame, current_fps, results)
            
            cv2.imshow("Auto-Pi-Lot - Visao", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'): break
                
    except KeyboardInterrupt:
        print("\nINFO: Programa interrompido pelo usuário.")
    finally:
        print("\n--- Desligando ---")
        if hasattr(motor_controller, 'cleanup'):
            motor_controller.cleanup()
        else:
            motor_controller.parar()
        cap.release()
        cv2.destroyAllWindows()
        # Remove o arquivo temporário
        if os.path.exists(temp_image_path):
            os.remove(temp_image_path)
        print("INFO: Sistema desligado com segurança.")

if __name__ == '__main__':
    main()
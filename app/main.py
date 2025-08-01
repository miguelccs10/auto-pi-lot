# app/main.py (Versão com Modo de Debug)

import cv2
import configparser
import os
import sys
import time

# --- NOVO: MODO DE DEBUG ---
# Mude para True para ver informações detalhadas no console e na tela.
DEBUG_MODE = True

# --- Lógica de Importação Condicional ---
try:
    from app.hardware.motor_control import MotorControl
    from edge_impulse_linux.image import ImageImpulseRunner
    IS_RASPBERRY_PI = True
except (ImportError, RuntimeError) as e:
    if "RPi" in str(e) or "edge_impulse_linux" in str(e):
        IS_RASPBERRY_PI = False
        class MotorControl:
            def __init__(self, config): print("[SIM] Motores inicializados.")
            def execute(self, command): print(f"[SIM] Comando: {command.upper()}")
            def parar(self): print("[SIM] Comando: PARAR")
        ImageImpulseRunner = None
    else:
        print(f"ERRO de importação inesperado: {e}")
        sys.exit(1)

def draw_detections(frame, results, threshold):
    """Desenha as caixas delimitadoras das detecções VÁLIDAS."""
    if not results or "bounding_boxes" not in results["result"].keys():
        return frame
        
    for bbox in results["result"]["bounding_boxes"]:
        if bbox['value'] >= threshold:
            label = bbox['label']
            confidence = bbox['value']
            x, y, w, h = bbox['x'], bbox['y'], bbox['width'], bbox['height']
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return frame

# --- NOVO: Função para desenhar informações de debug na tela ---
def draw_debug_info(frame, fps, results):
    """Desenha informações de FPS e detecções brutas na tela."""
    # Desenha o FPS
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    if results and "bounding_boxes" in results["result"].keys():
        all_detections = results["result"]["bounding_boxes"]
        # Desenha o número total de detecções (antes do filtro de confiança)
        cv2.putText(frame, f"Deteccoes Brutas: {len(all_detections)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return frame

def main():
    print("--- Iniciando Auto-Pi-Lot (Edge Impulse) ---")

    config = configparser.ConfigParser()
    config.read('config.ini', encoding='utf-8')
    model_path = config.get('Modelo', 'model_path')
    confidence_threshold = config.getfloat('Comportamento', 'confidence_threshold')
    camera_index = config.getint('Simulacao', 'camera_index')

    runner = None
    if IS_RASPBERRY_PI:
        if not os.path.exists(model_path):
            print(f"ERRO CRÍTICO: Arquivo do modelo '{model_path}' não encontrado!")
            sys.exit(1)
        runner = ImageImpulseRunner(model_path)
        model_info = runner.init()
        print(f"Modelo '{model_info['project']['name']}' carregado.")
    else:
        print("AVISO: Rodando em modo de simulação.")

    motor_controller = MotorControl(config)
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"ERRO CRÍTICO: Não foi possível abrir a câmera {camera_index}.")
        sys.exit(1)
    
    # --- NOVO: Variáveis para cálculo de FPS ---
    fps_counter = 0
    start_time = time.time()

    print("\n--- Sistema em execução. Pressione Ctrl+C ou 'q' para sair. ---\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            command_to_execute = "parar"
            results = None
            
            if runner:
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = runner.classify(img_rgb)
                
                # --- NOVO: Imprime o resultado bruto do modelo no console ---
                if DEBUG_MODE:
                    print(results) 

                if "bounding_boxes" in results["result"].keys():
                    detections = [bbox for bbox in results["result"]["bounding_boxes"] if bbox['value'] >= confidence_threshold]
                    if detections:
                        best_detection = max(detections, key=lambda x: x['value'])
                        command_to_execute = best_detection['label']
            
            motor_controller.execute(command_to_execute)

            # --- NOVO: Lógica de exibição de debug ---
            fps_counter += 1
            if (time.time() - start_time) > 1:
                current_fps = fps_counter / (time.time() - start_time)
                start_time = time.time()
                fps_counter = 0
            
            frame = draw_detections(frame, results, confidence_threshold)
            if DEBUG_MODE:
                frame = draw_debug_info(frame, current_fps if 'current_fps' in locals() else 0, results)
            
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
        if runner:
            runner.stop()
        cap.release()
        cv2.destroyAllWindows()
        print("INFO: Sistema desligado com segurança.")

if __name__ == '__main__':
    main()
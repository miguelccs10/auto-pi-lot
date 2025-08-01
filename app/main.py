# app/main.py (Versão de TESTE com redimensionamento manual)

import cv2
import configparser
import os
import sys
import time

DEBUG_MODE = True

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

# ... (funções draw_detections e draw_debug_info continuam as mesmas) ...
def draw_detections(frame, results, threshold, scale_x, scale_y):
    if not results or "bounding_boxes" not in results["result"].keys():
        return frame
    for bbox in results["result"]["bounding_boxes"]:
        if bbox['value'] >= threshold:
            # Escala as coordenadas da caixa de volta para o tamanho original do frame
            x = int(bbox['x'] * scale_x)
            y = int(bbox['y'] * scale_y)
            w = int(bbox['width'] * scale_x)
            h = int(bbox['height'] * scale_y)
            label = bbox['label']
            confidence = bbox['value']
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return frame

def draw_debug_info(frame, fps, results):
    # ... (código igual ao anterior) ...

def main():
    print("--- Iniciando Auto-Pi-Lot (TESTE DE REDIMENSIONAMENTO) ---")

    config = configparser.ConfigParser()
    config.read('config.ini', encoding='utf-8')
    model_path = config.get('Modelo', 'model_path')
    confidence_threshold = config.getfloat('Comportamento', 'confidence_threshold')
    camera_index = config.getint('Simulacao', 'camera_index')

    runner = None
    model_width, model_height = 120, 120 # Default

    # A inicialização do runner agora é um pouco diferente
    try:
        if IS_RASPBERRY_PI or (ImageImpulseRunner is not None):
            runner = ImageImpulseRunner(model_path)
            model_info = runner.init()
            model_width = model_info['model_parameters']['image_width']
            model_height = model_info['model_parameters']['image_height']
            print(f"Modelo '{model_info['project']['name']}' carregado. Espera input de {model_width}x{model_height}.")
        else:
            print("AVISO: Rodando em modo de simulação.")
    except Exception as e:
        print(f"ERRO ao iniciar o runner: {e}")
        print("Continuando em modo de simulação...")
        IS_RASPBERRY_PI = False

    motor_controller = MotorControl(config)
    cap = cv2.VideoCapture(camera_index)
    ret, frame = cap.read()
    if not ret:
        print("ERRO CRÍTICO: Não foi possível ler o primeiro frame da câmera.")
        sys.exit(1)
        
    original_height, original_width = frame.shape[:2]
    scale_x = original_width / model_width
    scale_y = original_height / model_height

    print("\n--- Sistema em execução... ---\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            command_to_execute = "parar"
            results = None
            
            if runner:
                # Redimensiona o frame para o tamanho esperado pelo modelo
                resized_frame = cv2.resize(frame, (model_width, model_height))
                img_rgb = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
                
                results = runner.classify(img_rgb)
                
                if DEBUG_MODE: print(results) 

                if "bounding_boxes" in results["result"].keys():
                    detections = [bbox for bbox in results["result"]["bounding_boxes"] if bbox['value'] >= confidence_threshold]
                    if detections:
                        best_detection = max(detections, key=lambda x: x['value'])
                        command_to_execute = best_detection['label']
            
            motor_controller.execute(command_to_execute)
            
            # Desenha as caixas no frame original, escalando as coordenadas
            frame = draw_detections(frame, results, confidence_threshold, scale_x, scale_y)
            
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
# app/main.py (Versão Edge Impulse)

import cv2
import os
import sys
import time
from edge_impulse_linux.image import ImageImpulseRunner

# O motor_control e a lógica condicional continuam os mesmos
try:
    from app.hardware.motor_control import MotorControl
except (ImportError, RuntimeError):
    class MotorControl:
        def __init__(self, config): print("[SIM] Motores inicializados.")
        def execute(self, command): print(f"[SIM] Comando: {command.upper()}")
        def parar(self): print("[SIM] Comando: PARAR")

def main():
    # O caminho para o seu modelo .eim baixado do Edge Impulse
    model_path = 'models/seu-modelo.eim' 
    if not os.path.exists(model_path):
        print(f"ERRO: Arquivo do modelo '{model_path}' não encontrado!")
        sys.exit(1)

    # Abre um runner para o modelo
    with ImageImpulseRunner(model_path) as runner:
        model_info = runner.init()
        print(f"Modelo '{model_info['project']['name']}' carregado.")
        labels = model_info['model_parameters']['labels']

        cap = cv2.VideoCapture(0) # Use o índice correto da câmera
        if not cap.isOpened():
            print("ERRO: Não foi possível abrir a câmera.")
            sys.exit(1)

        motor_controller = MotorControl(None) # Config pode ser adaptada
        print("\n--- Sistema em execução. Pressione Ctrl+C para sair. ---\n")

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                # O frame precisa ser convertido para RGB para o runner
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # Classifica a imagem
                res = runner.classify(img_rgb)
                
                command_to_execute = "parar"
                
                if "bounding_boxes" in res["result"].keys():
                    if len(res["result"]["bounding_boxes"]) > 0:
                        # Pega a detecção com maior confiança
                        best_detection = max(res["result"]["bounding_boxes"], key=lambda x: x['value'])
                        label = best_detection['label']
                        confidence = best_detection['value']
                        
                        # Lógica de decisão
                        if confidence > 0.75: # Limiar de confiança
                            command_to_execute = label
                            
                            # Desenha a caixa na tela
                            x = best_detection['x']
                            y = best_detection['y']
                            w = best_detection['width']
                            h = best_detection['height']
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            cv2.putText(frame, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                motor_controller.execute(command_to_execute)
                cv2.imshow("Auto-Pi-Lot - Edge Impulse", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            print("\n--- Desligando ---")
            motor_controller.parar()
            if runner:
                runner.stop()
            cap.release()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
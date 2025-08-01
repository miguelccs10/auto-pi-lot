# app/hardware/motor_control.py

import RPi.GPIO as GPIO
import time

class MotorControl:
    """
    Classe para controlar um chassi de 4 rodas (4WD) com um driver L298N.
    Assume que os motores de cada lado estão conectados em paralelo a um canal.
    """
    def __init__(self, config):
        print("[HW] Inicializando o controlador de motores 4WD...")
        
        try:
            # Lado Esquerdo (conectado ao Canal A do L298N)
            self.ENA = int(config.get('Hardware', 'motor_a_enable'))
            self.IN1 = int(config.get('Hardware', 'motor_a_in1'))
            self.IN2 = int(config.get('Hardware', 'motor_a_in2'))

            # Lado Direito (conectado ao Canal B do L298N)
            self.ENB = int(config.get('Hardware', 'motor_b_enable'))
            self.IN3 = int(config.get('Hardware', 'motor_b_in3'))
            self.IN4 = int(config.get('Hardware', 'motor_b_in4'))
            
            self.velocidade_padrao = int(config.get('Comportamento', 'velocidade'))
            # Velocidade reduzida para curvas mais suaves (uma porcentagem da velocidade padrão)
            self.velocidade_curva = int(self.velocidade_padrao * 0.5) 

        except Exception as e:
            print(f"ERRO CRÍTICO: Falha ao ler configuração de hardware. Detalhes: {e}")
            raise SystemExit

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)

        self.pwm_a = GPIO.PWM(self.ENA, 500)
        self.pwm_b = GPIO.PWM(self.ENB, 500)
        
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        
        print(f"[HW] Motores prontos. Velocidade padrão: {self.velocidade_padrao}%.")

    def _set_motores(self, vel_esquerda, dir_esquerda, vel_direita, dir_direita):
        """Função de baixo nível para controlar cada lado independentemente."""
        # Lado Esquerdo
        if dir_esquerda == "frente":
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        else: # "tras"
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(vel_esquerda)

        # Lado Direito
        if dir_direita == "frente":
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        else: # "tras"
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_b.ChangeDutyCycle(vel_direita)

    def mover_frente(self):
        self._set_motores(self.velocidade_padrao, "frente", self.velocidade_padrao, "frente")

    def virar_direita(self):
        # Lado esquerdo acelera, lado direito desacelera para uma curva suave
        self._set_motores(self.velocidade_padrao, "frente", self.velocidade_curva, "frente")

    def virar_esquerda(self):
        # Lado direito acelera, lado esquerdo desacelera
        self._set_motores(self.velocidade_curva, "frente", self.velocidade_padrao, "frente")

    def parar(self):
        """Para todos os motores."""
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)

    def execute(self, command):
        """Executa um comando de movimento de alto nível."""
        if command == "frente":
            self.mover_frente()
        elif command == "direita":
            self.virar_direita()
        elif command == "esquerda":
            self.virar_esquerda()
        elif command == "parar":
            self.parar()
        # Adicione outros comandos se necessário
        
    def cleanup(self):
        """Para os motores e limpa os pinos GPIO."""
        print("[HW] Limpando pinos GPIO...")
        self.parar()
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
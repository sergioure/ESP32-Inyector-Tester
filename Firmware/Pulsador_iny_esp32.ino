/*
  Proyecto: Control de Inyectores con PWM por Hardware 
  Autor: Sergio Ureña
  Fecha: 30 de Julio de 2025
  Microcontrolador: ESP32

  Descripción: Sistema completo para controlar inyectores de combustible mediante PWM por hardware,
  con múltiples modos de operación y una interfaz web para control remoto.
*/

// ==================== INCLUDES ====================
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Encoder.h>
#include <driver/ledc.h>  // Añadir esta línea al inicio del archivo con los demás includes
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <WebServer.h>

// ==================== CONFIGURACIÓN WIFI/SERVIDOR ====================
WebServer server(80); // Servidor en puerto 80
// Configuración del Access Point (AP)
const char* apSSID = "ESP32-Inyectores"; // Nombre de la red
const char* apPassword = "12345678";     // Contraseña (mínimo 8 caracteres

// ==================== VARIABLES DE CONTROL ====================
int rpm_f = 2500;
int pwm_f = 50;
bool estado = false;
bool inyec[4] = {false, false, false, false};

// ==================== CONFIGURACIÓN HARDWARE ====================
//Configuracion del encoder rotativo
#define CLK 27 // CLK ENCODER
#define DT 26 // DT ENCODER
ESP32Encoder encoder;

// Configuración LCD I2C (0x27 es la dirección común, cambiar si es necesario)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Configuración de pines
const int buzzerPin = 5;  // Buzzer digital (ON/OFF)

// Pines de botones
const int boton1Pin = 32;    // Botón 1 (normalmente para salir/retroceder)
const int boton2Pin = 33;    // Botón 2 (normalmente para funciones adicionales)
const int botonEncPin = 25;  // Botón del encoder (normalmente para enter/confirmar)

// Pines PWM para inyectores
const int inyec1Pin = 16;
const int inyec2Pin = 17;
const int inyec3Pin = 18;
const int inyec4Pin = 19;

// ==================== CONFIGURACIÓN PWM ====================
const uint32_t pwmFrequency = 500; // 500 Hz
const uint8_t pwmResolution = 8;   // 8 bits (0-255)
const int dutyCycle = 128;         // 50% de 255

// ==================== VARIABLES GLOBALES ====================
// Variables para el manejo de botones
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // tiempo de debounce en ms

// Variables globales necesarias
int indice_seleccion = 0;
int opcion_anterior = -1;  // Inicializado a -1 para forzar la primera impresión
int menu = 0;

// Encoder y navegación
volatile int contador = 1;       // Posición del encoder
int sub_menus = 0;               // Indica si se ingresó a un submenú
int sub_menu_modo = 0;           // Indica si está en el submenú de modo
int modo_abcd = 0;               // Modo seleccionado (A, B, C o D)

// Configuración de RPM y PWM para Motor A
const int RPM_MIN_A = 900;
const int RPM_MAX_A = 5000;
const int PASO_RPM_A = 50;       // Incremento/decremento por paso
int rpm_a = 2500;                // RPM inicial

const int PWM_MIN_A = 1;
const int PWM_MAX_A = 99;
const int PASO_PWM_A = 1;        // Incremento/decremento por paso
int pwm_a = 50;                  // PWM inicial

const int Tmin_MIN_A = 1;
const int Tmin_MAX_A = 99;
const int PASO_Tmin_A = 1;       // Incremento/decremento por paso
int tmin_a = 1;                  // Tmin inicial
int tseg_a = 0;                  // Tseg inicial

// Configuración de RPM y PWM para Motor B
const int RPM_MIN_B = 900;
const int RPM_MAX_B = 5000;
const int PASO_RPM_B = 50;
int rpm_b = 2500;

const int PWM_MIN_B = 1;
const int PWM_MAX_B = 99;
const int PASO_PWM_B = 1;
int pwm_b = 50;

const int Tmin_MIN_B = 1;
const int Tmin_MAX_B = 99;
const int PASO_Tmin_B = 1;
int tmin_b = 1;
int tseg_b = 0;

// Configuración para Motor C
const int PASO_RPM_C = 50;
int rpm_c = 900;
int pwm_c = 50;

// Configuración para Motor D
const int PASO_PWM_D = 1;
int rpm_d = 2500;
int pwm_d = 1;

// Variables de menú
int menu_modo_a = 0, menu_modo_b = 0, menu_modo_c = 0, menu_modo_d = 0;
int config_rpm_a = 0, config_pwm_a = 0, config_tmin_a = 0, config_a = 0;
int config_rpm_b = 0, config_pwm_b = 0, config_tmin_b = 0, config_b = 0;
int config_c = 0, config_d = 0;

//Variables modo e
int rpm_e = 2500;  // RPM inicial para Modo E
int pwm_e = 50;    // PWM inicial para Modo E
int menu_modo_e = 0;

// -------------------- CARACTERES PERSONALIZADOS --------------------
byte medio_vacio[8] = {0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F};
byte lleno[8] = {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};  // Lleno
byte vacio[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // Vacío
byte guion[8] = {0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00};  // Guion
byte flecha[8] = {0x00, 0x00, 0x04, 0x06, 0x1F, 0x06, 0x04, 0x00}; // Flecha
byte barra_progreso[8] = {0b11111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111 };

// Opciones del menú
const char* menu_opciones[] = {"A", "B", "C", "D", "E", "F"};
const int num_opciones = 6;

void setupPWM() {
  // Configurar frecuencia y resolución para cada pin PWM
  analogWriteFrequency(inyec1Pin, pwmFrequency);
  analogWriteResolution(inyec1Pin, pwmResolution);
  
  analogWriteFrequency(inyec2Pin, pwmFrequency);
  analogWriteResolution(inyec2Pin, pwmResolution);
  
  analogWriteFrequency(inyec3Pin, pwmFrequency);
  analogWriteResolution(inyec3Pin, pwmResolution);
  
  analogWriteFrequency(inyec4Pin, pwmFrequency);
  analogWriteResolution(inyec4Pin, pwmResolution);
}

// ==================== SETUP INICIAL ====================
/**
 * Función: setup()
 * Descripción: Configuración inicial del hardware y periféricos.
 * Configura pines, LCD, PWM, encoder y muestra pantalla de inicio.
 */
void setup() {
  // Inicializar LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
 
  // Configuración básica de pines
  pinMode(buzzerPin, OUTPUT); // Buzzer como salida digital
  pinMode(boton1Pin, INPUT_PULLDOWN);
  pinMode(boton2Pin, INPUT_PULLDOWN);
  pinMode(botonEncPin, INPUT_PULLUP);
  
  // Configuración PWM
  setupPWM();
  
  // Inicializar todos los PWM a 0
  analogWrite(inyec1Pin, 0);
  analogWrite(inyec2Pin, 0);
  analogWrite(inyec3Pin, 0);
  analogWrite(inyec4Pin, 0);
  
  // Beep de inicio con el buzzer
  digitalWrite(buzzerPin, HIGH);
  delay(200);
  digitalWrite(buzzerPin, LOW);

 // Configuracion encoder
  encoder.attachHalfQuad(DT, CLK);
  encoder.setCount(0);
  Serial.begin ( 115200 );

  // Pantalla de inicio
  zumbido(2, 200); // Dos pitidos de 200ms cada uno
  lcd.clear();
  imprimir_lcd(2, 0, "Pulsador de");
  imprimir_lcd(0, 1, "Inyectores ESP32");
  delay(2000);
  lcd.clear();

  // Registrar caracteres personalizados
  lcd.createChar(0, medio_vacio);
  lcd.createChar(1, lleno);
  lcd.createChar(2, vacio);
  lcd.createChar(3, guion);
  lcd.createChar(4, flecha);
    
  // Mostrar menú principal
  menu_principal();
}

// Función helper para formatear
void imprimir_formateado(uint8_t x, uint8_t y, const char* prefijo, int valor, int ancho = 0) {
  char buffer[17];
  if(ancho > 0) {
    char formato[10];
    sprintf(formato, "%%s%%0%dd", ancho);
    sprintf(buffer, formato, prefijo, valor);
  } else {
    sprintf(buffer, "%s%d", prefijo, valor);
  }
  imprimir_lcd(x, y, buffer);
}

// ==================== FUNCIONES PRINCIPALES ====================
/**
 * Función: menu_principal()
 * Descripción: Muestra el menú principal con las opciones disponibles (A-F).
 * Gestiona la navegación mediante el encoder y muestra la selección actual.
 */
void menu_principal() {
  menu = 1;
  menu_modo_a = 0;
    
  // Mostrar en terminal si cambió la selección (versión corregida)
  const char* opcion_actual = menu_opciones[indice_seleccion];
  if (opcion_anterior == -1 || strcmp(opcion_actual, menu_opciones[opcion_anterior]) != 0) {
    Serial.print("Opción seleccionada: ");
    Serial.println(opcion_actual);
    opcion_anterior = indice_seleccion;
  }
  
  // Primera línea: MODO:   A   B
  imprimir_lcd(0, 0, "MODO:  ");
  
  // Limpiar todas las posibles posiciones de flechas primero
  imprimir_lcd(6, 0, " ");  // Posición flecha A
  imprimir_lcd(10, 0, " "); // Posición flecha B
  imprimir_lcd(1, 1, " ");  // Posición flecha C
  imprimir_lcd(5, 1, " ");  // Posición flecha D
  imprimir_lcd(9, 1, " ");  // Posición flecha E
  imprimir_lcd(13, 1, " "); // Posición flecha F
  
  // Mostrar las opciones sin flechas
  imprimir_lcd(7, 0, "A");
  imprimir_lcd(11, 0, "B");
  imprimir_lcd(2, 1, "C");
  imprimir_lcd(6, 1, "D");
  imprimir_lcd(10, 1, "E");
  imprimir_lcd(14, 1, "F");
  
  // Colocar la flecha en la opción seleccionada
  switch(indice_seleccion) {
    case 0: // Opción A
      imprimir_lcd(6, 0, "\4"); // Flecha antes de A
      break;
    case 1: // Opción B
      imprimir_lcd(10, 0, "\4"); // Flecha antes de B
      break;
    case 2: // Opción C
      imprimir_lcd(1, 1, "\4"); // Flecha antes de C
      break;
    case 3: // Opción D
      imprimir_lcd(5, 1, "\4"); // Flecha antes de D
      break;
    case 4: // Opción E
      imprimir_lcd(9, 1, "\4"); // Flecha antes de E
      break;
    case 5: // Opción F
      imprimir_lcd(13, 1, "\4"); // Flecha antes de F
      break;
  }
}

/**
 * Función: ejecutar_temporizador()
 * Descripción: Controla un temporizador con visualización en LCD.
 * Parámetros:
 *   - tmin: Minutos del temporizador
 *   - tseg: Segundos del temporizador
 *   - modo: Modo de operación (1=simultáneo, 2=secuencial)
 *   - inyector_num: Número de inyector (para modo secuencial)
 */
void ejecutar_temporizador(int tmin, int tseg, int modo, int inyector_num = 0) {
    /* Subrutina para manejar el temporizador usando millis() */
    int total_segundos = tmin * 60 + tseg;
    
    // Convertir a décimas de segundo
    unsigned long tiempo_restante = total_segundos * 10;
    unsigned long ultima_actualizacion = 0;
    const unsigned long intervalo_actualizacion = 100; // 100 ms = 0.1 segundos
    
    // Buffer para el mensaje LCD
    char lcd_buffer[17]; // 16 caracteres + null terminator
    // Limpiar la línea primero para evitar residuos
    imprimir_lcd(0, 1, "                "); // Limpiar toda la línea
    
    // Mostrar tiempo inicial - Aseguramos posición fija
    if (modo == 2) { // Modo secuencial
        snprintf(lcd_buffer, sizeof(lcd_buffer), "Iny:%1d T:%02d:%02d", 
                inyector_num, 
                total_segundos / 60, 
                total_segundos % 60);
    } else {
        snprintf(lcd_buffer, sizeof(lcd_buffer), " T:%02d:%02d",  // Espacio inicial para alinear
                total_segundos / 60, 
                total_segundos % 60);
    }
    imprimir_lcd(0, 1, lcd_buffer);
    
    while (tiempo_restante > 0) {
        unsigned long ahora = millis();
        
        // Actualizar cada 100ms
        if (ahora - ultima_actualizacion >= intervalo_actualizacion) {
            ultima_actualizacion = ahora;
            tiempo_restante--;
            
            // Calcular minutos y segundos restantes
            int seg_rest = tiempo_restante / 10;
            int min_rest = seg_rest / 60;
            seg_rest = seg_rest % 60;
            
            // Actualizar display manteniendo la misma posición
            if (modo == 2) {
                snprintf(lcd_buffer, sizeof(lcd_buffer), "Iny:%1d T:%02d:%02d",
                        inyector_num,
                        min_rest,
                        seg_rest);
            } else {
                snprintf(lcd_buffer, sizeof(lcd_buffer), " T:%02d:%02d",  // Mismo espacio inicial
                        min_rest,
                        seg_rest);
            }
            imprimir_lcd(0, 1, lcd_buffer);  // Siempre en la misma posición
            
            // Mostrar progreso por serial
            Serial.print("Tiempo restante: ");
            if (modo == 2) {
                Serial.print("Inyector ");
                Serial.print(inyector_num);
                Serial.print(" - ");
            }
            Serial.print(min_rest);
            Serial.print(":");
            if (seg_rest < 10) Serial.print("0");
            Serial.println(seg_rest);
        }
        
        delay(10); // Pequeña pausa
    }
    
    // Tiempo completado
    zumbido(1, 100);
    if (modo == 2) {
        imprimir_lcd(0, 1, "Iny:%1d Fin       ", inyector_num);
    } else {
        imprimir_lcd(0, 1, " T:00:00         "); // Formato consistente
    }
}

/**
 * Función: arranque_modos()
 * Descripción: Ejecuta el modo de operación seleccionado (A-D).
 * Controla los inyectores según la configuración del modo.
 * Parámetros:
 *   - opcion: Modo a ejecutar (1=A, 2=B, etc.)
 */
void arranque_modos(int opcion) {
    if (opcion == 1) {  // Modo A (activación simultánea con LEDC)
      Serial.println("Inicio del Modo A - Simultáneo");
      zumbido(2, 250);
      
      // 1. Configuración inicial
      lcd.clear();
      imprimir_formateado(1, 0, "RPM:", rpm_a, 4);
      imprimir_formateado(10, 0, "P:", pwm_a, 2);
      lcd.setCursor(15, 0);
      lcd.print('%');
      
      // 2. Configuración LEDC para los 4 inyectores (simultáneos)
      const int pwmResolution = 10; // Resolución de 10 bits (0-1023)
      const ledc_channel_t channels[] = {
          LEDC_CHANNEL_0, // Inyector 1
          LEDC_CHANNEL_1, // Inyector 2
          LEDC_CHANNEL_2, // Inyector 3
          LEDC_CHANNEL_3  // Inyector 4
      };
      const int pins[] = {inyec1Pin, inyec2Pin, inyec3Pin, inyec4Pin};
      
      // Configurar timer LEDC (frecuencia basada en RPM)
      ledc_timer_config_t timer_conf = {
          .speed_mode = LEDC_LOW_SPEED_MODE,
          .duty_resolution = (ledc_timer_bit_t)pwmResolution,
          .timer_num = LEDC_TIMER_0,
          .freq_hz = (uint32_t)(rpm_a / 60.0), // Convertir RPM a Hz
          .clk_cfg = LEDC_AUTO_CLK
      };
      ledc_timer_config(&timer_conf);
      
      // Configurar canales y activar PWM
      uint32_t duty = map(pwm_a, 0, 100, 0, 1023); // Mapear % PWM a resolución de 10 bits
      for(int i = 0; i < 4; i++) {
          ledc_channel_config_t channel_conf = {
              .gpio_num = pins[i],
              .speed_mode = LEDC_LOW_SPEED_MODE,
              .channel = channels[i],
              .timer_sel = LEDC_TIMER_0,
              .duty = duty,
              .hpoint = 0
          };
          ledc_channel_config(&channel_conf);
      }
      
      // 3. Temporizador con barra de progreso (similar a tu versión original)
      unsigned long tiempo_total = tmin_a * 60 * 1000UL + tseg_a * 1000UL; // ms
      unsigned long tiempo_inicio = millis();
      unsigned long ultima_actualizacion = 0;
      const unsigned long intervalo_actualizacion = 200; // Actualizar cada 200ms
      
      // Mostrar tiempo inicial
      char tiempo_str[10];
      sprintf(tiempo_str, "%02d:%02d", tmin_a, tseg_a);
      imprimir_lcd(1, 1, "T: ");
      imprimir_lcd(3, 1, tiempo_str);
      
      // Barra de progreso (usando caracteres personalizados)
      lcd.setCursor(9, 1);
      for(int i = 0; i < 7; i++) lcd.write(5); // Caracteres vacíos
      
      // Bucle principal
      while (millis() - tiempo_inicio < tiempo_total) {
          unsigned long ahora = millis();
          
          if (ahora - ultima_actualizacion >= intervalo_actualizacion) {
              ultima_actualizacion = ahora;
              
              // Actualizar tiempo restante
              unsigned long tiempo_restante = tiempo_total - (ahora - tiempo_inicio);
              int minutos = tiempo_restante / 60000;
              int segundos = (tiempo_restante % 60000) / 1000;
              sprintf(tiempo_str, "%02d:%02d", minutos, segundos);
              imprimir_lcd(3, 1, tiempo_str);
              
              // Actualizar barra de progreso (simplificada)
              int progreso = map(ahora - tiempo_inicio, 0, tiempo_total, 0, 7);
              lcd.setCursor(9, 1);
              for (int i = 0; i < 7; i++) {
                  lcd.write(i < progreso ? 1 : 5); // Lleno (1) o vacío (5)
              }
          }
          delay(10);
      }
      
      // 4. Apagar todos los inyectores
      for(int i = 0; i < 4; i++) {
          ledc_stop(LEDC_LOW_SPEED_MODE, channels[i], 0);
      }
      
      // 5. Finalización
      Serial.println("\nSecuencia completada");
      imprimir_lcd(0, 1, "COMPLETADO!     ");
      lcd.setCursor(9, 1);
      for(int i = 0; i < 7; i++) lcd.write(1); // Barra completa
      
      // 4. Finalización
      zumbido(1, 1500);
      // 5. Limpieza
      lcd.clear();
      menu_principal();
}
    
    else if (opcion == 2) {  // Modo B (activación secuencial)
    Serial.println("Inicio del Modo B - Secuencial");
    zumbido(3, 200);
    
    // 1. Configuración inicial
    lcd.clear();
    
    // Mostrar valores fijos en la fila 0
    imprimir_formateado(1, 0, "RPM:", rpm_b, 4);
    imprimir_formateado(10, 0, "P:", pwm_b, 2);
    lcd.setCursor(15, 0);
    lcd.print('%');
    
    // 2. Configuración LEDC para los 4 inyectores
    const int pwmResolution = 10;
    const ledc_channel_t channels[] = {
        LEDC_CHANNEL_0, // Inyector 1
        LEDC_CHANNEL_1, // Inyector 2
        LEDC_CHANNEL_2, // Inyector 3
        LEDC_CHANNEL_3  // Inyector 4
    };
    const int pins[] = {inyec1Pin, inyec2Pin, inyec3Pin, inyec4Pin};
    
    // Configurar timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)pwmResolution,
        .timer_num = LEDC_TIMER_2,
        .freq_hz = (uint32_t)(rpm_b / 60.0),
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    // Configurar canales (inicialmente apagados)
    for(int i = 0; i < 4; i++) {
        ledc_channel_config_t channel_conf = {
            .gpio_num = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channels[i],
            .timer_sel = LEDC_TIMER_2,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel_conf);
    }

    // 3. Secuencia de activación
    for (int i = 0; i < 4; i++) {
        Serial.printf("\nActivando inyector %d\n", i+1);
        
        // Mostrar en LCD (fila 1)
        char lcd_buffer[17];
        snprintf(lcd_buffer, sizeof(lcd_buffer), "Iny:%d T:%02d:%02d",
                i+1, 
                tmin_b, 
                tseg_b);
        imprimir_lcd(0, 1, lcd_buffer);
        
        // Activar el inyector actual
        uint32_t duty = map(pwm_b, 0, 99, 0, 1023);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channels[i], duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channels[i]);
        
        // Temporizador
        unsigned long tiempo_total = tmin_b * 60000UL + tseg_b * 1000UL;
        unsigned long tiempo_inicio = millis();
        unsigned long ultima_actualizacion = 0;
        
        while (millis() - tiempo_inicio < tiempo_total) {
            unsigned long ahora = millis();
            
            // Actualizar cada 200ms
            if (ahora - ultima_actualizacion >= 200) {
                ultima_actualizacion = ahora;
                
                // Actualizar tiempo restante
                unsigned long restante = tiempo_total - (ahora - tiempo_inicio);
                int min_rest = restante / 60000;
                int seg_rest = (restante % 60000) / 1000;
                snprintf(lcd_buffer, sizeof(lcd_buffer), "Iny:%d T:%02d:%02d", 
                        i+1, min_rest, seg_rest);
                imprimir_lcd(0, 1, lcd_buffer);
            }
            delay(10);
        }
        
        // Apagar el inyector actual
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channels[i], 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channels[i]);
        
        // Mostrar finalización
        snprintf(lcd_buffer, sizeof(lcd_buffer), "Iny:%d Fin     ", i+1);
        imprimir_lcd(0, 1, lcd_buffer);
        
        // Pequeña pausa entre inyectores (excepto el último)
        if (i < 3) {
            delay(500);
        }
    }
    
    // 4. Finalización
    Serial.println("\nSecuencia completada");
    imprimir_lcd(0, 1, "COMPLETADO!     ");
    zumbido(1, 1500);
    
    // 5. Limpieza
    lcd.clear();
    menu_principal();
}
 
    else if (opcion == 3) {  // Modo C
    Serial.println("Inicio del MODO C");
    zumbido(2, 250);
    
    // Configuración inicial
    lcd.clear();
    imprimir_formateado(1, 0, "RPM:", rpm_c, 4);
    lcd.createChar(1, lleno); // Cargar carácter lleno
    imprimir_formateado(10, 0, "P:", pwm_c, 2);
    lcd.setCursor(15, 0);
    lcd.print('%');
    
    // Configurar PWM usando LEDC (similar al Modo E pero simplificado)
    const int pwmChannel = 0;
    const int pwmResolution = 10;
    
    // Configurar timer LEDC
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)pwmResolution,
        .timer_num = LEDC_TIMER_1, // Diferente timer que el usado en Modo E
        .freq_hz = (uint32_t)(rpm_c / 60.0),
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    // Configurar canales
    const int pins[] = {inyec1Pin, inyec2Pin, inyec3Pin, inyec4Pin};
    for(int i = 0; i < 4; i++) {
        ledc_channel_config_t channel_conf = {
            .gpio_num = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)(LEDC_CHANNEL_4 + i), // Canales diferentes
            .timer_sel = LEDC_TIMER_1,
            .duty = 512, // 50% duty cycle inicial
            .hpoint = 0
        };
        ledc_channel_config(&channel_conf);
    }

    unsigned long last_update = millis();
    const int update_interval = 300;
    
    // Secuencia de incremento de RPM
    while (rpm_c <= 5000) {
        unsigned long current_time = millis();
        
        if (current_time - last_update >= update_interval) {
            last_update = current_time;
            
            // Actualizar frecuencia PWM
            float freq = rpm_c / 60.0;
            ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, freq);
            
            // Mantener duty cycle al 50%
            uint32_t duty = 512; // 50% de 1023
            for(int i = 0; i < 4; i++) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(LEDC_CHANNEL_4 + i), duty);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(LEDC_CHANNEL_4 + i));
            }
            
            // Actualizar visualización
            imprimir_formateado(1, 0, "RPM:", rpm_c, 4);
            Serial.print("RPM: ");
            Serial.println(rpm_c);
            
            // Barra de progreso con carácter lleno
            int progress = map(rpm_c, 900, 5000, 0, 16);
            lcd.setCursor(0, 1);
            for(int i = 0; i < progress; i++) lcd.write(1);
            for(int i = progress; i < 16; i++) lcd.write(' ');
            
            rpm_c += PASO_RPM_C;
        }
        delay(1);
    }
    
    // Finalización
    for(int i = 0; i < 4; i++) {
        ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(LEDC_CHANNEL_4 + i), 0);
    }
    
    // Mostrar barra completa
    lcd.setCursor(0, 1);
    for(int i = 0; i < 16; i++) lcd.write(1);
    
    zumbido(1, 1250);
    imprimir_lcd(0, 1, "COMPLETADO!     ");
    
    // Espera no bloqueante
    unsigned long startTime = millis();
    while(millis() - startTime < 1500) {
        delay(10);
    }

    // 4. Finalización
    rpm_c = 900;
    menu = 1;
    Serial.println("\nSecuencia completada");
    imprimir_lcd(0, 1, "COMPLETADO!     ");
    zumbido(1, 1500);
            
    // 5. Limpieza
    lcd.clear();
    menu_principal();
}

else if (opcion == 4) {  // Modo D
    Serial.println("Inicio del MODO D");
    zumbido(2, 250);
    
    // 1. Configuración inicial
    lcd.clear();
    imprimir_formateado(1, 0, "RPM:", rpm_d, 4);
    imprimir_formateado(10, 0, "P:", pwm_d, 2);
    lcd.setCursor(15, 0);
    lcd.print('%');
    lcd.createChar(1, lleno);

    // 2. Configuración avanzada de PWM usando LEDC
    const float freq = rpm_d / 60.0;
    const int pwmResolution = 10; // 10 bits (0-1023)
    
    // Configurar canales PWM (LEDC)
    const ledc_channel_t channels[] = {
        LEDC_CHANNEL_0, 
        LEDC_CHANNEL_1, 
        LEDC_CHANNEL_2, 
        LEDC_CHANNEL_3
    };
    
    const int pins[] = {inyec1Pin, inyec2Pin, inyec3Pin, inyec4Pin};
    
    // Configurar timer LEDC
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)pwmResolution,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = (uint32_t)freq,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    // Configurar canales
    for(int i = 0; i < 4; i++) {
        ledc_channel_config_t channel_conf = {
            .gpio_num = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channels[i],
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel_conf);
    }

    // 3. Bucle principal con control preciso
    const unsigned long updateInterval = 500; // ms
    unsigned long previousMillis = millis();
    bool pwmCompleted = false;

    while(!pwmCompleted) {
        unsigned long currentMillis = millis();
        
        if(currentMillis - previousMillis >= updateInterval) {
            previousMillis = currentMillis;
            
            if(pwm_d <= 99) {
                // Calcular ciclo de trabajo
                uint32_t duty = (1023 * pwm_d) / 100;
                
                // Escribir en todos los canales
                for(int i = 0; i < 4; i++) {
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, channels[i], duty);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, channels[i]);
                }
                
                // Actualizar visualización
                imprimir_formateado(10, 0, "P:", pwm_d, 2);
                Serial.printf("PWM: %d%%, Duty: %lu/1023\n", pwm_d, duty);
                
                // Barra de progreso
                int progress = map(pwm_d, 1, 99, 0, 16);
                lcd.setCursor(0, 1);
                for(int i = 0; i < progress; i++) lcd.write(1);
                for(int i = progress; i < 16; i++) lcd.write(' ');
                
                pwm_d += PASO_PWM_D;
            } else {
                pwmCompleted = true;
            }
        }
        delay(1); // Pequeña pausa
    }

    // 4. Finalización
    for(int i = 0; i < 4; i++) {
        ledc_stop(LEDC_LOW_SPEED_MODE, channels[i], 0);
    }
    
    // Mostrar finalización
    lcd.setCursor(0, 1);
    for(int i = 0; i < 16; i++) lcd.write(1);
    imprimir_lcd(0, 1, "COMPLETADO!     ");
    zumbido(1, 1500);

    // 5. Limpieza y reinicio
    pwm_d = 1;
    lcd.clear();
    menu = 1;
    menu_principal();
  }
    // Finalización común
    zumbido(1, 1250);
  }

// ==================== FUNCIONES DE LOS MODOS ====================

/**
 * Función: modo_a()
 * Descripción: Interfaz de configuración para el Modo A.
 * Permite ajustar RPM, PWM y tiempo de operación.
 */
void modo_a() {
  sub_menu_modo = 1;  // 1=RPM, 2=PWM, 3=Tmin, 4=Inicio
  bool editing = false;
  long encoder_base = 0;  // Variable nueva para guardar posición base
  
  lcd.clear();
  
  while(true) {
    // Manejar navegación del encoder
    long current_encoder = encoder.getCount();
    
    if(!editing) {
      if(current_encoder > encoder_base) { // Giro derecha
        sub_menu_modo = (sub_menu_modo % 4) + 1;
        encoder_base = current_encoder;
      } 
      else if(current_encoder < encoder_base) { // Giro izquierda
        sub_menu_modo = (sub_menu_modo == 1) ? 4 : sub_menu_modo - 1;
        encoder_base = current_encoder;
      }
    }

    // Mostrar valores
    imprimir_formateado(1, 0, "RPM:", rpm_a, 4);
    imprimir_formateado(11, 0, "P:", pwm_a, 2);
    lcd.setCursor(15, 0);
    lcd.print('%');
    imprimir_formateado(1, 1, "T:", tmin_a, 2);
    imprimir_formateado(5, 1, ":", tseg_a, 2);
    imprimir_lcd(10, 1, "Inicio");

    // Actualizar flechas de navegación
    lcd.setCursor(0, 0);
    lcd.print(sub_menu_modo == 1 ? (editing ? '*' : '\4') : ' ');
    lcd.setCursor(10, 0);
    lcd.print(sub_menu_modo == 2 ? (editing ? '*' : '\4') : ' ');
    lcd.setCursor(0, 1);
    lcd.print(sub_menu_modo == 3 ? (editing ? '*' : '\4') : ' ');
    lcd.setCursor(9, 1);
    lcd.print(sub_menu_modo == 4 ? '\4' : ' ');

    // Manejar botón Enter
    if(digitalRead(botonEncPin) == LOW) {
      delay(50);
      if(digitalRead(botonEncPin) == LOW) {
        zumbido(1, 100);
        
        if(sub_menu_modo == 4) { // Inicio
          arranque_modos(1);
          lcd.clear();
          return;
        } else {
          editing = !editing;
          if(editing) {
            encoder.setCount(0);  // Reiniciamos el contador del encoder
            encoder_base = 0;     // Reiniciamos la base
          }
        }
        while(digitalRead(botonEncPin) == LOW) delay(10);
      }
    }

    // Manejar edición de valores (VERSIÓN CORREGIDA)
    if(editing) {
      long delta = encoder.getCount(); // Obtenemos el cambio acumulado
      
      if(delta != 0) {
        switch(sub_menu_modo) {
          case 1: // RPM
            rpm_a += (delta > 0) ? PASO_RPM_A : -PASO_RPM_A;
            rpm_a = constrain(rpm_a, RPM_MIN_A, RPM_MAX_A);
            break;
            
          case 2: // PWM
            pwm_a += (delta > 0) ? PASO_PWM_A : -PASO_PWM_A;
            pwm_a = constrain(pwm_a, PWM_MIN_A, PWM_MAX_A);
            break;
            
          case 3: // Tmin
            tmin_a += (delta > 0) ? PASO_Tmin_A : -PASO_Tmin_A;
            tmin_a = constrain(tmin_a, Tmin_MIN_A, Tmin_MAX_A);
            break;
        }
        encoder.setCount(0); // Reiniciamos después de aplicar el cambio
        encoder_base = 0;    // Actualizamos la base
      }
    }

    // Manejar botón de salida
    if(digitalRead(boton1Pin) == HIGH) {
      delay(50);
      if(digitalRead(boton1Pin) == HIGH) {
        lcd.clear();
        menu_principal();
        return;
      }
    }

    delay(50);
  }
}

/**
 * Función: modo_b()
 * Descripción: Interfaz de configuración para el Modo B.
 * Similar al Modo A pero con parámetros específicos para este modo.
 */
void modo_b() {
  sub_menu_modo = 1;  // Iniciar en posición (0,0)
  config_b = 0;       // Modo navegación (no configuración)
  lcd.clear();
  
  // Variables para el encoder
  long val_new = encoder.getCount();
  long val_old = val_new;
  char lcd_buffer[17]; // Buffer para formatear texto
  
  // Bucle principal del Modo B
  while (true) {
    // Manejar movimiento del encoder
    val_new = encoder.getCount();
    
    if (val_new > val_old) {  // Giro a la derecha
      sub_menu_modo = sub_menu_modo % 4 + 1;  // Ciclo 1→2→3→4→1
    } 
    else if (val_new < val_old) {  // Giro a la izquierda
      sub_menu_modo = (sub_menu_modo - 2 + 4) % 4 + 1;  // Ciclo 1→4→3→2→1
    }
    val_old = val_new;
          
    // Mostrar contenido fijo (formateado correctamente)
    imprimir_formateado(1, 0, "RPM:", rpm_b, 4);    // RPM
    imprimir_formateado(10, 0, "P:", pwm_b, 2);     // P
    lcd.setCursor(15, 0);
    lcd.write('%');
    imprimir_formateado(1, 1, "T:", tmin_b, 2);     // T
    imprimir_formateado(5, 1, ":", tseg_b, 2);      // 
    imprimir_lcd(10, 1, "Inicio");
    
    // Colocar la flecha/asterisco en la posición actual
    switch(sub_menu_modo) {
      case 1:  // Posición (0,0) - RPM
        if (config_rpm_b == 1) {
          imprimir_lcd(0, 0, "*");
          delay(150);
        } else {
          imprimir_lcd(0, 0, "\4");
          delay(150);
        }
        imprimir_lcd(9, 0, " ");
        imprimir_lcd(0, 1, " ");
        imprimir_lcd(9, 1, " ");
        break;
        
      case 2:  // Posición (9,0) - PWM
        if (config_pwm_b == 1) {
          imprimir_lcd(9, 0, "*");
          delay(150);
        } else {
          imprimir_lcd(9, 0, "\4");
          delay(150);
        }
        imprimir_lcd(0, 0, " ");
        imprimir_lcd(0, 1, " ");
        imprimir_lcd(9, 1, " ");
        break;
        
      case 3:  // Posición (0,1) - Tmin
        if (config_tmin_b == 1) {
          imprimir_lcd(0, 1, "*");
          delay(150);
        } else {
          imprimir_lcd(0, 1, "\4");
          delay(150);
        }
        imprimir_lcd(0, 0, " ");
        imprimir_lcd(9, 0, " ");
        imprimir_lcd(9, 1, " ");
        break;
        
      case 4:  // Posición (9,1) - Inicio
        imprimir_lcd(9, 1, "\4");
        delay(150);
        imprimir_lcd(0, 0, " ");
        imprimir_lcd(9, 0, " ");
        imprimir_lcd(0, 1, " ");
        break;
    }
    
    // Manejar botón Enter para seleccionar/editar
    if (digitalRead(botonEncPin) == LOW) {
      delay(150);
      if (digitalRead(botonEncPin) == LOW) {
        switch(sub_menu_modo) {
          case 1:  // Editar RPM
            config_rpm_b = 1; config_pwm_b = 0; config_tmin_b = 0; config_b = 0;
            imprimir_lcd(0, 0, "*");
            zumbido(1, 100); // Pitido al entrar en edición
            
            // Bucle de edición de RPM
            while (true) {
              val_new = encoder.getCount();
              
              if (val_new > val_old && rpm_b < RPM_MAX_B) {
                rpm_b += PASO_RPM_B;
                Serial.print("RPM B cambiado a: ");
                Serial.println(rpm_b);
              } else if (val_new < val_old && rpm_b > RPM_MIN_B) {
                rpm_b -= PASO_RPM_B;
                Serial.print("RPM B cambiado a: ");
                Serial.println(rpm_b);
              }
              val_old = val_new;
              
              rpm_b = constrain(rpm_b, RPM_MIN_B, RPM_MAX_B);
              imprimir_formateado(1, 0, "RPM:", rpm_b, 4);    // RPM
              
              if (digitalRead(botonEncPin) == LOW) {
                delay(150);
                if (digitalRead(botonEncPin) == LOW) {
                  break;
                }
              }
              delay(50);
            }
            config_rpm_b = 0;
            break;
            
          case 2:  // Editar PWM
            config_rpm_b = 0; config_pwm_b = 1; config_tmin_b = 0; config_b = 0;
            imprimir_lcd(9, 0, "*");
            zumbido(1, 100); // Pitido al entrar en edición
            
            // Bucle de edición de PWM
            while (true) {
              val_new = encoder.getCount();
              
              if (val_new > val_old && pwm_b < PWM_MAX_B) {
                pwm_b += PASO_PWM_B;
                Serial.print("PWM B cambiado a: ");
                Serial.println(pwm_b);
              } else if (val_new < val_old && pwm_b > PWM_MIN_B) {
                pwm_b -= PASO_PWM_B;
                Serial.print("PWM B cambiado a: ");
                Serial.println(pwm_b);
              }
              val_old = val_new;
              
              pwm_b = constrain(pwm_b, PWM_MIN_B, PWM_MAX_B);
              imprimir_formateado(10, 0, "P:", pwm_b, 2);     // P
              
              if (digitalRead(botonEncPin) == LOW) {
                delay(150);
                if (digitalRead(botonEncPin) == LOW) {
                  break;
                }
              }
              delay(50);
            }
            config_pwm_b = 0;
            break;
            
          case 3:  // Editar Tmin
            config_rpm_b = 0; config_pwm_b = 0; config_tmin_b = 1; config_b = 0;
            imprimir_lcd(0, 1, "*");
            zumbido(1, 100); // Pitido al entrar en edición
            
            // Bucle de edición de Tmin
            while (true) {
              val_new = encoder.getCount();
              
              if (val_new > val_old && tmin_b < Tmin_MAX_B) {
                tmin_b += PASO_Tmin_B;
                Serial.print("Tmin B cambiado a: ");
                Serial.println(tmin_b);
              } else if (val_new < val_old && tmin_b > Tmin_MIN_B) {
                tmin_b -= PASO_Tmin_B;
                Serial.print("Tmin B cambiado a: ");
                Serial.println(tmin_b);
              }
              val_old = val_new;
              
              tmin_b = constrain(tmin_b, Tmin_MIN_B, Tmin_MAX_B);
              imprimir_formateado(1, 1, "T:", tmin_b, 2);     // T
              
              if (digitalRead(botonEncPin) == LOW) {
                delay(150);
                if (digitalRead(botonEncPin) == LOW) {
                  break;
                }
              }
              delay(50);
            }
            config_tmin_b = 0;
            break;
            
          case 4:  // INICIO MODO B
            arranque_modos(2); 
            lcd.clear();
            sub_menu_modo = 1;
            return;
        }
      }
    }
    
    // Manejar botón1 para salir
    if (digitalRead(boton1Pin) == HIGH) {
      delay(150);
      if (digitalRead(boton1Pin) == HIGH) {
        menu_modo_b = 0;
        lcd.clear();
        menu_principal();
        return;
      }
    }
    delay(50);  // Pequeña pausa
  }
}

/**
 * Función: modo_c()
 * Descripción: Interfaz para el Modo C (RPM progresivo).
 * Ejecuta una secuencia automática de aumento de RPM.
 */
void modo_c() {
  menu_modo_c = 1;
  lcd.clear();
  
  // Bucle principal del Modo C
  while (true) {
    // Mostrar contenido fijo con flecha estática
    imprimir_formateado(1, 0, "RPM:", rpm_c, 4);    // RPM
    imprimir_formateado(10, 0, "P:", pwm_c, 2);     // P
    lcd.setCursor(15, 0);
    lcd.write('%');
    imprimir_lcd(0, 1, "          ");
    imprimir_lcd(9, 1, "\4");  // Flecha fija en INICIO
    imprimir_lcd(10, 1, "Inicio");
    
    // Manejar botón Enter para iniciar el modo
    if (digitalRead(botonEncPin) == LOW) {
      delay(150);  // Debounce
      if (digitalRead(botonEncPin) == LOW) {
        // Ejecutar secuencia del Modo C
        arranque_modos(3);
        sub_menu_modo = 1;
        return;
      }
    }
    
    // Manejar botón1 para salir
    if (digitalRead(boton1Pin) == HIGH) {
      delay(150);
      if (digitalRead(boton1Pin) == HIGH) {
        menu_modo_c = 0;
        lcd.clear();
        menu_principal();
        return;
      }
    }
    
    delay(50);
  }
}

/**
 * Función: modo_d()
 * Descripción: Interfaz para el Modo D (PWM progresivo).
 * Ejecuta una secuencia automática de aumento de PWM.
 */
void modo_d() {
  menu_modo_d = 1;
  lcd.clear();
  
  // Bucle principal del Modo D
  while (true) {
    // Mostrar contenido fijo con flecha estática
    imprimir_formateado(1, 0, "RPM:", rpm_d, 4);    // RPM
    imprimir_formateado(10, 0, "P:", pwm_d, 2);     // P
    lcd.setCursor(15, 0);
    lcd.write('%');
    imprimir_lcd(0, 1, "          ");
    imprimir_lcd(9, 1, "\4");  // Flecha fija en INICIO
    imprimir_lcd(10, 1, "Inicio");
    
    // Manejar botón Enter para iniciar el modo
    if (digitalRead(botonEncPin) == LOW) {
      delay(150);  // Debounce
      if (digitalRead(botonEncPin) == LOW) {
        // Ejecutar secuencia del Modo D
        arranque_modos(4);  // Usar opción 4 para Modo D****************
        return;
      }
    }
    
    // Manejar botón1 para salir
    if (digitalRead(boton1Pin) == HIGH) {
      delay(150);
      if (digitalRead(boton1Pin) == HIGH) {
        menu_modo_d = 0;
        lcd.clear();
        menu_principal();
        return;
      }
    }
    
    delay(50);
  }
}

/**
 * Función: modo_e()
 * Descripción: Modo de control manual continuo.
 * Permite ajustar RPM y PWM en tiempo real sin temporizador.
 */
void modo_e() {
  // Valores iniciales
  rpm_e = 2500;  // RPM inicial para Modo E
  pwm_e = 50;    // PWM inicial para Modo E
  sub_menu_modo = 1;  // 1=RPM, 2=PWM, 3=Inicio/Parar
  menu_modo_e = 1;
  bool inyectores_activos = false;
  bool editando = false;
  
  // Configuración LEDC
  const int pwmResolution = 10; // 10 bits (0-1023)
  const ledc_channel_t channels[] = {
      LEDC_CHANNEL_0, 
      LEDC_CHANNEL_1, 
      LEDC_CHANNEL_2, 
      LEDC_CHANNEL_3
  };
  const int pins[] = {inyec1Pin, inyec2Pin, inyec3Pin, inyec4Pin};
  
  // Configurar timer LEDC
  ledc_timer_config_t timer_conf = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = (ledc_timer_bit_t)pwmResolution,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = (uint32_t)(rpm_e / 60.0),
      .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);
  
  // Configurar canales
  for(int i = 0; i < 4; i++) {
      ledc_channel_config_t channel_conf = {
          .gpio_num = pins[i],
          .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = channels[i],
          .timer_sel = LEDC_TIMER_0,
          .duty = 0,
          .hpoint = 0
      };
      ledc_channel_config(&channel_conf);
  }

  // Variables para el encoder
  long val_new = encoder.getCount();
  long val_old = val_new;
  
  lcd.clear();
  
  // Bucle principal del Modo E
  while (true) {
    // Manejar movimiento del encoder (solo si no está editando)
    if (!editando) {
      val_new = encoder.getCount();
      if (val_new > val_old) {
        sub_menu_modo = sub_menu_modo % 3 + 1;
        val_old = val_new;
      } else if (val_new < val_old) {
        sub_menu_modo = (sub_menu_modo - 2 + 3) % 3 + 1;
        val_old = val_new;
      }
    }
    
    // Mostrar contenido fijo
      imprimir_formateado(1, 0, "RPM:", rpm_e, 4);
      imprimir_formateado(10, 0, "P:", pwm_e, 2);
      lcd.setCursor(15, 0);
      lcd.print('%');
      
      // Mostrar flecha o asterisco según estado
      lcd.setCursor(0, 0);
      lcd.print(sub_menu_modo == 1 ? (editando ? '*' : '\4') : ' ');
      
      lcd.setCursor(9, 0);
      lcd.print(sub_menu_modo == 2 ? (editando ? '*' : '\4') : ' ');
      
      lcd.setCursor(9, 1);
      lcd.print(sub_menu_modo == 3 ? '\4' : ' ');
      
      // Mostrar Inicio/Parar
      imprimir_lcd(10, 1, inyectores_activos ? "Parar " : "Inicio");
      
      // Parpadeo del cursor en modo edición
      if (editando && (millis() % 1000 < 500)) {
        if (sub_menu_modo == 1) {
          lcd.setCursor(0, 0);
          lcd.print('*');
        } else if (sub_menu_modo == 2) {
          lcd.setCursor(9, 0);
          lcd.print('*');
        }
      }
    
    // Manejar botón Enter
    if (digitalRead(botonEncPin) == LOW) {
      delay(150); // Debounce
      if (digitalRead(botonEncPin) == LOW) {
        zumbido(1, 100);
        
        switch(sub_menu_modo) {
          case 1: // RPM
          case 2: // PWM
            editando = !editando;
            if (editando) {
              encoder.setCount(0); // Resetear encoder para edición
              val_old = 0;
              val_new = 0;
            }
            break;
            
          case 3: // Inicio/Parar
            inyectores_activos = !inyectores_activos;
            if (inyectores_activos) {
              // Configurar frecuencia y duty cycle
              ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, rpm_e / 60.0);
              uint32_t duty = (1023 * pwm_e) / 100;
              for(int i = 0; i < 4; i++) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, channels[i], duty);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, channels[i]);
              }
              zumbido(1, 200);
            } else {
              // Apagar inyectores
              for(int i = 0; i < 4; i++) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, channels[i], 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, channels[i]);
              }
              zumbido(1, 100);
            }
            break;
        }
        while(digitalRead(botonEncPin) == LOW) delay(10); // Esperar a soltar
      }
    }
    
    // Manejar edición de valores
    if (editando) {
      val_new = encoder.getCount();
      if (val_new != val_old) {
        if (sub_menu_modo == 1) { // Editar RPM
          rpm_e = constrain(rpm_e + (val_new > val_old ? 50 : -50), 900, 5000);
          imprimir_formateado(1, 0, "RPM:", rpm_e, 4);
          
          // Si los inyectores están activos, actualizar frecuencia
          if (inyectores_activos) {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, rpm_e / 60.0);
          }
        } 
        else if (sub_menu_modo == 2) { // Editar PWM
          pwm_e = constrain(pwm_e + (val_new > val_old ? 1 : -1), 1, 99);
          imprimir_formateado(10, 0, "P:", pwm_e, 2);
          
          // Si los inyectores están activos, actualizar duty cycle
          if (inyectores_activos) {
            uint32_t duty = (1023 * pwm_e) / 100;
            for(int i = 0; i < 4; i++) {
              ledc_set_duty(LEDC_LOW_SPEED_MODE, channels[i], duty);
              ledc_update_duty(LEDC_LOW_SPEED_MODE, channels[i]);
            }
          }
        }
        val_old = val_new;
      }
    }
    
    // Manejar botón1 para salir
    if (digitalRead(boton1Pin) == HIGH) {
      delay(150);
      if (digitalRead(boton1Pin) == LOW) {
        // Apagar inyectores
        for(int i = 0; i < 4; i++) {
          ledc_stop(LEDC_LOW_SPEED_MODE, channels[i], 0);
        }
        menu_modo_e = 0;
        lcd.clear();
        menu_principal();
        return;
      }
    }
    
    delay(50); // Pequeña pausa
  }
}

// ==================== MODO F (SERVIDOR WEB) ====================

/**
 * Función: modo_f()
 * Descripción: Crea un access point WiFi y servidor web para control remoto.
 * Muestra una interfaz web para controlar los inyectores desde un navegador.
 */
void modo_f() {
    Serial.println("\nIniciando Modo F (AP + Servidor Web)");
    lcd.clear();
    imprimir_lcd(0, 0, "Modo F: AP ESP32");
    imprimir_lcd(0, 1, "Conectate a WiFi");
    delay(1500);

    // 1. Crear Access Point
    WiFi.softAP(apSSID, apPassword);
    Serial.print("SSID: ");
    Serial.println(apSSID);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());

    // Mostrar IP en LCD
    lcd.clear();
    imprimir_lcd(0, 0, "SSID: ESP32-Inyec");
    imprimir_lcd(0, 1, "IP: ");
    imprimir_lcd(4, 1, WiFi.softAPIP().toString().c_str());
    delay(1500);

    // 2. Configurar rutas del servidor
    server.on("/", handleRoot);
    server.on("/set", handleSet);
    server.on("/estado", handleEstado);
    server.onNotFound(handleNotFound);

    // 3. Iniciar servidor
    server.begin();
    zumbido(1, 500);
    lcd.clear();

    // 4. Bucle principal
    while (true) {
        server.handleClient();

        // Actualizar LCD
        imprimir_lcd(0, 0, "IP:");
        imprimir_lcd(3, 0, WiFi.softAPIP().toString().c_str());
        imprimir_lcd(0, 1, "RPM:");
        imprimir_lcd(4, 1, String(rpm_f).c_str());
        imprimir_lcd(9, 1, "P:");
        imprimir_lcd(11, 1, String(pwm_f).c_str());
        lcd.setCursor(13, 1);
        lcd.print("%");

        // Salir si se presiona botón1
        if (digitalRead(boton1Pin) == HIGH) {
            delay(50);
            if (digitalRead(boton1Pin) == HIGH) {
                server.stop();
                WiFi.softAPdisconnect(true);
                lcd.clear();
                zumbido(1, 200);
                menu_principal();
                return;
            }
        }
        delay(10);
    }
}


// ==================== FUNCIONES AUXILIARES ====================

/**
 * Función: zumbido()
 * Descripción: Genera sonidos con el buzzer.
 * Parámetros:
 *   - repeticiones: Número de pitidos
 *   - tiempo_ms: Duración de cada pitido en ms
 */
void zumbido(int repeticiones, int tiempo_ms) {
  for (int i = 0; i < repeticiones; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(tiempo_ms);
    digitalWrite(buzzerPin, LOW);
    delay(tiempo_ms);
  }
}

/**
 * Función: imprimir_lcd()
 * Descripción: Muestra texto en el LCD con formato.
 * Sobrecargada para manejar diferentes tipos de parámetros.
 */
// Función mejorada que maneja tanto texto como caracteres especiales
void imprimir_lcd(uint8_t x, uint8_t y, const char* formato, ...) {
    char buffer[17];
    va_list args;
    va_start(args, formato);
    vsnprintf(buffer, sizeof(buffer), formato, args);
    va_end(args);
    
    lcd.setCursor(x, y);
    lcd.print(buffer);
}

void imprimir_lcd(uint8_t posx, uint8_t posy, uint8_t caracter) {
  lcd.setCursor(posx, posy);
  lcd.write(caracter);
}


/**
 * Función: botones()
 * Descripción: Gestiona las pulsaciones de los botones.
 * Controla la navegación entre menús y la selección de opciones.
 */
void botones() {
    static bool estado_enter_anterior = HIGH;
    
    bool estado_enter_actual = digitalRead(botonEncPin);
    
    if (estado_enter_actual == LOW && estado_enter_anterior == HIGH) {
        delay(100);
        if (menu == 1) {
            switch(indice_seleccion) {
                case 0:
                    Serial.println("Entrando al Modo A");
                    modo_a();
                    break;
                case 1:
                    Serial.println("Entrando al Modo B");
                    modo_b();
                    break;
                case 2:
                    Serial.println("Entrando al Modo C");
                    modo_c();
                    break;
                case 3:
                    Serial.println("Entrando al Modo D");
                    modo_d();
                    break;
                case 4:
                    Serial.println("Entrando al Modo E");
                    modo_e();
                    break;
                case 5:
                    Serial.println("Entrando al Modo F");
                    modo_f();
                    break;
            }
        }
        delay(100);
    }
    
    estado_enter_anterior = estado_enter_actual;
}

// Página HTML (similar a tu versión Python)
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Control Inyector ESP32</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; text-align: center; padding: 20px; background-color: #f0f8ff; color: #003366; margin: 0; }
        .container { max-width: 600px; margin: 0 auto; padding: 20px; background-color: white; border-radius: 15px; box-shadow: 0 4px 8px rgba(0, 100, 200, 0.1); border: 1px solid #d4e6f7; }
        h2 { color: #0066cc; margin-bottom: 25px; border-bottom: 2px solid #b3d9ff; padding-bottom: 10px; }
        .control-group { background-color: #e6f2ff; padding: 15px; border-radius: 10px; margin-bottom: 15px; border-left: 4px solid #4da6ff; }
        label { display: block; margin-bottom: 8px; font-weight: 600; color: #004080; }
        input[type=range] { width: 100%; height: 10px; -webkit-appearance: none; background: linear-gradient(to right, #b3d9ff, #0066cc); border-radius: 5px; outline: none; margin: 10px 0; }
        input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 20px; height: 20px; background: #0066cc; border-radius: 50%; cursor: pointer; border: 2px solid white; box-shadow: 0 0 3px rgba(0,0,0,0.3); }
        .value { font-weight: bold; font-size: 1.1em; color: #004d99; display: inline-block; min-width: 50px; }
        button { padding: 12px 25px; font-size: 16px; margin: 10px 5px; border: none; border-radius: 25px; cursor: pointer; font-weight: 600; transition: all 0.3s ease; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        #toggleBtn { background-color: #0066cc; color: white; width: 120px; }
        #toggleBtn:hover { background-color: #004d99; }
        button:not(#toggleBtn) { background-color: #4da6ff; color: white; }
        button:not(#toggleBtn):hover { background-color: #1a8cff; }
        .injector-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin: 15px 0; }
        .injector-column { display: flex; flex-direction: column; gap: 10px; }
        .injector-option { display: flex; align-items: center; transition: all 0.3s ease; padding: 5px 10px; border-radius: 20px; }
        .injector-option:hover { background-color: #d4e6f7; }
        input[type=checkbox] { -webkit-appearance: none; width: 22px; height: 22px; background: white; border: 2px solid #4da6ff; border-radius: 4px; margin-right: 8px; position: relative; cursor: pointer; transition: all 0.2s ease; }
        input[type=checkbox]:checked { background-color: #ff4d4d; border-color: #ff1a1a; }
        input[type=checkbox]:checked::after { content: "X"; color: white; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); font-size: 14px; font-weight: bold; }
        .injector-label { color: #666; transition: all 0.3s ease; }
        input[type=checkbox]:checked + .injector-label { color: #0066cc; font-weight: bold; }
    </style>
</head>
<body>
    <div class="container">
        <h2>Control de Inyectores ESP32</h2>
        
        <div class="control-group">
            <label for="rpm">RPM: <span id="rpm_val" class="value">%RPM%</span></label>
            <input type="range" id="rpm" min="900" max="5000" value="%RPM%" step="50" oninput="updateLabel('rpm')">
        </div>
        
        <div class="control-group">
            <label for="pwm">PWM (%): <span id="pwm_val" class="value">%PWM%</span></label>
            <input type="range" id="pwm" min="1" max="99" value="%PWM%" oninput="updateLabel('pwm')">
        </div>
        
        <div class="control-group">
            <label>Seleccion de Inyectores:</label>
            <div class="injector-grid">
                <div class="injector-column">
                    <label class="injector-option">
                        <input type="checkbox" id="iny1" %INY1_CHECKED%>
                        <span class="injector-label">INY1</span>
                    </label>
                    <label class="injector-option">
                        <input type="checkbox" id="iny2" %INY2_CHECKED%>
                        <span class="injector-label">INY2</span>
                    </label>
                </div>
                <div class="injector-column">
                    <label class="injector-option">
                        <input type="checkbox" id="iny3" %INY3_CHECKED%>
                        <span class="injector-label">INY3</span>
                    </label>
                    <label class="injector-option">
                        <input type="checkbox" id="iny4" %INY4_CHECKED%>
                        <span class="injector-label">INY4</span>
                    </label>
                </div>
            </div>
        </div>
        
        <button onclick="sendValues()">Actualizar Valores</button>
        
        <div style="margin-top: 20px;">
            <button id="toggleBtn" onclick="toggleState()">%BOTON_TEXTO%</button>
        </div>
        
        <div id="statusMsg" class="status"></div>
    </div>

    <script>
        let estado = %ESTADO_JS%;
        const toggleBtn = document.getElementById('toggleBtn');
        if(estado) {
            toggleBtn.innerText = "Detener Inyectores";
            toggleBtn.style.backgroundColor = "#cc3300";
        }

        function updateLabel(id) {
            document.getElementById(id + "_val").innerText = document.getElementById(id).value;
        }

        function showStatus(message, isSuccess) {
            const statusElement = document.getElementById('statusMsg');
            statusElement.textContent = message;
            statusElement.className = isSuccess ? 'status success' : 'status error';
            statusElement.style.display = 'block';
            
            setTimeout(() => {
                statusElement.style.display = 'none';
            }, 3000);
        }

        function sendValues() {
            const rpm = document.getElementById('rpm').value;
            const pwm = document.getElementById('pwm').value;
            const iny1 = document.getElementById('iny1').checked ? 1 : 0;
            const iny2 = document.getElementById('iny2').checked ? 1 : 0;
            const iny3 = document.getElementById('iny3').checked ? 1 : 0;
            const iny4 = document.getElementById('iny4').checked ? 1 : 0;
            
            fetch(`/set?rpm=${rpm}&pwm=${pwm}&iny1=${iny1}&iny2=${iny2}&iny3=${iny3}&iny4=${iny4}`)
                .then(res => {
                    showStatus("¡Valores actualizados correctamente!", true);
                    console.log("Valores enviados");
                })
                .catch(err => {
                    showStatus("Error al enviar los valores", false);
                    console.log("Error:", err);
                });
        }

        function toggleState() {
            estado = !estado;
            toggleBtn.innerText = estado ? "Detener Inyectores" : "Activar Inyectores";
            toggleBtn.style.backgroundColor = estado ? "#cc3300" : "#0066cc";
            
            fetch(`/estado?valor=${estado ? 1 : 0}`)
                .then(res => {
                    showStatus(estado ? "Inyectores activados" : "Inyectores detenidos", true);
                    console.log("Estado actualizado");
                })
                .catch(err => {
                    showStatus("Error al cambiar estado", false);
                    console.log("Error:", err);
                });
        }
    </script>
</body>
</html>
)rawliteral";

// ==================== FUNCIONES DEL SERVIDOR WEB ====================

/**
 * Función: handleRoot()
 * Descripción: Maneja la petición a la raíz del servidor web.
 * Devuelve la página HTML con la interfaz de control.
 */
void handleRoot() {
    String html = HTML;
    html.replace("%RPM%", String(rpm_f));
    html.replace("%PWM%", String(pwm_f));
    html.replace("%BOTON_TEXTO%", estado ? "Detener Inyectores" : "Activar Inyectores");
    html.replace("%ESTADO_JS%", estado ? "true" : "false");
    
    // Checkboxes de inyectores
    html.replace("%INY1_CHECKED%", inyec[0] ? "checked" : "");
    html.replace("%INY2_CHECKED%", inyec[1] ? "checked" : "");
    html.replace("%INY3_CHECKED%", inyec[2] ? "checked" : "");
    html.replace("%INY4_CHECKED%", inyec[3] ? "checked" : "");

    server.send(200, "text/html", html);
}

/**
 * Función: handleSet()
 * Descripción: Procesa los parámetros de configuración enviados desde la web.
 * Actualiza RPM, PWM y estado de los inyectores.
 */
void handleSet() {
    if (server.hasArg("rpm")) rpm_f = server.arg("rpm").toInt();
    if (server.hasArg("pwm")) pwm_f = server.arg("pwm").toInt();
    
    // Actualizar estado de inyectores
    for (int i = 0; i < 4; i++) {
        String argName = "iny" + String(i+1);
        if (server.hasArg(argName)) {
            inyec[i] = (server.arg(argName).toInt() == 1);
        }
    }
    
    // Aplicar cambios si están activos
    if (estado) {
        activar_inyectores();
    }
    
    server.send(200, "text/plain", "OK");
}

/**
 * Función: handleEstado()
 * Descripción: Controla el estado general (activar/desactivar) desde la web.
 */
void handleEstado() {
    if (server.hasArg("valor")) {
        estado = (server.arg("valor").toInt() == 1);
        if (estado) {
            activar_inyectores();
        } else {
            desactivar_inyectores();
        }
    }
    server.send(200, "text/plain", "OK");
}

/**
 * Función: handleNotFound()
 * Descripción: Maneja las rutas no encontradas en el servidor web.
 */
void handleNotFound() {
    server.send(404, "text/plain", "404: Pagina no encontrada");
}

/**
 * Función: activar_inyectores()
 * Descripción: Activa los inyectores según la configuración actual.
 * Usa PWM por hardware para un control preciso.
 */
void activar_inyectores() {
    // Configuración LEDC para ESP32 (versión corregida)
    const int pwmResolution = 10; // 10 bits (0-1023)
    const ledc_channel_t channels[] = {
        LEDC_CHANNEL_0, // Inyector 1
        LEDC_CHANNEL_1, // Inyector 2
        LEDC_CHANNEL_2, // Inyector 3
        LEDC_CHANNEL_3  // Inyector 4
    };
    const int pins[] = {inyec1Pin, inyec2Pin, inyec3Pin, inyec4Pin};

    // Configurar timer (solo una vez)
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)pwmResolution,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = (uint32_t)(rpm_f / 60.0), // Convertir RPM a Hz
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // Configurar canales y activar PWM
    uint32_t duty = map(pwm_f, 0, 100, 0, 1023); // Mapear % PWM a resolución de 10 bits
    
    for(int i = 0; i < 4; i++) {
        if(inyec[i]) { // Solo configurar inyectores activados
            ledc_channel_config_t channel_conf = {
                .gpio_num = pins[i],
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = channels[i],
                .timer_sel = LEDC_TIMER_0,
                .duty = duty,
                .hpoint = 0
            };
            ledc_channel_config(&channel_conf);
        } else {
            ledc_stop(LEDC_LOW_SPEED_MODE, channels[i], 0); // Apagar canal
        }
    }
    Serial.println("Inyectores activados");
}

/**
 * Función: desactivar_inyectores()
 * Descripción: Apaga todos los inyectores.
 */
void desactivar_inyectores() {
    const ledc_channel_t channels[] = {
        LEDC_CHANNEL_0, LEDC_CHANNEL_1, 
        LEDC_CHANNEL_2, LEDC_CHANNEL_3
    };
    
    for(int i = 0; i < 4; i++) {
        ledc_stop(LEDC_LOW_SPEED_MODE, channels[i], 0);
    }
    Serial.println("Inyectores desactivados");
}

// ==================== LOOP PRINCIPAL ====================
/**
 * Función: loop()
 * Descripción: Bucle principal del programa.
 * Gestiona la navegación con el encoder y las pulsaciones de botones.
 */
void loop() {
    static long oldPosition = encoder.getCount();  // Inicializar con posición actual
    
    // Manejar movimiento del encoder
    long newPosition = encoder.getCount();
    
    if (newPosition != oldPosition) {
        // Solo procesar cambios si estamos en el menú principal
        if (menu == 1) {  // Variable global que indica que estamos en el menú principal
            if (newPosition > oldPosition) {
                indice_seleccion = (indice_seleccion + 1) % num_opciones;
            } else {
                indice_seleccion = (indice_seleccion - 1 + num_opciones) % num_opciones;
            }
            
            // Actualizar menú solo si hubo cambio
            if (indice_seleccion != opcion_anterior) {
                menu_principal();
                opcion_anterior = indice_seleccion;
            }
        }
        oldPosition = newPosition;
    }
    
    // Manejar botones
    botones();
    delay(10);  // Pequeña pausa para evitar rebotes
}
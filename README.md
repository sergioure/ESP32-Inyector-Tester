# ESP32_Injector_tester

# ESP32 Fuel Injector Tester 🚗💨

<img width="713" height="489" alt="image" src="https://github.com/user-attachments/assets/ca423f54-f8e9-4ca2-9b0c-2d547cea1459" />

## 1. Introducción
Dispositivo para probar y controlar inyectores de combustible mediante señales PWM. Simula condiciones de operación (RPM, ciclo de trabajo, tiempo) e incluye un servidor web para control remoto.

### 🔑 Características clave
- ✅ **6 modos de operación** (A-F)
- ✅ Control por **encoder rotatorio + botón**
- ✅ Visualización en **LCD 16x2**
- ✅ **Control WiFi** desde navegador (smartphone/PC)
- ✅ Activación **individual o simultánea** de inyectores
- ✅ Barra de progreso en tiempo real

---

## 2. Componentes y Conexiones
### 🔌 Conexiones de inyectores
| Pin ESP32 | Inyector | Color recomendado |
|-----------|----------|-------------------|
| 16        | 1        | Amarillo          |
| 17        | 2        | Verde             |
| 18        | 3        | Azul              |
| 19        | 4        | Rojo              |

> ⚠️ **Verificar polaridad** antes de conectar los inyectores.

### 🎛️ Controles físicos
| Componente       | Función                                  |
|------------------|------------------------------------------|
| Encoder rotatorio| Navegar/ajustar valores                  |
| Botón central    | Confirmar selección                      |
| Botón 1 (↩)      | Volver al menú anterior                  |
| Botón 2          | *(Reservado para futuras funciones)*     |

---

## 3. Modos de Operación
### 📌 Menú Principal

### 🔧 **Modo A**: Activación Simultánea
- Todos los inyectores se activan al mismo tiempo
- **Ajustes**: RPM (900-5000), PWM (1-99%), Tiempo (MM:SS)

### 🔄 **Modo B**: Activación Secuencial
- Inyectores se activan en secuencia (1→2→3→4)
- Configuración individual por inyector

### 📈 **Modo C**: RPM Progresivo
- Aumenta RPM automáticamente desde 900 hasta 5000
- Ideal para pruebas de respuesta en diferentes regímenes

### ⚡ **Modo D**: PWM Progresivo
- Incrementa ciclo de trabajo del 1% al 99%
- Perfecto para calibración de inyectores

### 🎮 **Modo E**: Control Manual
- Ajuste en tiempo real de RPM/PWM
- Activación manual con botón

### 🌐 **Modo F**: Control Web (WiFi)
1. Conéctate a la red:  
   - **SSID**: `ESP32-Inyectores`  
   - **Password**: `12345678`
2. Abre navegador en: `192.168.4.1`
3. Interfaz web permite:
   - Control individual de inyectores
   - Ajuste remoto de parámetros

---

## 4. Indicadores LCD
| Símbolo  | Significado               |
|----------|---------------------------|
| ➤       | Opción seleccionada       |
| *        | Modo edición activo       |
| T:00:00  | Temporizador              |
| Inj:X    | Inyector activo (1-4)     |

---

## 5. ⚠️ Seguridad
- No exceder límites de voltaje/corriente
- Usar fuente de alimentación estable (12V recomendado)
- Verificar conexiones antes de energizar
- Desconectar después de usar

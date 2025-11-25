#include "config.h"
// #include "functions.h"

// ============ VARIABLES GLOBALES ============
QueueHandle_t adc_data_queue;
QueueHandle_t uart_command_queue;
SemaphoreHandle_t mux_control_mutex;
adc_oneshot_unit_handle_t adc_handle;
esp_timer_handle_t adc_timer;

mux_control_t mux_control = {
    .current_state = MUX_STATE_00,
    .running = false,
    .manual_mode = false,
    .delay_ms = MUX_DEFAULT_DELAY_MS,
    .state_changed = false
};

// ============ INICIALIZACIÓN DEL SISTEMA ============
void system_init(void) {
    ESP_LOGI(TAG, "Iniciando sistema ECG...");
    
    queues_init();
    mutexes_init();
    adc_init();
    uart_init();
    gpio_init();
    timers_init();
    
    ESP_LOGI(TAG, "Sistema ECG inicializado correctamente");
}

void queues_init(void) {
    adc_data_queue = xQueueCreate(ADC_QUEUE_SIZE, sizeof(adc_data_t));
    uart_command_queue = xQueueCreate(10, sizeof(uart_command_t));
    
    if (adc_data_queue == NULL || uart_command_queue == NULL) {
        ESP_LOGE(TAG, "Error creando colas");
        return;
    }
    
    ESP_LOGI(TAG, "Colas creadas exitosamente");
}

void mutexes_init(void) {
    mux_control_mutex = xSemaphoreCreateMutex();
    
    if (mux_control_mutex == NULL) {
        ESP_LOGE(TAG, "Error creando mutex MUX");
        return;
    }
    
    ESP_LOGI(TAG, "Mutex creado exitosamente");
}

void adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_USED,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    adc_oneshot_new_unit(&init_config, &adc_handle);
    
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_USED,
        .atten = ADC_ATTEN_USED,
    };
    
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config);
    
    ESP_LOGI(TAG, "ADC inicializado - Canal: %d, Resolución: 12 bits", ADC_CHANNEL);
}

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_BITS_USED,
        .parity = UART_PARITY_USED,
        .stop_bits = UART_STOP_BITS_USED,
        .flow_ctrl = UART_FLOW_CTRL_USED,
    };
    
    uart_driver_install(UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 20, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    
    ESP_LOGI(TAG, "UART inicializado - Baudrate: %d", UART_BAUD_RATE);
}

void gpio_init(void) {
    // Configurar pines del multiplexor como salida
    gpio_set_direction(MUX_PIN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MUX_PIN_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    // Estado inicial del multiplexor
    gpio_set_level(MUX_PIN_A, 0);
    gpio_set_level(MUX_PIN_B, 0);
    gpio_set_level(LED_PIN, 0);
    
    ESP_LOGI(TAG, "GPIOs inicializados");
}

void timers_init(void) {
    const esp_timer_create_args_t adc_timer_args = {
        .callback = &adc_timer_callback,
        .name = "adc_timer"
    };
    
    esp_timer_create(&adc_timer_args, &adc_timer);
    esp_timer_start_periodic(adc_timer, 1000000 / SAMPLE_RATE_HZ); // Microsegundos
    
    ESP_LOGI(TAG, "Timer ADC iniciado - Frecuencia: %d Hz", SAMPLE_RATE_HZ);
}

// ============ CALLBACKS DE TIMER ============
static void IRAM_ATTR adc_timer_callback(void* arg) {
    adc_data_t adc_data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Leer ADC
    adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_data.raw_value);
    adc_data.timestamp = esp_timer_get_time();
    
    // Enviar a cola sin bloquear
    xQueueSendFromISR(adc_data_queue, &adc_data, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// ============ TAREAS PRINCIPALES ============
void adc_acquisition_task(void* pvParameters) {
    adc_data_t adc_data;
    
    ESP_LOGI(TAG, "Tarea de adquisición ADC iniciada en Core %d", xPortGetCoreID());
    
    while (1) {
        if (xQueueReceive(adc_data_queue, &adc_data, pdMS_TO_TICKS(QUEUE_TIMEOUT_MS)) == pdTRUE) {
            // Enviar datos sin filtrado (procesamiento en Python)
            send_data_packet(adc_data.raw_value);
        }
        
        // Permitir que otras tareas se ejecuten
        taskYIELD();
    }
}

void uart_communication_task(void* pvParameters) {
    uint8_t uart_buffer[RD_BUF_SIZE];
    uart_command_t command;
    
    ESP_LOGI(TAG, "Tarea de comunicación UART iniciada en Core %d", xPortGetCoreID());
    
    while (1) {
        // Leer datos UART con timeout corto para no bloquear
        int len = uart_read_bytes(UART_PORT, uart_buffer, RD_BUF_SIZE - 1, pdMS_TO_TICKS(UART_CMD_TIMEOUT_MS));
        
        if (len > 0) {
            parse_uart_data(uart_buffer, len);
        }
        
        // Procesar comandos en cola
        if (xQueueReceive(uart_command_queue, &command, 0) == pdTRUE) {
            process_mux_command(command.command);
        }
        
        // Pequeño delay para no saturar la CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void mux_control_task(void* pvParameters) {
    ESP_LOGI(TAG, "Tarea de control MUX iniciada en Core %d", xPortGetCoreID());
    
    while (1) {
        if (xSemaphoreTake(mux_control_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
            if (mux_control.running) {
                if (mux_control.state_changed) {
                    // Cambio inmediato de estado
                    set_mux_state(mux_control.current_state);
                    mux_control.state_changed = false;
                    ESP_LOGI(TAG, "Estado MUX cambiado inmediatamente a: %d", mux_control.current_state);
                }
                
                if (!mux_control.manual_mode) {
                    // Modo automático - avanzar estado después del delay
                    mux_control.current_state = (mux_control.current_state + 1) % 4;
                    set_mux_state(mux_control.current_state);
                }
                
                led_blink();
                xSemaphoreGive(mux_control_mutex);
                vTaskDelay(pdMS_TO_TICKS(mux_control.delay_ms));
            } else {
                xSemaphoreGive(mux_control_mutex);
                vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar más tiempo cuando está detenido
            }
        } else {
            ESP_LOGW(TAG, "No se pudo obtener mutex MUX");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ============ FUNCIONES DE CONTROL ============
void set_mux_state(mux_state_t state) {
    switch(state) {
        case MUX_STATE_00:
            gpio_set_level(MUX_PIN_A, 0);
            gpio_set_level(MUX_PIN_B, 0);
            ESP_LOGI(TAG, "MUX -> Lead I: A(0), B(0)");
            break;
        case MUX_STATE_01:
            gpio_set_level(MUX_PIN_A, 0);
            gpio_set_level(MUX_PIN_B, 1);
            ESP_LOGI(TAG, "MUX -> Lead II: A(0), B(1)");
            break;
        case MUX_STATE_10:
            gpio_set_level(MUX_PIN_A, 1);
            gpio_set_level(MUX_PIN_B, 0);
            ESP_LOGI(TAG, "MUX -> Lead III: A(1), B(0)");
            break;
        case MUX_STATE_11:
            gpio_set_level(MUX_PIN_A, 1);
            gpio_set_level(MUX_PIN_B, 1);
            ESP_LOGI(TAG, "MUX -> aVF: A(1), B(1)");
            break;
        default:
            ESP_LOGW(TAG, "Estado MUX desconocido: %d", state);
            break;
    }
}

void process_mux_command(const char* command) {
    if (xSemaphoreTake(mux_control_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        ESP_LOGI(TAG, "Procesando comando: %s", command);
        
        if (strncmp(command, "START", 5) == 0) {
            mux_control.running = true;
            mux_control.manual_mode = false;
            ESP_LOGI(TAG, "MUX modo automático iniciado");
            
        } else if (strncmp(command, "STOP", 4) == 0) {
            mux_control.running = false;
            ESP_LOGI(TAG, "MUX detenido");
            
        } else if (strncmp(command, "STATE", 5) == 0) {
            if (strlen(command) >= 7) {
                int state = command[6] - '0';
                if (state >= 0 && state <= 3) {
                    mux_control.current_state = (mux_state_t)state;
                    mux_control.running = true;
                    mux_control.manual_mode = true;
                    mux_control.state_changed = true; // Marcar para cambio inmediato
                    ESP_LOGI(TAG, "MUX estado manual: %d", state);
                }
            }
            
        } else if (strncmp(command, "DELAY", 5) == 0) {
            if (strlen(command) >= 7) {
                int delay = atoi(&command[6]);
                if (delay >= MUX_MIN_DELAY_MS && delay <= MUX_MAX_DELAY_MS) {
                    mux_control.delay_ms = delay;
                    ESP_LOGI(TAG, "MUX delay: %lu ms", (unsigned long)delay);
                }
            }
            
        } else if (strncmp(command, "STATUS", 6) == 0) {
            ESP_LOGI(TAG, "MUX - Running:%s, Manual:%s, State:%d, Delay:%lu ms",
                mux_control.running ? "SI" : "NO",
                mux_control.manual_mode ? "SI" : "NO",
                (int)mux_control.current_state,
                (unsigned long)mux_control.delay_ms);
        }
        
        xSemaphoreGive(mux_control_mutex);
    }
}

// ============ FUNCIONES AUXILIARES ============
void led_blink(void) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // Blink más rápido
    gpio_set_level(LED_PIN, 0);
}

void send_data_packet(int adc_value) {
    uint8_t packet[DATA_PACKET_SIZE];
    
    packet[0] = DATA_START_BYTE;           // Start byte
    packet[1] = adc_value & 0xFF;          // LSB
    packet[2] = (adc_value >> 8) & 0xFF;   // MSB
    packet[3] = packet[0] ^ packet[1] ^ packet[2]; // Checksum
    
    uart_write_bytes(UART_PORT, (const char*)packet, DATA_PACKET_SIZE);
}

void parse_uart_data(uint8_t* data, int length) {
    static char cmd_buffer[32];
    int cmd_len = 0;
    
    // Extraer comando limpio (sin CR/LF/espacios)
    for (int i = 0; i < length && cmd_len < 31; i++) {
        if (data[i] != '\r' && data[i] != '\n' && data[i] != ' ') {
            cmd_buffer[cmd_len++] = data[i];
        }
    }
    cmd_buffer[cmd_len] = '\0';
    
    if (cmd_len > 0) {
        uart_command_t command;
        strncpy(command.command, cmd_buffer, sizeof(command.command) - 1);
        command.command[sizeof(command.command) - 1] = '\0';
        command.timestamp = esp_timer_get_time();
        
        // Enviar a cola de comandos (no bloqueante)
        xQueueSend(uart_command_queue, &command, 0);
    }
}
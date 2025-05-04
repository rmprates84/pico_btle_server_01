#include <stdio.h>                  // Biblioteca padrão para entrada e saída
#include "pico/stdlib.h"            // Biblioteca padrão da Pico para GPIO, temporização, etc.
#include "pico/cyw43_arch.h"        // Biblioteca de arquitetura CYW43 para Raspberry Pi Pico
#include "hardware/adc.h"           // Biblioteca para controle do ADC (Conversor Analógico-Digital)
#include "btstack.h"                // Biblioteca para manipulação da pilha Bluetooth (BTstack)
#include "pico/btstack_cyw43.h"     // Biblioteca para integrar BTstack com CYW43
#include "temp_sensor.h"            // Arquivo .gatt define a estrutura e os atributos de um perfil de dispositivo BLE


//Definições iniciais de execução
#define ADC_CHANNEL_TEMPSENSOR 4
#define APP_AD_FLAGS 0x06
#define HEARTBEAT_PERIOD_MS 1000 // Define o período de batimento cardíaco em milissegundos

static uint8_t adv_data[] = {
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Nome do periférico
    0x17, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', ' ', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0',
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1a, 0x18,
};
static const uint8_t adv_data_len = sizeof(adv_data);
int le_notification_enabled;
hci_con_handle_t con_handle;
uint16_t current_temp;
extern int le_notification_enabled;
extern hci_con_handle_t con_handle;
extern uint16_t current_temp;
extern uint8_t const profile_data[];

// Variáveis estáticas e estruturas usadas no programa
static btstack_timer_source_t heartbeat; // Estrutura para gerenciar o temporizador
static btstack_packet_callback_registration_t hci_event_callback_registration; // Registro de callback para eventos HCI (Host Controller Interface)

// Variáveis para conexão Bluetooth
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID; // Handle da conexão Bluetooth, inicializado como inválido
static int notification_enabled = 0; // Flag para indicar se notificações estão habilitadas

// servidor ATT (Attribute Protocol) - Protocolo de leitura
uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
// Servidor ATT (Attribute Protocol) - Protocolo de escrita
int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
// Callback para pacotes do servidor ATT
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
// Coleta dos dados de temperatura do sensor interno do RP2040
void poll_temp(void);
// Função chamada a cada batimento cardíaco
static void heartbeat_handler(struct btstack_timer_source *ts);

int main()
{
    // Inicializa a entrada/saída padrão (printf, scanf, etc.)
    stdio_init_all(); 

    // Inicializa o Periférico wifi - bluetooth / CYW43
    if (cyw43_arch_init()) { // Inicializa a arquitetura CYW43
        printf("failed to initialise cyw43_arch\n"); // Mensagem de erro caso falhe
        return -1; // Finaliza o programa com erro
    }

    // Conversor analógico digital - sensor de temperatura
    adc_init(); // Inicializa o ADC para uso do sensor de temperatura
    adc_select_input(ADC_CHANNEL_TEMPSENSOR); // Seleciona o canal do sensor de temperatura
    adc_set_temp_sensor_enabled(true); // Habilita o sensor de temperatura interno

    //Protocolos bluetooth
    l2cap_init(); // Inicializa o protocolo L2CAP (Logical Link Control and Adaptation Protocol)
    sm_init(); // Inicializa o Security Manager (gerenciamento de segurança Bluetooth)
    
    // Configura o servidor ATT (Attribute Protocol), com callbacks
    att_server_init(profile_data, att_read_callback, att_write_callback);  
    
    // Registra o callback para eventos HCI
    // Eventos HCI são notificações do controlador ao host sobre a ocorrência de eventos.
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Registra o callback para pacotes do servidor ATT
    att_server_register_packet_handler(packet_handler);

    // Configura e adiciona o temporizador de batimento cardíaco
    heartbeat.process = &heartbeat_handler; // Define a função de processamento do batimento
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS); // Configura o temporizador
    btstack_run_loop_add_timer(&heartbeat); // Adiciona o temporizador à fila

    // Liga o controlador HCI para ativar o Bluetooth
    hci_power_control(HCI_POWER_ON); 

    /*
    Código condicional que permite alternar entre usar `btstack_run_loop_execute()`
    ou um loop manual que apenas dorme (desativado por padrão).
    */
    #if 1 
        btstack_run_loop_execute(); // Inicia o loop de execução principal do BTstack
    #else
        while (true) { // Loop infinito alternativo
        printf("Teste de bluetooth - v2!\n");
        sleep_ms(1000); // Aguarda 1 segundo
    }
    #endif

    return 0; // Finaliza o programa com sucesso

}

//------------------------------------- Funções --------------------------------------------------------

uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);

    if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE){
        return att_read_callback_handle_blob((const uint8_t *)&current_temp, sizeof(current_temp), offset, buffer, buffer_size);
    }
    return 0;
}

int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(buffer_size);
    
    if (att_handle != ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE) return 0;
    le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    con_handle = connection_handle;
    if (le_notification_enabled) {
        att_server_request_can_send_now_event(con_handle);
    }
    return 0;
}

//callback para pacotes do servidor ATT
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    switch(event_type){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            gap_local_bd_addr(local_addr);
            printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));

            // setup advertisements
            uint16_t adv_int_min = 800;
            uint16_t adv_int_max = 800;
            uint8_t adv_type = 0;
            bd_addr_t null_addr;
            memset(null_addr, 0, 6);
            gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
            assert(adv_data_len <= 31); // ble limitation
            gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
            gap_advertisements_enable(1);

            poll_temp();

            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            le_notification_enabled = 0;
            break;
        case ATT_EVENT_CAN_SEND_NOW:
            att_server_notify(con_handle, ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE, (uint8_t*)&current_temp, sizeof(current_temp));
            break;
        default:
            break;
    }
}

// Coleta dos dados de temperatura do sensor interno do RP2040
void poll_temp(void) {
    adc_select_input(ADC_CHANNEL_TEMPSENSOR);
    uint32_t raw32 = adc_read();
    const uint32_t bits = 12;

    // Scale raw reading to 16 bit value using a Taylor expansion (for 8 <= bits <= 16)
    uint16_t raw16 = raw32 << (16 - bits) | raw32 >> (2 * bits - 16);

    // ref https://github.com/raspberrypi/pico-micropython-examples/blob/master/adc/temperature.py
    const float conversion_factor = 3.3 / (65535);
    float reading = raw16 * conversion_factor;
    
    // The temperature sensor measures the Vbe voltage of a biased bipolar diode, connected to the fifth ADC channel
    // Typically, Vbe = 0.706V at 27 degrees C, with a slope of -1.721mV (0.001721) per degree. 
    float deg_c = 27 - (reading - 0.706) / 0.001721;
    current_temp = deg_c * 100;
    printf("Write temp %.2f degc\n", deg_c);
 }

 // Função chamada a cada batimento cardíaco
static void heartbeat_handler(struct btstack_timer_source *ts) {
    static uint32_t counter = 0; // Contador para rastrear os ciclos do temporizador
    counter++; // Incrementa o contador

    // Atualiza a temperatura a cada 10 batimentos
    if (counter % 10 == 0) {
        poll_temp(); // Função para ler a temperatura
        if (le_notification_enabled) { // Verifica se notificações estão habilitadas
            att_server_request_can_send_now_event(con_handle); // Solicita permissão para enviar dados
        }
    }

    // Inverte o estado do LED
    static int led_on = true; // Variável para armazenar o estado do LED
    led_on = !led_on; // Alterna o estado
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on); // Atualiza o estado do LED GPIO

    // Reinicia o temporizador
    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS); // Configura o próximo batimento
    btstack_run_loop_add_timer(ts); // Adiciona o temporizador à fila de execução
}
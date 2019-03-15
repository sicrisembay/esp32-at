#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "bt_zplBridge.h"
#include "driver/uart.h"
#include "freertos/ringbuf.h"

#include "time.h"
#include "sys/time.h"

#define ZPL_BRIDGE_TAG "ZPL_BT_BRIDGE"
#define SPP_SERVER_NAME "SPP_SERVER"
#define ZPL_BRIDGE_NAME "ZPL_BT_BRIDGE"

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static uint32_t bt_initiator_hdl = 0;
static RingbufHandle_t bridgeTxBuf_handle;
static RingbufHandle_t bridgeRxBuf_handle;
static bool bCongest = false;
static bool bBtWriteDone = true;

static esp_err_t _bridgeUartInit(void);
static void _TxUartTask(void *pxParam);
static void _RxUartTask(void *pxParam);

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(ZPL_BRIDGE_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(ZPL_BRIDGE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(ZPL_BRIDGE_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(ZPL_BRIDGE_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(ZPL_BRIDGE_TAG, "ESP_SPP_CLOSE_EVT");
        ESP_LOGI(ZPL_BRIDGE_TAG, "Client Handle=%d", param->close.handle);
        bt_initiator_hdl = 0;
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(ZPL_BRIDGE_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(ZPL_BRIDGE_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(ZPL_BRIDGE_TAG, "ESP_SPP_DATA_IND_EVT (Client Hdl=%d, data=%d)", param->data_ind.handle, *(param->data_ind.data));
        if(xRingbufferGetCurFreeSize(bridgeTxBuf_handle) >= param->data_ind.len) {
            xRingbufferSend(bridgeTxBuf_handle, param->data_ind.data, param->data_ind.len, (TickType_t)0);
        }
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(ZPL_BRIDGE_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(ZPL_BRIDGE_TAG, "ESP_SPP_WRITE_EVT");
        bBtWriteDone = true;
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(ZPL_BRIDGE_TAG, "ESP_SPP_SRV_OPEN_EVT");
        ESP_LOGI(ZPL_BRIDGE_TAG, "client handle=%d new handle=%d", param->srv_open.handle, param->srv_open.new_listen_handle);
        bt_initiator_hdl = param->srv_open.handle;
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(ZPL_BRIDGE_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(ZPL_BRIDGE_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(ZPL_BRIDGE_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(ZPL_BRIDGE_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(ZPL_BRIDGE_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(ZPL_BRIDGE_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
    default: {
        ESP_LOGI(ZPL_BRIDGE_TAG, "event: %d", event);
        break;
    }
    }
    return;
}


void bt_bridge_init(void)
{
    esp_bt_cod_t cod;
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    if((ret = _bridgeUartInit()) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s bridge Uart Init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    /* refer to
     * https://www.bluetooth.com/specifications/assigned-numbers/baseband
     */
    cod.major = 0b00001;
    cod.minor = 0b000101;
    cod.service = 0b00000010000;
    if ((ret = esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s Failed setting COD: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
}

static esp_err_t _bridgeUartInit(void)
{
    esp_err_t ret = ESP_OK;
    const uart_config_t uart_config = {
            .baud_rate = CONFIG_BRIDGE_UART_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    bridgeTxBuf_handle = xRingbufferCreate(CONFIG_BRIDGE_TX_BUFF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if(bridgeTxBuf_handle == NULL) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s bridge tx buff creation failed: %s\n", __func__, "ESP_ERR_NO_MEM");
        return ESP_ERR_NO_MEM;
    }

    bridgeRxBuf_handle = xRingbufferCreate(CONFIG_BRIDGE_RX_BUFF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if(bridgeRxBuf_handle == NULL) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s bridge rx buff creation failed: %s\n", __func__, "ESP_ERR_NO_MEM");
        return ESP_ERR_NO_MEM;
    }

    if((ret = uart_param_config(CONFIG_BRIDGE_UART_PORT, &uart_config)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s uart config failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    if((ret = uart_set_pin(CONFIG_BRIDGE_UART_PORT,
                 CONFIG_BRIDGE_UART_PORT_TX_PIN,
                 CONFIG_BRIDGE_UART_PORT_RX_PIN,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s uart config pin failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    if((ret = uart_driver_install(CONFIG_BRIDGE_UART_PORT, CONFIG_BRIDGE_RX_BUFF_SIZE, CONFIG_BRIDGE_TX_BUFF_SIZE, 0, NULL, 0)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s uart install failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

#if defined( CONFIG_BRIDGE_TX_TASK_PINNED_TO_CORE_0)
    if(pdPASS != xTaskCreatePinnedToCore(_TxUartTask, "BridgeUartTx", CONFIG_BRIDGE_TX_TASK_STACK_SIZE, NULL, CONFIG_BRIDGE_TX_TASK_PRIORITY, NULL, PRO_CPU_NUM)) {
#elif defined( CONFIG_BRIDGE_TX_TASK_PINNED_TO_CORE_1)
    if(pdPASS != xTaskCreatePinnedToCore(_TxUartTask, "BridgeUartTx", CONFIG_BRIDGE_TX_TASK_STACK_SIZE, NULL, CONFIG_BRIDGE_TX_TASK_PRIORITY, NULL, APP_CPU_NUM)) {
#else
    if(pdPASS != xTaskCreatePinnedToCore(_TxUartTask, "BridgeUartTx", CONFIG_BRIDGE_TX_TASK_STACK_SIZE, NULL, CONFIG_BRIDGE_TX_TASK_PRIORITY, NULL, tskNO_AFFINITY)) {
#endif
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s Tx Task creation failed: %s\n", __func__, "ESP_ERR_NO_MEM");
        return ESP_ERR_NO_MEM;
    }

#if defined( CONFIG_BRIDGE_RX_TASK_PINNED_TO_CORE_0)
    if(pdPASS != xTaskCreatePinnedToCore(_RxUartTask, "BridgeUartRx", CONFIG_BRIDGE_RX_TASK_STACK_SIZE, NULL, CONFIG_BRIDGE_RX_TASK_PRIORITY, NULL, PRO_CPU_NUM)) {
#elif defined( CONFIG_BRIDGE_RX_TASK_PINNED_TO_CORE_1)
    if(pdPASS != xTaskCreatePinnedToCore(_RxUartTask, "BridgeUartRx", CONFIG_BRIDGE_RX_TASK_STACK_SIZE, NULL, CONFIG_BRIDGE_RX_TASK_PRIORITY, NULL, APP_CPU_NUM)) {
#else
    if(pdPASS != xTaskCreatePinnedToCore(_RxUartTask, "BridgeUartRx", CONFIG_BRIDGE_RX_TASK_STACK_SIZE, NULL, CONFIG_BRIDGE_RX_TASK_PRIORITY, NULL, tskNO_AFFINITY)) {
#endif
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s Rx Task creation failed: %s\n", __func__, "ESP_ERR_NO_MEM");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* Received from Bluetooth SPP ---> Transmit to Uart */
static void _TxUartTask(void *pxParam)
{
    uint8_t *pTxData = (uint8_t *)0;
    size_t nTxData = 0;
    while(1) {
        uart_wait_tx_done(CONFIG_BRIDGE_UART_PORT, (TickType_t)portMAX_DELAY);
        while(1) {
            pTxData = (uint8_t *)xRingbufferReceiveUpTo(bridgeTxBuf_handle, &nTxData, (TickType_t)(5 / portTICK_RATE_MS), UART_FIFO_LEN);
            if(pTxData != NULL) {
                ESP_LOGI(ZPL_BRIDGE_TAG, "%s data to uart tx=%d", __func__, *pTxData);
                uart_tx_chars(CONFIG_BRIDGE_UART_PORT, (char *)pTxData, nTxData);
                vRingbufferReturnItem(bridgeTxBuf_handle, (void*)pTxData);
                break;
            }
        }
    }
}

/* Received from Uart ---> Transmit via Bluetooth */
static void _RxUartTask(void *pxParam)
{
    uint8_t readBuffer[128];
    size_t nRxData;
    bBtWriteDone = true;

    while(1) {
        /* Receive from UART */
        nRxData = uart_read_bytes(CONFIG_BRIDGE_UART_PORT, readBuffer, sizeof(readBuffer), (TickType_t)(10 / portTICK_RATE_MS));
        if(nRxData > 0) {
            if(bt_initiator_hdl) {
                ESP_LOGI(ZPL_BRIDGE_TAG, "%s Rx data=%d to bt client=%d", __func__, readBuffer[0], bt_initiator_hdl);
                while(1) {
                    if(bBtWriteDone) {
                        esp_spp_write(bt_initiator_hdl, nRxData, readBuffer);
                        bBtWriteDone = false;
                        break;
                    } else {
                        vTaskDelay((TickType_t)(5 / portTICK_RATE_MS));
                    }
                }
            }
        }
    }
}

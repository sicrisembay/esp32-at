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
#include "FreeRTOS_CLI.h"
#include "bt_zplCmds.h"

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
static esp_err_t _bridgePinInit(void);
static void _TxUartTask(void *pxParam);
static void _RxUartTask(void *pxParam);

static const char BT_CTRL_ESCAPE_SEQUENCE[] = {'\4', '\4', '\4', '!'};
static const uint8_t BT_CTRL_ESCAPE_SEQUENCE_LEN = sizeof(BT_CTRL_ESCAPE_SEQUENCE) / sizeof(BT_CTRL_ESCAPE_SEQUENCE[0]);
#define BT_CTRL_ESCAPE_SEQUENCE_INTERCHARACTER_DELAY_MS     (TickType_t)(500 / portTICK_RATE_MS)
static bool bEscapeEnabled = false;

#define CLI_MAX_INPUT_LENGTH        (128)
#define CLI_MAX_OUTPUT_LENGTH       (256)
static int8_t pcOutputString[CLI_MAX_OUTPUT_LENGTH];
static int8_t pcInputString[CLI_MAX_INPUT_LENGTH];
static const char * const pPromptStr = "BT> ";
static const char * const pWelcomeStr = "\n\n\rZPL Bridge Command Line Interface.\n\rType help to view a list of registered commands.\n\r";
static void SendToConsole(uint8_t *pData, size_t nData);


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
        } else {
            /// TODO: Received BT data ignored due to buffer full
        }
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGW(ZPL_BRIDGE_TAG, "ESP_SPP_CONG_EVT");
        bCongest = true;
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

    if((ret = _bridgePinInit()) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s bridge Pin Init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

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

void bt_bridge_ExitEscapedMode(void)
{
    uint8_t *pRxData = (uint8_t *)0;
    size_t nRxData = 0;

    if(bEscapeEnabled) {
        /* Flush Rx */
        pRxData = (uint8_t *)xRingbufferReceiveUpTo(bridgeRxBuf_handle, &nRxData, (TickType_t)(5 / portTICK_RATE_MS), 64);
        if(pRxData != NULL) {
            while(1) {
                if(bBtWriteDone) {
                    esp_spp_write(bt_initiator_hdl, nRxData, pRxData);
                    bBtWriteDone = false;
                    break;
                } else {
                    vTaskDelay((TickType_t)(5 / portTICK_RATE_MS));
                }
            }
            vRingbufferReturnItem(bridgeRxBuf_handle, (void*)pRxData);
        }

        /* Disable Escaped Mode */
        bEscapeEnabled = false;
    }
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

static esp_err_t _bridgePinInit(void)
{
    esp_err_t ret = ESP_OK;
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_BRIDGE_MXRT1052_RESET_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    if((ret = gpio_config(&io_conf)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s reset pin init failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(CONFIG_BRIDGE_MXRT1052_RESET_PIN, 1U);

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_BRIDGE_DOWNSTREAM_CLIENT_ID_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    if((ret = gpio_config(&io_conf)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s client id downstream pin init failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(CONFIG_BRIDGE_DOWNSTREAM_CLIENT_ID_PIN, 0U);

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_BRIDGE_UPSTREAM_CLIENT_ID_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    if((ret = gpio_config(&io_conf)) != ESP_OK) {
        ESP_LOGE(ZPL_BRIDGE_TAG, "%s bridge init pin failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/* Received from Bluetooth SPP ---> Transmit to Uart */
static void _TxUartTask(void *pxParam)
{
    uint8_t *pTxData = (uint8_t *)0;
    uint8_t *pRxData = (uint8_t *)0;
    size_t nTxData = 0;
    size_t nRxData = 0;
    bEscapeEnabled = false;
    TickType_t receiveTick;
    TickType_t receiveLastTick;
    uint8_t escapeSequencePos;

    receiveTick = xTaskGetTickCount();
    receiveLastTick = receiveTick;
    escapeSequencePos = 0;

    bt_zpl_RegisterCommands();

    while(1) {
        uart_wait_tx_done(CONFIG_BRIDGE_UART_PORT, (TickType_t)portMAX_DELAY);
        while(1) {
            pTxData = (uint8_t *)xRingbufferReceiveUpTo(bridgeTxBuf_handle, &nTxData, (TickType_t)(5 / portTICK_RATE_MS), UART_FIFO_LEN);
            if(pTxData != NULL) {
                receiveTick = xTaskGetTickCount();
                ESP_LOGI(ZPL_BRIDGE_TAG, "%s data to uart tx=%d", __func__, *pTxData);

                if(bEscapeEnabled == true) {
                    /* Redirect to ESP32 command line interface */
                    SendToConsole(pTxData, nTxData);
                    vRingbufferReturnItem(bridgeTxBuf_handle, (void*)pTxData);
                } else {
                    /* Check for Escape Sequence */
                    if((*pTxData == BT_CTRL_ESCAPE_SEQUENCE[escapeSequencePos]) &&
                       ((receiveTick - receiveLastTick) > BT_CTRL_ESCAPE_SEQUENCE_INTERCHARACTER_DELAY_MS)) {
                        escapeSequencePos++;
                        if(escapeSequencePos >= BT_CTRL_ESCAPE_SEQUENCE_LEN) {
                            bEscapeEnabled = true;
                            escapeSequencePos = 0;
                            ESP_LOGI(ZPL_BRIDGE_TAG, "Escape Sequence Received");
                            /* Empty the Rx Ring Buffer */
                            while(NULL != (pRxData = xRingbufferReceiveUpTo(bridgeRxBuf_handle, &nRxData, 0, 64))) {
                                vRingbufferReturnItem(bridgeRxBuf_handle, (void *)pRxData);
                            }
                            /* Send Welcome Message */
                            xRingbufferSend(bridgeRxBuf_handle, pWelcomeStr, strlen((char *)pWelcomeStr), (TickType_t)0);
                            xRingbufferSend(bridgeRxBuf_handle, pPromptStr, strlen((char *)pPromptStr), (TickType_t)0);
                        }
                    } else {
                        escapeSequencePos = 0;
                    }

                    if(bEscapeEnabled != true) {
                        /* Redirect to ESP32 UART */
                        uart_tx_chars(CONFIG_BRIDGE_UART_PORT, (char *)pTxData, nTxData);
                    }
                    vRingbufferReturnItem(bridgeTxBuf_handle, (void*)pTxData);
                    break;
                }
                receiveLastTick = receiveTick;
            }
        }
    }
}

/* Received from Uart ---> Transmit via Bluetooth */
static void _RxUartTask(void *pxParam)
{
    uint8_t readBuffer[128];
    uint8_t *pRxData = (uint8_t *)0;
    size_t nRxData = 0;
    bBtWriteDone = true;

    while(1) {
        if(bEscapeEnabled) {
            pRxData = (uint8_t *)xRingbufferReceiveUpTo(bridgeRxBuf_handle, &nRxData, (TickType_t)(5 / portTICK_RATE_MS), 64);
            if(pRxData != NULL) {
                while(1) {
                    if(bBtWriteDone) {
                        esp_spp_write(bt_initiator_hdl, nRxData, pRxData);
                        bBtWriteDone = false;
                        break;
                    } else {
                        vTaskDelay((TickType_t)(5 / portTICK_RATE_MS));
                    }
                }
                vRingbufferReturnItem(bridgeRxBuf_handle, (void*)pRxData);
            }
        } else {
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
}


static void SendToConsole(uint8_t *pData, size_t nData)
{
    size_t idx = 0;
    BaseType_t xMoreDataToFollow;
    static uint32_t inputIdx = 0;

    for(idx = 0; idx < nData; idx++) {
        /* Echo */
        if((pData[idx] != '\n') && (pData[idx] != '\r') && (pData[idx] != '\b')) {
            xRingbufferSend(bridgeRxBuf_handle, &pData[idx], 1, (TickType_t)0);
        }

        if( pData[idx] == '\n' ) {
            /* A newline character was received, so the input command string is
            complete and can be processed.  Transmit a line separator, just to
            make the output easier to read. */
            xRingbufferSend(bridgeRxBuf_handle, (uint8_t *)"\r\n", strlen("\r\n"), (TickType_t)0);

            /* The command interpreter is called repeatedly until it returns
            pdFALSE.  See the "Implementing a command" documentation for an
            exaplanation of why this is. */
            do {
                /* Send the command string to the command interpreter.  Any
                output generated by the command interpreter will be placed in the
                pcOutputString buffer. */
                xMoreDataToFollow = FreeRTOS_CLIProcessCommand(
                                          (char *)pcInputString,   /* The command string.*/
                                          (char *)pcOutputString,  /* The output buffer. */
                                          CLI_MAX_OUTPUT_LENGTH/* The size of the output buffer. */
                                      );

                /* Write the output generated by the command interpreter to the
                console. */
                xRingbufferSend(bridgeRxBuf_handle, (uint8_t *)pcOutputString, strlen((char *)pcOutputString), (TickType_t)0);
            } while( xMoreDataToFollow != pdFALSE );

            if(bEscapeEnabled == true) {
                xRingbufferSend(bridgeRxBuf_handle, (uint8_t *)pPromptStr, strlen((char *)pPromptStr), (TickType_t)0);
            } else {
                /* User entered "exit". No need to send prompt */
                xRingbufferSend(bridgeRxBuf_handle, (uint8_t*)"\n\r", strlen("\n\r"), (TickType_t)0);
            }

            /* All the strings generated by the input command have been sent.
            Processing of the command is complete.  Clear the input string ready
            to receive the next command. */
            inputIdx = 0;
            memset( pcInputString, 0x00, CLI_MAX_INPUT_LENGTH );
        }
        else
        {
            /* The if() clause performs the processing after a newline character
            is received.  This else clause performs the processing if any other
            character is received. */
            if( pData[idx] == '\r' ) {
                /* Ignore carriage returns. */
            } else if( pData[idx] == '\b' ) {
                /* Backspace was pressed.  Erase the last character in the input
                buffer - if there are any. */
                if( inputIdx > 0 ) {
                    inputIdx--;
                    pcInputString[ inputIdx ] = '\0';
                    xRingbufferSend(bridgeRxBuf_handle, (uint8_t *)&pData[idx], 1, (TickType_t)0);
                }
            } else {
                /* A character was entered.  It was not a new line, backspace
                or carriage return, so it is accepted as part of the input and
                placed into the input buffer.  When a \n is entered the complete
                string will be passed to the command interpreter. */
                if( inputIdx < CLI_MAX_INPUT_LENGTH ) {
                    pcInputString[ inputIdx ] = pData[idx];
                    inputIdx++;
                }
            }
        }
    }
}

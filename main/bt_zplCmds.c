#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FreeRTOS_CLI.h"
#include "bt_zplCmds.h"
#include "bt_zplBridge.h"
#include "driver/gpio.h"
#include "string.h"

static BaseType_t _prvExit(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    bt_bridge_ExitEscapedMode();
    return pdFALSE;
}

static const CLI_Command_Definition_t xExit = {
        "exit",                                     /* The command string to type. */
        "exit: Exits the command line mode.\r\n",   /* Help String */
        _prvExit,                                   /* The function to run. */
        0                                           /* Param1: Volume Name */
};

#define RESET_DELAY     (1000 / portTICK_RATE_MS)
static BaseType_t _prvReset(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    int tx_sig;

    if(gpio_get_level(CONFIG_BRIDGE_MXRT1052_RESET_PIN)) {
        /* Switch off MXRT1052 power */
        gpio_set_level(CONFIG_BRIDGE_MXRT1052_RESET_PIN, 0);
        vTaskDelay(RESET_DELAY);
        /* Set TX to Open drain (MCU Rx pin has pull up to MCU 3.3V) */
#if(CONFIG_BRIDGE_UART_PORT == 0)
        tx_sig = U0TXD_OUT_IDX;
#elif(CONFIG_BRIDGE_UART_PORT == 1)
        tx_sig = U1TXD_OUT_IDX;
#elif(CONFIG_BRIDGE_UART_PORT == 2)
        tx_sig = U2TXD_OUT_IDX;
#else
#error "Invalid Bridge Uart Port!"
#endif
        gpio_set_level(CONFIG_BRIDGE_UART_PORT_TX_PIN, 1);
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CONFIG_BRIDGE_UART_PORT_TX_PIN], PIN_FUNC_GPIO);
        gpio_set_direction(CONFIG_BRIDGE_UART_PORT_TX_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
        gpio_matrix_out(CONFIG_BRIDGE_UART_PORT_TX_PIN, tx_sig, 0, 0);
        return pdTRUE;
    } else {
        vTaskDelay(RESET_DELAY);
        /* Set TX to Push-Pull */
#if(CONFIG_BRIDGE_UART_PORT == 0)
        tx_sig = U0TXD_OUT_IDX;
#elif(CONFIG_BRIDGE_UART_PORT == 1)
        tx_sig = U1TXD_OUT_IDX;
#elif(CONFIG_BRIDGE_UART_PORT == 2)
        tx_sig = U2TXD_OUT_IDX;
#else
#error "Invalid Bridge Uart Port!"
#endif
        gpio_set_level(CONFIG_BRIDGE_UART_PORT_TX_PIN, 1);
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CONFIG_BRIDGE_UART_PORT_TX_PIN], PIN_FUNC_GPIO);
        gpio_set_direction(CONFIG_BRIDGE_UART_PORT_TX_PIN, GPIO_MODE_OUTPUT);
        gpio_matrix_out(CONFIG_BRIDGE_UART_PORT_TX_PIN, tx_sig, 0, 0);

        /* Switch on MXRT1052 power */
        gpio_set_level(CONFIG_BRIDGE_MXRT1052_RESET_PIN, 1);
        bt_bridge_ExitEscapedMode();
        strncpy(pcWriteBuffer, "Reset Done.\n\r", (xWriteBufferLen - 1));
        pcWriteBuffer[xWriteBufferLen - 1] = '\0';
        return pdFALSE;
    }
    return pdFALSE;
}
static const CLI_Command_Definition_t xReset = {
        "reset",                                    /* The command string to type. */
        "reset: Resets RT1052 MCU and exits CLI mode.\r\n",   /* Help String */
        _prvReset,                                   /* The function to run. */
        0                                           /* Param1: Volume Name */
};

void bt_zpl_RegisterCommands(void)
{
    FreeRTOS_CLIRegisterCommand(&xExit);
    FreeRTOS_CLIRegisterCommand(&xReset);
}

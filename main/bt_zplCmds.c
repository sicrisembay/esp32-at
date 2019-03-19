#include "freertos/FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "bt_zplCmds.h"
#include "bt_zplBridge.h"

static BaseType_t _prvExit(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static const CLI_Command_Definition_t xExit = {
        "exit",                                     /* The command string to type. */
        "exit: Exits the command mode.\r\n",        /* Help String */
        _prvExit,                                   /* The function to run. */
        0                                           /* Param1: Volume Name */
};

void bt_zpl_RegisterCommands(void)
{
    FreeRTOS_CLIRegisterCommand(&xExit);
}


static BaseType_t _prvExit(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    bt_bridge_ExitEscapedMode();
    return pdFALSE;
}

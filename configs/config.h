#ifndef CONFIG_H_
#define CONFIG_H_



//Task names
#define SYSTEM_TASK_NAME        "SYSTEM"
//#define ADC_TASK_NAME           "ADC"
//#define PM_TASK_NAME            "PWRMGNT"
//#define CRTP_TX_TASK_NAME       "CRTP-TX"
//#define CRTP_RX_TASK_NAME       "CRTP-RX"
//#define CRTP_RXTX_TASK_NAME     "CRTP-RXTX"
//#define LOG_TASK_NAME           "LOG"
//#define MEM_TASK_NAME           "MEM"
//#define PARAM_TASK_NAME         "PARAM"
//#define STABILIZER_TASK_NAME    "STABILIZER"
//#define NRF24LINK_TASK_NAME     "NRF24LINK"
//#define ESKYLINK_TASK_NAME      "ESKYLINK"
//#define SYSLINK_TASK_NAME       "SYSLINK"
//#define USBLINK_TASK_NAME       "USBLINK"
//Task stack sizes
#define SYSTEM_TASK_STACKSIZE         (2* configMINIMAL_STACK_SIZE)
//#define ADC_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
//#define PM_TASK_STACKSIZE             configMINIMAL_STACK_SIZE
//#define CRTP_TX_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
//#define CRTP_RX_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
//#define CRTP_RXTX_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
//#define LOG_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
//#define MEM_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
//#define PARAM_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
//#define STABILIZER_TASK_STACKSIZE     (3 * configMINIMAL_STACK_SIZE)
//#define NRF24LINK_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
//#define ESKYLINK_TASK_STACKSIZE       configMINIMAL_STACK_SIZE
//#define SYSLINK_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
//#define USBLINK_TASK_STACKSIZE        configMINIMAL_STACK_SIZE

//Task priorities. Higher number higher priority
#define SYSTEM_TASK_PRI         2
//#define CRTP_RXTX_TASK_PRI      2
//#define LOG_TASK_PRI            1
//#define MEM_TASK_PRI            1
//#define PARAM_TASK_PRI          1
//#define STABILIZER_TASK_PRI     4
//#define SYSLINK_TASK_PRI        3
//#define USBLINK_TASK_PRI        3





#endif /* CONFIG_H_ */

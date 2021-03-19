#include <string.h>

#include "cy_device_headers.h"
#include "cy_project.h"
#include "glue_functions.c"
#include "sdl/common/src/drivers/gpio/cy_gpio.c"
#include "sdl/common/src/drivers/smif/ver3_1/cy_smif.c"
#include "sdl/common/src/drivers/smif/ver3_1/cy_smif_memslot.c"
#include "sdl/common/src/drivers/syslib/cy_syslib.c"
#include "sdl/common/src/drivers/syspm/cy_syspm.c"
#include "sdl/common/src/drivers/systick/cy_systick.c"
#include "sdl/common/src/drivers/syswdt/cy_syswdt.c"
#include "sdl/tviic2d4m/src/drivers/sysclk/cy_sysclk.c"
#include "sdl/tviic2d4m/src/drivers/syspmic/cy_syspmic.c"
#include "sdl/tviic2d4m/src/mw/power/cy_power.c"
#include "sdl/tviic2d4m/src/mw/smif_mem/cy_smif_device_common.c"
#include "sdl/tviic2d4m/src/mw/smif_mem/cy_smif_semp.c"
#include "sdl/tviic2d4m/src/system/rev_a/system_tviic2d4m_cm0plus.c"
#include "tvii_series_smif_ex_adopter.h"

#define CHECK(x)     \
  if (!(x)) {        \
    return __LINE__; \
  }

typedef struct {
  volatile stc_GPIO_PRT_t* port;
  uint8_t pin;
  en_hsiom_sel_t hsiom;
  uint32_t driveMode;
} cy_stc_smif_port_t;

cy_stc_smif_port_t smifPortCfg[] = {
    {CY_SMIF_CLK_PORT, CY_SMIF_CLK_PIN, CY_SMIF_CLK_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_CLK_INV_PORT, CY_SMIF_CLK_INV_PIN, CY_SMIF_CLK_INV_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_RWDS_PORT, CY_SMIF_RWDS_PIN, CY_SMIF_RWDS_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_SELECT0_PORT, CY_SMIF_SELECT0_PIN, CY_SMIF_SELECT0_PIN_MUX, CY_GPIO_DM_PULLUP},
    {CY_SMIF_SELECT1_PORT, CY_SMIF_SELECT1_PIN, CY_SMIF_SELECT1_PIN_MUX, CY_GPIO_DM_PULLUP},
    {CY_SMIF_DATA0_PORT, CY_SMIF_DATA0_PIN, CY_SMIF_DATA0_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA1_PORT, CY_SMIF_DATA1_PIN, CY_SMIF_DATA1_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA2_PORT, CY_SMIF_DATA2_PIN, CY_SMIF_DATA2_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA3_PORT, CY_SMIF_DATA3_PIN, CY_SMIF_DATA3_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA4_PORT, CY_SMIF_DATA4_PIN, CY_SMIF_DATA4_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA5_PORT, CY_SMIF_DATA5_PIN, CY_SMIF_DATA5_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA6_PORT, CY_SMIF_DATA6_PIN, CY_SMIF_DATA6_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA7_PORT, CY_SMIF_DATA7_PIN, CY_SMIF_DATA7_PIN_MUX, CY_GPIO_DM_STRONG},
};
#define SIZE_OF_PORT (sizeof(smifPortCfg) / sizeof(cy_stc_smif_port_t))

static void SmifPortInit(cy_stc_smif_port_t cfg[], uint8_t size) {
  cy_stc_gpio_pin_config_t pinCfg = {0};
  for (uint32_t i = 0; i < size; i++) {
    pinCfg.driveMode = cfg[i].driveMode;
    pinCfg.hsiom = cfg[i].hsiom;
    Cy_GPIO_Pin_Init(cfg[i].port, cfg[i].pin, &pinCfg);
  }
}

// return 1, when one of selected device return error during erasing.
// return 0, when erasing completed without error.
static bool WaitDeviceBecomesReady(cy_en_smif_slave_select_t slave, cy_en_smif_semp_reg_latency_code_t regLc,
                                   cy_en_smif_trx_type_t transMode, cy_stc_smif_context_t* smifContext) {
  for (uint8_t i_slaveNum = 0; i_slaveNum < CY_SMIF_GetDeviceNumber(SMIF_USED); i_slaveNum++) {
    cy_en_smif_slave_select_t slaveToBeChecked = (cy_en_smif_slave_select_t)((uint8_t)slave & (0x01 << i_slaveNum));
    if ((uint8_t)slaveToBeChecked == 0u) {
      // not selected slave
      continue;
    }

    // only selected slave will reach hare.
    while (1) {
      un_cy_smif_semp_STR1_t status1;
      Cy_SMIF_SEMP_ReadStatusRegister1(SMIF_USED, slaveToBeChecked, &status1.u8, regLc, transMode, smifContext);
      // After transfer from Regacy SPI to OCTAL(SDR/DDR). RDYBSY may be also "0".
      if (status1.field.u1ERSERR == 1) return 1;
      if (status1.field.u1PRGERR == 1) return 1;
      if (status1.field.u1RDYBSY == 0) break;
    }
  }

  return 0;
}

static void SetModeWithBusyCheck(volatile stc_SMIF_t* base, cy_en_smif_mode_t mode) {
  cy_en_smif_status_t status;
  do {
    status = Cy_SMIF_SetMode(base, mode);
  } while (status != CY_SMIF_SUCCESS);
}

static int WriteRegisterSequence(cy_en_smif_slave_select_t slave, cy_en_smif_semp_reg_addr_t addr, uint8_t value,
                                 cy_en_smif_trx_addr_len_t addrLen, cy_en_smif_semp_reg_latency_code_t regLc,
                                 cy_en_smif_trx_type_t transMode, cy_stc_smif_context_t* smifContext) {
  Cy_SMIF_SEMP_WriteEnable(SMIF_USED, slave, transMode, smifContext);
  Cy_SMIF_SEMP_WriteAnyRegister(SMIF_USED, slave, value, addr, addrLen, transMode, smifContext);
  return WaitDeviceBecomesReady(slave, regLc, transMode, smifContext);
}

/***** User Modifiable definitions *****/
#define TEST_WRITE_LC CY_SMIF_SEMP_WT_LATENCY0
// SDR SPI: up to 81MHz, SDR OCTAL: up to 92MHz, DDR OCTAL: up to 85MHz
#define TEST_READ_LC CY_SMIF_SEMP_RD_LATENCY2
// SDR SPI: up to 133MHz, SDR OCTAL: up to 133MHz, DDR OCTAL: up to 66MHz
#define TEST_READ_REG_LC CY_SMIF_SEMP_REG_LATENCY1

//#define TEST_SLAVE_NO (0)
/**************************************/

#define DATA_WIDTH_1_OR_2_BITS (0)
#define DATA_WIDTH_4_BITS (1)
#define TRX_DATA_WIDTH (CY_SPI_TRANSACTION_4S4S4S)

#define TEST_SLAVE0 (CY_SMIF_SLAVE_SELECT_0)
#define TEST_SLAVE0_DEVICE (SMIF_DEVICE0)
#define TEST_SLAVE0_BIT_ALLOC (CY_SMIF_DEV_QUAD_BIT_0To3)

#define TEST_SLAVE1 (CY_SMIF_SLAVE_SELECT_1)
#define TEST_SLAVE1_DEVICE (SMIF_DEVICE1)
#define TEST_SLAVE1_BIT_ALLOC (CY_SMIF_DEV_QUAD_BIT_4To7)

static const cy_stc_smif_config_t smifConfig = {
    .txClk = CY_SMIF_INV_FOR_DDR,
    .rxClk = CY_SMIF_INV_OUTPUT_CLK,  // Note
    .dlpAuto = CY_SMIF_DLP_UPDATE_MANUAL,
    .capDelay = CY_SMIF_CAPTURE_DELAY_0_CYCLE,
    .delaySel = CY_SMIF_1_SEN_SEL_PER_TAP,
    .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
    .setupDelay = CY_SMIF_SETUP_0_CLK_PULUS_MIN,
    .holdDelay = CY_SMIF_HOLD_0_CLK_PULUS_MIN,
    .mode = CY_SMIF_NORMAL,  // MMIO mode
    .blockEvent = CY_SMIF_BUS_ERROR,
};

static cy_stc_smif_context_t smifContext;

static int configure_device(cy_en_smif_slave_select_t slave, cy_en_smif_bit_alloc_t bit_alloc,
                            volatile stc_SMIF_DEVICE_t* smif_device, const cy_stc_device_config_t* dev_cfg,
                            cy_stc_ex_dev_context_t* dev_context) {
  /********* Issue Software Reset ******/
  Cy_SMIF_SEMP_SoftwareResetEnable(SMIF_USED, slave, CY_SPI_TRANSACTION_1S1S1S, &smifContext);
  Cy_SMIF_SEMP_SoftwareReset(SMIF_USED, slave, CY_SPI_TRANSACTION_1S1S1S, &smifContext);
  WaitDeviceBecomesReady(slave, CY_SMIF_SEMP_REG_LATENCY0, CY_SPI_TRANSACTION_1S1S1S, &smifContext);

  /*********************************************************/
  /********* Write Status and Configuration registers ******/
  /*********************************************************/

  /*** Write Configuration register ***/
  // Setting Status Register 1
  un_cy_smif_semp_STR1_t writeStatusReg1 = {0};

  // Setting Configuration Register 1
  un_cy_smif_semp_CFR1_t writeConfigReg1 = {0};
  writeConfigReg1.field.u1QUADIT = DATA_WIDTH_1_OR_2_BITS;

  // Setting Configuration Register 2
  un_cy_smif_semp_CFR2_t writeConfigReg2 = {0};
  writeConfigReg2.field.u4MEMLAT = TEST_READ_LC;
  writeConfigReg2.field.u1QPI_IT = DATA_WIDTH_4_BITS;
  writeConfigReg2.field.u1ADRBYT = CY_TRX_ADDR_4BYTE;

  // Setting Configuration Register 3
  un_cy_smif_semp_CFR3_t writeConfigReg3 = {0};
  writeConfigReg3.field.u2VRGLAT = TEST_READ_REG_LC;

  // Setting Configuration Register 4
  un_cy_smif_semp_CFR4_t writeConfigReg4 = {0};
  writeConfigReg4.field.u1RBSTWP = 0;  // Read Wrapped Burst disable

  // Write Enable Volatile register
  // !! Attention if "Write Enable" command issued, following
  //    "Write Register" command will write to Non-Volatile registers.
  Cy_SMIF_SEMP_WriteEnableVolatileRegister(SMIF_USED, slave, CY_SPI_TRANSACTION_1S1S1S, &smifContext);

  // Write registers
  Cy_SMIF_SEMP_WriteRegister(SMIF_USED, slave, writeStatusReg1.u8, writeConfigReg1.u8, writeConfigReg2.u8,
                             writeConfigReg3.u8, writeConfigReg4.u8, CY_SPI_TRANSACTION_1S1S1S, &smifContext);
  CHECK(WaitDeviceBecomesReady(slave, TEST_READ_REG_LC, TRX_DATA_WIDTH, &smifContext) == 0);

  /*** Read back and Verify Configuration register ***/
  // Read and Verify Setting Configuration Register 1
  un_cy_smif_semp_CFR1_t readConfigReg1 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, slave, &readConfigReg1.u8, CY_SEMP_REG_ADDR_CFR1_V, TEST_READ_REG_LC,
                                       CY_TRX_ADDR_4BYTE, TRX_DATA_WIDTH, &smifContext);
  CHECK(writeConfigReg1.field.u1QUADIT == readConfigReg1.field.u1QUADIT);

  // Read and Verify Setting Configuration Register 2
  un_cy_smif_semp_CFR2_t readConfigReg2 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, slave, &readConfigReg2.u8, CY_SEMP_REG_ADDR_CFR2_V, TEST_READ_REG_LC,
                                       CY_TRX_ADDR_4BYTE, TRX_DATA_WIDTH, &smifContext);
  CHECK(writeConfigReg2.field.u4MEMLAT == readConfigReg2.field.u4MEMLAT);
  CHECK(writeConfigReg2.field.u1QPI_IT == readConfigReg2.field.u1QPI_IT);
  CHECK(writeConfigReg2.field.u1ADRBYT == readConfigReg2.field.u1ADRBYT);

  // Read and Verify Setting Configuration Register 3
  un_cy_smif_semp_CFR3_t readConfigReg3 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, slave, &readConfigReg3.u8, CY_SEMP_REG_ADDR_CFR3_V, TEST_READ_REG_LC,
                                       CY_TRX_ADDR_4BYTE, TRX_DATA_WIDTH, &smifContext);
  CHECK(writeConfigReg3.field.u2VRGLAT == readConfigReg3.field.u2VRGLAT);

  // Read and Verify Setting Configuration Register 4
  un_cy_smif_semp_CFR4_t readConfigReg4 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, slave, &readConfigReg4.u8, CY_SEMP_REG_ADDR_CFR4_V, TEST_READ_REG_LC,
                                       CY_TRX_ADDR_4BYTE, TRX_DATA_WIDTH, &smifContext);
  CHECK(writeConfigReg4.field.u1RBSTWP == readConfigReg4.field.u1RBSTWP);

  /***************************************************************************/
  un_cy_smif_semp_CFR3_t cfr3 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, slave, &cfr3.u8, CY_SEMP_REG_ADDR_CFR3_V, TEST_READ_REG_LC,
                                       CY_TRX_ADDR_4BYTE, TRX_DATA_WIDTH, &smifContext);
  cfr3.field.u1UNHYSA = 1;
  CHECK(WriteRegisterSequence(slave, CY_SEMP_REG_ADDR_CFR3_NV, cfr3.u8, CY_TRX_ADDR_4BYTE, TEST_READ_REG_LC,
                              TRX_DATA_WIDTH, &smifContext) == 0);

  uint8_t rb_cfr3 = 0u;
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, slave, &rb_cfr3, CY_SEMP_REG_ADDR_CFR3_V, TEST_READ_REG_LC,
                                       CY_TRX_ADDR_4BYTE, TRX_DATA_WIDTH, &smifContext);

  CHECK(rb_cfr3 == cfr3.u8);
  /***************************************************************************/

  // just for read/write test for now
  CHECK(WriteRegisterSequence(slave, CY_SEMP_REG_ADDR_DLP_V, 0xAA, CY_TRX_ADDR_4BYTE, TEST_READ_REG_LC, TRX_DATA_WIDTH,
                              &smifContext) == 0);
  uint8_t readbackDLP = 0u;
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, slave, &readbackDLP, CY_SEMP_REG_ADDR_DLP_V, TEST_READ_REG_LC,
                                       CY_TRX_ADDR_4BYTE, TRX_DATA_WIDTH, &smifContext);
  CHECK(readbackDLP == 0xAA);

  CHECK(Cy_SMIF_InitXIPModeForExMem(smif_device, bit_alloc, dev_cfg, dev_context) == CY_SMIF_SUCCESS);

  return 0;
}

static const cy_stc_device_config_t smifDev0Cfg = {
    .xipReadCmdId = CY_SMIF_SEMP_RDID_4S4S4S_4ADR,
    .xipReadMode = Cy_SMIF_SEMP_MODE_NOT_CONTINUOUS,
    .writeEn = false,
    .xipWriteCmdId = CY_SMIF_SEMP_WTID_1S1S1S_4ADR,
    .xipWriteMode = 0u,
    .mergeEnable = true,
    .mergeTimeout = CY_SMIF_MER_TIMEOUT_1_CYCLE,
    .totalTimeoutEnable = true,
    .totalTimeout = 1000u,
    .addr = 0x80000000,
    .size = CY_SMIF_DEVICE_64M_BYTE,
    .rdLatencyCode = TEST_READ_LC,
    .wtLatencyCode = TEST_WRITE_LC,
};

static const cy_stc_device_config_t smifDev1Cfg = {
    .xipReadCmdId = CY_SMIF_SEMP_RDID_4S4S4S_4ADR,
    .xipReadMode = Cy_SMIF_SEMP_MODE_NOT_CONTINUOUS,
    .writeEn = false,
    .xipWriteCmdId = CY_SMIF_SEMP_WTID_1S1S1S_4ADR,
    .xipWriteMode = 0u,
    .mergeEnable = true,
    .mergeTimeout = CY_SMIF_MER_TIMEOUT_1_CYCLE,
    .totalTimeoutEnable = true,
    .totalTimeout = 1000u,
    .addr = 0x84000000,
    .size = CY_SMIF_DEVICE_64M_BYTE,
    .rdLatencyCode = TEST_READ_LC,
    .wtLatencyCode = TEST_WRITE_LC,
};

static cy_stc_ex_dev_context_t smifDevice0Context;
static cy_stc_ex_dev_context_t smifDevice1Context;

__attribute__((used)) int Init(unsigned long adr, unsigned long clk, unsigned long fnc) {
  (void)adr;
  (void)clk;
  (void)fnc;

  memset(g_sempReadLatencyCodeToDummyCycle, 0, sizeof(g_sempReadLatencyCodeToDummyCycle));
  g_sempReadLatencyCodeToDummyCycle[0] = g_sempLatTblOtherThanOctalRead;
  g_sempReadLatencyCodeToDummyCycle[1] = g_sempLatTblOctalRead;
  memset(g_sempWriteLatencyCodeToDummyCycle, 0, sizeof(g_sempWriteLatencyCodeToDummyCycle));
  g_sempWriteLatencyCodeToDummyCycle[0] = g_latancyTableOfWrite;
  memset(cmdLatTbls, 0, sizeof(cmdLatTbls));
  cmdLatTbls[0] = NULL;
  cmdLatTbls[1] = g_sempLatTblRegType1;
  cmdLatTbls[2] = g_sempLatTblRegType2;
  cmdLatTbls[3] = g_sempLatTblRegType3;
  cmdLatTbls[4] = g_sempLatTblOtherThanOctalRead;
  cmdLatTbls[5] = g_sempLatTblOctalRead;

  SystemInit();

  Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
  Cy_SysClk_HfClockSetDivider(SMIF_HF_CLOCK, CY_SYSCLK_HFCLK_DIVIDE_BY_4);

  // Please modify according to your HW condition.
  ChangePLLFrequency(64000000);  // SMIF out clock will be 50,000,000 //YOTS: for initial test

  SmifPortInit(smifPortCfg, SIZE_OF_PORT);
  CHECK(Cy_SMIF_InitExMemContext(CY_SMIF_SEMP, &smifDevice0Context) == CY_SMIF_SUCCESS);
  CHECK(Cy_SMIF_InitExMemContext(CY_SMIF_SEMP, &smifDevice1Context) == CY_SMIF_SUCCESS);

  /*************************/
  /* 1. Setting for SMIF 0 */
  /*************************/
  Cy_SMIF_DeInit(SMIF_USED);
  Cy_SMIF_Init(SMIF_USED, &smifConfig, 1000, &smifContext);

  /************************************/
  /* 2. Setting for SMIF 0 Device 0/1 */
  /************************************/
  Cy_SMIF_DeviceSetDataSelect(SMIF_DEVICE0, CY_SMIF_DATA_SEL0);
  Cy_SMIF_DeviceWriteEnable(SMIF_DEVICE0);
  Cy_SMIF_DeviceSetDataSelect(SMIF_DEVICE1, CY_SMIF_DATA_SEL2);
  Cy_SMIF_DeviceWriteEnable(SMIF_DEVICE1);

  /********************/
  /* 3. Enable SMIF 0 */
  /********************/
  Cy_SMIF_Enable(SMIF_USED, &smifContext);

  CHECK(configure_device(TEST_SLAVE0, TEST_SLAVE0_BIT_ALLOC, TEST_SLAVE0_DEVICE, &smifDev0Cfg, &smifDevice0Context) ==
        0);
  CHECK(configure_device(TEST_SLAVE1, TEST_SLAVE1_BIT_ALLOC, TEST_SLAVE1_DEVICE, &smifDev1Cfg, &smifDevice1Context) ==
        0);
  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_MEMORY);

  return 0;
}

__attribute__((used)) int UnInit(unsigned long fnc) {
  (void)fnc;
  return (0);
}

__attribute__((used)) int EraseSector(unsigned long adr) {
  if (adr < CY_SMIF_GetXIP_Address(SMIF_USED)) return __LINE__;
  adr -= CY_SMIF_GetXIP_Address(SMIF_USED);

  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_NORMAL);

  cy_en_smif_slave_select_t test_slave = (adr & 0x04000000) ? TEST_SLAVE1 : TEST_SLAVE0;
  Cy_SMIF_SEMP_WriteEnable(SMIF_USED, test_slave, TRX_DATA_WIDTH, &smifContext);
  Cy_SMIF_SEMP_Erase_256KB_Sector(SMIF_USED, test_slave, adr, TRX_DATA_WIDTH, &smifContext);
  CHECK(WaitDeviceBecomesReady(test_slave, TEST_READ_REG_LC, TRX_DATA_WIDTH, &smifContext) == 0);

  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_MEMORY);

  return (0);
}

#if(0) /* Does not work for some reason */
__attribute__((used)) int EraseChip(void) {
  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_NORMAL);
  Cy_SMIF_SEMP_WriteEnable(SMIF_USED, TEST_SLAVE0, CY_SPI_TRANSACTION_1S1S1S, &smifContext);
  Cy_SMIF_SEMP_EraseChip(SMIF_USED, TEST_SLAVE0, CY_SPI_TRANSACTION_1S1S1S, &smifContext);
  CHECK(WaitDeviceBecomesReady(TEST_SLAVE0, TEST_READ_REG_LC, CY_SPI_TRANSACTION_1S1S1S, &smifContext) == 0);
  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_MEMORY);

  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_NORMAL);
  Cy_SMIF_SEMP_WriteEnable(SMIF_USED, TEST_SLAVE1, CY_SPI_TRANSACTION_1S1S1S, &smifContext);
  Cy_SMIF_SEMP_EraseChip(SMIF_USED, TEST_SLAVE1, CY_SPI_TRANSACTION_1S1S1S, &smifContext);
  CHECK(WaitDeviceBecomesReady(TEST_SLAVE1, TEST_READ_REG_LC, CY_SPI_TRANSACTION_1S1S1S, &smifContext) == 0);
  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_MEMORY);

  return (0);
}
#endif

__attribute__((used)) int ProgramPage(unsigned long adr, unsigned long sz, const unsigned char* buf) {
  if (sz != 0x100) return __LINE__;
  if (adr < CY_SMIF_GetXIP_Address(SMIF_USED)) return __LINE__;
  adr -= CY_SMIF_GetXIP_Address(SMIF_USED);

  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_NORMAL);

  cy_en_smif_slave_select_t test_slave = (adr & 0x04000000) ? TEST_SLAVE1 : TEST_SLAVE0;
  Cy_SMIF_SEMP_WriteEnable(SMIF_USED, test_slave, TRX_DATA_WIDTH, &smifContext);
  Cy_SMIF_MMIO_Program_ExMem(SMIF_USED, test_slave, CY_SMIF_SEMP_WTID_4S4S4S_4ADR, adr, sz, buf, CY_SMIF_BLOCKING,
                             TEST_WRITE_LC, Cy_SMIF_SEMP_MODE_NOT_CONTINUOUS, &smifDevice0Context, &smifContext);
  CHECK(WaitDeviceBecomesReady(test_slave, TEST_READ_REG_LC, TRX_DATA_WIDTH, &smifContext) == 0);

  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_MEMORY);

  return (0);
}

/* [] END OF FILE */

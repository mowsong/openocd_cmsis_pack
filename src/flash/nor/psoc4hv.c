/***************************************************************************
 *   Copyright (C) 2020 by Bohdan Tymkiv, Andrii Lishchynskyi              *
 *   bohdan.tymkiv@infineon.com bohdan200@gmail.com                        *
 *   andrii.lishchynskyi@infineon.com                                      *
 *                                                                         *
 *   Copyright (C) <2019-2021>                                             *
 *     <Cypress Semiconductor Corporation (an Infineon company)>           *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <flash/progress.h>
#include <helper/binarybuffer.h>
#include <jtag/jtag.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#include "imp.h"
#include "target/breakpoints.h"
#include "target/cortex_m.h"
#include "target/register.h"
#include "time_support.h"

#define PSOC4_SYSREQ_KEY1					0xB6
#define PSOC4_SYSREQ_KEY2					0xD3
#define PSOC4_SYSREQ_BIT					(1u << 31)
#define PSOC4_SYSREQ_HMASTER_BIT			(1u << 30)
#define PSOC4_SYSREQ_PRIVILEGED_BIT			(1u << 28)
#define PSOC4_SYSREQ_DIS_RESET_VECT_REL_BIT (1u << 27)
#define PSOC4_SYSREQ_NO_REMAP				(PSOC4_SYSREQ_HMASTER_BIT | PSOC4_SYSREQ_DIS_RESET_VECT_REL_BIT)

#define PSOC4_SROM_STATUS_MASK		0xF0000000
#define PSOC4_SROM_STATUS_SUCCEEDED 0xA0000000

#define PSOC4_CMD_GET_SILICON_ID   0x00
#define PSOC4_CMD_LOAD_LATCH	   0x04
#define PSOC4_CMD_WRITE_ROW		   0x05
#define PSOC4_CMD_PROGRAM_ROW	   0x06
#define PSOC4_CMD_ERASE_ALL		   0x0A
#define PSOC4_CMD_CHECKSUM		   0x0B
#define PSOC4_CMD_WRITE_PROTECTION 0x0D
#define PSOC4_CMD_SET_IMO48		   0x15
#define PSOC4_CMD_WRITE_SFLASH_ROW 0x18
#define PSOC4_CMD_BULK_ERASE       0x1D

#define PSOC4_CHIP_PROT_VIRGIN	  0x00
#define PSOC4_CHIP_PROT_OPEN	  0x01
#define PSOC4_CHIP_PROT_PROTECTED 0x02
#define PSOC4_CHIP_PROT_KILL	  0x04

#define PSOC4_ROMTABLE_PID0			  0xF0000FE0
#define PSOC4_ROMTABLE_DESIGNER_CHECK 0xB4

#define NVIC_VTOR 0xE000ED08

#define PSOC4_STACK_WA_SIZE	 512u
#define PSOC4_IPC_TIMEOUT_MS 1500u

#define FLASHC_GEOMETRY(base, index) ((base) + 0x10000 * (index))

#define SFLASH_BASE_v1 0x0FFFF000
#define SFLASH_BASE_v2 0x0FFFE000

#define BANK_ADDRESS_MASK		0xFFF00000
#define BANK_MAIN_ADDR_MASKED	0x00000000
#define BANK_WORK_ADDR_MASKED	0x1F000000
#define BANK_SFLASH_ADDR_MASKED 0x0FF00000

#define FAULT_STRUCT_SIZE		0x00000100
#define FAULT_BASE(n)			(0x40130000 + (n)*FAULT_STRUCT_SIZE)
#define FAULT_STATUS_VALID_MASK 0x80000000

#define FAULT_CTL(n)	(FAULT_BASE((n)) + 0x00)
#define FAULT_STATUS(n) (FAULT_BASE((n)) + 0x0C)
#define FAULT_DATA0(n)	(FAULT_BASE((n)) + 0x10)
#define FAULT_DATA1(n)	(FAULT_BASE((n)) + 0x14)
#define FAULT_MASK0(n)	(FAULT_BASE((n)) + 0x50)

#define FAULT_MASK_FLASH0_C_ECC	 (0x01 << 4)
#define FAULT_MASK_FLASH0_NC_ECC (0x01 << 5)
#define FAULT_MASK_FLASH1_C_ECC	 (0x01 << 7)
#define FAULT_MASK_FLASH1_NC_ECC (0x01 << 8)

#define FAULT_MASK0_ENABLE_ECC_MASK                                                                                    \
	(FAULT_MASK_FLASH0_C_ECC | FAULT_MASK_FLASH0_NC_ECC | FAULT_MASK_FLASH1_C_ECC | FAULT_MASK_FLASH1_NC_ECC)

struct cpuss_type {
	uint32_t geometry;
	uint32_t sysreq;
	uint32_t sysarg;
};

static const struct cpuss_type cpuss_legacy = {
	.geometry = 0x400E0000,
	.sysreq = 0x40000004,
	.sysarg = 0x40000008,
};

static const struct cpuss_type cpuss_new = {
	.geometry = 0x40110000,
	.sysreq = 0x40100004,
	.sysarg = 0x40100008,
};

struct sflash_type {
	uint32_t row_prot_addr[4];
	uint32_t chip_prot_offs;
	uint32_t siid_offs;
};

struct spcif_type {
	uint32_t mflash_size_mask;
	uint32_t mflash_size_pos;
	uint32_t sflash_size_mask;
	uint32_t sflash_size_pos;
	uint32_t macro_num_mask;
	uint32_t macro_num_pos;
	uint32_t row_size_mask;
	uint32_t rows_size_pos;
};

static const struct spcif_type spcif_v2 = {
	.mflash_size_mask = 0xffff << 0, /* 0:15 */
	.mflash_size_pos = 0,
	.sflash_size_mask = 0x0F << 16, /* 16:19 */
	.sflash_size_pos = 16,
	.macro_num_mask = 0x03 << 20, /* 20:21 */
	.macro_num_pos = 20,
	.row_size_mask = 0x03 << 22, /* 22:23 */
	.rows_size_pos = 22,
};

static const struct spcif_type spcif_v3 = {
	.mflash_size_mask = 0x3FFF << 0, /* 0:13 */
	.mflash_size_pos = 0,
	.sflash_size_mask = 0x3F << 14, /* 14:19 */
	.sflash_size_pos = 14,
	.macro_num_mask = 0x03 << 20, /* 20:21 */
	.macro_num_pos = 20,
	.row_size_mask = 0x03 << 22, /* 22:23 */
	.rows_size_pos = 22,
};

struct psoc4_info;
struct psoc4_family_type {
	const uint8_t id;
	const char *name;
	const uint32_t sflash_base;
	const struct cpuss_type *cpuss;
	const struct spcif_type *spcif;
	int (*prepare)(struct target *target, struct psoc4_info *p4_info);
	int (*sflash_probe)(struct flash_bank *bank);
};

static int imo_48mhz(struct target *target, struct psoc4_info *p4_info);
static int sflash_p4(struct flash_bank *bank);
static int sflash_p4hv(struct flash_bank *bank);

// clang-format off
static const struct psoc4_family_type psoc4_families[] = {
	{ 0x93, "PSoC 4100/4200",                      SFLASH_BASE_v1, &cpuss_legacy, &spcif_v2, NULL,      sflash_p4,   }, /* has known SROM bug (CDT#184422) */
	{ 0x9A, "PSoC 4000",                           SFLASH_BASE_v1, &cpuss_new,    &spcif_v2, imo_48mhz, sflash_p4,   }, /* has known SROM bug (CDT#184422) */
	{ 0x9E, "PSoC 4xx7 BLE",                       SFLASH_BASE_v1, &cpuss_new,    &spcif_v2, imo_48mhz, sflash_p4,   },
	{ 0xA0, "PSoC 4200L",                          SFLASH_BASE_v1, &cpuss_new,    &spcif_v2, NULL,      sflash_p4,   },
	{ 0xA1, "PSoC 4100M/4200M",                    SFLASH_BASE_v1, &cpuss_new,    &spcif_v2, NULL,      sflash_p4,   },
	{ 0xA3, "PSoC 4xx8 BLE",                       SFLASH_BASE_v1, &cpuss_new,    &spcif_v2, NULL,      sflash_p4,   },
	{ 0xA7, "PSoC 4000DS/4200DS",                  SFLASH_BASE_v1, &cpuss_new,    &spcif_v2, imo_48mhz, sflash_p4,   },
	{ 0xA9, "PSoC 4000S/4700S",                    SFLASH_BASE_v1, &cpuss_new,    &spcif_v3, imo_48mhz, sflash_p4,   },
	{ 0xAA, "PSoC 4xx8 BLE",                       SFLASH_BASE_v1, &cpuss_new,    &spcif_v2, NULL,      sflash_p4,   },
	{ 0xAB, "PSoC 4100S",                          SFLASH_BASE_v1, &cpuss_new,    &spcif_v3, imo_48mhz, sflash_p4,   },
	{ 0xAC, "PSoC 4100PS/PSoC Analog Coprocessor", SFLASH_BASE_v1, &cpuss_new,    &spcif_v3, imo_48mhz, sflash_p4,   },
	{ 0xAE, "PSoC 4xx8 BLE",                       SFLASH_BASE_v1, &cpuss_new,    &spcif_v2, NULL,      sflash_p4,   },
	{ 0xB5, "PSoC 4100S Plus",                     SFLASH_BASE_v1, &cpuss_new,    &spcif_v3, imo_48mhz, sflash_p4,   },
	{ 0xB7, "TrueTouch TSG7L",                     SFLASH_BASE_v1, &cpuss_new,    &spcif_v3, imo_48mhz, sflash_p4,   },
	{ 0xB8, "PSoC 4100S Plus/PSoC 4500",           SFLASH_BASE_v1, &cpuss_new,    &spcif_v3, imo_48mhz, sflash_p4,   },
	{ 0xBE, "PSoC 4100S Max",                      SFLASH_BASE_v2, &cpuss_new,    &spcif_v3, imo_48mhz, sflash_p4,   },
	{ 0xC0, "CCG6DF USB Type-C Port Controller",   SFLASH_BASE_v1, &cpuss_new,    &spcif_v3, imo_48mhz, sflash_p4,   },
	{ 0xC2, "PSoC4 HV PA",                         SFLASH_BASE_v2, &cpuss_new,    &spcif_v3, imo_48mhz, sflash_p4hv, },
	{ 0xC3, "CCG6SF USB Type-C Port Controller",   SFLASH_BASE_v1, &cpuss_new,    &spcif_v3, imo_48mhz, sflash_p4,   },
	{ 0, NULL, 0, NULL, NULL, NULL, NULL, },
};
// clang-format on

struct psoc4_info {
	uint16_t family_id;
	uint16_t revision_id;

	uint32_t mflash_size;
	uint32_t sflash_size;
	uint32_t num_macro;
	uint32_t row_size;
	uint32_t controller_idx;
	uint32_t area_idx;

	uint32_t die_bank_base;
	uint32_t die_bank_size;
	struct sflash_type sflash;
	const struct psoc4_family_type *family;
	bool probed;
};

static struct armv7m_algorithm g_armv7m_algo;
static struct working_area *g_stack_area;
static bool g_ecc_enabled;

/* Basic helper functions */
static inline bool is_wflash_bank(struct flash_bank *bank)
{
	return (bank->base & BANK_ADDRESS_MASK) == BANK_WORK_ADDR_MASKED;
}

static inline bool is_sflash_bank(struct flash_bank *bank)
{
	return (bank->base & BANK_ADDRESS_MASK) == BANK_SFLASH_ADDR_MASKED;
}

static inline bool is_psoc4hv(struct flash_bank *bank)
{
	struct psoc4_info *p4_info = bank->driver_priv;
	return p4_info->family->sflash_probe == sflash_p4hv;
}

/** **********************************************************************************************
 * @brief This is the very first step during target detection. Function retrieves family and
 * revision IDs from the ROM table and populates the following fields in the p4_info structure:
 * p4_info->family_id = family_id;
 * p4_info->revision_id = revision_id;
 * p4_info->family = family;
 *
 * @param target current target
 * @param p4_info pointer, will be populated
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_read_family(struct target *target, struct psoc4_info *p4_info)
{
	int retval, i;
	uint32_t pidbf[4];
	uint8_t pid[4];

	retval = target_read_memory(target, PSOC4_ROMTABLE_PID0, 4, 4, (uint8_t *)pidbf);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < 4; i++) {
		uint32_t tmp = target_buffer_get_u32(target, (uint8_t *)(pidbf + i));
		if (tmp & 0xffffff00) {
			LOG_ERROR("Unexpected data in ROMTABLE");
			return ERROR_FAIL;
		}
		pid[i] = tmp & 0xff;
	}

	uint16_t family_id = pid[0] | ((pid[1] & 0xf) << 8);
	uint16_t revision_id = (pid[2] & 0xF0) | ((pid[3] & 0xF0) >> 4);
	uint32_t designer = ((pid[1] & 0xf0) >> 4) | ((pid[2] & 0xf) << 4);

	if (designer != PSOC4_ROMTABLE_DESIGNER_CHECK) {
		LOG_ERROR("ROMTABLE designer is not Cypress");
		return ERROR_FAIL;
	}

	const struct psoc4_family_type *family = NULL;
	for (const struct psoc4_family_type *p = psoc4_families; p->id; p++) {
		if (p->id == family_id) {
			family = p;
			break;
		}
	}

	if (family == NULL) {
		LOG_ERROR("PSoC4 family 0x%02X is not supported", family_id);
		return ERROR_FAIL;
	}

	p4_info->family_id = family_id;
	p4_info->revision_id = revision_id;
	p4_info->family = family;

	return ERROR_OK;
}

/** **********************************************************************************************
 * @brief based on the p4_info.controller_idx, reads one of the SPCIF.GEOMETRY registers and
 * populates the following fields in the p4_info structure:
 * p4_info->mflash_size
 * p4_info->sflash_size
 * p4_info->num_macros
 * p4_info->row_size
 * @param target current target
 * @param p4_info pointer, will be populated
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_read_geometry(struct target *target, struct psoc4_info *p4_info)
{
	const struct spcif_type *spcif = p4_info->family->spcif;

	uint32_t geom = 0;
	const uint32_t geom_base = p4_info->family->cpuss->geometry;
	const uint32_t flash_ctl_idx = p4_info->controller_idx;
	int hr = target_read_u32(target, FLASHC_GEOMETRY(geom_base, flash_ctl_idx), &geom);
	if (hr != ERROR_OK)
		return hr;

	uint32_t val;
	val = (geom & spcif->mflash_size_mask) >> spcif->mflash_size_pos;
	p4_info->mflash_size = (val + 1) * 256;

	val = (geom & spcif->sflash_size_mask) >> spcif->sflash_size_pos;
	p4_info->sflash_size = (val + 1) * 256;

	val = (geom & spcif->macro_num_mask) >> spcif->macro_num_pos;
	p4_info->num_macro = (val + 1);

	val = (geom & spcif->row_size_mask) >> spcif->rows_size_pos;
	p4_info->row_size = (val + 1) * 64;

	return ERROR_OK;
}

/** **********************************************************************************************
 * @brief translates given address of the flash bank to it's macro number
 * @param bank flash bank
 * @param address absolute address within given bank
 * @return number of the flash macro
 *************************************************************************************************/
static uint8_t flash_addr_to_macro(struct flash_bank *bank, uint32_t address)
{
	struct psoc4_info *p4_info = bank->driver_priv;
	assert(address >= p4_info->die_bank_base);
	assert(address < p4_info->die_bank_base + p4_info->die_bank_size);

	return (address - p4_info->die_bank_base) * p4_info->num_macro / p4_info->die_bank_size;
}

/** **********************************************************************************************
 * @brief translates sysreq error code to the human-readable format
 * @param error_code sysreq error code
 * @return string representing sysreq code in a human-readable format
 *************************************************************************************************/
static const char *psoc4_sysreq_errorstr(uint32_t error_code)
{
	switch (error_code) {
	case 0xF0000001:
		return "Invalid Chip Protection Mode - This API is not available in current chip protection mode";
	case 0xF0000002:
		return "Invalid NVL Address - The NVL address provided must be within the range 0x00 - 0x07";
	case 0xF0000003:
		return "Invalid Page Latch Address/Size. Address is out of bounds or size is too large/unsupported";
	case 0xF0000004:
		return "Invalid Address - The row id or byte address provided is outside of the available memory";
	case 0xF0000005:
		return "Row Protected - The row id provided is a protected row";
	case 0xF000000A:
		return "Checksum Zero Failed - The calculated checksum was not zero";
	case 0xF000000B:
		return "Invalid Opcode - The opcode is not a valid API opcode";
	case 0xF000000C:
		return "Key Opcode Mismatch - The opcode provided does not match key1 and key2";
	case 0xF000000D:
		return "Macro Protected - The macro id provided is protected";
	case 0xF000000E:
		return "Invalid start address - The start address is greater than the end address provided";
	case 0xF000000F:
		return "No NVL Used - This is the return code for any NVL specific commands called for parts without NVLs";
	case 0xF0000010:
		return "No Sector Erase - The Erase Sector operation is not supported";
	case 0xF0000011:
		return "API Not Instantiated - The SRAM API for the opcode is not instantiated";
	case 0xF0000012:
		return "Invalid Flash Clock - set IMO to 48MHz before Write/Erase operations";
	case 0xF0000013:
		return "Invalid Macro ID - the macro provided is outside of the available macros";
	case 0xF0000014:
		return "API not Available in DEAD Mode";
	default:
		LOG_WARNING("Unknown sysreq error 0x%" PRIx32, error_code);
		return "";
	}
}

/** **********************************************************************************************
 * @brief translates chip protection value to the human-readable format
 * @param protection device protection state
 * @return string representing chip protection in a human-readable format
 *************************************************************************************************/
static const char *psoc4_protection_str(uint8_t protection)
{
	static const char *const protection_str[5] = {
		"VIRGIN", "OPEN", "PROTECTED", "UNKNOWN", "KILL",
	};

	if (protection > 4)
		protection = 3; /* UNKNOWN */

	return protection_str[protection];
}

/** ***********************************************************************************************
 * @brief starts pseudo flash algorithm and leaves it running. Function allocates working area for
 * algorithm code and CPU stack, adjusts stack pointer, uploads and starts the algorithm.
 * Algorithm (a basic infinite loop) runs asynchronously while driver performs Flash operations.
 * @param target current target
 * @param p4_info pointer to structure with target-specific information
 *************************************************************************************************/
static int psoc4_sysreq_prepare(struct target *target, struct psoc4_info *p4_info)
{
	if (target->coreid == 0xFF) { /* Chip is protected */
		if (p4_info->family->prepare)
			return p4_info->family->prepare(target, p4_info);

		return ERROR_OK;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Initialize Vector Table Offset register (in case FW modified it) */
	int hr = target_write_u32(target, NVIC_VTOR, 0x00000000);
	if (hr != ERROR_OK)
		return hr;

	/* Allocate Working Area for Stack and Flash algorithm */
	hr = target_alloc_working_area(target, PSOC4_STACK_WA_SIZE, &g_stack_area);
	if (hr != ERROR_OK)
		return hr;

	g_armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	g_armv7m_algo.core_mode = ARM_MODE_THREAD;

	struct reg_param reg_params;
	init_reg_param(&reg_params, "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params.value, 0, 32, g_stack_area->address + g_stack_area->size);

	/* Write 'cpsie	i' + basic infinite loop algorithm to target RAM */
	hr = target_write_u32(target, g_stack_area->address, 0xE7FEB662);
	if (hr != ERROR_OK)
		goto destroy_rp_free_wa;

	hr = target_start_algorithm(target, 0, NULL, 1, &reg_params, g_stack_area->address, 0, &g_armv7m_algo);
	if (hr != ERROR_OK)
		goto destroy_rp_free_wa;

	if (p4_info->family->prepare) {
		if (p4_info->family->prepare(target, p4_info))
			goto destroy_rp_free_wa;
	}

	destroy_reg_param(&reg_params);
	return hr;

destroy_rp_free_wa:
	/* Something went wrong, do some cleanup */
	destroy_reg_param(&reg_params);

	if (g_stack_area) {
		target_free_working_area(target, g_stack_area);
		g_stack_area = NULL;
	}

	return hr;
}

/** ***********************************************************************************************
 * @brief invokes given SROM API passing additional parameters either via SYSARG register or
 * via in RAM depending on the number of parameters. If ram_params is not NULL all parameters,
 * including mandatory cmd_param, will be passed via RAM.
 * @param target current target
 * @param p4_info pointer to structure with target-specific information
 * @param cmd SROM API request code
 * @param cmd_param mandatory SROM API parameter
 * @param ram_params optional pointer to additional parameters
 * @param ram_params_size size of additional parameters, in bytes
 * @param sysarg_out SROM API return code
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_sysreq(struct target *target, struct psoc4_info *p4_info, uint8_t cmd, uint16_t cmd_param,
                        uint8_t *ram_params, uint32_t ram_params_size, uint32_t *sysarg_out, bool quiet)
{
	int hr;
	if (ram_params) {
		uint32_t ram_param;
		buf_set_buf(ram_params, 0, &ram_param, 0, 32);
		LOG_DEBUG("sysreq cmd: 0x%02X, param: 0x%04X, ram_param: 0x%08X", cmd, cmd_param, ram_param);
	} else {
		LOG_DEBUG("sysreq cmd: 0x%02X, param: 0x%04X, ram_param: none", cmd, cmd_param);
	}

	const uint32_t sysarg_addr = p4_info->family->cpuss->sysarg;
	const uint32_t sysreq_addr = p4_info->family->cpuss->sysreq;
	const uint32_t param = PSOC4_SYSREQ_KEY1 | ((PSOC4_SYSREQ_KEY2 + cmd) << 8) | (cmd_param << 16);
	struct working_area *ram_param_area = NULL;

	if (ram_params) {
		hr = target_alloc_working_area(target, ram_params_size + 4, &ram_param_area);
		if (hr != ERROR_OK)
			return hr;

		hr = target_write_u32(target, ram_param_area->address, param);
		if (hr != ERROR_OK)
			goto cleanup_ram_wa;

		hr = target_write_buffer(target, ram_param_area->address + 4, ram_params_size, ram_params);
		if (hr != ERROR_OK)
			goto cleanup_ram_wa;

		hr = target_write_u32(target, sysarg_addr, ram_param_area->address);
		if (hr != ERROR_OK)
			goto cleanup_ram_wa;
	} else {
		hr = target_write_u32(target, sysarg_addr, param);
		if (hr != ERROR_OK)
			return hr;
	}

	hr = target_write_u32(target, sysreq_addr, PSOC4_SYSREQ_BIT | PSOC4_SYSREQ_HMASTER_BIT | cmd);
	if (hr != ERROR_OK)
		goto cleanup_ram_wa;

	uint32_t val;
	int64_t t0 = timeval_ms();
	do {
		int64_t elapsed_ms = timeval_ms() - t0;
		if (elapsed_ms > PSOC4_IPC_TIMEOUT_MS)
			break;

		hr = target_read_u32(target, sysreq_addr, &val);
		if (hr != ERROR_OK)
			goto cleanup_ram_wa;

	} while (val & (PSOC4_SYSREQ_BIT | PSOC4_SYSREQ_PRIVILEGED_BIT));

	hr = target_read_u32(target, sysarg_addr, &val);
	if (hr != ERROR_OK)
		goto cleanup_ram_wa;

	if ((val & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED) {
		if(!quiet)
			LOG_ERROR("%s", psoc4_sysreq_errorstr(val));

		hr = ERROR_FAIL;
	}

	if (sysarg_out)
		*sysarg_out = val;

cleanup_ram_wa:
	if (ram_param_area)
		target_free_working_area(target, ram_param_area);

	return hr;
}

/** ***********************************************************************************************
 * @brief stops running flash algorithm and releases associated resources.
 * This function is also used for cleanup in case of errors so g_stack_area may be NULL.
 * These cases have to be handled gracefully.
 * @param target current target
 * @param p4_info pointer to structure with target-specific information
 *************************************************************************************************/
static void psoc4_sysreq_release(struct target *target, struct psoc4_info *p4_info)
{
	if (g_stack_area) {
		/* Special core_id for fake SysAP */
		if (target->coreid == 0xFF)
			return;

		/* Stop flash algorithm if it is running */
		if (target->running_alg) {
			int hr = target_halt(target);
			if (hr != ERROR_OK)
				goto exit_free_wa;

			hr = target_wait_algorithm(target, 0, NULL, 0, NULL, 0, PSOC4_IPC_TIMEOUT_MS, &g_armv7m_algo);
			if (hr != ERROR_OK)
				goto exit_free_wa;
		}

	exit_free_wa:
		/* Free Stack/Flash algorithm working area */
		target_free_working_area(target, g_stack_area);
		g_stack_area = NULL;
	}

	/* Possible error is not critical here */
	target_write_u32(target, p4_info->family->cpuss->sysreq, PSOC4_SYSREQ_NO_REMAP);
}

/** ***********************************************************************************************
 * @brief configures 48MHz IMO clock. This is required on some families, flash programming will
 * fail otherwise.
 * @param target current target
 * @param p4_info pointer to structure with target-specific information
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int imo_48mhz(struct target *target, struct psoc4_info *p4_info)
{
	uint32_t sysreq_status;
	int hr = psoc4_sysreq(target, p4_info, PSOC4_CMD_SET_IMO48, 0, NULL, 0, &sysreq_status, false);

	return hr;
}

/** ***********************************************************************************************
 * @brief configures SFlash layout for PSoC4 parts
 * @param bank SFlash bank to configure
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int sflash_p4(struct flash_bank *bank)
{
	struct psoc4_info *p4_info = bank->driver_priv;

	p4_info->die_bank_base = p4_info->family->sflash_base;
	p4_info->die_bank_size = p4_info->sflash_size;

	p4_info->sflash.chip_prot_offs = 0x7C;
	p4_info->sflash.siid_offs = 0x0144;
	for (size_t i = 0; i < p4_info->num_macro; i++)
		p4_info->sflash.row_prot_addr[i] = p4_info->family->sflash_base + i * p4_info->row_size * 8;

	switch (p4_info->row_size) {
	case 64:
		bank->size = 0;
		bank->base = p4_info->die_bank_base;
		break;
	case 128:
		bank->size = 4 * p4_info->row_size;
		bank->base = p4_info->die_bank_base + 0x200;
		break;
	case 256:
		bank->size = 4 * p4_info->row_size;
		p4_info->sflash.chip_prot_offs = 0x0FC;
		p4_info->sflash.siid_offs = 0x244;
		bank->base = p4_info->die_bank_base + 0x400;
		break;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief configures SFlash layout for PSoC4-HV parts
 * @param bank SFlash bank to configure
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int sflash_p4hv(struct flash_bank *bank)
{
	struct psoc4_info *p4_info = bank->driver_priv;

	const uint32_t area_idx = p4_info->area_idx;
	p4_info->die_bank_base = p4_info->family->sflash_base;
	p4_info->die_bank_size = p4_info->sflash_size;

	uint32_t sflash_area;
	const uint32_t user_sflash_tab = p4_info->die_bank_base + 0x40;
	int hr = target_read_u32(bank->target, user_sflash_tab + 4 * area_idx, &sflash_area);
	if (hr != ERROR_OK)
		return hr;

	bank->base = 0x0FFF0000 | (sflash_area & 0x0000FFFF);
	bank->size = sflash_area >> 16;

	p4_info->sflash.chip_prot_offs = 0x7C;
	p4_info->sflash.siid_offs = 0x00;
	for (size_t i = 0; i < p4_info->num_macro; i++) {
		const uint32_t prot_tab = p4_info->die_bank_base + 0x20;
		uint16_t prot_area;
		hr = target_read_u16(bank->target, prot_tab + 2 * i, &prot_area);
		if (hr != ERROR_OK)
			return hr;
		p4_info->sflash.row_prot_addr[i] = 0x0FFF0000 | prot_area;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Executes system ROM API to read silicon/family ID and protection from the device
 * @param target current target
 * @param p4_info pointer to structure with target-specific information
 * @param silicon_id pointer to variable, will be populated with silicon ID
 * @param family_id pointer to variable, will be populated with family ID
 * @param protection pointer to variable, will be populated with protection state
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_sysreq_silicon_id(struct target *target, struct psoc4_info *p4_info, uint32_t *silicon_id,
								   uint16_t *family_id, uint8_t *protection)
{
	uint32_t part0, part1;

	int hr = psoc4_sysreq_prepare(target, p4_info);
	if (hr != ERROR_OK)
		return hr;

	hr = psoc4_sysreq(target, p4_info, PSOC4_CMD_GET_SILICON_ID, 0, NULL, 0, &part0, false);
	if (hr != ERROR_OK)
		goto release;

	hr = target_read_u32(target, p4_info->family->cpuss->sysreq, &part1);
	if (hr != ERROR_OK)
		goto release;

	/* build ID as Cypress sw does:
	 * bit 31..16 silicon ID
	 * bit 15..8  revision ID (so far 0x11 for all devices)
	 * bit 7..0   family ID (lowest 8 bits) */
	if (silicon_id)
		*silicon_id = ((part0 & 0x0000ffff) << 16) | ((part0 & 0x00ff0000) >> 8) | (part1 & 0x000000ff);

	if (family_id)
		*family_id = part1 & 0x0fff;

	if (protection)
		*protection = (part1 >> 12) & 0x0f;

release:
	psoc4_sysreq_release(target, p4_info);
	return hr;
}

/** ***********************************************************************************************
 * Rounds the 32-bit integer up to the next highest power of two, e.g.
 * 1000 -> 1024, 1024 -> 1024, 2040 -> 2048 etc.
 * Borrowed from the 'Bit Twiddling Hacks', see
 * https://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
 *************************************************************************************************/
static inline uint32_t next_pow2(uint32_t val)
{
	assert(val != 0);

	val--;
	val |= (val >> 1);
	val |= (val >> 2);
	val |= (val >> 4);
	val |= (val >> 8);
	val |= (val >> 16);

	return val + 1;
}

/** ***********************************************************************************************
 * @brief get_wounded_size
 * @param bank
 * @param max_size
 * @return
 *************************************************************************************************/
static void calculate_wounded_geometry(struct flash_bank *bank)
{
	const struct armv7m_common *cm = target_to_armv7m(bank->target);
	struct psoc4_info *p4_info = bank->driver_priv;

	/* Set initial bank size to full size, no wounding */
	bank->size = p4_info->die_bank_size;

	uint32_t dummy;
	int hr = mem_ap_read_atomic_u32(cm->debug_ap, bank->base + p4_info->die_bank_size - 4u, &dummy);
	if (hr == ERROR_OK)
		return;

	/* Round initial size up to the nearest power of two */
	uint32_t size = next_pow2(p4_info->die_bank_size);

	/* Round initial num_macro up to the nearest even number */
	uint32_t num_macro = (p4_info->num_macro + 1) & ~1u;

	do {
		size >>= 1;
		num_macro >>= 1;
		hr = mem_ap_read_atomic_u32(cm->debug_ap, bank->base + size - 4u, &dummy);
	} while (size && hr != ERROR_OK);

	/* There is at least one macro */
	if (num_macro == 0)
		num_macro = 1;

	LOG_DEBUG("%s: wounding detected (%d -> %d KiB, %d -> %d macro)", bank->name, p4_info->die_bank_size / 1024u,
			  size / 1024u, p4_info->num_macro, num_macro);

	bank->size = size;
	p4_info->num_macro = num_macro;
}

/** ***********************************************************************************************
 * @brief Probes the device and populates related data structures with target flash geometry
 * data. This is done by reading ROM Table and (one of the) SPCIF.GEOMETRY register(s) describing
 * available amount of memory.
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_probe(struct flash_bank *bank)
{
	struct psoc4_info *p4_info = bank->driver_priv;

	int hr = psoc4_read_family(bank->target, bank->driver_priv);
	if (hr != ERROR_OK)
		return hr;

	hr = psoc4_read_geometry(bank->target, bank->driver_priv);
	if (hr != ERROR_OK)
		return hr;

	if (is_sflash_bank(bank)) {
		hr = p4_info->family->sflash_probe(bank);
		if (hr != ERROR_OK)
			return hr;
	} else {
		p4_info->die_bank_base = bank->base;
		p4_info->die_bank_size = p4_info->mflash_size;
		calculate_wounded_geometry(bank);
	}

	bank->num_sectors = bank->size / p4_info->row_size;

	free(bank->sectors);
	bank->sectors = alloc_block_array(0, p4_info->row_size, bank->size / p4_info->row_size);
	if (bank->sectors == NULL)
		return ERROR_FAIL;

	bank->write_start_alignment = p4_info->row_size;
	bank->write_end_alignment = p4_info->row_size;
	p4_info->probed = true;

	/* I do not know why dap re-initialization  is needed, but following
	 * write operation fails sometimes on PSoC4-HV */
	const struct armv7m_common *cm = target_to_armv7m(bank->target);
	dap_dp_init(cm->debug_ap->dap);
	mem_ap_write_atomic_u32(cm->debug_ap, p4_info->family->cpuss->sysreq, PSOC4_SYSREQ_NO_REMAP);

	LOG_DEBUG("Flash bank '%s': %d bytes (%d rows, %d bytes/row, %d macro)", bank->name, bank->size,
			  bank->size / p4_info->row_size, p4_info->row_size, p4_info->num_macro);

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Probes given flash bank only if it hasn't been probed yet
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_auto_probe(struct flash_bank *bank)
{
	struct psoc4_info *p4_info = bank->driver_priv;
	if (p4_info->probed)
		return ERROR_OK;
	return psoc4_probe(bank);
}

/** ***********************************************************************************************
 * @brief performs flash bank list traversal and locates main and supervisory flash banks
 * @param target current target
 * @param main will be populated with pointer to main flash bank
 * @param super will be populated with pointer to supervisory flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int get_main_super_banks(struct target *target, struct flash_bank **main, struct flash_bank **super)
{
	*main = *super = NULL;
	int hr = get_flash_bank_by_addr(target, 0, true, main);
	if (hr != ERROR_OK) {
		LOG_ERROR("Unable to locate Main Flash bank");
		return hr;
	}

	for (*super = flash_bank_list(); *super; *super = (*super)->next) {
		if ((*super)->target == target && is_sflash_bank(*super))
			break;
	}

	if (*super == NULL) {
		LOG_ERROR("Unable to locate Supervisory Flash bank");
		return ERROR_FAIL;
	}

	hr = (*super)->driver->auto_probe(*super);
	return hr;
}

/** ***********************************************************************************************
 * @brief reads whole row protection array and stores it in given buffer
 * @param target current target
 * @param prot_array pointer to buffer to write data to
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_read_row_protection(struct target *target, uint8_t **prot_array)
{
	*prot_array = NULL;

	struct flash_bank *main_bank, *super_bank;
	int hr = get_main_super_banks(target, &main_bank, &super_bank);
	if (hr != ERROR_OK)
		return hr;

	struct psoc4_info *p4_minfo = main_bank->driver_priv;
	struct psoc4_info *p4_sinfo = super_bank->driver_priv;

	size_t num_prot_bytes = main_bank->size / p4_minfo->row_size / 8;
	size_t bytes_per_macro = num_prot_bytes / p4_minfo->num_macro;
	uint32_t *prot_table = p4_sinfo->sflash.row_prot_addr;

	*prot_array = malloc(num_prot_bytes);
	if (*prot_array == NULL)
		return ERROR_FAIL;

	uint8_t *ptr = *prot_array;
	for (size_t macro = 0; macro < p4_minfo->num_macro; macro++) {
		hr = target_read_buffer(target, prot_table[macro], bytes_per_macro, ptr);
		LOG_DEBUG("Reading prot block @0x%08" PRIx32 ", %zu bytes", prot_table[macro], bytes_per_macro);
		if (hr != ERROR_OK) {
			free(*prot_array);
			*prot_array = NULL;
			return hr;
		}
		ptr += bytes_per_macro;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief programs whole flash protection array stored in a given buffer
 * @param target current target
 * @param prot_array pointer to buffer to read data from
 * @param chip_protect if true, function will perform chip protection
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_write_row_protection(struct target *target, uint8_t *prot_array, bool chip_protect)
{
	struct flash_bank *main_bank, *super_bank;
	int hr = get_main_super_banks(target, &main_bank, &super_bank);
	if (hr != ERROR_OK)
		return hr;

	struct psoc4_info *p4_minfo = main_bank->driver_priv;
	struct psoc4_info *p4_sinfo = super_bank->driver_priv;

	size_t bytes_per_macro = main_bank->size / p4_minfo->row_size / p4_minfo->num_macro / 8;
	hr = psoc4_sysreq_prepare(target, p4_minfo);
	if (hr != ERROR_OK)
		return hr;

	uint8_t req_buffer[bytes_per_macro + 4];
	for (size_t macro = 0; macro < p4_minfo->num_macro; macro++) {
		uint16_t load_macro = flash_addr_to_macro(super_bank, p4_sinfo->sflash.row_prot_addr[macro]);
		memcpy(&req_buffer[4], prot_array, bytes_per_macro);
		buf_set_u32(req_buffer, 0, 32, bytes_per_macro - 1);
		hr = psoc4_sysreq(target, p4_minfo, PSOC4_CMD_LOAD_LATCH, ((load_macro & 0x7F) << 8), req_buffer,
						  sizeof(req_buffer), NULL, false);
		if (hr != ERROR_OK)
			goto release;

		uint16_t protect_macro = (macro & 0x7F) << 8;
		protect_macro |= chip_protect ? PSOC4_CHIP_PROT_PROTECTED : PSOC4_CHIP_PROT_OPEN;
		psoc4_sysreq(target, p4_minfo, PSOC4_CMD_WRITE_PROTECTION, protect_macro, NULL, 0, NULL, false);
		if (hr != ERROR_OK)
			goto release;

		prot_array += bytes_per_macro;
	}

release:
	psoc4_sysreq_release(target, p4_minfo);
	return hr;
}

/** ***********************************************************************************************
 * @brief sector protection routine
 * @param bank the bank to protect or unprotect
 * @param set if non-zero, enable protection; if 0, disable it
 * @param first the first sector to (un)protect, typically 0
 * @param last the last sector to (un)project, typically N-1
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	if (is_sflash_bank(bank) || is_wflash_bank(bank)) {
		LOG_ERROR("Row Protection only supported for Main Flash");
		return ERROR_FAIL;
	}

	struct target *target = bank->target;

	uint8_t *prot_array;
	int hr = psoc4_read_row_protection(target, &prot_array);
	if (hr != ERROR_OK) {
		free(prot_array);
		return hr;
	}

	for (size_t bit = first; bit <= last; bit++) {
		size_t byte = bit / 8;
		size_t shift = bit % 8;
		if (set) {
			prot_array[byte] |= (0x01u << shift);
		} else {
			prot_array[byte] &= ~(0x01u << shift);
		}
	}

	hr = psoc4_write_row_protection(target, prot_array, false);
	free(prot_array);

	return hr;
}

/** ***********************************************************************************************
 * @brief determine if the specific bank is "protected" or not, populates bank.sectors.is_protected
 * field for each of the flash sectors
 * @param bank - the bank to check
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_protect_check(struct flash_bank *bank)
{
	if (is_sflash_bank(bank) || is_wflash_bank(bank)) {
		for (size_t i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_protected = false;
		return ERROR_OK;
	}

	uint8_t *prot_data;
	int hr = psoc4_read_row_protection(bank->target, &prot_data);
	if (hr != ERROR_OK)
		return hr;

	for (uint32_t i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = prot_data[i / 8] & (1 << (i % 8)) ? 1 : 0;

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief performs program operation, see psoc4hv_write.c for details
 * @param bank current flash bank
 * @param buffer pointer to the buffer with data
 * @param offset starting offset in flash bank
 * @param count number of bytes in buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
#define SFLASH_PROGRAMMING_MASK 0x00000100
static int psoc4_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	static const uint8_t write_algo[] = {
#include "../../../contrib/loaders/flash/psoc4hv/psoc4hv_write.inc"
	};

	struct target *target = bank->target;
	struct psoc4_info *p4_info = bank->driver_priv;

	struct working_area *wa_algo_stack;
	int hr = target_alloc_working_area(target, sizeof(write_algo) + PSOC4_STACK_WA_SIZE, &wa_algo_stack);
	if (hr != ERROR_OK)
		return hr;

	size_t wa_avail = target_get_working_area_avail(target);
	/* Header - uint32_t[4], chunk - uint32_t[2] + data */
	wa_avail -= (wa_avail - 16) % (8 + p4_info->row_size);
	const size_t max_num_chunks = wa_avail / (8 + p4_info->row_size);
	uint8_t chunks[wa_avail];

	if (max_num_chunks == 0) {
		LOG_ERROR("No large enough working area available");
		goto exit0;
	}

	struct working_area *wa_buffers;
	hr = target_alloc_working_area(target, wa_avail, &wa_buffers);
	if (hr != ERROR_OK)
		goto exit0;

	hr = target_write_buffer(target, wa_algo_stack->address, sizeof(write_algo), write_algo);
	if (hr != ERROR_OK)
		goto exit1;

	uint32_t header[4] = {
		p4_info->family->cpuss->sysreq,
		p4_info->family->cpuss->sysarg,
		p4_info->row_size,
		0,
	};

	hr = target_write_buffer(target, wa_buffers->address, sizeof(header), (uint8_t *)header);
	if (hr != ERROR_OK)
		goto exit1;

	/* Pass all parameters to the entry point (see psoc4_write.c) */
	struct reg_param reg_params[3];
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, wa_buffers->address);

	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, bank->base + offset);

	init_reg_param(&reg_params[2], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, wa_algo_stack->address + wa_algo_stack->size);

	/* Start the algorithm in the background */
	struct armv7m_algorithm armv7m_algo;
	armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_algo.core_mode = ARM_MODE_THREAD;
	hr = target_start_algorithm(target, 0, NULL, 3, reg_params, wa_algo_stack->address, 0, &armv7m_algo);
	if (hr != ERROR_OK)
		goto exit2;

	if (p4_info->family->prepare) {
		hr = p4_info->family->prepare(target, p4_info);
		if (hr != ERROR_OK)
			goto exit3;
	}

	size_t total_chunks = count / p4_info->row_size;
	size_t chunk_idx = 0;
	progress_init(total_chunks, PROGRAMMING);

	while (total_chunks) {
		uint32_t row_id = offset / p4_info->row_size;

		const uint32_t macro_id = flash_addr_to_macro(bank, bank->base + offset);
		uint32_t param0 = (p4_info->controller_idx << 31) | (macro_id & 0x7F) << 24 | (p4_info->row_size - 1);
		uint32_t param1 = (p4_info->controller_idx << 31) | (row_id & 0x7FFF) << 16 | (row_id & 0xFF);
		if (is_sflash_bank(bank))
			param1 |= SFLASH_PROGRAMMING_MASK;

		buf_set_u32(&chunks[chunk_idx * (8 + p4_info->row_size) + 0], 0, 32, param0);
		buf_set_u32(&chunks[chunk_idx * (8 + p4_info->row_size) + 4], 0, 32, param1);
		memcpy(&chunks[chunk_idx * (8 + p4_info->row_size) + 8], buffer, p4_info->row_size);

		chunk_idx++;
		total_chunks--;
		buffer += p4_info->row_size;
		offset += p4_info->row_size;
		if (chunk_idx == max_num_chunks || total_chunks == 0) {
			hr = target_write_buffer(target, wa_buffers->address + 16, sizeof(chunks), chunks);
			if (hr != ERROR_OK)
				goto exit4;

			hr = target_write_u32(target, wa_buffers->address + 12, chunk_idx);
			if (hr != ERROR_OK)
				goto exit4;

			uint32_t status = UINT32_MAX;
			int64_t t0 = timeval_ms();
			while (status) {
				if (timeval_ms() - t0 > PSOC4_IPC_TIMEOUT_MS) {
					LOG_ERROR("Timeout waiting for flash write algorithm");
					hr = ERROR_FLASH_OPERATION_FAILED;
					goto exit4;
				}

				hr = target_poll(target);
				if (hr != ERROR_OK)
					goto exit4;

				hr = target_read_u32(target, wa_buffers->address + 12, &status);
				if (hr != ERROR_OK)
					goto exit4;

				if (status & PSOC4_SROM_STATUS_MASK || target->state != TARGET_DEBUG_RUNNING) {
					LOG_ERROR("%s", psoc4_sysreq_errorstr(status));
					hr = ERROR_FLASH_OPERATION_FAILED;
					goto exit4;
				}
			}

			chunk_idx = 0;
		}

		progress_left(total_chunks);
	}

exit4:
	progress_done(hr);
exit3:
	target_halt(target);
	target_wait_algorithm(target, 0, NULL, 0, NULL, 0, PSOC4_IPC_TIMEOUT_MS, &armv7m_algo);
exit2:
	for (size_t i = 0; i < sizeof(reg_params) / sizeof(reg_params[0]); i++)
		destroy_reg_param(&reg_params[i]);
exit1:
	target_free_working_area(target, wa_buffers);
exit0:
	target_free_working_area(target, wa_algo_stack);
	target_write_u32(target, p4_info->family->cpuss->sysreq, PSOC4_SYSREQ_NO_REMAP);
	return hr;
}

/** ***********************************************************************************************
 * @brief Performs MassErase operation. This will erase ALL flash banks on PSoC4-HV!
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_mass_erase(struct target *target) {
	struct psoc4_info psoc4_info;
	memset(&psoc4_info, 0, sizeof(psoc4_info));

	int hr = psoc4_read_family(target, &psoc4_info);
	if (hr != ERROR_OK)
		return hr;

	uint8_t protection = 0;
	hr = psoc4_sysreq_silicon_id(target, &psoc4_info, NULL, NULL, &protection);
	if (hr != ERROR_OK)
		return hr;

	hr = psoc4_sysreq_prepare(target, &psoc4_info);
	if (hr != ERROR_OK)
		return hr;

	if (protection == PSOC4_CHIP_PROT_PROTECTED) {
		LOG_INFO("Entering OPEN protection mode, this will erase internal Flash...");
		hr = psoc4_sysreq(target, &psoc4_info, PSOC4_CMD_WRITE_PROTECTION, PSOC4_CHIP_PROT_OPEN, NULL, 0, NULL, false);
		if (hr == ERROR_OK)
			LOG_INFO("Chip protection will take effect after Reset");
	} else {
		LOG_INFO("Performing Mass Erase...");
		uint8_t ram_param[4];
		buf_set_u32(ram_param, 0, 32, 0);
		hr = psoc4_sysreq(target, &psoc4_info, PSOC4_CMD_ERASE_ALL, 0, ram_param, sizeof(ram_param), NULL, false);
	}

	psoc4_sysreq_release(target, &psoc4_info);
	return hr;
}

/** ***********************************************************************************************
 * @brief Intended to perform sector erase which is not supported in PSoC4
 * @param bank current flash bank
 * @param first The first sector to erase
 * @param last The last sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	if (is_sflash_bank(bank)) {
		LOG_WARNING("Erase of the SFlash is not supported by the chip, erase skipped!");
		return ERROR_OK;
	}

	const bool is_p4hv = is_psoc4hv(bank);
	if (first != 0 || last != bank->num_sectors - 1) {
		if (!is_p4hv) {
			LOG_WARNING("Only mass erase available, erase skipped! (psoc4 mass_erase)");
		} else {
			LOG_WARNING("Partial flash erase is not supported by the chip, erase skipped!");
		}
		return ERROR_OK;
	}

	if (!is_p4hv)
		return psoc4_mass_erase(bank->target);

	struct psoc4_info *psoc4_info = bank->driver_priv;
	struct target *target = bank->target;

	int hr = psoc4_sysreq_prepare(target, psoc4_info);
	if (hr != ERROR_OK)
		goto exit0;

	LOG_INFO("Attempting to Bulk-Erase the flash bank %s...", bank->name);

	uint32_t ctrl_idx = (psoc4_info->controller_idx & 0x01u);
	uint32_t ram_param = 0x07 | (ctrl_idx << 3u);
	uint32_t status = 0;
	hr = psoc4_sysreq(target, psoc4_info, PSOC4_CMD_BULK_ERASE, ram_param, NULL, 0, &status, true);
	psoc4_sysreq_release(target, psoc4_info);

	if(hr != ERROR_OK && status == 0xF000000B) {
		LOG_WARNING("Bulk-Erase API is not supported, erase skipped! Use mass erase to erase whole chip (psoc4 mass_erase)");
		hr = ERROR_OK;
	}

exit0:
	return hr;
}

/** ************************************************************************************************
 * @brief display human-readable information about the target into the given buffer
 * @param target current target
 * @param buf - where to put the text for the human to read
 * @param buf_size - the size of the human buffer.
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_silicon_info(struct target *target, char *buf, int buf_size)
{
	uint32_t si_siid;
	uint32_t protection;

	struct psoc4_info p4_minfo;
	memset(&p4_minfo, 0, sizeof(p4_minfo));

	if (target->coreid != 0xFF) {
		struct flash_bank *main_bank, *super_bank;
		int hr = get_main_super_banks(target, &main_bank, &super_bank);
		if (hr != ERROR_OK)
			return hr;

		memcpy(&p4_minfo, main_bank->driver_priv, sizeof(p4_minfo));
		struct psoc4_info *p4_sinfo = super_bank->driver_priv;

		hr = target_read_u32(target, p4_sinfo->family->sflash_base + p4_sinfo->sflash.siid_offs, &si_siid);
		if (hr != ERROR_OK)
			return hr;

		hr = target_read_u32(target, p4_sinfo->family->sflash_base + p4_sinfo->sflash.chip_prot_offs, &protection);
		if (hr != ERROR_OK)
			return hr;

		protection = (protection >> 24) & 0x0f;
		if (protection == PSOC4_CHIP_PROT_VIRGIN)
			protection = PSOC4_CHIP_PROT_OPEN;
		else if (protection == PSOC4_CHIP_PROT_OPEN)
			protection = PSOC4_CHIP_PROT_VIRGIN;

	} else {
		int hr = psoc4_read_family(target, &p4_minfo);
		if (hr != ERROR_OK)
			return hr;

		uint8_t prot;
		hr = psoc4_sysreq_silicon_id(target, &p4_minfo, &si_siid, NULL, &prot);
		if (hr != ERROR_OK)
			return hr;

		si_siid >>= 16;
		protection = prot;
	}

	const char *tcl = "source [find target/cympn.cfg]; "
					  "set siid \"%04X\"; "
					  "if { [dict exists $MPN $siid ] } {"
					  "  set pn [lindex $MPN($siid) 0]; "
					  "  set main_size [lindex $MPN($siid) 2]; "
					  "  return [list $pn $main_size]; "
					  "}";

	char *si_mpn = NULL;
	uint32_t udd_mflash_size = 0;
	char *command = alloc_printf(tcl, si_siid);

	extern struct command_context *global_cmd_ctx;
	Jim_Interp *interp = global_cmd_ctx->interp;
	Jim_Eval(interp, command);
	Jim_Obj *list = Jim_GetResult(interp);
	if (Jim_IsList(list) && Jim_ListLength(interp, list) == 2) {
		si_mpn = strdup(Jim_GetString(Jim_ListGetIndex(interp, list, 0), NULL));
		long l;
		Jim_GetLong(interp, Jim_ListGetIndex(interp, list, 1), &l);
		udd_mflash_size = (uint32_t)(l * 1024u);
	}
	free(command);

	char si_rev_major = "XABCDEFGHIJKLMNOP"[p4_minfo.revision_id >> 4u];
	char si_rev_minor = "X0123456789abcdef"[p4_minfo.revision_id & 0x0F];
	const char *format = "*****************************************\n"
						 "** Silicon: 0x%04X, Family: 0x%02X, Rev.: 0x%02X (%c%c)\n"
						 "** Detected Family: %s\n"
						 "%s"
						 "%s"
						 "** Chip Protection: %s\n"
						 "*****************************************";

	char mpn_str[64];
	char mflash_size_str[64];
	if (si_mpn) {
		snprintf(mpn_str, sizeof(mpn_str), "** Detected Device: %s\n", si_mpn);
		snprintf(mflash_size_str, sizeof(mflash_size_str), "** Detected Main Flash size, kb: %d\n",
				 udd_mflash_size / 1024u);
		free(si_mpn);
	} else {
		snprintf(mpn_str, sizeof(mpn_str), "** Device is not present in the UDD\n");
		snprintf(mflash_size_str, sizeof(mflash_size_str), "** Main Flash size will be auto-detected\n");
	}

	snprintf(buf, buf_size, format, si_siid, p4_minfo.family_id, p4_minfo.revision_id, si_rev_major, si_rev_minor,
			 p4_minfo.family->name, mpn_str, mflash_size_str, psoc4_protection_str(protection));

	return ERROR_OK;
}

static int psoc4_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct target *target = bank->target;
	struct psoc4_info *p4_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		command_print_sameline(cmd,
				 "%s: target state: %s, halt the target for more information\n"
				 "%s: %d bytes (%d rows, %d bytes/row, %d macro)",
				 p4_info->family->name, target_state_name(target), bank->name, bank->size,
				 bank->size / p4_info->row_size, p4_info->row_size, p4_info->num_macro);
		return ERROR_OK;
	}

	uint32_t silicon_id;
	uint8_t protection;
	int hr = psoc4_sysreq_silicon_id(target, p4_info, &silicon_id, NULL, &protection);
	if (hr != ERROR_OK)
		return hr;

	command_print_sameline(cmd,
			 "%s: silicon id: 0x%08X, protection: %s\n"
			 "%s: %d bytes (%d rows, %d bytes/row, %d macro)",
			 p4_info->family->name, silicon_id, psoc4_protection_str(protection), bank->name, bank->size,
			 bank->size / p4_info->row_size, p4_info->row_size, p4_info->num_macro);

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Performs Flash Read operation with ECC error reporting
 * @param bank The bank to read
 * @param buffer The data bytes read
 * @param offset The offset into the chip to read
 * @param count The number of bytes to read
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_flash_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	if (!g_ecc_enabled)
		return default_flash_read(bank, buffer, offset, count);

	assert(offset % 4 == 0);
	assert(count % 4 == 0);
	int hr = target_write_u32(bank->target, FAULT_STATUS(0), 0x00);
	if (hr != ERROR_OK)
		return hr;

	uint32_t address = bank->base + offset;

	while (count) {
		const int lvl = change_debug_level(LOG_LVL_USER);
		target_read_buffer(bank->target, address, 4, buffer);
		change_debug_level(lvl);

		uint32_t status = 0;
		hr = target_read_u32(bank->target, FAULT_STATUS(0), &status);
		if (hr != ERROR_OK)
			return hr;

		if (status & FAULT_STATUS_VALID_MASK) {
			uint32_t er_addr;
			hr = target_read_u32(bank->target, FAULT_DATA0(0), &er_addr);
			if (hr != ERROR_OK)
				return hr;

			LOG_WARNING("ECC Error at address 0x%08X", er_addr);

			hr = target_write_u32(bank->target, FAULT_STATUS(0), 0x00);
			if (hr != ERROR_OK)
				return hr;
		}

		count -= 4;
		buffer += 4;
		address += 4;
	}

	return ERROR_OK;
}

/* flash bank <name> psoc <base> <size> 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(psoc4_flash_bank_command)
{
	struct psoc4_info *p4_info;

	if (CMD_ARGC < 6 || CMD_ARGC > 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	p4_info = calloc(1, sizeof(struct psoc4_info));
	if (CMD_ARGC == 7) {
		if (is_sflash_bank(bank)) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], p4_info->area_idx);
		} else {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], p4_info->controller_idx);
		}
	}

	bank->driver_priv = p4_info;
	bank->default_padded_value = bank->erased_value = 0x00;

	return ERROR_OK;
}

COMMAND_HANDLER(psoc4_handle_mass_erase)
{
	/* Ignore first parameter for backward compatibility */
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	return psoc4_mass_erase(target);
}

COMMAND_HANDLER(psoc4_handle_chip_protect)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	if (target->coreid == 0xFF) {
		LOG_ERROR("The 'chip_protect' command is not available via mem_ap");
		return ERROR_FAIL;
	}

	struct flash_bank *mflash_bank;
	int hr = get_flash_bank_by_addr(target, 0, true, &mflash_bank);
	if (hr != ERROR_OK)
		return hr;

	struct psoc4_info *p4_info = mflash_bank->driver_priv;
	uint8_t protection = 0;
	hr = psoc4_sysreq_silicon_id(target, p4_info, NULL, NULL, &protection);
	if (hr != ERROR_OK)
		return hr;

	if (protection != PSOC4_CHIP_PROT_OPEN) {
		LOG_ERROR("Operation not supported when chip is in %s state", psoc4_protection_str(protection));
		return ERROR_FAIL;
	}

	uint8_t *row_protection;
	hr = psoc4_read_row_protection(target, &row_protection);
	if (hr != ERROR_OK)
		return hr;

	hr = psoc4_write_row_protection(target, row_protection, true);
	if (hr == ERROR_OK)
		LOG_INFO("Chip protection will take effect after Reset");

	free(row_protection);
	return hr;
}

COMMAND_HANDLER(psoc4_handle_reset_halt)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int hr;
	struct target *target = get_current_target(CMD_CTX);

	if (target->coreid == 0xFF) {
		LOG_ERROR("The 'reset_halt' command is not available via mem_ap");
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED) {
		hr = target_halt(target);
		if (hr != ERROR_OK)
			return hr;

		hr = target_wait_state(target, TARGET_HALTED, PSOC4_IPC_TIMEOUT_MS);
		if (hr != ERROR_OK)
			return hr;
	}

	struct flash_bank *main_flash;
	hr = get_flash_bank_by_addr(target, 0, true, &main_flash);
	if (hr != ERROR_OK)
		return hr;

	struct psoc4_info *p4_info = main_flash->driver_priv;
	hr = target_write_u32(target, p4_info->family->cpuss->sysreq, PSOC4_SYSREQ_NO_REMAP);
	if (hr != ERROR_OK)
		return hr;

	uint32_t reset_addr;
	/* Read and validate Reset Vector value */
	hr = target_read_u32(target, 0x00000004, &reset_addr);
	if (hr != ERROR_OK)
		return hr;

	if (reset_addr > main_flash->size || reset_addr < 0x40) {
		LOG_INFO("Entry Point address invalid (0x%08X), reset_halt skipped", reset_addr);
		return ERROR_OK;
	}

	/* Set breakpoint at User Application entry point */
	hr = breakpoint_add(target, reset_addr, 2, BKPT_HARD);
	if (hr != ERROR_OK)
		return hr;

	const struct armv7m_common *cm = target_to_armv7m(target);

	/* M0S8 platform reboots immediately after issuing SYSRESETREQ
	 * this disables SWD/JTAG pins momentarily and may break communication
	 * Ignoring return value of mem_ap_write_atomic_u32 seems to be ok here */

	/* Reset the CM0 by asserting SYSRESETREQ */
	LOG_INFO("%s: bkpt @0x%08X, issuing SYSRESETREQ", target->cmd_name, reset_addr);
	mem_ap_write_atomic_u32(cm->debug_ap, NVIC_AIRCR, AIRCR_VECTKEY | AIRCR_SYSRESETREQ);

	dap_invalidate_cache(cm->debug_ap->dap);
	register_cache_invalidate(target->reg_cache);

	/* Target is now running, call appropriate callbacks */
	target->state = TARGET_RUNNING;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	enum log_levels lvl = change_debug_level(LOG_LVL_USER);
	int64_t t0 = timeval_ms();
	while ((timeval_ms() - t0) < PSOC4_IPC_TIMEOUT_MS) {
		dap_dp_init(cm->debug_ap->dap);
		target->examined = false;
		hr = cortex_m_examine(target);
		if (hr == ERROR_OK)
			break;
		alive_sleep(5);
	}
	change_debug_level(lvl);

	hr = target_wait_state(target, TARGET_HALTED, PSOC4_IPC_TIMEOUT_MS);
	if (hr != ERROR_OK)
		return hr;

	/* Remove the break point */
	breakpoint_remove(target, reset_addr);

	return ERROR_OK;
}

COMMAND_HANDLER(psoc4_handle_silicon_info)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	char buf[1024];
	int hr = psoc4_silicon_info(get_current_target(CMD_CTX), buf, sizeof(buf));
	if (hr == ERROR_OK)
		LOG_USER("%s", buf);

	return hr;
}

COMMAND_HANDLER(psoc4_handle_ecc_error_reporting)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], g_ecc_enabled);

	struct target *target = get_current_target(CMD_CTX);

	/* Disable FAULT[1] */
	int hr = target_write_u32(target, FAULT_CTL(1), 0);
	if (hr != ERROR_OK)
		return hr;

	hr = target_write_u32(target, FAULT_MASK0(1), 0);
	if (hr != ERROR_OK)
		return hr;

	/* Configure FAULT[0] */
	hr = target_write_u32(target, FAULT_CTL(0), 0);
	if (hr != ERROR_OK)
		return hr;

	const uint32_t fault_mask0 = g_ecc_enabled ? FAULT_MASK0_ENABLE_ECC_MASK : 0;
	hr = target_write_u32(target, FAULT_MASK0(0), fault_mask0);
	if (hr != ERROR_OK)
		return hr;

	LOG_INFO("ECC error reporting is now %s", g_ecc_enabled ? "Enabled" : "Disabled");
	return ERROR_OK;
}

static const struct command_registration psoc4_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = psoc4_handle_mass_erase,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Erases all flash banks and/or unlocks the chip",
	},
	{
		.name = "chip_protect",
		.handler = psoc4_handle_chip_protect,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Moves the part to PROTECTED state",
	},
	{
		.name = "reset_halt",
		.handler = psoc4_handle_reset_halt,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Tries to simulate broken Vector Catch",
	},
	{
		.name = "silicon_info",
		.handler = psoc4_handle_silicon_info,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Prints detailed info about the connected silicon",
	},
	{
		.name = "ecc_error_reporting",
		.handler = psoc4_handle_ecc_error_reporting,
		.mode = COMMAND_EXEC,
		.usage = "on|off",
		.help = "Controls ECC error reporting",
	},
	COMMAND_REGISTRATION_DONE,
};

static const struct command_registration psoc4_command_handlers[] = {
	{
		.name = "psoc4",
		.mode = COMMAND_ANY,
		.help = "PSoC 4 flash command group",
		.usage = "",
		.chain = psoc4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE,
};

const struct flash_driver psoc4hv_flash = {
	.name = "psoc4hv",
	.commands = psoc4_command_handlers,
	.flash_bank_command = psoc4_flash_bank_command,
	.erase = psoc4_erase,
	.protect = psoc4_protect,
	.write = psoc4_write,
	.read = psoc4_flash_read,
	.probe = psoc4_probe,
	.auto_probe = psoc4_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = psoc4_protect_check,
	.info = psoc4_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

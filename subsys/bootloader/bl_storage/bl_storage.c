/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bl_storage.h"
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <nrf.h>
#include <assert.h>
#include <nrfx_nvmc.h>
#include <pm_config.h>
#include <zephyr/sys/printk.h>

/** A single monotonic counter. It consists of a description value, a 'type',
 *  to know what this counter counts. Further, it has a number of slots
 *  allocated to it. Each time the counter is updated, a new slot is written.
 *  This way, the counter can be updated without erasing anything. This is
 *  necessary so the counter can be used in the OTP section of the UICR
 *  (available on e.g. nRF91 and nRF53).
 */
struct monotonic_counter {
	uint16_t description; /* Counter description. What the counter is used for. See COUNTER_DESC_*. */
	uint16_t num_counter_slots; /* Number of entries in 'counter_slots' list. */
	uint16_t counter_slots[1];
};

/** The second data structure in the provision page. It has unknown length since
 *  'counters' is repeated. Note that each entry in counters also has unknown
 *  length, and each entry can have different length from the others, so the
 *  entries beyond the first cannot be accessed via array indices.
 */
struct counter_collection {
	uint16_t type; /* Must be "monotonic counter". */
	uint16_t num_counters; /* Number of entries in 'counters' list. */
	struct monotonic_counter counters[1];
};

#define TYPE_COUNTERS 1 /* Type referring to counter collection. */
#define COUNTER_DESC_VERSION 1 /* Counter description value for firmware version. */

#ifdef PM_S0_OFFSET /* partition manager available */
/* upgradable mcuboot bootloader option, to validate and select which mcuboot slot to boot:
 * optional trailer at the end of of flash partitions for mcuboot slot0 and slot1
 * 1) mcuboot erase flash sector for mcuboot_image_trailer
 * 2) mcuboot writes new image to flash (e.g. "mcumgr image upload")
 * 3) mcuboot writes mcuboot_image_trailer with value mcuboot_image_trailer_init as final step of image upload
 * 4) device is rebooted via "mcumgr reboot" or external reset
 * 5) b0 checks if one or both mcuboot slots have a trailer.magic, if not normal boot proceeds
 * 6) b0 updates mcuboot_image_trailer.status appropriately:
 *    From TESTING to BOOTING (if any TESTING)
 *    From BOOTING to INACTIVE (if any BOOTING)
 * 7) b0 gives a slot 1st priority if its trailer.status=TESTING
	  otherwise it prioritizes trailer.status=ACTIVE
 * 8) mcuboot changes trailer.status of booted image to ACTIVE if confirmed by "mcumgr image confirm"
 *    and simultaneously marks the not-booted image as INACTIVE
 * NOTE: if any image fails to validate, the other bank will still be attempted as well.
 *       Hence b0 can "fallback" to an image otherwise marked as INACTIVE.
 */

/* Note1: interface cloned by mcuboot/boot/boot_serial/src/boot_serial.c */
/* Note2: these constants rely on bits in flash can be changed from 1 to 0 without erasing */
#define MCUBOOT_IMAGE_TRAILER_STATUS_UNKNOWN  0xFFFFFFFF
#define MCUBOOT_IMAGE_TRAILER_STATUS_TESTING  0xFFFFFFFE
#define MCUBOOT_IMAGE_TRAILER_STATUS_BOOTING  0xFFFFFFFC
#define MCUBOOT_IMAGE_TRAILER_STATUS_ACTIVE   0xFFFFFFF8
#define MCUBOOT_IMAGE_TRAILER_STATUS_INACTIVE 0x00000000
#define MCUBOOT_IMAGE_TRAILER_MAGIC_SZ 3

typedef struct {
    uint32_t status; /* mcuboot writes TESTING or ACTIVE, b0 writes BOOTING if TESTING found */
    uint32_t magic[MCUBOOT_IMAGE_TRAILER_MAGIC_SZ]; /* mcuboot writes*/
} mcuboot_image_trailer;

const mcuboot_image_trailer mcuboot_image_trailer_init = {
	.status = MCUBOOT_IMAGE_TRAILER_STATUS_TESTING,
	.magic = {
		0x1234beef,
		0xbeef1234,
		0x1234beef
	}
};

static uint32_t check_and_fix_image_trailer(const char* prefix, const mcuboot_image_trailer* trailer)
{
	int result = MCUBOOT_IMAGE_TRAILER_STATUS_UNKNOWN;
	if (memcmp(&trailer->magic, &mcuboot_image_trailer_init.magic, sizeof(trailer->magic)) == 0) {
		result = trailer->status;
		switch (trailer->status) {
		case MCUBOOT_IMAGE_TRAILER_STATUS_TESTING:
			/* change status to BOOTING */
			printk("%s: TESTING => BOOTING\n\r", prefix);
			__DSB(); /* Because of nRF9160 Erratum 7 */
			nrfx_nvmc_word_write((uint32_t)&trailer->status, MCUBOOT_IMAGE_TRAILER_STATUS_BOOTING);
			break;
		case MCUBOOT_IMAGE_TRAILER_STATUS_BOOTING:
			printk("%s: BOOTING => INACTIVE (boot once)\n\r", prefix);
			__DSB(); /* Because of nRF9160 Erratum 7 */
			nrfx_nvmc_word_write((uint32_t)&trailer->status, MCUBOOT_IMAGE_TRAILER_STATUS_INACTIVE);
			break;
		case MCUBOOT_IMAGE_TRAILER_STATUS_ACTIVE:
			printk("%s: ACTIVE\n\r", prefix);
			break;
		case MCUBOOT_IMAGE_TRAILER_STATUS_INACTIVE:
			printk("%s: INACTIVE\n\r", prefix);
			break;
		case MCUBOOT_IMAGE_TRAILER_STATUS_UNKNOWN:
			printk("%s: UNKNOWN\n\r", prefix);
			break;
		default:
			printk("%s: UNKNOWN, status=0x%"PRIu32"\n\r", prefix, trailer->status);
		}
	} else {
		printk("%s: UNKNOWN (no image trailer)\n\r", prefix);
	}
	return result;
}

slot_priority slot_priority_from_image_trailers(void)
{
	/* preserve behavior of b0 if no magic headers are present, by "prioritizing" S0 as default */
	int result = SLOT_PRIORITY_S0;
	uint32_t s0_addr = s0_address_read();
	uint32_t s1_addr = s1_address_read();
	/* sanity check that the addresses from "provisioning" and the PM config match */
	if (s0_addr == PM_S0_OFFSET && s1_addr == PM_S1_OFFSET) {
		const mcuboot_image_trailer* s0 = (mcuboot_image_trailer*)(s0_addr + PM_S0_SIZE - sizeof(mcuboot_image_trailer));
		const mcuboot_image_trailer* s1 = (mcuboot_image_trailer*)(s1_addr + PM_S1_SIZE - sizeof(mcuboot_image_trailer));
		uint32_t s0_status = check_and_fix_image_trailer("slot0", s0);
		uint32_t s1_status = check_and_fix_image_trailer("slot1", s1);
		switch (s1_status) {
		case MCUBOOT_IMAGE_TRAILER_STATUS_TESTING:
		case MCUBOOT_IMAGE_TRAILER_STATUS_BOOTING:
		case MCUBOOT_IMAGE_TRAILER_STATUS_ACTIVE:
			switch (s0_status) {
			case MCUBOOT_IMAGE_TRAILER_STATUS_TESTING:
			case MCUBOOT_IMAGE_TRAILER_STATUS_BOOTING:
				break; /* SLOT_PRIORITY_S0 */
			default:
				result = SLOT_PRIORITY_S1;
			}
		}
	}
	return result;
}

#else

slot_priority slot_priority_from_image_trailers(void)
{
	return SLOT_PRIORITY_S0;
}
#endif


uint32_t s0_address_read(void)
{
	uint32_t addr = BL_STORAGE->s0_address;

	__DSB(); /* Because of nRF9160 Erratum 7 */
	return addr;
}

uint32_t s1_address_read(void)
{
	uint32_t addr = BL_STORAGE->s1_address;

	__DSB(); /* Because of nRF9160 Erratum 7 */
	return addr;
}

uint32_t num_public_keys_read(void)
{
	uint32_t num_pk = BL_STORAGE->num_public_keys;

	__DSB(); /* Because of nRF9160 Erratum 7 */
	return num_pk;
}

/* Value written to the invalidation token when invalidating an entry. */
#define INVALID_VAL 0xFFFF0000

static bool key_is_valid(uint32_t key_idx)
{
	bool ret = (BL_STORAGE->key_data[key_idx].valid != INVALID_VAL);

	__DSB(); /* Because of nRF9160 Erratum 7 */
	return ret;
}

int verify_public_keys(void)
{
	for (uint32_t n = 0; n < num_public_keys_read(); n++) {
		if (key_is_valid(n)) {
			for (uint32_t i = 0; i < SB_PUBLIC_KEY_HASH_LEN / 2; i++) {
				const uint16_t *hash_as_halfwords =
					(const uint16_t *)BL_STORAGE->key_data[n].hash;
				uint16_t halfword = nrfx_nvmc_otp_halfword_read(
					(uint32_t)&hash_as_halfwords[i]);
				if (halfword == 0xFFFF) {
					return -EHASHFF;
				}
			}
		}
	}
	return 0;
}

int public_key_data_read(uint32_t key_idx, uint8_t *p_buf)
{
	const volatile uint8_t *p_key;

	if (!key_is_valid(key_idx)) {
		return -EINVAL;
	}

	if (key_idx >= num_public_keys_read()) {
		return -EFAULT;
	}

	p_key = BL_STORAGE->key_data[key_idx].hash;

	/* Ensure word alignment, since the data is stored in memory region
	 * with word sized read limitation. Perform both build time and run
	 * time asserts to catch the issue as soon as possible.
	 */
	BUILD_ASSERT(offsetof(struct bl_storage_data, key_data) % 4 == 0);
	__ASSERT(((uint32_t)p_key % 4 == 0), "Key address is not word aligned");

	otp_copy32(p_buf, (volatile uint32_t *restrict)p_key, SB_PUBLIC_KEY_HASH_LEN);

	return SB_PUBLIC_KEY_HASH_LEN;
}

void invalidate_public_key(uint32_t key_idx)
{
	const volatile uint32_t *invalidation_token =
			&BL_STORAGE->key_data[key_idx].valid;

	if (*invalidation_token != INVALID_VAL) {
		/* Write if not already written. */
		__DSB(); /* Because of nRF9160 Erratum 7 */
		nrfx_nvmc_word_write((uint32_t)invalidation_token, INVALID_VAL);
	}
}


/** Get the counter_collection data structure in the provision data. */
static const struct counter_collection *get_counter_collection(void)
{
	struct counter_collection *collection = (struct counter_collection *)
		&BL_STORAGE->key_data[num_public_keys_read()];
	return nrfx_nvmc_otp_halfword_read((uint32_t)&collection->type) == TYPE_COUNTERS
		? collection : NULL;
}


/** Get one of the (possibly multiple) counters in the provision data.
 *
 *  param[in]  description  Which counter to get. See COUNTER_DESC_*.
 */
static const struct monotonic_counter *get_counter_struct(uint16_t description)
{
	const struct counter_collection *counters = get_counter_collection();

	if (counters == NULL) {
		return NULL;
	}

	const struct monotonic_counter *current = counters->counters;

	for (size_t i = 0; i < nrfx_nvmc_otp_halfword_read(
		(uint32_t)&counters->num_counters); i++) {
		uint16_t num_slots = nrfx_nvmc_otp_halfword_read(
					(uint32_t)&current->num_counter_slots);

		if (nrfx_nvmc_otp_halfword_read((uint32_t)&current->description) == description) {
			return current;
		}

		current = (const struct monotonic_counter *)
					&current->counter_slots[num_slots];
	}
	return NULL;
}


uint16_t num_monotonic_counter_slots(void)
{
	const struct monotonic_counter *counter
			= get_counter_struct(COUNTER_DESC_VERSION);
	uint16_t num_slots = 0;

	if (counter != NULL) {
		num_slots = nrfx_nvmc_otp_halfword_read((uint32_t)&counter->num_counter_slots);
	}
	return num_slots != 0xFFFF ? num_slots : 0;
}


/** Function for getting the current value and the first free slot.
 *
 * @param[out]  free_slot  Pointer to the first free slot. Can be NULL.
 *
 * @return The current value of the counter (the highest value before the first
 *         free slot).
 */
static uint16_t get_counter(const uint16_t **free_slot)
{
	uint16_t highest_counter = 0;
	const uint16_t *addr = NULL;
	const uint16_t *slots =
			get_counter_struct(COUNTER_DESC_VERSION)->counter_slots;
	uint16_t num_slots = num_monotonic_counter_slots();

	for (uint32_t i = 0; i < num_slots; i++) {
		uint16_t counter = ~nrfx_nvmc_otp_halfword_read((uint32_t)&slots[i]);

		if (counter == 0) {
			addr = &slots[i];
			break;
		}
		if (highest_counter < counter) {
			highest_counter = counter;
		}
	}

	if (free_slot != NULL) {
		*free_slot = addr;
	}
	return highest_counter;
}


uint16_t get_monotonic_counter(void)
{
	return get_counter(NULL);
}


int set_monotonic_counter(uint16_t new_counter)
{
	const uint16_t *next_counter_addr;
	uint16_t counter = get_counter(&next_counter_addr);

	if (new_counter <= counter) {
		/* Counter value must increase. */
		return -EINVAL;
	}

	if (next_counter_addr == NULL) {
		/* No more room. */
		return -ENOMEM;
	}

	nrfx_nvmc_halfword_write((uint32_t)next_counter_addr, ~new_counter);
	return 0;
}

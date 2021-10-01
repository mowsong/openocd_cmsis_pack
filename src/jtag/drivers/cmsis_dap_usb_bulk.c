/***************************************************************************
 *   Copyright (C) 2018 by Mickaël Thomas                                  *
 *   mickael9@gmail.com                                                    *
 *                                                                         *
 *   Copyright (C) 2016 by Maksym Hilliaka                                 *
 *   oter@frozen-team.com                                                  *
 *                                                                         *
 *   Copyright (C) 2016 by Phillip Pearson                                 *
 *   pp@myelin.co.nz                                                       *
 *                                                                         *
 *   Copyright (C) 2014 by Paul Fertser                                    *
 *   fercerpav@gmail.com                                                   *
 *                                                                         *
 *   Copyright (C) 2013 by mike brown                                      *
 *   mike@theshedworks.org.uk                                              *
 *                                                                         *
 *   Copyright (C) 2013 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2021 by Bohdan Tymkiv                                   *
 *   bohdan.tymkiv@infineon.com bohdan200@gmail.com                        *
 *                                                                         *
 *   Copyright (C) 2021                                                    *
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

#include <helper/system.h>
#include <libusb.h>
#include <helper/log.h>
#include <helper/replacements.h>
#include <helper/time_support.h>

#include "cmsis_dap.h"

struct cmsis_dap_backend_data {
	struct libusb_context *usb_ctx;
	struct libusb_device_handle *dev_handle;
	unsigned int ep_out;
	unsigned int ep_in;
	int interface;
};

static int cmsis_dap_usb_interface = -1;

static void cmsis_dap_usb_close(struct cmsis_dap *dap);
static int cmsis_dap_usb_alloc(struct cmsis_dap *dap, unsigned int pkt_sz);

static int cmsis_dap_usb_open(struct cmsis_dap *dap, uint16_t vids[], uint16_t pids[], char *serial)
{
	int err;
	struct libusb_context *ctx;
	struct libusb_device **device_list;

	err = libusb_init(&ctx);
	if (err) {
		LOG_ERROR("libusb initialization failed: %s", libusb_strerror(err));
		return ERROR_FAIL;
	}

	int num_devices = libusb_get_device_list(ctx, &device_list);
	if (num_devices < 0) {
		LOG_ERROR("could not enumerate USB devices: %s", libusb_strerror(num_devices));
		libusb_exit(ctx);
		return ERROR_FAIL;
	}

	for (int i = 0; i < num_devices; i++) {
		struct libusb_device *dev = device_list[i];
		struct libusb_device_descriptor dev_desc;

		err = libusb_get_device_descriptor(dev, &dev_desc);
		if (err) {
			LOG_ERROR("could not get device descriptor for device %d: %s", i, libusb_strerror(err));
			continue;
		}

		/* Match VID/PID */

		bool id_match = false;
		bool id_filter = vids[0] || pids[0];
		for (int id = 0; vids[id] || pids[id]; id++) {
			id_match = !vids[id] || dev_desc.idVendor == vids[id];
			id_match &= !pids[id] || dev_desc.idProduct == pids[id];

			if (id_match)
				break;
		}

		if (id_filter && !id_match)
			continue;

		/* Don't continue if we asked for a serial number and the device doesn't have one */
		if (dev_desc.iSerialNumber == 0 && serial && serial[0])
			continue;

		struct libusb_device_handle *dev_handle = NULL;
		err = libusb_open(dev, &dev_handle);
		if (err) {
			/* It's to be expected that most USB devices can't be opened
			 * so only report an error if it was explicitly selected
			 */
			if (id_filter) {
				LOG_ERROR("could not open device 0x%04x:0x%04x: %s",
						dev_desc.idVendor, dev_desc.idProduct, libusb_strerror(err));
			} else {
				LOG_DEBUG("could not open device 0x%04x:0x%04x: %s",
						dev_desc.idVendor, dev_desc.idProduct, libusb_strerror(err));
			}
			continue;
		}

		/* Match serial number */

		bool serial_match = false;
		char dev_serial[256] = {0};
		if (dev_desc.iSerialNumber > 0) {
			err = libusb_get_string_descriptor_ascii(
					dev_handle, dev_desc.iSerialNumber,
					(uint8_t *)dev_serial, sizeof(dev_serial));

			if (err < 0) {
				const char *msg = "could not read serial number for device 0x%04x:0x%04x: %s";
				if (serial)
					LOG_WARNING(msg, dev_desc.idVendor, dev_desc.idProduct,
								libusb_strerror(err));
				else
					LOG_DEBUG(msg, dev_desc.idVendor, dev_desc.idProduct,
								libusb_strerror(err));
			} else if (serial && strncmp(dev_serial, serial, sizeof(dev_serial)) == 0) {
				serial_match = true;
			}
		}

		if (serial && !serial_match) {
			libusb_close(dev_handle);
			continue;
		}

		/* Find the CMSIS-DAP string in product string */

		bool cmsis_dap_in_product_str = false;
		char product_string[256] = {0};
		if (dev_desc.iProduct > 0) {
			err = libusb_get_string_descriptor_ascii(
					dev_handle, dev_desc.iProduct,
					(uint8_t *)product_string, sizeof(product_string));
			if (err < 0) {
				LOG_WARNING("could not read product string for device 0x%04x:0x%04x: %s",
						dev_desc.idVendor, dev_desc.idProduct, libusb_strerror(err));
			} else if (strstr(product_string, "CMSIS-DAP")) {
				LOG_DEBUG("found product string of 0x%04x:0x%04x '%s'",
						  dev_desc.idVendor, dev_desc.idProduct, product_string);
				cmsis_dap_in_product_str = true;
			}
		}

		bool device_identified_reliably = cmsis_dap_in_product_str
											|| serial_match || id_match;

		/* Find the CMSIS-DAP interface */

		for (int config = 0; config < dev_desc.bNumConfigurations; config++) {
			struct libusb_config_descriptor *config_desc;
			err = libusb_get_config_descriptor(dev, config, &config_desc);
			if (err) {
				LOG_ERROR("could not get configuration descriptor %d for device 0x%04x:0x%04x: %s",
						config, dev_desc.idVendor, dev_desc.idProduct, libusb_strerror(err));
				continue;
			}

			LOG_DEBUG("enumerating interfaces of 0x%04x:0x%04x",
					  dev_desc.idVendor, dev_desc.idProduct);
			int config_num = config_desc->bConfigurationValue;
			const struct libusb_interface_descriptor *intf_desc_candidate = NULL;
			const struct libusb_interface_descriptor *intf_desc_found = NULL;

			for (int interface = 0; interface < config_desc->bNumInterfaces; interface++) {
				const struct libusb_interface_descriptor *intf_desc = &config_desc->interface[interface].altsetting[0];
				int interface_num = intf_desc->bInterfaceNumber;

				/* Skip this interface if another one was requested explicitly */
				if (cmsis_dap_usb_interface != -1 && cmsis_dap_usb_interface != interface_num)
					continue;

				/* CMSIS-DAP v2 spec says:
				 *
				 * CMSIS-DAP with default V2 configuration uses WinUSB and is therefore faster.
				 * Optionally support for streaming SWO trace is provided via an additional USB endpoint.
				 *
				 * The WinUSB configuration requires custom class support with the interface setting
				 *     Class Code: 0xFF (Vendor specific)
				 *     Subclass: 0x00
				 *     Protocol code: 0x00
				 *
				 * Depending on the configuration it uses the following USB endpoints which should be configured
				 * in the interface descriptor in this order:
				 *  - Endpoint 1: Bulk Out – used for commands received from host PC.
				 *  - Endpoint 2: Bulk In – used for responses send to host PC.
				 *  - Endpoint 3: Bulk In (optional) – used for streaming SWO trace (if enabled with SWO_STREAM).
				 */

				/* Search for "CMSIS-DAP" in the interface string */
				bool cmsis_dap_in_interface_str = false;
				if (intf_desc->iInterface != 0) {

					char interface_str[256] = {0};

					err = libusb_get_string_descriptor_ascii(
							dev_handle, intf_desc->iInterface,
							(uint8_t *)interface_str, sizeof(interface_str));
					if (err < 0) {
						LOG_DEBUG("could not read interface string %d for device 0x%04x:0x%04x: %s",
								  intf_desc->iInterface,
								  dev_desc.idVendor, dev_desc.idProduct,
								  libusb_strerror(err));
					} else if (strstr(interface_str, "CMSIS-DAP")) {
						cmsis_dap_in_interface_str = true;
						LOG_DEBUG("found interface %d string '%s'",
								  interface_num, interface_str);
					}
				}

				/* Bypass the following check if this interface was explicitly requested. */
				if (cmsis_dap_usb_interface == -1) {
					if (!cmsis_dap_in_product_str && !cmsis_dap_in_interface_str)
						continue;
				}

				/* check endpoints */
				if (intf_desc->bNumEndpoints < 2) {
					LOG_DEBUG("skipping interface %d, has only %d endpoints",
							  interface_num, intf_desc->bNumEndpoints);
					continue;
				}

				if ((intf_desc->endpoint[0].bmAttributes & 3) != LIBUSB_TRANSFER_TYPE_BULK ||
						(intf_desc->endpoint[0].bEndpointAddress & 0x80) != LIBUSB_ENDPOINT_OUT) {
					LOG_DEBUG("skipping interface %d, endpoint[0] is not bulk out",
							  interface_num);
					continue;
				}

				if ((intf_desc->endpoint[1].bmAttributes & 3) != LIBUSB_TRANSFER_TYPE_BULK ||
						(intf_desc->endpoint[1].bEndpointAddress & 0x80) != LIBUSB_ENDPOINT_IN) {
					LOG_DEBUG("skipping interface %d, endpoint[1] is not bulk in",
							  interface_num);
					continue;
				}

				/* We can rely on the interface is really CMSIS-DAP if
				 * - we've seen CMSIS-DAP in the interface string
				 * - config asked explicitly for an interface number
				 * - the device has only one interface
				 * The later two cases should be honored only if we know
				 * we are on the right device */
				bool intf_identified_reliably = cmsis_dap_in_interface_str
							|| (device_identified_reliably &&
									(cmsis_dap_usb_interface != -1
									 || config_desc->bNumInterfaces == 1));

				if (intf_desc->bInterfaceClass != LIBUSB_CLASS_VENDOR_SPEC ||
						intf_desc->bInterfaceSubClass != 0 || intf_desc->bInterfaceProtocol != 0) {
					/* If the interface is reliably identified
					 * then we need not insist on setting USB class, subclass and protocol
					 * exactly as the specification requires.
					 * At least KitProg3 uses class 0 contrary to the specification */
					if (intf_identified_reliably) {
						LOG_WARNING("Using CMSIS-DAPv2 interface %d with wrong class %" PRId8
								  " subclass %" PRId8 " or protocol %" PRId8,
								  interface_num,
								  intf_desc->bInterfaceClass,
								  intf_desc->bInterfaceSubClass,
								  intf_desc->bInterfaceProtocol);
					} else {
						LOG_DEBUG("skipping interface %d, class %" PRId8
								  " subclass %" PRId8 " protocol %" PRId8,
								  interface_num,
								  intf_desc->bInterfaceClass,
								  intf_desc->bInterfaceSubClass,
								  intf_desc->bInterfaceProtocol);
						continue;

					}
				}

				if (intf_identified_reliably) {
					/* That's the one! */
					intf_desc_found = intf_desc;
					break;
				}

				if (!intf_desc_candidate && device_identified_reliably) {
					/* This interface looks suitable for CMSIS-DAP. Store the pointer to it
					 * and keep searching for another one with CMSIS-DAP in interface string */
					intf_desc_candidate = intf_desc;
				}
			}

			if (!intf_desc_found) {
				/* We were not able to identify reliably which interface is CMSIS-DAP.
				 * Let's use the first suitable if we found one */
				intf_desc_found = intf_desc_candidate;
			}

			if (!intf_desc_found) {
				libusb_free_config_descriptor(config_desc);
				continue;
			}

			/* We've chosen an interface, connect to it */
			int interface_num = intf_desc_found->bInterfaceNumber;
			int packet_size = intf_desc_found->endpoint[0].wMaxPacketSize;
			int ep_out = intf_desc_found->endpoint[0].bEndpointAddress;
			int ep_in = intf_desc_found->endpoint[1].bEndpointAddress;

			libusb_free_config_descriptor(config_desc);
			libusb_free_device_list(device_list, true);

			LOG_INFO("Using CMSIS-DAPv2 interface with VID:PID=0x%04x:0x%04x, serial=%s",
					dev_desc.idVendor, dev_desc.idProduct, dev_serial);

			dap->vid = dev_desc.idVendor;
			dap->pid = dev_desc.idProduct;

			int current_config;
			err = libusb_get_configuration(dev_handle, &current_config);
			if (err) {
				LOG_ERROR("could not find current configuration: %s", libusb_strerror(err));
				libusb_close(dev_handle);
				libusb_exit(ctx);
				return ERROR_FAIL;
			}

			if (config_num != current_config) {
				err = libusb_set_configuration(dev_handle, config_num);
				if (err) {
					LOG_ERROR("could not set configuration: %s", libusb_strerror(err));
					libusb_close(dev_handle);
					libusb_exit(ctx);
					return ERROR_FAIL;
				}
			}

			err = libusb_claim_interface(dev_handle, interface_num);
			if (err)
				LOG_WARNING("could not claim interface: %s", libusb_strerror(err));

			dap->bdata = malloc(sizeof(struct cmsis_dap_backend_data));
			if (!dap->bdata) {
				LOG_ERROR("unable to allocate memory");
				libusb_release_interface(dev_handle, interface_num);
				libusb_close(dev_handle);
				libusb_exit(ctx);
				return ERROR_FAIL;
			}

			dap->packet_size = packet_size;
			dap->packet_buffer_size = packet_size;
			dap->bdata->usb_ctx = ctx;
			dap->bdata->dev_handle = dev_handle;
			dap->bdata->ep_out = ep_out;
			dap->bdata->ep_in = ep_in;
			dap->bdata->interface = interface_num;

			dap->packet_buffer = malloc(dap->packet_buffer_size);
			if (!dap->packet_buffer) {
				LOG_ERROR("unable to allocate memory");
				cmsis_dap_usb_close(dap);
				return ERROR_FAIL;
			}

			dap->command = dap->packet_buffer;
			dap->response = dap->packet_buffer;

			return ERROR_OK;
		}

		libusb_close(dev_handle);
	}

	libusb_free_device_list(device_list, true);

	libusb_exit(ctx);
	return ERROR_FAIL;
}

static void cmsis_dap_usb_close(struct cmsis_dap *dap)
{
	libusb_release_interface(dap->bdata->dev_handle, dap->bdata->interface);
	libusb_close(dap->bdata->dev_handle);
	libusb_exit(dap->bdata->usb_ctx);
	free(dap->bdata);
	dap->bdata = NULL;
	free(dap->packet_buffer);
	dap->packet_buffer = NULL;
}

static int cmsis_dap_usb_read(struct cmsis_dap *dap, int timeout_ms)
{
	int transferred = 0;
	int err;

	err = libusb_bulk_transfer(dap->bdata->dev_handle, dap->bdata->ep_in,
							dap->packet_buffer, dap->packet_size, &transferred, timeout_ms);
	if (err) {
		if (err == LIBUSB_ERROR_TIMEOUT) {
			return ERROR_TIMEOUT_REACHED;
		} else {
			LOG_ERROR("error reading data: %s", libusb_strerror(err));
			return ERROR_FAIL;
		}
	}

	memset(&dap->packet_buffer[transferred], 0, dap->packet_buffer_size - transferred);

	return transferred;
}

static int cmsis_dap_usb_write(struct cmsis_dap *dap, int txlen, int timeout_ms)
{
	int transferred = 0;
	int err;

	/* skip the first byte that is only used by the HID backend */
	err = libusb_bulk_transfer(dap->bdata->dev_handle, dap->bdata->ep_out,
							dap->packet_buffer, txlen, &transferred, timeout_ms);
	if (err) {
		if (err == LIBUSB_ERROR_TIMEOUT) {
			return ERROR_TIMEOUT_REACHED;
		} else {
			LOG_ERROR("error writing data: %s", libusb_strerror(err));
			return ERROR_FAIL;
		}
	}

	return transferred;
}

static int cmsis_dap_usb_alloc(struct cmsis_dap *dap, unsigned int pkt_sz)
{
	uint8_t *buf = malloc(pkt_sz);
	if (!buf) {
		LOG_ERROR("unable to allocate CMSIS-DAP packet buffer");
		return ERROR_FAIL;
	}

	dap->packet_buffer = buf;
	dap->packet_size = pkt_sz;
	dap->packet_buffer_size = pkt_sz;

	dap->command = dap->packet_buffer;
	dap->response = dap->packet_buffer;

	return ERROR_OK;
}

/* Maximum number of transfers which have been submitted but did not reached
 * the destination yet. In-flight transfers are effectively queued in the
 * USB controller's driver so when any transfer completes the driver can submit
 * next request immediately */
#define MAX_INFLIGHT_REQUESTS 256

/* Fixed pool of transfer structures. USB transfers are taken from the pool when
 * they are submitted and returned to the pool upon completion for later reuse */
static struct cmsis_dap_async_xfer {
	struct libusb_transfer *xfer;
	int completed;
} cmsis_xfer_pool[MAX_INFLIGHT_REQUESTS];

/* Queued error status. Since all OUT transfers are queued it is not possible to
 * retrieve their execution status immediately. This information will be accessible
 * later when completion callback is called. Errors are accumulated in this variable */
static int queued_out_xfer_status;

/* Temporary structure with various stats, to be removed in the future */
static struct {
	size_t in_fifo;
	size_t in_flight;
} stats;

/** ***********************************************************************************************
 * @brief Allocates new async transfer from the pool
 * @param dap pointer to cmsis_dap object
 * @return pointer to pool member with allocated async transfer
 *************************************************************************************************/
static struct cmsis_dap_async_xfer *cmsis_dap_alloc_xfer(struct cmsis_dap *dap)
{
	struct cmsis_dap_async_xfer *cmsis_xfer;
	int64_t time_start = timeval_ms();

	while (true) {
		/* Loop through the pool, locate first unused descriptors */
		for (size_t i = 0; i < MAX_INFLIGHT_REQUESTS; i++) {
			cmsis_xfer = &cmsis_xfer_pool[i];

			/* Exit the loop if free descriptor is found */
			if (cmsis_xfer->xfer == NULL)
				goto xfer_found;
		}

		/* There are no free transfer descriptors in the pool. This call will block until any pending
		 * transfer is completed, associated transfer descriptor will be returned to the pool in the
		 * completion callback */
		libusb_handle_events(dap->bdata->usb_ctx);

		/* Timeout here means there is a bug somewhere and descriptors are leaking */
		if (timeval_ms() - time_start > 1000) {
			LOG_ERROR("Timeout waiting for transfer completion, this should never happen");
			return NULL;
		}
	}

xfer_found:
	/* Allocate new libusb_transfer structure, this takes the descriptor from the pool */
	cmsis_xfer->xfer = libusb_alloc_transfer(0);
	cmsis_xfer->completed = 0;
	return cmsis_xfer;
}

/** ***********************************************************************************************
 * @brief De-allocates completed async transfer and returns it to a pool for reuse
 * @param cmsis_xfer pointer to pool object to de-allocate
 * @param free_buffer if true, de-allocate data buffer associated with a transfer
 *************************************************************************************************/
static void cmsis_dap_free_xfer(struct cmsis_dap_async_xfer *cmsis_xfer, bool free_buffer)
{
	/* Free data buffer, if required */
	if (free_buffer)
		free(cmsis_xfer->xfer->buffer);

	/* Free the libusb_transfer and return the descriptor back to the pool */
	libusb_free_transfer(cmsis_xfer->xfer);
	cmsis_xfer->xfer = NULL;
}

/** ***********************************************************************************************
 * @brief Completion callback for async OUT transfers
 * @param xfr pointer to completed async transfer
 *************************************************************************************************/
static void LIBUSB_CALL out_xfer_cb(struct libusb_transfer *xfr)
{
	struct cmsis_dap_async_xfer *cmsis_xfer = (struct cmsis_dap_async_xfer *)xfr->user_data;
	assert(cmsis_xfer->xfer == xfr);

	/* Accumulate errors from (possibly) multiple transactions, errors will be checked
	 * in further cmsis_dap_usb_write_async() calls */
	if (cmsis_xfer->xfer->status != LIBUSB_TRANSFER_COMPLETED)
		queued_out_xfer_status = cmsis_xfer->xfer->status;

	/* Data buffer is not needed anymore for OUT transfers, free the data and return
	 * the descriptor to the pool */
	cmsis_xfer->completed = 1;
	cmsis_dap_free_xfer(cmsis_xfer, true);

	stats.in_flight--;
	stats.in_fifo++;
}

/** ***********************************************************************************************
 * @brief Allocates and submits async OUT transfer
 * @param dap pointer to cmsis_dap object
 * @param txlen number of bytes to transmit
 * @param timeout_ms transfer timeout
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_dap_usb_write_async(struct cmsis_dap *dap, int txlen, int timeout_ms)
{
	/* Check for any errors in already completed OUT transfers */
	if (queued_out_xfer_status != LIBUSB_TRANSFER_COMPLETED) {
		queued_out_xfer_status = LIBUSB_TRANSFER_COMPLETED;
		return ERROR_FAIL;
	}

	/* Take transfer descriptor from the pool */
	struct cmsis_dap_async_xfer *cmsis_xfer = cmsis_dap_alloc_xfer(dap);
	if (!cmsis_xfer || !cmsis_xfer->xfer)
		return ERROR_FAIL;

	/* Allocate data buffer */
	uint8_t *buffer = malloc(txlen);
	if (!buffer) {
		cmsis_dap_free_xfer(cmsis_xfer, false);
		return ERROR_FAIL;
	}

	/* Populate data buffer and transfer descriptor */
	memcpy(buffer, dap->packet_buffer, txlen);
	libusb_fill_bulk_transfer(cmsis_xfer->xfer, dap->bdata->dev_handle, dap->bdata->ep_out,
							  buffer, txlen, out_xfer_cb, cmsis_xfer, timeout_ms);

	/* Submit the transfer. This call returns immediately, all data is queued in the kernel */
	int hr = libusb_submit_transfer(cmsis_xfer->xfer);
	if (hr != LIBUSB_SUCCESS) {
		cmsis_dap_free_xfer(cmsis_xfer, true);
		return ERROR_FAIL;
	}

	stats.in_flight++;
	return txlen;
}

/** ***********************************************************************************************
 * @brief Completion callback for async IN transfers
 * @param xfr pointer to completed async transfer
 *************************************************************************************************/
static void LIBUSB_CALL in_xfer_cb(struct libusb_transfer *xfr)
{
	struct cmsis_dap_async_xfer *cmsis_xfer = (struct cmsis_dap_async_xfer *)xfr->user_data;
	assert(cmsis_xfer->xfer == xfr);

	/* Mark transfer as completed, all IN transfers are synchronization points */
	cmsis_xfer->completed = 1;

	if (xfr->status == LIBUSB_TRANSFER_COMPLETED) {
		if (stats.in_fifo)
			stats.in_fifo--;
	}
}

/** ***********************************************************************************************
 * @brief Allocates, submits and waits for completion of async IN transfer
 * @param dap pointer to cmsis_dap object
 * @param timeout_ms transfer timeout
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_dap_usb_read_async(struct cmsis_dap *dap, int timeout_ms)
{
	struct cmsis_dap_async_xfer *cmsis_xfer = cmsis_dap_alloc_xfer(dap);
	if (!cmsis_xfer || !cmsis_xfer->xfer)
		return ERROR_FAIL;

	/* Populate the transfer descriptor */
	libusb_fill_bulk_transfer(cmsis_xfer->xfer, dap->bdata->dev_handle, dap->bdata->ep_in,
							  dap->packet_buffer, dap->packet_size, in_xfer_cb, cmsis_xfer, timeout_ms);

	/* Submit the transfer, this call returns immediately */
	int hr = libusb_submit_transfer(cmsis_xfer->xfer);
	if (hr != LIBUSB_SUCCESS) {
		cmsis_dap_free_xfer(cmsis_xfer, false);
		return ERROR_FAIL;
	}

	/* Wait for the transfer completion */
	while(!cmsis_xfer->completed)
		libusb_handle_events_completed(dap->bdata->usb_ctx, &cmsis_xfer->completed);

	/* Retrieve the transfer status and return descriptor to the pool */
	int transferred = cmsis_xfer->xfer->actual_length;
	hr = cmsis_xfer->xfer->status;
	cmsis_dap_free_xfer(cmsis_xfer, false);

	/* Timeout is a spacial case which is handled in upper layers of CMSIS-DAP driver */
	if (hr == LIBUSB_TRANSFER_TIMED_OUT) {
		/* This is for debug only, to be removed in the future */
		if (stats.in_fifo != 0) {
			LOG_ERROR("In Flight: %zu, In FIFO: %zu", stats.in_flight, stats.in_fifo);
			libusb_reset_device(dap->bdata->dev_handle);
			assert(false);
		}

		return ERROR_TIMEOUT_REACHED;
	}

	if (hr != LIBUSB_TRANSFER_COMPLETED)
		return ERROR_FAIL;

	/* Cleanup the buffer's tail */
	memset(&dap->packet_buffer[transferred], 0, dap->packet_buffer_size - transferred);
	return transferred;
}

COMMAND_HANDLER(cmsis_dap_handle_usb_interface_command)
{
	if (CMD_ARGC == 1)
		cmsis_dap_usb_interface = strtoul(CMD_ARGV[0], NULL, 10);
	else
		LOG_ERROR("expected exactly one argument to cmsis_dap_usb_interface <interface_number>");

	return ERROR_OK;
}

const struct command_registration cmsis_dap_usb_subcommand_handlers[] = {
	{
		.name = "interface",
		.handler = &cmsis_dap_handle_usb_interface_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB interface number to use (for USB bulk backend only)",
		.usage = "<interface_number>",
	},
	COMMAND_REGISTRATION_DONE
};

const struct cmsis_dap_backend cmsis_dap_usb_backend = {
	.name = "usb_bulk",
	.open = cmsis_dap_usb_open,
	.close = cmsis_dap_usb_close,
	.read = cmsis_dap_usb_read,
	.write = cmsis_dap_usb_write,
	.packet_buffer_alloc = cmsis_dap_usb_alloc,
};

const struct cmsis_dap_backend cmsis_dap_usb_backend_async = {
	.name = "usb_bulk_async",
	.open = cmsis_dap_usb_open,
	.close = cmsis_dap_usb_close,
	.read = cmsis_dap_usb_read_async,
	.write = cmsis_dap_usb_write_async,
	.packet_buffer_alloc = cmsis_dap_usb_alloc,
};

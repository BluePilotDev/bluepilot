#include "selfdrive/pandad/panda_comms.h"

#include <stdexcept>
#include <string>
#include <vector>

#include <libusb-1.0/libusb.h>

#include "common/swaglog.h"

static const uint16_t USB_VID_COMMA = 0x3801;
static const uint16_t USB_VID_PANDA = 0xbbaa;
static const uint16_t USB_PID_PANDA = 0xddcc;
static const unsigned int USB_CTRL_TIMEOUT_MS = 1000;
static const unsigned int USB_BULK_RX_TIMEOUT_MS = 10;   // short; PSCM sends ~2kfps when truck on
static const unsigned int USB_BULK_TX_TIMEOUT_MS = 5000;

std::vector<std::string> PandaUsbHandle::list() {
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) != 0) return {};

  libusb_device **devs;
  ssize_t cnt = libusb_get_device_list(ctx, &devs);

  std::vector<std::string> serials;
  for (ssize_t i = 0; i < cnt; i++) {
    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(devs[i], &desc) != 0) continue;
    if ((desc.idVendor != USB_VID_COMMA && desc.idVendor != USB_VID_PANDA) ||
        desc.idProduct != USB_PID_PANDA) continue;

    libusb_device_handle *h = nullptr;
    if (libusb_open(devs[i], &h) != 0) continue;

    char buf[17] = {};
    int r = libusb_get_string_descriptor_ascii(h, desc.iSerialNumber,
                                               (unsigned char *)buf, 16);
    libusb_close(h);
    if (r > 0) serials.emplace_back(buf);
  }

  libusb_free_device_list(devs, 1);
  libusb_exit(ctx);
  return serials;
}

PandaUsbHandle::PandaUsbHandle(std::string serial) {
  if (libusb_init(&ctx) != 0) {
    throw std::runtime_error("libusb_init failed");
  }

  libusb_device **devs;
  ssize_t cnt = libusb_get_device_list(ctx, &devs);
  libusb_device *found = nullptr;

  for (ssize_t i = 0; i < cnt; i++) {
    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(devs[i], &desc) != 0) continue;
    if ((desc.idVendor != USB_VID_COMMA && desc.idVendor != USB_VID_PANDA) ||
        desc.idProduct != USB_PID_PANDA) continue;

    libusb_device_handle *h = nullptr;
    if (libusb_open(devs[i], &h) != 0) continue;

    char buf[17] = {};
    int r = libusb_get_string_descriptor_ascii(h, desc.iSerialNumber,
                                               (unsigned char *)buf, 16);
    libusb_close(h);

    if (r > 0 && (serial.empty() || serial == std::string(buf))) {
      found = devs[i];
      hw_serial = std::string(buf);
      break;
    }
  }
  libusb_free_device_list(devs, 1);

  if (!found) {
    libusb_exit(ctx);
    ctx = nullptr;
    throw std::runtime_error("USB panda not found: " + serial);
  }

  if (libusb_open(found, &dev_handle) != 0) {
    libusb_exit(ctx);
    ctx = nullptr;
    throw std::runtime_error("libusb_open failed for " + serial);
  }

  libusb_set_auto_detach_kernel_driver(dev_handle, 1);

  if (libusb_claim_interface(dev_handle, 0) != 0) {
    libusb_close(dev_handle);
    dev_handle = nullptr;
    libusb_exit(ctx);
    ctx = nullptr;
    throw std::runtime_error("libusb_claim_interface failed for " + serial);
  }
}

PandaUsbHandle::~PandaUsbHandle() {
  cleanup();
}

void PandaUsbHandle::cleanup() {
  if (dev_handle) {
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
    dev_handle = nullptr;
  }
  if (ctx) {
    libusb_exit(ctx);
    ctx = nullptr;
  }
}

int PandaUsbHandle::control_write(uint8_t request, uint16_t param1, uint16_t param2,
                                   unsigned int timeout) {
  int ret = libusb_control_transfer(
      dev_handle,
      LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
      request, param1, param2, nullptr, 0,
      timeout == TIMEOUT ? USB_CTRL_TIMEOUT_MS : timeout);
  if (ret < 0) {
    LOGE("USB control_write 0x%02x error: %s", request, libusb_error_name(ret));
    if (ret == LIBUSB_ERROR_NO_DEVICE) connected = false;
    comms_healthy = false;
  }
  return ret;
}

int PandaUsbHandle::control_read(uint8_t request, uint16_t param1, uint16_t param2,
                                  unsigned char *data, uint16_t length,
                                  unsigned int timeout) {
  int ret = libusb_control_transfer(
      dev_handle,
      LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
      request, param1, param2, data, length,
      timeout == TIMEOUT ? USB_CTRL_TIMEOUT_MS : timeout);
  if (ret < 0) {
    LOGE("USB control_read 0x%02x error: %s", request, libusb_error_name(ret));
    if (ret == LIBUSB_ERROR_NO_DEVICE) connected = false;
    comms_healthy = false;
  }
  return ret;
}

int PandaUsbHandle::bulk_write(unsigned char endpoint, unsigned char *data, int length,
                                unsigned int timeout) {
  int transferred = 0;
  int ret = libusb_bulk_transfer(
      dev_handle, endpoint,
      data, length, &transferred,
      timeout == TIMEOUT ? USB_BULK_TX_TIMEOUT_MS : timeout);
  if (ret != 0 && ret != LIBUSB_ERROR_TIMEOUT) {
    LOGE("USB bulk_write ep=0x%02x error: %s", endpoint, libusb_error_name(ret));
    if (ret == LIBUSB_ERROR_NO_DEVICE) connected = false;
    comms_healthy = false;
    return -1;
  }
  return transferred;
}

int PandaUsbHandle::bulk_read(unsigned char endpoint, unsigned char *data, int length,
                               unsigned int timeout) {
  int transferred = 0;
  int ret = libusb_bulk_transfer(
      dev_handle, endpoint,
      data, length, &transferred,
      timeout == TIMEOUT ? USB_BULK_RX_TIMEOUT_MS : timeout);
  if (ret == LIBUSB_ERROR_TIMEOUT) {
    return 0;  // no data available; not an error
  }
  if (ret != 0) {
    LOGE("USB bulk_read ep=0x%02x error: %s", endpoint, libusb_error_name(ret));
    if (ret == LIBUSB_ERROR_NO_DEVICE) connected = false;
    comms_healthy = false;
    return -1;
  }
  return transferred;
}

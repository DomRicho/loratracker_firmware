#ifndef HEADER_LORA_H
#define HEADER_LORA_H

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>

#define DEFAULT_LORA_CFG {      \
    .frequency = 915000000,     \
	.bandwidth = BW_500_KHZ,    \
	.datarate = SF_8,           \
	.preamble_len = 8,          \
	.coding_rate = CR_4_5,      \
	.iq_inverted = false,       \
	.public_network = false,    \
	.tx_power = 14,              \
	.tx = false,                \
}                               \

#endif

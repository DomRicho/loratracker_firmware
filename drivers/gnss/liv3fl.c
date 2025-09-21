#define DT_DRV_COMPAT st_liv3fl

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gnss.h>

#include <zephyr/modem/backend/uart.h>
#include <zephyr/modem/chat.h>

#include <drivers/gnss/liv3fl.h>
#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/drivers/gnss.h>

#include <drivers/gnss/nmea0183_match.h>
#include <drivers/gnss/st_match.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(liv3fl, CONFIG_GNSS_LOG_LEVEL);

#define UART_RX_BUF_SZ (512 + IS_ENABLED(CONFIG_GNSS_SATELLITES) * 1024)
#define UART_TX_BUF_SZ 128
#define CHAT_RECV_BUF_SZ 2048
#define CHAT_ARGV_SZ 32
#define LIV3FL_INIT_TIMEOUT 1U

struct gnss_liv3fl_config {
	const struct device *uart;
	const struct modem_chat_script *const init_chat_script;
	uint16_t fix_rate_ms;
    const struct gpio_dt_spec reset;
    // const struct gpio_dt_spec pps;
	struct {
		uint32_t initial;
		uint32_t desired;
	} baudrate;
};

struct gnss_liv3fl_data {
	struct gnss_nmea0183_match_data match_data;
#if CONFIG_GNSS_SATELLITES
	struct gnss_satellite satellites[CONFIG_GNSS_NMEA_GENERIC_SATELLITES_COUNT];
#endif

	/* UART backend */
	struct modem_pipe *uart_pipe;
	struct modem_backend_uart uart_backend;
	uint8_t uart_backend_receive_buf[UART_RX_BUF_SZ];
	uint8_t uart_backend_transmit_buf[UART_TX_BUF_SZ];

	/* Modem chat */
	struct modem_chat chat;
	uint8_t chat_receive_buf[CHAT_RECV_BUF_SZ];
	uint8_t *chat_argv[CHAT_ARGV_SZ];
};


MODEM_CHAT_MATCHES_DEFINE(unsol_matches,
	MODEM_CHAT_MATCH_WILDCARD("$??GGA,", ",*", gnss_nmea0183_match_gga_callback),
	MODEM_CHAT_MATCH_WILDCARD("$??RMC,", ",*", gnss_nmea0183_match_rmc_callback),
    MODEM_CHAT_MATCH_WILDCARD("$PSTMPOSHOLD,", ",*", gnss_teseo_match_poshold_callback),
#if CONFIG_GNSS_SATELLITES
#endif
);

static int liv3fl_set_fix_rate(const struct device *dev, uint32_t fix_interval_ms)
{
    return -ENOTSUP;
}

static int liv3fl_get_fix_rate(const struct device *dev, uint32_t *fix_interval_ms)
{
    return -ENOTSUP;
}

static int liv3fl_set_navigation_mode(const struct device *dev,
						  enum gnss_navigation_mode mode)
{
    switch(mode) {
        case GNSS_NAVIGATION_MODE_ZERO_DYNAMICS:
            // Set Position hold
            break;
        case GNSS_NAVIGATION_MODE_LOW_DYNAMICS:
        case GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS:
        case GNSS_NAVIGATION_MODE_HIGH_DYNAMICS:
            // Turn on HIGH_DYNAMICS_ON
        default:
            return -ENOTSUP;
    }
    return -ENOTSUP;
}

static int liv3fl_get_navigation_mode(const struct device *dev, 
        enum gnss_navigation_mode *mode)
{
    return -ENOTSUP;
}

static int liv3fl_set_enabled_systems(const struct device *dev, gnss_systems_t systems)
{
    return -ENOTSUP;
}

static int liv3fl_get_enabled_systems(const struct device *dev, gnss_systems_t *systems)
{
    return -ENOTSUP;
}

static int liv3fl_get_supported_systems(const struct device *dev, gnss_systems_t *systems)
{
    return -ENOTSUP;
}

static int liv3fl_get_latest_timepulse(const struct device *dev, k_ticks_t *timestamp)
{
    return -ENOTSUP;
}

static int gnss_liv3fl_resume(const struct device *dev)
{
	const struct gnss_liv3fl_config *cfg = dev->config;
	struct gnss_liv3fl_data *data = dev->data;
	int ret;

	ret = modem_pipe_open(data->uart_pipe, K_SECONDS(10));
	if (ret < 0) {
		return ret;
	}

	ret = modem_chat_attach(&data->chat, data->uart_pipe);

    // // reset the device
    // gpio_pin_set_dt(&cfg->reset, 1);
    // k_msleep(10);
    // gpio_pin_set_dt(&cfg->reset, 0);
    // k_msleep(200);

	if (ret == 0) {
		ret = modem_chat_run_script(&data->chat, cfg->init_chat_script);
	}

	if (ret < 0) {
        LOG_ERR("Something went wrong :(");
		modem_pipe_close(data->uart_pipe, K_SECONDS(10));
	}
	return ret;
}

static DEVICE_API(gnss, gnss_liv3fl_api) = {
	.set_fix_rate = liv3fl_set_fix_rate,
	.get_fix_rate = liv3fl_get_fix_rate,
	.set_navigation_mode = liv3fl_set_navigation_mode,
	.get_navigation_mode = liv3fl_get_navigation_mode,
	.set_enabled_systems = liv3fl_set_enabled_systems,
	.get_enabled_systems = liv3fl_get_enabled_systems,
	.get_supported_systems = liv3fl_get_supported_systems,
    .get_latest_timepulse = liv3fl_get_latest_timepulse,
};

static int gnss_liv3fl_init_nmea0183_match(const struct device *dev)
{
	struct gnss_liv3fl_data *data = dev->data;

	const struct gnss_nmea0183_match_config match_config = {
		.gnss = dev,
#if CONFIG_GNSS_SATELLITES
		.satellites = data->satellites,
		.satellites_size = ARRAY_SIZE(data->satellites),
#endif
	};

	return gnss_nmea0183_match_init(&data->match_data, &match_config);
}

static void gnss_liv3fl_init_pipe(const struct device *dev)
{
	const struct gnss_liv3fl_config *cfg = dev->config;
	struct gnss_liv3fl_data *data = dev->data;
	const struct modem_backend_uart_config uart_backend_config = {
		.uart = cfg->uart,
		.receive_buf = data->uart_backend_receive_buf,
		.receive_buf_size = sizeof(data->uart_backend_receive_buf),
		.transmit_buf = data->uart_backend_transmit_buf,
		.transmit_buf_size = sizeof(data->uart_backend_transmit_buf),
	};

	data->uart_pipe = modem_backend_uart_init(&data->uart_backend, &uart_backend_config);
}

static uint8_t gnss_liv3fl_char_delimiter[] = {'\r', '\n'};

static int gnss_liv3fl_init_chat(const struct device *dev)
{
	struct gnss_liv3fl_data *data = dev->data;

	const struct modem_chat_config chat_config = {
		.user_data = data,
		.receive_buf = data->chat_receive_buf,
		.receive_buf_size = sizeof(data->chat_receive_buf),
		.delimiter = gnss_liv3fl_char_delimiter,
		.delimiter_size = ARRAY_SIZE(gnss_liv3fl_char_delimiter),
		.filter = NULL,
		.filter_size = 0,
		.argv = data->chat_argv,
		.argv_size = ARRAY_SIZE(data->chat_argv),
		.unsol_matches = unsol_matches,
		.unsol_matches_size = ARRAY_SIZE(unsol_matches),
	};

	return modem_chat_init(&data->chat, &chat_config);
}

// Configure new baud rate: 
// Configure new PPS clock (64 MHz reconm for timing applications)
static int liv3fl_init(const struct device *dev)
{
	const struct gnss_liv3fl_config *cfg = dev->config;
    gpio_pin_configure_dt(&cfg->reset, (GPIO_OUTPUT_INACTIVE));
	int ret;
	ret = gnss_liv3fl_init_nmea0183_match(dev);
	if (ret < 0) {
		return ret;
	}

	gnss_liv3fl_init_pipe(dev);

	ret = gnss_liv3fl_init_chat(dev);
	if (ret < 0) {
		return ret;
	}

#if CONFIG_PM_DEVICE
	pm_device_init_suspended(dev);
#else
	ret = gnss_liv3fl_resume(dev);
	if (ret < 0) {
		return ret;
	}
#endif
    return 0;
}

// const static struct modem_chat_match init_finished = MODEM_CHAT_MATCH_WILDCARD(
//     "$PSTMSWCONFIG,1,11,12,????????????????????????????????????????????????????????????????????????????????*??",
//     "\r\n", NULL
// );
const static struct modem_chat_match setpar_ok = MODEM_CHAT_MATCH_WILDCARD("$PSTMSETPAROK", ",*", NULL);
const static struct modem_chat_match savepar_ok = MODEM_CHAT_MATCH_WILDCARD("$PSTMSAVEPAROK", ",*", NULL);
// const static struct modem_chat_match pps_ok = MODEM_CHAT_MATCH_WILDCARD("$PSTMPPSOK", ",*", NULL);
// const static struct modem_chat_match restore_ok = MODEM_CHAT_MATCH_WILDCARD("$PSTMRESTOREPAROK", ",*", NULL);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(
    init_cmd,
    // MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMRESTOREPAR*11\r\n", restore_ok),
    // MODEM_CHAT_SCRIPT_CMD_RESP("Hello World", init_finished),
    // MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMSETPAR,1102,0xA*10\r\n", setpar_ok), // Update Baudrate
    MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMSETPAR,1197,40*11\r\n", setpar_ok), // set pps clk to 64 MHz. Recom for timing
    MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMSETPAR,1215,3C*6C\r\n", setpar_ok), // Auto-hold to 60 samples
    MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMSETPAR,1201,0000000004000042*1B", setpar_ok), // change Commands
    MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMSETPAR,1228,0000000000000000*12", setpar_ok),
    MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMSAVEPAR*58\r\n", savepar_ok),
    MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMGETPAR,1102*21\r\n", modem_chat_any_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMGETPAR,1201*21\r\n", modem_chat_any_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMGETPAR,1228*2A\r\n", modem_chat_any_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMGETPAR,1197*2D\r\n", modem_chat_any_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMGETPAR,1215*24\r\n", modem_chat_any_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("$PSTMSRR*49\r\n", modem_chat_any_match),
);

MODEM_CHAT_SCRIPT_NO_ABORT_DEFINE(st_liv3fl_init_chat_script, init_cmd, NULL, 
                LIV3FL_INIT_TIMEOUT);


#define LIV3FL(inst)										                   \
	BUILD_ASSERT(										                       \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 9600) ||				   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 19200) ||				   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 38400) ||				   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 57600) ||				   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 115200) ||			   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 230400) ||			   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 460800),				   \
		"Invalid current-speed. Please set the UART current-speed to a baudrate" \
		"compatible with the modem.");							               \
												                               \
	BUILD_ASSERT((DT_INST_PROP(inst, fix_rate) >= 50) &&					   \
		     (DT_INST_PROP(inst, fix_rate) < 65536),					       \
		     "Invalid fix-rate. Please set it higher than 50-ms"			   \
		     " and must fit in 16-bits.");						               \
												                               \
	static const struct gnss_liv3fl_config liv3fl_cfg_##inst = {	           \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),					           \
		.init_chat_script = &_CONCAT(DT_DRV_COMPAT, _init_chat_script),        \
        .reset = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                     \
		.baudrate = {									                       \
			.initial = DT_INST_PROP(inst, initial_baudrate),			       \
			.desired = DT_PROP(DT_INST_BUS(inst), current_speed),			   \
		},										                               \
		.fix_rate_ms = DT_INST_PROP(inst, fix_rate),					       \
	};											                               \
												                               \
	static struct gnss_liv3fl_data liv3fl_data_##inst;						   \
												                               \
	DEVICE_DT_INST_DEFINE(inst,								                   \
			      liv3fl_init,							                       \
			      NULL,								                           \
			      &liv3fl_data_##inst,						                   \
			      &liv3fl_cfg_##inst,						                   \
			      POST_KERNEL,							                       \
			      CONFIG_GNSS_INIT_PRIORITY,					               \
			      &gnss_liv3fl_api);

DT_INST_FOREACH_STATUS_OKAY(LIV3FL)

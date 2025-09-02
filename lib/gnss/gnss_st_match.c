#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/kernel.h>
#include <zephyr/modem/chat.h>

#include <string.h>
#include <errno.h>

#include <drivers/gnss/parse.h>
#include <drivers/gnss/nmea0183.h>
#include <drivers/gnss/nmea0183_match.h>
#include <drivers/gnss/parse.h>
#include <drivers/gnss/st_match.h>

/*
 * $PSTMPOSHOLD,<on_off>,<Lat>,<N/S>,<Long>,<E/W>,<Alt>*<checksum><cr><lf>
 */  
int gnss_teseo_parse_poshold(const char** argv, uint16_t argc, struct gnss_data* data)
{
    data->info.pos_hold = (argv[1][0]  == '1');
	/* Validate cardinal directions */
	if (((argv[3][0] != 'N') && (argv[3][0] != 'S')) ||
	    ((argv[5][0] != 'E') && (argv[5][0] != 'W'))) {
		return -EINVAL;
	}
	/* Parse coordinates */
	if ((gnss_nmea0183_ddmm_mmmm_to_ndeg(argv[2], &data->nav_hold.latitude) < 0) ||
	    (gnss_nmea0183_ddmm_mmmm_to_ndeg(argv[4], &data->nav_hold.longitude) < 0)) {
		return -EINVAL;
	}

	/* Align sign of coordinates with cardinal directions */
	data->nav_hold.latitude = argv[3][0] == 'N'
				    ? data->nav_hold.latitude
				    : -data->nav_hold.latitude;

	data->nav_hold.longitude = argv[5][0] == 'E'
				     ? data->nav_hold.longitude
				     : -data->nav_hold.longitude;

	data->nav_hold.speed = 0;

    uint64_t tmp64;
	if ((gnss_parse_dec_to_milli(argv[6], &tmp64) < 0) ||
	    (tmp64 > INT32_MAX) ||
	    (tmp64 < INT32_MIN)) {
		return -EINVAL;
	}
	data->nav_hold.altitude = (int32_t)tmp64;
    return 0;
}

void gnss_teseo_match_poshold_callback(struct modem_chat *chat, char **argv, 
                                uint16_t argc, void *user_data)
{
	struct gnss_nmea0183_match_data *data = user_data;

	if (gnss_teseo_parse_poshold((const char **)argv, argc, &data->data) < 0) {
		return;
	}
}


/****************************************************************************
 *
 *   Copyright (c) 2024 ModalAI Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ModalAI nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file teseo.cpp
 *
 * STMicro Teseo LIV3F/LIV4F GPS driver implementation.
 * Fully self-contained message handling — does not call base class handleMessage(), due to it handling messages differently. 
 *
 * Teseo NMEA epoch message order (confirmed from raw NMEA captures):
 *   LIV3: RMC -> GGA -> GST -> GSA(x2-3) -> GSV(pages) -> KFCOV -> PSTMPV
 *   LIV4: KFCOV -> PSTMPVRAW -> RMC -> GGA -> GST -> GSA(x2-6) -> GSV(pages)
 *
 * Publish triggers:
 *   LIV3: PSTMPV  (last message in epoch) — requires all flags set
 *   LIV4: GSA     (last message before GSV) — requires all flags set
 *
 * Epoch integrity (LIV3): UTC timestamp from RMC must match PSTMPV.
 *   LIV3 timestamps are 100% consistent across all tested captures.
 *  
 *
 * LIV3 vs LIV4 detection:
 *   - PSTMPV present   -> LIV3 (vertical velocity only, ddd.d precision)
 *   - PSTMPVRAW present -> LIV4 (position + NED velocity, ddd.ddd precision)
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>

#include "teseo.h"

#ifndef M_PI_F
# define M_PI_F 3.14159265358979323846f
#endif

#define NMEA_UNUSED(x) (void)x;
#define TESEO_WARN(...)  {GPS_WARN(__VA_ARGS__);}

GPSDriverTeseo::GPSDriverTeseo(GPSCallbackPtr callback, void *callback_user,
			       sensor_gps_s *gps_position,
			       satellite_info_s *satellite_info,
			       float heading_offset) :
	GPSDriverNMEA(callback, callback_user, gps_position, satellite_info, heading_offset)
{
}

int GPSDriverTeseo::handleMessage(int len)
{
	char *endp;

	if (len < 7) {
		return 0;
	}

	int uiCalcComma = 0;

	for (int i = 0; i < len; i++) {
		if (_rx_buffer[i] == ',') { uiCalcComma++; }
	}

	char *bufptr = (char *)(_rx_buffer + 6);
	int ret = 0;
	bool sat_info_updated = false;

	// ================================================================================
	// RMC — first message of LIV3 epoch, provides time + horizontal velocity + cog rad
	// ================================================================================
	if ((memcmp(_rx_buffer + 3, "RMC,", 4) == 0) && (uiCalcComma >= 11)) {

		double utc_time = 0.0;
		char Status = 'V';
		double lat = 0.0, lon = 0.0;
		float ground_speed_K = 0.f;
		float track_true = 0.f;
		int nmea_date = 0;
		float Mag_var = 0.f;
		char ns = '?', ew = '?';
		NMEA_UNUSED(Mag_var);

		if (bufptr && *(++bufptr) != ',') { utc_time = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { Status = *(bufptr++); }
		if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }
		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }
		if (bufptr && *(++bufptr) != ',') { ground_speed_K = strtof(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { track_true = strtof(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { nmea_date = static_cast<int>(strtol(bufptr, &endp, 10)); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { Mag_var = strtof(bufptr, &endp); bufptr = endp; }

		_epoch_rmc_utc = utc_time;

		if (ns == 'S') { lat = -lat; }
		if (ew == 'W') { lon = -lon; }

		if (Status == 'V') {
			_gps_position->fix_type = 0;
		}

		float track_rad = track_true * M_PI_F / 180.0f;
		if (track_rad > M_PI_F) {
			track_rad -= 2.f * M_PI_F;
		}

		float velocity_ms = ground_speed_K / 1.9438445f;
		float velocity_north = velocity_ms * cosf(track_rad);
		float velocity_east  = velocity_ms * sinf(track_rad);

		_gps_position->vel_m_s = velocity_ms;
		_gps_position->cog_rad = track_rad;
		_gps_position->c_variance_rad = 0.1f;

		// LIV3: horizontal velocity from RMC
		// LIV4: PSTMPVRAW provides vel_n/vel_e, RMC still provides vel_m_s/cog
		if (!_is_liv4) {
			_gps_position->vel_n_m_s = velocity_north;
			_gps_position->vel_e_m_s = velocity_east;
			_gps_position->vel_ned_valid = true;
		}

		// Time
		int utc_hour = static_cast<int>(utc_time / 10000);
		int utc_minute = static_cast<int>((utc_time - utc_hour * 10000) / 100);
		double utc_sec = static_cast<double>(utc_time - utc_hour * 10000 - utc_minute * 100);
		int nmea_day = static_cast<int>(nmea_date / 10000);
		int nmea_mth = static_cast<int>((nmea_date - nmea_day * 10000) / 100);
		int nmea_year = static_cast<int>(nmea_date - nmea_day * 10000 - nmea_mth * 100);

		struct tm timeinfo = {};
		timeinfo.tm_year = nmea_year + 100;
		timeinfo.tm_mon = nmea_mth - 1;
		timeinfo.tm_mday = nmea_day;
		timeinfo.tm_hour = utc_hour;
		timeinfo.tm_min = utc_minute;
		timeinfo.tm_sec = int(utc_sec);
		timeinfo.tm_isdst = 0;

#ifndef NO_MKTIME
		time_t epoch = mktime(&timeinfo);

		if (epoch > GPS_EPOCH_SECS) {
			uint64_t usecs = static_cast<uint64_t>((utc_sec - static_cast<uint64_t>(utc_sec)) * 1000000);

			if (!_clock_set) {
				timespec ts{};
				ts.tv_sec = epoch;
				ts.tv_nsec = usecs * 1000;
				setClock(ts);
				_clock_set = true;
			}

			_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
			_gps_position->time_utc_usec += usecs;
		} else {
			_gps_position->time_utc_usec = 0;
		}

#else
		_gps_position->time_utc_usec = 0;
#endif
		_TESEO_time_received = true;

	// =========================================================================
	// GGA — position, fix type, satellites
	// =========================================================================
	} else if ((memcmp(_rx_buffer + 3, "GGA,", 4) == 0) && (uiCalcComma >= 14)) {

		double utc_time = 0.0, lat = 0.0, lon = 0.0;
		float alt = 0.f, geoid_h = 0.f;
		float hdop = 99.9f, dgps_age = NAN;
		int num_of_sv = 0, fix_quality = 0;
		char ns = '?', ew = '?';
		NMEA_UNUSED(dgps_age);
		NMEA_UNUSED(utc_time);

		if (bufptr && *(++bufptr) != ',') { utc_time = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }
		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }
		if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { hdop = strtof(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { alt = strtof(bufptr, &endp); bufptr = endp; }
		while (*(++bufptr) != ',') {} // skip M
		if (bufptr && *(++bufptr) != ',') { geoid_h = strtof(bufptr, &endp); bufptr = endp; }
		while (*(++bufptr) != ',') {} // skip M
		if (bufptr && *(++bufptr) != ',') { dgps_age = strtof(bufptr, &endp); bufptr = endp; }

		if (ns == 'S') { lat = -lat; }
		if (ew == 'W') { lon = -lon; }

		// LIV3: position from GGA. LIV4: PSTMPVRAW overrides position.
		if (!_is_liv4) {
			_gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);
			_gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
			_gps_position->alt = static_cast<int>(alt * 1000);
			_gps_position->alt_ellipsoid = _gps_position->alt + static_cast<int>(geoid_h * 1000);
		}

		_gps_position->hdop = hdop;

		if (fix_quality <= 0) {
			_gps_position->fix_type = 0;
		} else {
			if (fix_quality == 5) { fix_quality = 3; }
			_gps_position->fix_type = 3 + fix_quality - 1;
		}

		// Before fix, GGA reports tracked sats which is misleading for OSD
		if (_gps_position->fix_type > 0) {
			_gps_position->satellites_used = static_cast<int>(num_of_sv);
		} else {
			_gps_position->satellites_used = 0;
		}

		_TESEO_pos_received = true;
		_teseo_pos_timestamp = gps_absolute_time();
		_last_timestamp_time = _teseo_pos_timestamp;

	// =========================================================================
	// GST — position accuracy (eph, epv)
	// =========================================================================
	} else if ((memcmp(_rx_buffer + 3, "GST,", 4) == 0) && (uiCalcComma == 8)) {

		double utc_time = 0.0;
		float lat_err = 0.f, lon_err = 0.f, alt_err = 0.f;
		float min_err = 0.f, maj_err = 0.f, deg_from_north = 0.f, rms_err = 0.f;
		NMEA_UNUSED(utc_time);
		NMEA_UNUSED(min_err);
		NMEA_UNUSED(maj_err);
		NMEA_UNUSED(deg_from_north);
		NMEA_UNUSED(rms_err);

		if (bufptr && *(++bufptr) != ',') { utc_time = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { rms_err = strtof(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { maj_err = strtof(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { min_err = strtof(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { deg_from_north = strtof(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { lat_err = strtof(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { lon_err = strtof(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { alt_err = strtof(bufptr, &endp); bufptr = endp; }

		_gps_position->eph = sqrtf(lat_err * lat_err + lon_err * lon_err);
		_gps_position->epv = alt_err;

		_TESEO_eph_received = true;

	// =========================================================================
	// GSA — DOP values, publish trigger for LIV4
	// =========================================================================
	} else if ((memcmp(_rx_buffer + 3, "GSA,", 4) == 0) && (uiCalcComma >= 17)) {

		char M_pos = ' ';
		int fix_mode = 0;
		int sat_id[12] {};
		float pdop = 99.9f, hdop = 99.9f, vdop = 99.9f;
		NMEA_UNUSED(M_pos);
		NMEA_UNUSED(sat_id);
		NMEA_UNUSED(pdop);

		if (bufptr && *(++bufptr) != ',') { M_pos = *(bufptr++); }
		if (bufptr && *(++bufptr) != ',') { fix_mode = strtol(bufptr, &endp, 10); bufptr = endp; }

		for (int y = 0; y < 12; y++) {
			if (bufptr && *(++bufptr) != ',') { sat_id[y] = strtol(bufptr, &endp, 10); bufptr = endp; }
		}

		if (bufptr && *(++bufptr) != ',') { pdop = strtof(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { hdop = strtof(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { vdop = strtof(bufptr, &endp); bufptr = endp; }

		// Only use GSA for DOP values, not fix_type.
		// Multiple GSA lines arrive per epoch (one per constellation).
		// Empty constellation GSAs report fix_mode=1 which would incorrectly
		// overwrite the valid fix_type already set by GGA.
		if (fix_mode > 1) {
			_gps_position->hdop = static_cast<float>(hdop);
			_gps_position->vdop = static_cast<float>(vdop);
			_TESEO_dop_received = true;
		}

		// LIV4: GSA is the last standard message before GSV — publish here
		if (_is_liv4 && _TESEO_dop_received) {
			ret = tryPublishEpoch();
		}

	// =========================================================================
	// GSV — satellite info (PRN, CN0, elevation, azimuth)
	// =========================================================================
	} else if ((memcmp(_rx_buffer + 3, "GSV,", 4) == 0)) {

		int all_page_num = 0, this_page_num = 0, tot_sv_visible = 0;
		struct gsv_sat {
			int svid;
			int elevation;
			int azimuth;
			int snr;
		} sat[4] {};

		if (bufptr && *(++bufptr) != ',') { all_page_num = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { this_page_num = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { tot_sv_visible = strtol(bufptr, &endp, 10); bufptr = endp; }

		if ((this_page_num < 1) || (this_page_num > all_page_num)) {
			return 0;
		}

		// Track GSV counts per constellation prefix
		if (memcmp(_rx_buffer, "$GP", 3) == 0) {
			_sat_num_gpgsv = tot_sv_visible;
		} else if (memcmp(_rx_buffer, "$GL", 3) == 0) {
			_sat_num_glgsv = tot_sv_visible;
		} else if (memcmp(_rx_buffer, "$GA", 3) == 0) {
			_sat_num_gagsv = tot_sv_visible;
		} else if (memcmp(_rx_buffer, "$GB", 3) == 0) {
			_sat_num_gbgsv = tot_sv_visible;
		} else if (memcmp(_rx_buffer, "$BD", 3) == 0) {
			_sat_num_bdgsv = tot_sv_visible;
		}

		int end = 4;

		if (this_page_num == all_page_num) {
			end = tot_sv_visible - (this_page_num - 1) * 4;
		}

		if (_satellite_info) {
			uint64_t now = gps_absolute_time();

			for (int y = 0; y < end; y++) {
				if (bufptr && *(++bufptr) != ',') { sat[y].svid = strtol(bufptr, &endp, 10); bufptr = endp; }
				if (bufptr && *(++bufptr) != ',') { sat[y].elevation = strtol(bufptr, &endp, 10); bufptr = endp; }
				if (bufptr && *(++bufptr) != ',') { sat[y].azimuth = strtol(bufptr, &endp, 10); bufptr = endp; }
				if (bufptr && *(++bufptr) != ',') { sat[y].snr = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (sat[y].svid == 0 || sat[y].snr == 0) {
					continue;
				}

				/*
				 * Map NMEA 3.10 PRNs to UBX NAV-SVINFO numbering scheme.
				 * Same scheme used by the UBX driver for consistent satellite IDs.
				 *
				 * NMEA 3.10 raw PRNs:          NAV-SVINFO target:
				 *   GPS:      1-32              GPS:      1-32     (no change)
				 *   SBAS:     33-51             SBAS:     120-158  (NMEA + 87)
				 *   GLONASS:  65-92             GLONASS:  65-96    (no change)
				 *   BeiDou:   141-145 (GEO)     BeiDou:   159-163  (NMEA + 18)
				 *   BeiDou:   146-177 (MEO)     BeiDou:   33-64    (NMEA - 113)
				 *   QZSS:     183-197           QZSS:     193-207  (NMEA + 10)
				 *   Galileo:  301-336           Galileo:  211-246  (NMEA - 90)
				 */
				int svid_mapped = sat[y].svid;

				if (sat[y].svid >= 1 && sat[y].svid <= 32) {
					// GPS: no change
				} else if (sat[y].svid >= 33 && sat[y].svid <= 51) {
					svid_mapped = sat[y].svid + 87;
				} else if (sat[y].svid >= 65 && sat[y].svid <= 92) {
					// GLONASS: no change
				} else if (sat[y].svid >= 141 && sat[y].svid <= 145) {
					svid_mapped = sat[y].svid + 18;
				} else if (sat[y].svid >= 146 && sat[y].svid <= 177) {
					svid_mapped = sat[y].svid - 113;
				} else if (sat[y].svid >= 183 && sat[y].svid <= 197) {
					svid_mapped = sat[y].svid + 10;
				} else if (sat[y].svid >= 301 && sat[y].svid <= 336) {
					svid_mapped = sat[y].svid - 90;
				}

				_sat_buf.upsert(static_cast<uint8_t>(svid_mapped),
						static_cast<uint8_t>(sat[y].snr > 0),
						static_cast<uint8_t>(sat[y].snr),
						static_cast<uint8_t>(sat[y].elevation),
						static_cast<uint8_t>(sat[y].azimuth),
						now);
			}

			publishSatelliteInfo();
			sat_info_updated = true;
		}

	// =========================================================================
	// PSTMPV — LIV3 vertical velocity (last message in LIV3 epoch)
	// =========================================================================
	} else if ((memcmp(_rx_buffer + 1, "PSTMPV,", 7) == 0) && (uiCalcComma == 22)) {

		if (!_variant_detected) {
			_is_liv4 = false;
			_variant_detected = true;
			TESEO_WARN("Teseo LIV3 detected (PSTMPV)");
		}

		handlePSTMPV((char *)(_rx_buffer + 7));

		// LIV3 publish trigger
		ret = tryPublishEpoch();

	// =========================================================================
	// PSTMPVRAW — LIV4 position + NED velocity (first timestamped msg in LIV4 epoch)
	// =========================================================================
	} else if ((memcmp(_rx_buffer + 1, "PSTMPVRAW,", 10) == 0) && (uiCalcComma == 15)) {

		if (!_variant_detected) {
			_is_liv4 = true;
			_variant_detected = true;
			TESEO_WARN("Teseo LIV4 detected (PSTMPVRAW)");
		}

		handlePSTMPVRAW((char *)(_rx_buffer + 10));

	// =========================================================================
	// PSTMKFCOV — Kalman filter covariance (VelStd for sacc)
	// =========================================================================
	} else if ((memcmp(_rx_buffer + 1, "PSTMKFCOV,", 10) == 0) && (uiCalcComma == 8)) {

		handlePSTMKFCOV((char *)(_rx_buffer + 10));
		_TESEO_sacc_received = true;
	}

	// Satellite info publish
	if (sat_info_updated) {
		ret |= 2;
	}

	return ret;
}

void GPSDriverTeseo::handlePSTMPV(char *bufptr)
{
	/*
	 * PSTMPV — LIV3 vertical velocity (last message in LIV3 epoch).
	 * Horizontal velocity comes from RMC. Position from GGA. eph/epv from GST.
	 *
	 * Field 1:  UTC timestamp (for epoch verification)
	 * Field 10: Velocity Vertical/Up component (m/s) — ddd.d precision
	 */
	char *endp;
	double utc_time = 0.0;

	if (bufptr && *(++bufptr) != ',') { utc_time = strtod(bufptr, &endp); bufptr = endp; }

	// Skip fields 2-9 (lat, ns, lon, ew, alt, M, vel_n, vel_e)
	for (int i = 0; i < 8; i++) {
		if (bufptr) { bufptr = strchr(bufptr + 1, ','); }
	}

	// Field 10: Velocity Vertical/Up (m/s)
	float vel_v = 0.f;

	if (bufptr && *(++bufptr) != ',') { vel_v = strtof(bufptr, &endp); bufptr = endp; }

	if (!isnan(vel_v) && _gps_position->fix_type > 0) {
		_gps_position->vel_d_m_s = -vel_v;  // "up" -> NED "down"
	}

	// Epoch verification: PSTMPV UTC must match RMC UTC (100% reliable on LIV3)
	_TESEO_vel_received = true;

	double utc_diff = utc_time - _epoch_rmc_utc;
	if (utc_diff < 0) { utc_diff = -utc_diff; }

	if (_epoch_rmc_utc > 0 && utc_time > 0 && utc_diff > 0.001) {
		_TESEO_vel_received = false;
	}
}

void GPSDriverTeseo::handlePSTMPVRAW(char *bufptr)
{
	/*
	 * PSTMPVRAW — LIV4 position + NED velocity (first timestamped msg in LIV4 epoch).
	 *
	 * $PSTMPVRAW,UTC,lat,NS,lon,EW,qual,sats,hdop,alt,M,geoid,M,vel_n,vel_e,vel_v*cs
	 */
	char *endp;

	double lat = 0.0, lon = 0.0;
	float alt = 0.f;
	char ns = '?', ew = '?';
	float vel_n = 0.f, vel_e = 0.f, vel_v = 0.f;
	int gps_qual = 0;

	// Field 1: UTC (skip — epoch matching for LIV4 deferred)
	if (bufptr && *(++bufptr) != ',') { strtod(bufptr, &endp); bufptr = endp; }

	if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }
	if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }
	if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }
	if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }
	if (bufptr && *(++bufptr) != ',') { gps_qual = strtol(bufptr, &endp, 10); bufptr = endp; }

	// Skip sats, hdop
	if (bufptr && *(++bufptr) != ',') { strtol(bufptr, &endp, 10); bufptr = endp; }
	if (bufptr && *(++bufptr) != ',') { strtof(bufptr, &endp); bufptr = endp; }

	// Altitude
	if (bufptr && *(++bufptr) != ',') { alt = strtof(bufptr, &endp); bufptr = endp; }

	// Skip alt unit M, geoid, geoid unit M
	if (bufptr && *(++bufptr) != ',') { bufptr++; }
	if (bufptr && *(++bufptr) != ',') { strtof(bufptr, &endp); bufptr = endp; }
	if (bufptr && *(++bufptr) != ',') { bufptr++; }

	// Velocity NED
	if (bufptr && *(++bufptr) != ',') { vel_n = strtof(bufptr, &endp); bufptr = endp; }
	if (bufptr && *(++bufptr) != ',') { vel_e = strtof(bufptr, &endp); bufptr = endp; }
	if (bufptr && *(++bufptr) != ',') { vel_v = strtof(bufptr, &endp); bufptr = endp; }

	if (gps_qual > 0) {
		if (ns == 'S') { lat = -lat; }
		if (ew == 'W') { lon = -lon; }

		_gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);
		_gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
		_gps_position->alt = static_cast<int>(alt * 1000);

		_gps_position->vel_n_m_s = vel_n;
		_gps_position->vel_e_m_s = vel_e;
		_gps_position->vel_d_m_s = -vel_v;
		_gps_position->vel_ned_valid = true;
	} else {
		_gps_position->vel_ned_valid = false;
	}

	_TESEO_vel_received = true;
	_TESEO_pos_received = true;
	_teseo_pos_timestamp = gps_absolute_time();
	_last_timestamp_time = _teseo_pos_timestamp;
}

void GPSDriverTeseo::handlePSTMKFCOV(char *bufptr)
{
	/*
	 * PSTMKFCOV — Teseo Kalman filter covariance.
	 * Field 5: VelStd (m/s) — used as s_variance_m_s (sacc).
	 */
	char *endp;
	float vel_std = 0.f;

	// Skip fields 1-4
	if (bufptr && *(++bufptr) != ',') { strtof(bufptr, &endp); bufptr = endp; }
	if (bufptr && *(++bufptr) != ',') { strtof(bufptr, &endp); bufptr = endp; }
	if (bufptr && *(++bufptr) != ',') { strtof(bufptr, &endp); bufptr = endp; }
	if (bufptr && *(++bufptr) != ',') { strtof(bufptr, &endp); bufptr = endp; }

	// Field 5: VelStd (m/s)
	if (bufptr && *(++bufptr) != ',') { vel_std = strtof(bufptr, &endp); bufptr = endp; }

	_gps_position->s_variance_m_s = vel_std;
}

int GPSDriverTeseo::tryPublishEpoch()
{
	int ret = 0;

	if (_TESEO_pos_received && _TESEO_vel_received && _TESEO_eph_received &&
	    _TESEO_time_received && _TESEO_sacc_received) {

		_gps_position->timestamp = _teseo_pos_timestamp;
		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
		_clock_set = false;

		_TESEO_pos_received = false;
		_TESEO_vel_received = false;
		_TESEO_eph_received = false;
		_TESEO_time_received = false;
		_TESEO_sacc_received = false;
		_TESEO_dop_received = false;

		_rate_count_vel++;
		_rate_count_lat_lon++;
		ret = 1;
	}

	return ret;
}

void GPSDriverTeseo::publishSatelliteInfo()
{
	uint64_t now = gps_absolute_time();

	_sat_buf.evictStale(now, SAT_STALE_TIMEOUT_US);

	// Sort by SNR descending
	for (uint8_t i = 0; i < _sat_buf.count; i++) {
		for (uint8_t j = i + 1; j < _sat_buf.count; j++) {
			if (_sat_buf.sats[j].snr > _sat_buf.sats[i].snr) {
				SatEntry swap = _sat_buf.sats[i];
				_sat_buf.sats[i] = _sat_buf.sats[j];
				_sat_buf.sats[j] = swap;
			}
		}
	}

	uint8_t publish_count = (_sat_buf.count < satellite_info_s::SAT_INFO_MAX_SATELLITES)
				? _sat_buf.count : satellite_info_s::SAT_INFO_MAX_SATELLITES;

	for (uint8_t i = 0; i < publish_count; i++) {
		_satellite_info->svid[i]      = _sat_buf.sats[i].svid;
		_satellite_info->used[i]      = _sat_buf.sats[i].used;
		_satellite_info->snr[i]       = _sat_buf.sats[i].snr;
		_satellite_info->elevation[i] = _sat_buf.sats[i].elevation;
		_satellite_info->azimuth[i]   = _sat_buf.sats[i].azimuth;
		_satellite_info->prn[i]       = _sat_buf.sats[i].svid;
	}

	for (uint8_t i = publish_count; i < satellite_info_s::SAT_INFO_MAX_SATELLITES; i++) {
		_satellite_info->svid[i]      = 0;
		_satellite_info->used[i]      = 0;
		_satellite_info->snr[i]       = 0;
		_satellite_info->elevation[i] = 0;
		_satellite_info->azimuth[i]   = 0;
		_satellite_info->prn[i]       = 0;
	}

	_satellite_info->count = publish_count;
	_satellite_info->timestamp = now;
}

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
 * @file teseo.h
 *
 * STMicro Teseo LIV3F/LIV4F GPS driver.
 * Extends the base NMEA driver with Teseo proprietary messages
 * (PSTMPV, PSTMPVRAW, PSTMKFCOV) and epoch-aligned publishing.
 *
 * Automatically detects LIV3 vs LIV4 based on PSTMPV vs PSTMPVRAW.
 */

#pragma once

#include "nmea.h"

/**
 * Satellite tracking entry for constellation-agnostic satellite buffer.
 * SVIDs are stored in UBX NAV-SVINFO numbering scheme.
 */
struct SatEntry {
	uint8_t svid;
	uint8_t used;
	uint8_t snr;
	uint8_t elevation;
	uint8_t azimuth;
	uint64_t timestamp;
};

/**
 * Single unified satellite buffer. Tracks all constellations in one array.
 * Stale satellites (not seen in GSV for >timeout) are automatically evicted.
 */
struct SatBuffer {
	static constexpr uint8_t MAX_SATS = 40;
	SatEntry sats[MAX_SATS];
	uint8_t count{0};

	int find(uint8_t svid) const {
		for (uint8_t i = 0; i < count; i++) {
			if (sats[i].svid == svid) { return i; }
		}

		return -1;
	}

	void upsert(uint8_t svid, uint8_t used, uint8_t snr, uint8_t elevation,
		    uint8_t azimuth, uint64_t now) {
		int idx = find(svid);

		if (idx >= 0) {
			if (snr >= sats[idx].snr) {
				sats[idx].used = used;
				sats[idx].snr = snr;
				sats[idx].elevation = elevation;
				sats[idx].azimuth = azimuth;
			}

			sats[idx].timestamp = now;

		} else if (count < MAX_SATS) {
			sats[count].svid = svid;
			sats[count].used = used;
			sats[count].snr = snr;
			sats[count].elevation = elevation;
			sats[count].azimuth = azimuth;
			sats[count].timestamp = now;
			count++;
		}
	}

	void evictStale(uint64_t now, uint64_t max_age_us) {
		uint8_t i = 0;

		while (i < count) {
			if ((now - sats[i].timestamp) > max_age_us) {
				sats[i] = sats[count - 1];
				count--;

			} else {
				i++;
			}
		}
	}
};

class GPSDriverTeseo : public GPSDriverNMEA
{
public:
	GPSDriverTeseo(GPSCallbackPtr callback, void *callback_user,
		       sensor_gps_s *gps_position,
		       satellite_info_s *satellite_info,
		       float heading_offset = 0.f);

	~GPSDriverTeseo() {}

protected:
	int handleMessage(int len) override;

private:
	void handlePSTMPV(char *bufptr);
	void handlePSTMPVRAW(char *bufptr);
	void handlePSTMKFCOV(char *bufptr);

	/**
	 * Try to publish a complete epoch. Checks that all required data has been
	 * received and that the UTC timestamps from the first and last messages match.
	 * Returns 1 if published, 0 otherwise.
	 */
	int tryPublishEpoch();

	/**
	 * Evict stale satellites, sort by SNR, and publish to satellite_info.
	 */
	void publishSatelliteInfo();

	bool _is_liv4{false};
	bool _variant_detected{false};

	double _epoch_rmc_utc{0};

	// Flags for Teseo-specific publish gate
	bool _TESEO_pos_received{false};
	bool _TESEO_vel_received{false};
	bool _TESEO_eph_received{false};
	bool _TESEO_time_received{false};
	bool _TESEO_sacc_received{false};
	bool _TESEO_dop_received{false};

	uint64_t _teseo_pos_timestamp{0};

	static constexpr uint64_t SAT_STALE_TIMEOUT_US = 2000000;
	SatBuffer _sat_buf;
};

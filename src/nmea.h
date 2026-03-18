/****************************************************************************
 *
 *   Copyright (c) 2020, 2021 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file nmea.h
 *
 * NMEA protocol definitions
 *
 * @author WeiPeng Guo <guoweipeng1990@sina.com>
 * @author Stone White <stone@thone.io>
 * @author Jose Jimenez-Berni <berni@ias.csic.es>
 *
 */

#pragma once

#include "gps_helper.h"
#include "../../definitions.h"
#include "unicore.h"

class RTCMParsing;

#define NMEA_RECV_BUFFER_SIZE 1024
#define NMEA_DEFAULT_BAUDRATE 115200

struct SatEntry {
	uint8_t svid;
	uint8_t used;
	uint8_t snr;
	uint8_t elevation;
	uint8_t azimuth;
	uint64_t timestamp;
};

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

class GPSDriverNMEA : public GPSHelper
{
public:
	/**
	 * @param heading_offset heading offset in radians [-pi, pi]. It is substracted from the measurement.
	 */
	GPSDriverNMEA(GPSCallbackPtr callback, void *callback_user,
		      sensor_gps_s *gps_position,
		      satellite_info_s *satellite_info,
		      float heading_offset = 0.f);

	virtual ~GPSDriverNMEA();

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, const GPSConfig &config) override;

private:
	void handleHeading(float heading_deg, float heading_stddev_deg);

	UnicoreParser _unicore_parser;

	enum class NMEADecodeState {
		uninit,
		got_sync1,
		got_asteriks,
		got_first_cs_byte,
		decode_rtcm3
	};

	void decodeInit(void);
	int handleMessage(int len);
	int parseChar(uint8_t b);

	int32_t read_int();
	double read_float();
	char read_char();

	sensor_gps_s *_gps_position {nullptr};
	satellite_info_s *_satellite_info {nullptr};
	double _last_POS_timeUTC{0};
	double _last_VEL_timeUTC{0};
	double _last_FIX_timeUTC{0};
	uint64_t _last_timestamp_time{0};
	uint64_t _pos_timestamp{0};		///< system time when GGA position was received (used as publish timestamp)

	uint8_t _sat_num_gga{0};
	uint8_t _sat_num_gns{0};
	uint8_t _sat_num_gsv{0};
	uint8_t _sat_num_gpgsv{0};
	uint8_t _sat_num_glgsv{0};
	uint8_t _sat_num_gagsv{0};
	uint8_t _sat_num_gbgsv{0};
	uint8_t _sat_num_bdgsv{0};

	static constexpr uint64_t SAT_STALE_TIMEOUT_US = 2000000;
	SatBuffer _sat_buf;

	void publishSatelliteInfo();

	bool _clock_set {false};

//  check if we got all basic essential packages we need
	bool _TIME_received{false};
	bool _POS_received{false};
	bool _ALT_received{false};
	bool _SVNUM_received{false};
	bool _SVINFO_received{false};
	bool _FIX_received{false};
	bool _DOP_received{false};
	bool _VEL_received{false};
	bool _EPH_received{false};
	bool _SACC_received{false};
	bool _HEAD_received{false};

	NMEADecodeState _decode_state{NMEADecodeState::uninit};
	uint8_t _rx_buffer[NMEA_RECV_BUFFER_SIZE] {};
	uint16_t _rx_buffer_bytes{0};

	OutputMode _output_mode{OutputMode::GPS};

	RTCMParsing *_rtcm_parsing{nullptr};

	float _heading_offset;

	bool _pstmpv_active{false};  // Set true on first PSTMPV receipt; defers VTG/RMC triggers
};

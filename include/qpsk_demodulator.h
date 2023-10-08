/**
 * @file qpsk_demodulator.h
 * @author Fern Lane
 * @brief QPSK demodulator (decoder)
 * @version 1.0.0
 * @date 2023-09-26
 *
 * @copyright Copyright (c) 2023
 *
 * Copyright (C) 2023 Fern Lane, QPSK modulator / demodulator libraries
 * Licensed under the GNU Affero General Public License, Version 3.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *       https://www.gnu.org/licenses/agpl-3.0.en.html
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#ifndef QPSK_MODULATOR_H__
#define QPSK_MODULATOR_H__

#include "rrc_filter.h"
#include "pll.h"
#include "butterworth_filter.h"

#define M_PI 3.1415927f

// RRC input filter number of positive lobes, alpha (roll-off factor) and order
#define FILTER_LOBES_N 40
#define FILTER_ALPHA .98f
#define FILTER_ORDER 1

// AGC accumulator release gain when signal <= accumulator
#define AGC_RELEASE_K .98f

// Proportional and integral gain terms of PLL's PI controller
#define PLL_K_P 0.2f
#define PLL_K_I 8.0f

// Filtering gain of pll's internal actual frequency
#define PLL_FREQUENCY_FILTER .994f

// Order of butterworth low-pass filters for I and Q
#define IQ_FILTER_ORDER 3

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)       \
    ((byte)&0x80 ? '1' : '0'),     \
        ((byte)&0x40 ? '1' : '0'), \
        ((byte)&0x20 ? '1' : '0'), \
        ((byte)&0x10 ? '1' : '0'), \
        ((byte)&0x08 ? '1' : '0'), \
        ((byte)&0x04 ? '1' : '0'), \
        ((byte)&0x02 ? '1' : '0'), \
        ((byte)&0x01 ? '1' : '0')

typedef struct
{
    float sample_rate;
    float lo_frequency;
    float bandwidth;
    uint16_t halfcycles_per_symbol;
    float carrier_start_threshold, carrier_lost_threshold;
    float pll_lock_threshold, pll_lock_time;

    rrc_filter_s *rrc_filter;
    pll_s *pll;
    butter_filter_s *butter_filter_i;
    butter_filter_s *butter_filter_q;

    float sample_time;
    float filtered_sample;
    float peak_detector;
    float signal_strength_rms;
    uint8_t carrier_detected, pll_locked;
    float pll_lock_timer;
    float iq_error;
    float i_mixed, q_mixed, i_sign, q_sign, i_sign_prev, q_sign_prev;
    float reference_signs_i[4], reference_signs_q[4];
    uint8_t reference_signs_set_position;
    uint16_t samples_after_zcd;
    int16_t sampling_position;
    uint16_t samples_per_symbol;
} qpsk_demodulator_s;

float clampf(float value, float min, float max);

qpsk_demodulator_s *qpsk_demodulator_init(float sample_rate,
                                          float lo_frequency,
                                          float bandwidth,
                                          uint16_t halfcycles_per_symbol,
                                          float carrier_start_threshold,
                                          float carrier_lost_threshold,
                                          float pll_lock_threshold,
                                          float pll_lock_time);

void qpsk_demodulator_demodulate_chunk(qpsk_demodulator_s *qpsk_demodulator,
                                       float *samples_chunk,
                                       uint32_t samples_chunk_size,
                                       uint8_t *symbols_chunk,
                                       uint8_t debug_messages);

void qpsk_demodulator_reset(qpsk_demodulator_s *qpsk_demodulator);

void qpsk_demodulator_destroy(qpsk_demodulator_s *qpsk_demodulator);

#endif
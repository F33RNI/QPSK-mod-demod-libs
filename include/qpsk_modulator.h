/**
 * @file qpsk_modulator.h
 * @author Fern Lane
 * @brief QPSK modulator (encoder)
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

#define M_PI 3.1415927f

// It is necessary that the sum of the amplitudes of the sin and cos be -1 to 1. Default: 1 / sqrt(2)
#define AMPLITUDE_CORRECTION_FACTOR .70710678f

// RRC output filter number of positive lobes, alpha (roll-off factor) and order
#define FILTER_LOBES_N 40
#define FILTER_ALPHA .98f
#define FILTER_ORDER 1

// AGC accumulator release gain when signal <= accumulator
#define AGC_RELEASE_K .8f

// AGC accumulator must be above this threshold to be applied to the modulated signal
#define AGC_APPLY_THRESHOLD .2f

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
    float carrier_frequency;
    float bandwidth;
    uint16_t halfcycles_per_symbol;
    float amplitude_peak;

    uint16_t samples_per_symbol;

    rrc_filter_s *rrc_filter;

    uint16_t samples_per_symbol_counter;
    uint32_t sample_counter, symbol_counter;
    float sample_time;
    float i_sign, q_sign, iq_sample;
    uint8_t current_symbol;
    float peak_detector;
} qpsk_modulator_s;

qpsk_modulator_s *qpsk_modulator_init(float sample_rate,
                                      float carrier_frequency,
                                      float bandwidth,
                                      uint16_t halfcycles_per_symbol,
                                      float amplitude_peak);

uint32_t qpsk_modulator_calculate_samples_chunk_length(qpsk_modulator_s *qpsk_modulator, uint32_t symbols_chunk_length);

void qpsk_modulator_modulate_chunk(qpsk_modulator_s *qpsk_modulator,
                                   uint8_t *symbols_chunk,
                                   uint32_t symbols_chunk_length,
                                   float *samples_chunk,
                                   uint8_t debug_messages);

void qpsk_modulator_reset(qpsk_modulator_s *qpsk_modulator);

void qpsk_modulator_destroy(qpsk_modulator_s *qpsk_modulator);

#endif

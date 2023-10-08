/**
 * @file qpsk_modulator.c
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#include "qpsk_modulator.h"

/**
 * @brief Initializes QPSK demodulator
 *
 * @param sample_rate Sampling rate (in Hz)
 * @param carrier_frequency Carrier frequency (in Hz)
 * @param bandwidth Output signal required bandwidth (in Hz)
 * @param halfcycles_per_symbol Rate of symbols in carrier halfcycles (2 halfcycles = 1 full carrier wave cycle)
 * @param amplitude_peak Amplitude of modulated signal (peak value)
 * @return qpsk_modulator_s* QPSK modulator's struct
 */
qpsk_modulator_s *qpsk_modulator_init(float sample_rate,
                                      float carrier_frequency,
                                      float bandwidth,
                                      uint16_t halfcycles_per_symbol,
                                      float amplitude_peak)
{
    // Allocate struct
    qpsk_modulator_s *qpsk_modulator = malloc(sizeof(qpsk_modulator_s));

    // Initialize basic fields
    qpsk_modulator->sample_rate = sample_rate;
    qpsk_modulator->carrier_frequency = carrier_frequency;
    qpsk_modulator->bandwidth = bandwidth;
    qpsk_modulator->halfcycles_per_symbol = halfcycles_per_symbol;
    qpsk_modulator->amplitude_peak = amplitude_peak;

    // Calculate how much samples one symbol takes
    qpsk_modulator->samples_per_symbol = (uint32_t)((1.f / qpsk_modulator->carrier_frequency / 2.f) *
                                                    qpsk_modulator->sample_rate *
                                                    (float)qpsk_modulator->halfcycles_per_symbol);

    // Use a band-pass filter if the bandwidth doesn't "touch" 0
    if (carrier_frequency - bandwidth / 2.f > 0)
        qpsk_modulator->rrc_filter = rrc_filter_init(FILTER_TYPE_BANDPASS,
                                                     sample_rate,
                                                     FILTER_LOBES_N,
                                                     FILTER_ORDER,
                                                     FILTER_ALPHA,
                                                     carrier_frequency - bandwidth / 2.f,
                                                     carrier_frequency + bandwidth / 2.f);

    // Use a low-pass filter otherwise
    else
        qpsk_modulator->rrc_filter = rrc_filter_init(FILTER_TYPE_LOWPASS,
                                                     sample_rate,
                                                     FILTER_LOBES_N,
                                                     FILTER_ORDER,
                                                     FILTER_ALPHA,
                                                     carrier_frequency + bandwidth / 2.f,
                                                     0.f);

    // Initialize loop variables
    qpsk_modulator->samples_per_symbol_counter = 0;
    qpsk_modulator->sample_counter = 0;
    qpsk_modulator->symbol_counter = 0;
    qpsk_modulator->sample_time = 0.f;
    qpsk_modulator->i_sign = 0.f;
    qpsk_modulator->q_sign = 0.f;
    qpsk_modulator->peak_detector = 0.f;

    return qpsk_modulator;
}

/**
 * @brief Calculates the size of the array of sample after modulation
 *
 * @param qpsk_modulator QPSK modulator's struct
 * @param symbols_chunk_length number of symbols
 * @return uint32_t number of samples
 */
uint32_t qpsk_modulator_calculate_samples_chunk_length(qpsk_modulator_s *qpsk_modulator, uint32_t symbols_chunk_length)
{
    // Multiply samples_per_symbol by number of symbols
    return qpsk_modulator->samples_per_symbol * symbols_chunk_length;
}

/**
 * @brief Modulates chunk of symbols
 *
 * @param qpsk_modulator QPSK modulator's struct
 * @param symbols_chunk Array of symbols
 * @param symbols_chunk_length Length of array of symbols
 * @param samples_chunk Allocated array of samples
 * (size must be qpsk_modulator_calculate_samples_chunk_length * sizeof(float))
 * @param debug_messages Enable printf() debug messages (greatly affects performance)
 */
void qpsk_modulator_modulate_chunk(qpsk_modulator_s *qpsk_modulator,
                                   uint8_t *symbols_chunk,
                                   uint32_t symbols_chunk_length,
                                   float *samples_chunk,
                                   uint8_t debug_messages)
{
    // Log number of samples
    if (debug_messages)
        printf("Symbols to modulate: %d\n", symbols_chunk_length);

    // Record start time
    clock_t time_started = clock();

    // Modulate entire chunk
    while (1)
    {
        // Calculate current time (in seconds)
        qpsk_modulator->sample_time += 1.f / qpsk_modulator->sample_rate;

        // New symbol
        if (qpsk_modulator->samples_per_symbol_counter == 0)
        {
            // Check if we have data to modulate
            if (qpsk_modulator->symbol_counter <= symbols_chunk_length - 1)
            {
                // Extract symbol
                qpsk_modulator->current_symbol = symbols_chunk[qpsk_modulator->symbol_counter];

                // 00, 01, 10 or 11
                if (qpsk_modulator->current_symbol <= 0b11)
                {
                    qpsk_modulator->i_sign = qpsk_modulator->current_symbol & 0b10 ? 1.f : -1.f;
                    qpsk_modulator->q_sign = qpsk_modulator->current_symbol & 0b01 ? 1.f : -1.f;
                }

                // Anything else (0xFF)
                else
                {
                    qpsk_modulator->i_sign = 0.f;
                    qpsk_modulator->q_sign = 0.f;
                }

                // "Play" new symbol for some amount of cycles
                qpsk_modulator->samples_per_symbol_counter = qpsk_modulator->samples_per_symbol;

                // Log
                if (debug_messages)
                    printf("Modulating %d/%d symbol: %d (0b" BYTE_TO_BINARY_PATTERN "). I: %.0f, Q: %.0f\n",
                           qpsk_modulator->symbol_counter + 1, symbols_chunk_length,
                           qpsk_modulator->current_symbol, BYTE_TO_BINARY(qpsk_modulator->current_symbol),
                           qpsk_modulator->i_sign, qpsk_modulator->q_sign);

                // Increment counter
                qpsk_modulator->symbol_counter++;
            }

            // Exit if no more data
            else
                break;
        }

        // Decrement samples counter
        if (qpsk_modulator->samples_per_symbol_counter > 0)
            qpsk_modulator->samples_per_symbol_counter--;

        // Calculate IQ sample
        qpsk_modulator->iq_sample = cosf(2.f * M_PI * qpsk_modulator->carrier_frequency *
                                         qpsk_modulator->sample_time) *
                                    qpsk_modulator->i_sign * AMPLITUDE_CORRECTION_FACTOR;
        qpsk_modulator->iq_sample -= sinf(2.f * M_PI * qpsk_modulator->carrier_frequency *
                                          qpsk_modulator->sample_time) *
                                     qpsk_modulator->q_sign * AMPLITUDE_CORRECTION_FACTOR;

        // Filter sample
        qpsk_modulator->iq_sample = rrc_filter_filter(qpsk_modulator->rrc_filter, qpsk_modulator->iq_sample);

        // Calculate peak detector
        if (fabsf(qpsk_modulator->iq_sample) > qpsk_modulator->peak_detector)
            qpsk_modulator->peak_detector = fabsf(qpsk_modulator->iq_sample);
        else
            qpsk_modulator->peak_detector = qpsk_modulator->peak_detector * AGC_RELEASE_K +
                                            fabsf(qpsk_modulator->iq_sample) * (1. - AGC_RELEASE_K);

        // Apply AGC and ignore silence
        if (qpsk_modulator->peak_detector > AGC_APPLY_THRESHOLD)
            qpsk_modulator->iq_sample *= qpsk_modulator->amplitude_peak / qpsk_modulator->peak_detector;
        else
            qpsk_modulator->iq_sample *= qpsk_modulator->amplitude_peak;

        // Write to the array
        samples_chunk[qpsk_modulator->sample_counter] = qpsk_modulator->iq_sample;

        // Increment chunk array counter
        qpsk_modulator->sample_counter++;
    }

    if (debug_messages)
    {
        // Record stop time
        clock_t time_stopped = clock();

        // Calculate time
        double time_taken_millis = (((double)(time_stopped - time_started)) / CLOCKS_PER_SEC) * 1000.;
        printf("Modulation is complete. Time taken: ~%.0f ms\n", time_taken_millis);
    }
}

/**
 * @brief Resets QPSK modulator to the initial state
 *
 * @param qpsk_modulator
 */
void qpsk_modulator_reset(qpsk_modulator_s *qpsk_modulator)
{
    // Reset filter
    rrc_filter_reset(qpsk_modulator->rrc_filter);

    // Reset loop variables
    qpsk_modulator->samples_per_symbol_counter = 0;
    qpsk_modulator->sample_counter = 0;
    qpsk_modulator->symbol_counter = 0;
    qpsk_modulator->sample_time = 0.f;
    qpsk_modulator->i_sign = 0.f;
    qpsk_modulator->q_sign = 0.f;
    qpsk_modulator->peak_detector = 0.f;
}

/**
 * @brief Frees all memory allocated by QPSK modulator
 *
 * @param qpsk_modulator QPSK modulator's struct
 */
void qpsk_modulator_destroy(qpsk_modulator_s *qpsk_modulator)
{
    rrc_filter_destroy(qpsk_modulator->rrc_filter);
    free(qpsk_modulator);
}

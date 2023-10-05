/**
 * @file qpsk_demodulator.c
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#include "qpsk_demodulator.h"

/**
 * @brief Clamps float value to a range
 *
 * @param value Value to clamp
 * @param min Range minimum value
 * @param max Range maximum value
 * @return float Clamped value
 */
float clampf(float value, float min, float max)
{
    const float clamped_min = value < min ? min : value;
    return clamped_min > max ? max : clamped_min;
}

/**
 * @brief Initializes QPSK demodulator
 *
 * @param sample_rate Sampling rate (in Hz)
 * @param lo_frequency Carrier frequency (local oscillator) (in Hz)
 * @param bandwidth Input signal bandwidth (in Hz)
 * @param halfcycles_per_symbol Rate of symbols in carrier halfcycles (2 halfcycles = 1 full carrier wave cycle)
 * @param carrier_start_threshold Signal must be above this threshold to start PLL locking and demodulating (in dBFS RMS)
 * @param carrier_lost_threshold Signal must be below this threshold to stop PLL locking and demodulating (in dBFS RMS)
 * @param pll_lock_threshold PLL's input (IQ error) should't change during pll_lock_time more than this value
 * @param pll_lock_time PLL's input (IQ error) should't change during this time (in seconds) more than pll_lock_threshold
 * @return qpsk_demodulator_s* QPSK demodulator's struct
 */
qpsk_demodulator_s *qpsk_demodulator_init(float sample_rate,
                                          float lo_frequency,
                                          float bandwidth,
                                          uint16_t halfcycles_per_symbol,
                                          float carrier_start_threshold,
                                          float carrier_lost_threshold,
                                          float pll_lock_threshold,
                                          float pll_lock_time)
{
    // Allocate struct
    qpsk_demodulator_s *qpsk_demodulator = malloc(sizeof(qpsk_demodulator_s));

    // Initialize basic fields
    qpsk_demodulator->sample_rate = sample_rate;
    qpsk_demodulator->lo_frequency = lo_frequency;
    qpsk_demodulator->bandwidth = bandwidth;
    qpsk_demodulator->halfcycles_per_symbol = halfcycles_per_symbol;
    qpsk_demodulator->carrier_start_threshold = carrier_start_threshold;
    qpsk_demodulator->carrier_lost_threshold = carrier_lost_threshold;
    qpsk_demodulator->pll_lock_threshold = pll_lock_threshold;
    qpsk_demodulator->pll_lock_time = pll_lock_time;

    // Use a band-pass filter if the bandwidth doesn't "touch" 0
    if (lo_frequency - bandwidth / 2.f > 0)
        qpsk_demodulator->rrc_filter = rrc_filter_init(FILTER_TYPE_BANDPASS,
                                                       sample_rate,
                                                       FILTER_LOBES_N,
                                                       FILTER_ORDER,
                                                       FILTER_ALPHA,
                                                       lo_frequency - bandwidth / 2.f,
                                                       lo_frequency + bandwidth / 2.f);

    // Use a low-pass filter otherwise
    else
        qpsk_demodulator->rrc_filter = rrc_filter_init(FILTER_TYPE_LOWPASS,
                                                       sample_rate,
                                                       FILTER_LOBES_N,
                                                       FILTER_ORDER,
                                                       FILTER_ALPHA,
                                                       lo_frequency + bandwidth / 2.f,
                                                       0.f);

    // Initialize PLL
    qpsk_demodulator->pll = pll_init(sample_rate,
                                     lo_frequency,
                                     PLL_K_P, PLL_K_I, qpsk_demodulator->lo_frequency / 2.f,
                                     halfcycles_per_symbol,
                                     PLL_FREQUENCY_FILTER);

    // Initialize IQ filters
    qpsk_demodulator->butter_filter_i = butter_filter_init(FILTER_TYPE_LOWPASS,
                                                           sample_rate,
                                                           IQ_FILTER_ORDER,
                                                           lo_frequency * .7f,
                                                           sqrtf(2.f));
    qpsk_demodulator->butter_filter_q = butter_filter_init(FILTER_TYPE_LOWPASS,
                                                           sample_rate,
                                                           IQ_FILTER_ORDER,
                                                           lo_frequency * .7f,
                                                           sqrtf(2.f));

    // Initialize loop variables
    qpsk_demodulator->sample_time = 0.f;
    qpsk_demodulator->peak_detector = 0.f;
    qpsk_demodulator->carrier_detected = 0;
    qpsk_demodulator->pll_locked = 0;
    qpsk_demodulator->pll_lock_timer = 0.f;
    qpsk_demodulator->iq_error = -1.f;
    qpsk_demodulator->i_sign_prev = 0.f;
    qpsk_demodulator->q_sign_prev = 0.f;
    for (uint8_t i = 0; i < 4; i++)
    {
        qpsk_demodulator->reference_signs_i[i] = 0.f;
        qpsk_demodulator->reference_signs_q[i] = 0.f;
    }
    qpsk_demodulator->reference_signs_set_position = 0;
    qpsk_demodulator->samples_after_zcd = 0;
    qpsk_demodulator->sampling_position = -1;

    return qpsk_demodulator;
}

/**
 * @brief Demodulates one chunk of samples into symbols
 *
 * @param qpsk_demodulator QPSK demodulator's struct
 * @param samples_chunk Aarray of samples
 * @param samples_chunk_size Length of array of symbols
 * @param symbols_chunk Allocated array of symbols (size equal to samples_chunk_size)
 * 0xFF - no symbol decoded at current sample, 0b00, 0b01, 0b10, 0b11 - decoded symbols at specific samples
 * @param debug_messages Enable printf() debug messages (greatly affects performance)
 */
void qpsk_demodulator_demodulate_chunk(qpsk_demodulator_s *qpsk_demodulator,
                                       float *samples_chunk,
                                       uint32_t samples_chunk_size,
                                       uint8_t *symbols_chunk,
                                       uint8_t debug_messages)
{
    // Log number of samples
    if (debug_messages)
        printf("Samples to demodulate: %d\n", samples_chunk_size);

    // Record start time
    clock_t time_started = clock();

    for (uint32_t i = 0; i < samples_chunk_size; i++)
    {
        // Calculate current time (in seconds)
        qpsk_demodulator->sample_time += 1.f / qpsk_demodulator->sample_rate;

        // Extract and filter current sample
        qpsk_demodulator->filtered_sample = rrc_filter_filter(qpsk_demodulator->rrc_filter, samples_chunk[i]);

        // Calculate peak detector
        if (fabsf(qpsk_demodulator->filtered_sample) > qpsk_demodulator->peak_detector)
            qpsk_demodulator->peak_detector = fabsf(qpsk_demodulator->filtered_sample);
        else
            qpsk_demodulator->peak_detector = qpsk_demodulator->peak_detector * AGC_RELEASE_K +
                                              fabsf(qpsk_demodulator->filtered_sample) * (1. - AGC_RELEASE_K);

        // Check if we have any signal
        if (qpsk_demodulator->peak_detector > 0.f)
        {
            // Calculate signal strength
            qpsk_demodulator->signal_strength_rms = 20.f * log10f(qpsk_demodulator->peak_detector / sqrtf(2.f));

            // Set carrier detected flag
            if (!qpsk_demodulator->carrier_detected &&
                qpsk_demodulator->signal_strength_rms > qpsk_demodulator->carrier_start_threshold)
            {
                qpsk_demodulator->carrier_detected = 1;
                if (debug_messages)
                    printf("Carrier detected @ %d/%d (%.3f s). RMS volume: %.1f dBFS\n",
                           i + 1, samples_chunk_size,
                           qpsk_demodulator->sample_time,
                           qpsk_demodulator->signal_strength_rms);
            }

            // Carrier lost
            if (qpsk_demodulator->carrier_detected &&
                qpsk_demodulator->signal_strength_rms < qpsk_demodulator->carrier_lost_threshold)
            {
                qpsk_demodulator->carrier_detected = 0;
                qpsk_demodulator->pll_locked = 0;
                if (debug_messages)
                    printf("Carrier lost and PLL unlocked @ %d/%d (%.3f s). RMS volume: %.1f dBFS\n",
                           i + 1, samples_chunk_size,
                           qpsk_demodulator->sample_time,
                           qpsk_demodulator->signal_strength_rms);
            }

            // Apply automatic gain
            if (qpsk_demodulator->carrier_detected)
            {
                // Apply AGC
                qpsk_demodulator->filtered_sample *= 1.f / qpsk_demodulator->peak_detector;

                // Hard-clip to -1 - 1 (to prevent huge AGC initial overshoot)
                qpsk_demodulator->filtered_sample = clampf(qpsk_demodulator->filtered_sample, -1.f, 1.f);
            }
        }

        // Mix input signal (real values only) with PLL real and imaginary output
        qpsk_demodulator->i_mixed = qpsk_demodulator->filtered_sample * qpsk_demodulator->pll->nco_imag;
        qpsk_demodulator->q_mixed = qpsk_demodulator->filtered_sample * qpsk_demodulator->pll->nco_real;

        // Apply LPFs
        qpsk_demodulator->i_mixed = butter_filter_filter(qpsk_demodulator->butter_filter_i,
                                                         qpsk_demodulator->i_mixed);
        qpsk_demodulator->q_mixed = butter_filter_filter(qpsk_demodulator->butter_filter_q,
                                                         qpsk_demodulator->q_mixed);

        // Calculate signs
        qpsk_demodulator->i_sign = qpsk_demodulator->i_mixed >= 0.f ? 1.f : -1.f;
        qpsk_demodulator->q_sign = qpsk_demodulator->q_mixed >= 0.f ? 1.f : -1.f;

        // Check if carrier is detected but PLL still not locked
        if (qpsk_demodulator->carrier_detected && !qpsk_demodulator->pll_locked)
        {
            // printf("%.2f\n", qpsk_demodulator->pll_lock_timer);
            //  Start timer
            if (qpsk_demodulator->pll_lock_timer == 0.f)
                qpsk_demodulator->pll_lock_timer = qpsk_demodulator->sample_time;

            // Reset timer on large error or first run
            if (fabsf(qpsk_demodulator->iq_error) > qpsk_demodulator->pll_lock_threshold)
                qpsk_demodulator->pll_lock_timer = 0.f;

            // Check timer
            if (qpsk_demodulator->pll_lock_timer > 0.f &&
                qpsk_demodulator->sample_time - qpsk_demodulator->pll_lock_timer > qpsk_demodulator->pll_lock_time)
            {
                // Reset timer
                qpsk_demodulator->pll_lock_timer = 0.f;

                // Set variables
                qpsk_demodulator->pll_locked = 1;

                qpsk_demodulator->reference_signs_i[0] = qpsk_demodulator->i_sign;
                qpsk_demodulator->reference_signs_q[0] = qpsk_demodulator->q_sign;
                qpsk_demodulator->reference_signs_set_position++;

                // Calculated PPM error
                float carrier_error_hz = fabsf(qpsk_demodulator->lo_frequency -
                                               qpsk_demodulator->pll->frequency_filtered);
                float carrier_error_ppm = (carrier_error_hz * 1.e6f) / qpsk_demodulator->lo_frequency;

                if (debug_messages)
                    printf("PLL locked @ %d/%d (%.3f s). VCO: %.2f Hz (error between %.2f Hz: %.0f ppm). Is: %.0f, Qs: %.0f\n",
                           i + 1, samples_chunk_size, qpsk_demodulator->sample_time,
                           qpsk_demodulator->pll->frequency_filtered,
                           qpsk_demodulator->lo_frequency,
                           carrier_error_ppm,
                           qpsk_demodulator->reference_signs_i[0],
                           qpsk_demodulator->reference_signs_q[0]);
            }
        }

        // Carrier not detected or PLL locked -> Reset timer
        else
            qpsk_demodulator->pll_lock_timer = 0.f;

        // Zero-crossing detector (falling edge) using PLL's carrier
        if (qpsk_demodulator->pll->zero_crossing_flag)
        {
            // Clear PLL's zero crossing flag
            qpsk_demodulator->pll->zero_crossing_flag = 0;

            // Reset samples counter
            qpsk_demodulator->samples_after_zcd = 0;
        }

        // No zero-crossing detected -> Increment samples counter
        else
            qpsk_demodulator->samples_after_zcd++;

        // Calculate how many samples one symbol takes
        if (qpsk_demodulator->pll->frequency_filtered > 0.f)
            qpsk_demodulator->samples_per_symbol = qpsk_demodulator->halfcycles_per_symbol *
                                                   (qpsk_demodulator->sample_rate /
                                                    (2.f * qpsk_demodulator->pll->frequency_filtered));
        else
            qpsk_demodulator->samples_per_symbol = 0;

        // Calculate sampling position
        if (qpsk_demodulator->carrier_detected && qpsk_demodulator->pll_locked)
        {
            // Calculate it only one time (at the first phase change)
            // I know that this is not very good, and we need to do a continuous calculation (correction)
            if (qpsk_demodulator->sampling_position < 0)
            {
                // If sign of I or sign of Q changed (phase change)
                if ((qpsk_demodulator->i_sign_prev >= 0 && qpsk_demodulator->i_sign < 0) ||
                    (qpsk_demodulator->i_sign_prev < 0 && qpsk_demodulator->i_sign >= 0) ||
                    (qpsk_demodulator->q_sign_prev >= 0 && qpsk_demodulator->q_sign < 0) ||
                    (qpsk_demodulator->q_sign_prev < 0 && qpsk_demodulator->q_sign >= 0))
                {
                    // Calculate sampling position
                    qpsk_demodulator->sampling_position = qpsk_demodulator->samples_after_zcd +
                                                          qpsk_demodulator->samples_per_symbol / 2.f;

                    // Fix if sampling position is after next zero-cross
                    if (qpsk_demodulator->sampling_position > qpsk_demodulator->samples_per_symbol)
                        qpsk_demodulator->sampling_position -= qpsk_demodulator->samples_per_symbol;

                    // Fix if sampling position too close to nex zero-cross
                    if (qpsk_demodulator->sampling_position > qpsk_demodulator->samples_per_symbol - 2)
                        qpsk_demodulator->sampling_position -= 2;

                    // Log phase change and sampling position
                    if (debug_messages)
                        printf("Detected phase change @ %d/%d (%.3f s). Sampling position: %d samples after zero-cross\n",
                               i + 1, samples_chunk_size, qpsk_demodulator->sample_time,
                               qpsk_demodulator->sampling_position);
                }
            }
        }
        // Reset sampling position
        else
            qpsk_demodulator->sampling_position = -1;

        // Sampling position is set -> Sample data
        if (qpsk_demodulator->sampling_position >= 0 &&
            qpsk_demodulator->samples_after_zcd == qpsk_demodulator->sampling_position)
        {
            // Set other reference phases if 0 is set
            if (0 < qpsk_demodulator->reference_signs_set_position &&
                qpsk_demodulator->reference_signs_set_position < 4)
            {
                // Check if sign changed
                if (qpsk_demodulator->i_sign !=
                        qpsk_demodulator->reference_signs_i[qpsk_demodulator->reference_signs_set_position - 1] ||
                    qpsk_demodulator->q_sign !=
                        qpsk_demodulator->reference_signs_q[qpsk_demodulator->reference_signs_set_position - 1])
                {
                    // Record new reference signs
                    qpsk_demodulator->reference_signs_i[qpsk_demodulator->reference_signs_set_position] =
                        qpsk_demodulator->i_sign;
                    qpsk_demodulator->reference_signs_q[qpsk_demodulator->reference_signs_set_position] =
                        qpsk_demodulator->q_sign;
                    if (debug_messages)
                        printf("IQ values of phase %d recorded @ %d/%d (%.3f s). Is: %.0f, Qs: %.0f\n",
                               qpsk_demodulator->reference_signs_set_position,
                               i + 1, samples_chunk_size, qpsk_demodulator->sample_time,
                               qpsk_demodulator->i_sign, qpsk_demodulator->q_sign);
                    qpsk_demodulator->reference_signs_set_position++;
                }

                // Save empty symbol
                symbols_chunk[i] = 0xFF;
            }

            // Check if we have all reference values
            else if (qpsk_demodulator->reference_signs_set_position >= 4)
            {
                // Decode data according to table of reference signs
                uint8_t iq_symbol = 0;
                for (uint8_t sign_n = 0; sign_n < 4; sign_n++)
                {
                    if (qpsk_demodulator->reference_signs_i[sign_n] == qpsk_demodulator->i_sign &&
                        qpsk_demodulator->reference_signs_q[sign_n] == qpsk_demodulator->q_sign)
                    {
                        iq_symbol = sign_n;
                        break;
                    }
                }

                // Save decoded symbol
                symbols_chunk[i] = iq_symbol;

                // Log decoded symbol
                if (debug_messages)
                    printf("Decoded symbol @ %d/%d (%.3f s): %d (0b" BYTE_TO_BINARY_PATTERN "). I: %.2f, Q: %.2f\n",
                           i + 1, samples_chunk_size, qpsk_demodulator->sample_time,
                           iq_symbol,
                           BYTE_TO_BINARY(iq_symbol),
                           qpsk_demodulator->i_mixed, qpsk_demodulator->q_mixed);
            }
            // Save empty symbol
            else
                symbols_chunk[i] = 0xFF;
        }
        // Save empty symbol
        else
            symbols_chunk[i] = 0xFF;

        // Save signs for the next cycle
        qpsk_demodulator->i_sign_prev = qpsk_demodulator->i_sign;
        qpsk_demodulator->q_sign_prev = qpsk_demodulator->q_sign;

        // Reset signs on carrier lost
        if (!qpsk_demodulator->pll_locked || !qpsk_demodulator->carrier_detected)
        {
            qpsk_demodulator->reference_signs_set_position = 0;
            for (uint8_t sign_n = 0; sign_n < 4; sign_n++)
            {
                qpsk_demodulator->reference_signs_i[sign_n] = 0.f;
                qpsk_demodulator->reference_signs_q[sign_n] = 0.f;
            }
        }

        // Calculate PLL input
        float pll_input_i = qpsk_demodulator->i_sign * qpsk_demodulator->q_mixed;
        float pll_input_q = qpsk_demodulator->q_sign * qpsk_demodulator->i_mixed;

        // Calculate IQ error
        qpsk_demodulator->iq_error = qpsk_demodulator->carrier_detected ? (pll_input_q - pll_input_i) : 0.f;

        // Take one PLL step
        pll_step(qpsk_demodulator->pll, qpsk_demodulator->iq_error, qpsk_demodulator->sample_time);
    }

    if (debug_messages)
    {
        // Record stop time
        clock_t time_stopped = clock();

        // Calculate time
        double time_taken_millis = (((double)(time_stopped - time_started)) / CLOCKS_PER_SEC) * 1000.;
        printf("Demodulation is complete. Time taken: ~%.0f ms\n", time_taken_millis);
    }
}

/**
 * @brief Frees all memory allocated by QPSK demodulator
 *
 * @param qpsk_demodulator QPSK demodulator's struct
 */
void qpsk_demodulator_destroy(qpsk_demodulator_s *qpsk_demodulator)
{
    rrc_filter_destroy(qpsk_demodulator->rrc_filter);
    pll_destroy(qpsk_demodulator->pll);
    butter_filter_destroy(qpsk_demodulator->butter_filter_i);
    butter_filter_destroy(qpsk_demodulator->butter_filter_q);
    free(qpsk_demodulator);
}

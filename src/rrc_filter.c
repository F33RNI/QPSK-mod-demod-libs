/**
 * @file rrc_filter.c
 * @author Fern Lane
 * @brief Root raised cosine (RRC) filter implementation by Fern Lane for filtering continues signal
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
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "rrc_filter.h"

/**
 * @brief Initializes root raised cosine (RRC) filter
 * implementation by Fern Lane
 * @param pass_type FILTER_TYPE_LOWPASS, FILTER_TYPE_HIGHPASS or FILTER_TYPE_BANDPASS
 * @param sample_rate Sampling rate (in Hz)
 * @param positive_lobes_n Used for calculating filter length (use something between 5 and 50)
 * @param order Filter order (number of times filter will be applied), minimum is 1 (>1 doesn't make sense)
 * @param alpha Roll-off factor (from 0 to 0.999 (steepest))
 * @param cutoff_frequency Low/High cutoff frequency (in Hz)
 * @param cutoff_frequency_2 High cutoff frequency for bandpass filter (in Hz) (for low/high-pass simply pass any value)
 * @return rrc_filter_s* Pointer to a struct to store data (aka filter object)
 */
rrc_filter_s *rrc_filter_init(uint8_t pass_type,
                              float sample_rate,
                              uint8_t positive_lobes_n,
                              uint8_t order,
                              float alpha,
                              float cutoff_frequency,
                              float cutoff_frequency_2)
{
    // Allocate struct
    rrc_filter_s *filter = malloc(sizeof(rrc_filter_s));

    // Initialize basic fields
    filter->pass_type = pass_type;
    filter->sample_rate = sample_rate;
    filter->positive_lobes_n = positive_lobes_n;
    filter->order = order;
    filter->alpha = alpha;
    filter->cutoff_frequency = cutoff_frequency;
    filter->cutoff_frequency_2 = cutoff_frequency_2;

    // Calculate filter length
    if (filter->pass_type == FILTER_TYPE_BANDPASS)
        filter->filter_length = (uint32_t)(filter->sample_rate /
                                           (2.f * ((filter->cutoff_frequency + filter->cutoff_frequency_2) / 2.f))) *
                                ((uint32_t)filter->positive_lobes_n * 2 + 1);
    else
        filter->filter_length = (uint32_t)(filter->sample_rate / (2.f * filter->cutoff_frequency)) *
                                ((uint32_t)filter->positive_lobes_n * 2.f + 1.f);

    // Allocate and initialize filter_state (for future convolution)
    filter->filter_state = malloc(order * sizeof(float *));
    for (uint8_t i = 0; i < order; i++)
    {
        filter->filter_state[i] = malloc(filter->filter_length * sizeof(float));
        for (uint32_t j = 0; j < filter->filter_length; j++)
            filter->filter_state[i][j] = 0.f;
    }

    // Generate low-pass impulse response
    filter->impulse_response = _rrc_filter_create_ir(filter, filter->cutoff_frequency);

    // Calculate impulse response for high-pass filter
    if (filter->pass_type == FILTER_TYPE_HIGHPASS)
    {
        // Create delta function
        float *delta_function = malloc(filter->filter_length * sizeof(float));
        for (uint32_t i = 0; i < filter->filter_length; i++)
            if (i == filter->filter_length / 2)
                delta_function[i] = 1.f;
            else
                delta_function[i] = 0.f;

        // Subtract to make impulse response high-pass
        for (uint32_t i = 0; i < filter->filter_length; i++)
            filter->impulse_response[i] = delta_function[i] - filter->impulse_response[i];

        // Delete delta function
        free(delta_function);
    }

    // Calculate impulse response for band-pass filter
    else if (filter->pass_type == FILTER_TYPE_BANDPASS)
    {
        // Generate low-pass impulse response for high-pass component
        float *impulse_response_upper = _rrc_filter_create_ir(filter, filter->cutoff_frequency_2);

        // Subtract upper impulse response from lower to make it band-pass
        for (uint32_t i = 0; i < filter->filter_length; i++)
            filter->impulse_response[i] = filter->impulse_response[i] - impulse_response_upper[i];

        // Delete upper impulse response
        free(impulse_response_upper);
    }

    return filter;
}

/**
 * @brief Generates RRC low-pass impulse response
 *
 * @param filter Filter's struct
 * @param cutoff_frequency low-pass last frequency
 * @return float* impulse response
 */
float *_rrc_filter_create_ir(rrc_filter_s *filter, float cutoff_frequency)
{
    // Calculate Ts
    // P.S. cutoff_frequency * filter->alpha * sqrtf(filter->alpha) is a compensation for alpha (added experimentally)
    float ts = 1.f / (cutoff_frequency + cutoff_frequency * filter->alpha * sqrtf(filter->alpha));

    // Invert alpha to calculate beta
    float beta = 1.f - filter->alpha;

    // Allocate memory
    float *impulse_response = malloc(filter->filter_length * sizeof(float));

    float t;
    float term_0, term_1, term_2, term_3, term_4;
    for (uint32_t i = 0; i < filter->filter_length; i++)
    {
        // Make "virtual" 0 at the center
        t = (-(float)filter->filter_length / 2.f + i) / filter->sample_rate;

        // t = 0
        if (t == 0.)
        {
            term_0 = 1.f / ts;
            term_1 = (1.f + beta * (4.f / M_PI - 1.f));
            impulse_response[i] = term_0 * term_1 / filter->sample_rate;
        }

        // t = +/- ts/(4 * beta)
        else if (abs(t) == (ts / (4.f * beta)))
        {
            term_0 = (beta / (ts * sqrtf(2.f)));
            term_1 = (1.f + 2.f / M_PI);
            term_2 = sinf(M_PI / (4.f * beta));
            term_3 = (1.f - 2.f / M_PI);
            term_4 = cosf(M_PI / (4.f * beta));
            impulse_response[i] = term_0 * (term_1 * term_2 + term_3 * term_4) / filter->sample_rate;
        }

        // Otherwise
        else
        {
            term_0 = 1.f / ts;
            term_1 = sinf(M_PI * (t / ts) * (1.f - beta));
            term_2 = 4.f * beta * (t / ts) * cosf(M_PI * (t / ts) * (1.f + beta));
            term_3 = M_PI * (t / ts);
            term_4 = 1.f - powf(4.f * beta * (t / ts), 2.f);
            impulse_response[i] = term_0 * (term_1 + term_2) / (term_3 * term_4) / filter->sample_rate;
        }
    }

    return impulse_response;
}

/**
 * @brief Filters given value (convolves it with impulse response)
 *
 * @param filter Filter's struct
 * @param input_value Value to filter
 * @return float Filtered value
 */
float rrc_filter_filter(rrc_filter_s *filter, float input_value)
{
    // Filter same value multiple times
    filter->filtered_value_temp = input_value;
    for (uint8_t order_i = 0; order_i < filter->order; order_i++)
    {
        // Fill rotating buffer
        for (uint32_t i = 0; i < filter->filter_length - 1; i++)
            filter->filter_state[order_i][i] = filter->filter_state[order_i][i + 1];
        filter->filter_state[order_i][filter->filter_length - 1] = filter->filtered_value_temp;

        // Convolve
        filter->output_temp = 0.f;
        for (uint32_t i = 0; i < filter->filter_length; i++)
            filter->output_temp += filter->filter_state[order_i][i] * filter->impulse_response[i];

        // Save for next order
        filter->filtered_value_temp = filter->output_temp;
    }

    return filter->filtered_value_temp;
}

/**
 * @brief Frees all memory allocated by filter
 *
 * @param filter Filter's struct
 */
void rrc_filter_destroy(rrc_filter_s *filter)
{
    for (uint8_t i = 0; i < filter->order; i++)
        free(filter->filter_state[i]);
    free(filter->filter_state);
    free(filter->impulse_response);
    free(filter);
}

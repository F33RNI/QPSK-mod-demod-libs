/**
 * @file butterworth_filter.c
 * @author Fern Lane
 * @brief Butterworth Filter implementation for filtering continues signal
 * Original code from: https://github.com/filoe/cscore
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

#include "butterworth_filter.h"

/**
 * @brief Initializes Butterworth filter
 *
 * @param pass_type FILTER_TYPE_LOWPASS or FILTER_TYPE_HIGHPASS
 * @param sample_rate Sampling rate (in Hz)
 * @param order Filter order (number of times filter will be applied to one sample), minimum is 1
 * @param cutoff_frequency Low/High cutoff frequency (in Hz)
 * @param resonance Amount of resonance. Values from sqrt(2) (no resonance (use by default)) to ~0.1 (high resonance)
 * @return butter_filter_s* Pointer to a struct to store data (aka filter object)
 */
butter_filter_s *butter_filter_init(uint8_t pass_type,
                                    float sample_rate,
                                    uint8_t order,
                                    float cutoff_frequency,
                                    float resonance)
{
    // Allocate struct
    butter_filter_s *filter = malloc(sizeof(butter_filter_s));

    // Initialize basic fields
    filter->pass_type = pass_type;
    filter->sample_rate = sample_rate;
    filter->order = order;
    filter->cutoff_frequency = cutoff_frequency;
    filter->resonance = resonance;

    // Initialize filtering constants
    if (pass_type == FILTER_TYPE_LOWPASS)
    {
        float c = 1.f / tanf(M_PI * cutoff_frequency / sample_rate);
        filter->a1 = 1.f / (1.f + resonance * c + c * c);
        filter->a2 = 2.f * filter->a1;
        filter->a3 = filter->a1;
        filter->b1 = 2.f * (1.f - c * c) * filter->a1;
        filter->b2 = (1.f - resonance * c + c * c) * filter->a1;
    }
    else
    {
        float c = tanf(M_PI * cutoff_frequency / sample_rate);
        filter->a1 = 1.f / (1.f + resonance * c + c * c);
        filter->a2 = -2.f * filter->a1;
        filter->a3 = filter->a1;
        filter->b1 = 2.f * (c * c - 1.f) * filter->a1;
        filter->b2 = (1.f - resonance * c + c * c) * filter->a1;
    }

    // Allocate and initialize input histories
    filter->input_histories = malloc(order * sizeof(float *));
    for (uint8_t i = 0; i < order; i++)
    {
        filter->input_histories[i] = malloc(2 * sizeof(float));
        for (uint8_t j = 0; j < 2; j++)
            filter->input_histories[i][j] = 0.f;
    }

    // Allocate and initialize output histories
    filter->output_histories = malloc(order * sizeof(float *));
    for (uint8_t i = 0; i < order; i++)
    {
        filter->output_histories[i] = malloc(3 * sizeof(float));
        for (uint8_t j = 0; j < 3; j++)
            filter->output_histories[i][j] = 0.f;
    }

    return filter;
}

/**
 * @brief Filters given value
 *
 * @param filter Filter's struct
 * @param input_value Value to filter
 * @return float Filtered value
 */
float butter_filter_filter(butter_filter_s *filter, float input_value)
{
    // Filter same value multiple times
    filter->filtered_value_temp = input_value;
    for (uint8_t order_i = 0; order_i < filter->order; order_i++)
    {
        filter->output_temp = filter->a1 * filter->filtered_value_temp +
                              filter->a2 * filter->input_histories[order_i][0] +
                              filter->a3 * filter->input_histories[order_i][1] -
                              filter->b1 * filter->output_histories[order_i][0] -
                              filter->b2 * filter->output_histories[order_i][1];

        filter->input_histories[order_i][1] = filter->input_histories[order_i][0];
        filter->input_histories[order_i][0] = filter->filtered_value_temp;

        filter->output_histories[order_i][2] = filter->output_histories[order_i][1];
        filter->output_histories[order_i][1] = filter->output_histories[order_i][0];
        filter->output_histories[order_i][0] = filter->output_temp;

        // Save for next order
        filter->filtered_value_temp = filter->output_temp;
    }

    return filter->filtered_value_temp;
}

/**
 * @brief Frees all memory allocated by the filter
 *
 * @param filter Filter's struct
 */
void butter_filter_destroy(butter_filter_s *filter)
{
    for (uint8_t i = 0; i < filter->order; i++)
    {
        free(filter->input_histories[i]);
        free(filter->output_histories[i]);
    }
    free(filter->input_histories);
    free(filter->output_histories);
    free(filter);
}

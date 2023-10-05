/**
 * @file butterworth_filter.h
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
#ifndef BUTTERWORTH_FILTER_H__
#define BUTTERWORTH_FILTER_H__

#define M_PI 3.1415927f

#define FILTER_TYPE_LOWPASS 0
#define FILTER_TYPE_HIGHPASS 1

typedef struct
{
    uint8_t pass_type;
    float sample_rate;
    uint8_t order;
    float cutoff_frequency;
    float resonance;

    float **input_histories;
    float **output_histories;
    float a1, a2, a3;
    float b1, b2;
    float filtered_value_temp;
    float output_temp;
} butter_filter_s;

butter_filter_s *butter_filter_init(uint8_t pass_type,
                                    float sample_rate,
                                    uint8_t order,
                                    float cutoff_frequency,
                                    float resonance);

float butter_filter_filter(butter_filter_s *filter, float input_value);

void butter_filter_destroy(butter_filter_s *filter);

#endif

/**
 * @file rrc_filter.h
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
#ifndef RRC_FILTER_H__
#define RRC_FILTER_H__

#define M_PI 3.1415927f

#define FILTER_TYPE_LOWPASS 0
#define FILTER_TYPE_HIGHPASS 1
#define FILTER_TYPE_BANDPASS 2

typedef struct
{
    uint8_t pass_type;
    float sample_rate;
    uint8_t positive_lobes_n;
    uint8_t order;
    float alpha;
    float cutoff_frequency;
    float cutoff_frequency_2;

    uint32_t filter_length;
    float *impulse_response;
    float **filter_state;
    float filtered_value_temp;
    float output_temp;
} rrc_filter_s;

rrc_filter_s *rrc_filter_init(uint8_t pass_type,
                              float sample_rate,
                              uint8_t positive_lobes_n,
                              uint8_t order,
                              float alpha,
                              float cutoff_frequency,
                              float cutoff_frequency_2);

float *_rrc_filter_create_ir(rrc_filter_s *filter, float cutoff_frequency);

float rrc_filter_filter(rrc_filter_s *filter, float input_value);

void rrc_filter_destroy(rrc_filter_s *filter);

#endif

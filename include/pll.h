/**
 * @file pll.h
 * @author Fern Lane
 * @brief Numerically-controlled Phase locked loop
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
#ifndef PLL_H__
#define PLL_H__

#define M_PI 3.1415927f

typedef struct
{
    float sample_rate;
    float lo_frequency;
    float k_p, k_i, k_nco;
    uint16_t halfcycles_per_symbol;
    float frequency_filter_k;

    float integrator;
    float output_phase;
    float nco_real, nco_imag;
    float frequency, frequency_filtered;
    uint8_t zero_crossing_flag;
    float omega;
    uint16_t halfcycles_counter;

    float pi_out;
    float omega_prev, omega_diff;
} pll_s;

pll_s *pll_init(float sample_rate,
                float lo_frequency,
                float k_p, float k_i, float k_nco,
                uint16_t halfcycles_per_symbol,
                float frequency_filter_k);

void pll_step(pll_s *pll, float error, float sample_time);

void pll_destroy(pll_s *pll);

#endif
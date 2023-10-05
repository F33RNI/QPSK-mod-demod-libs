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
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "pll.h"

/**
 * @brief Initializes PLL (Phase-locked-loop)
 *
 * @param sample_rate Sampling rate (in Hz)
 * @param lo_frequency Local oscillator frequency (free-running) (in Hz)
 * @param k_p Proportional gain term of PI controller (default: 0.2)
 * @param k_i Integral gain term of PI controller (default: 8)
 * @param k_nco Gain of NCO (Numerically controlled oscillator) (voltage to frequency transfer coefficient)
 * (default: >= than half of lo_frequency, ex. for lo_frequency=1000, k_nco=500)
 * @param halfcycles_per_symbol Rate of symbols in oscillator's halfcycles (2 halfcycles = 1 full wave cycle)
 * @param frequency_filter_k gain to filter frequency_filtered variable (default: 0.994)
 * @return pll_s* Pointer to a struct to store data (aka pll object)
 */
pll_s *pll_init(float sample_rate,
                float lo_frequency,
                float k_p, float k_i, float k_nco,
                uint16_t halfcycles_per_symbol,
                float frequency_filter_k)
{
    // Allocate struct
    pll_s *pll = malloc(sizeof(pll_s));

    // Initialize basic fields
    pll->sample_rate = sample_rate;
    pll->lo_frequency = lo_frequency;
    pll->k_p = k_p;
    pll->k_i = k_i;
    pll->k_nco = k_nco;
    pll->halfcycles_per_symbol = halfcycles_per_symbol;
    pll->frequency_filter_k = frequency_filter_k;

    // Initialize other variables
    pll->integrator = 0.f;
    pll->output_phase = 0.f;
    pll->nco_real = 1.f;
    pll->nco_imag = 0.f;
    pll->frequency = pll->lo_frequency;
    pll->frequency_filtered = pll->lo_frequency;
    pll->zero_crossing_flag = 0;
    pll->omega = 0.f;
    pll->halfcycles_counter = 0;

    return pll;
}

/**
 * @brief Calculates one step (one loop cycle) of PLL
 *
 * @param pll PLL's struct
 * @param error Input error (ex. for QPSK: sign_q * i_mixed - sign_i * q_mixed)
 * @param sample_time Current sample's time (in s) (aka current absolute time in seconds from init)
 */
void pll_step(pll_s *pll, float error, float sample_time)
{
    // Calculate integral controller
    pll->integrator += pll->k_i * error * (1.f / pll->sample_rate);

    // Calculate PI output using proportional controller and integral controller
    pll->pi_out = pll->k_p * error + pll->integrator;

    // Calculate output phase
    pll->output_phase += 2.f * M_PI * pll->pi_out * pll->k_nco * (1.f / pll->sample_rate);

    // Calculate base for sin and cos
    pll->omega_prev = pll->omega;
    pll->omega = 2.f * M_PI * pll->lo_frequency * sample_time + pll->output_phase;
    pll->omega = fmodf(pll->omega, 2.f * M_PI);

    // Calculate momentary phase difference
    if (pll->omega > pll->omega_prev)
        pll->omega_diff = pll->omega - pll->omega_prev;
    else
        pll->omega_diff = (pll->omega + (2.f * M_PI)) - pll->omega_prev;

    // Calculate current VCO frequency
    pll->frequency = (pll->omega_diff / (2.f * M_PI)) * pll->sample_rate;
    if (pll->frequency > 0.f)
        pll->frequency_filtered = pll->frequency_filtered * pll->frequency_filter_k +
                                  pll->frequency * (1.f - pll->frequency_filter_k);

    // Half-cycle detector
    // omega_prev <= pi <= omega and omega != omega_prev -> pi cross
    // omega_prev > omega -> 2*pi cross
    if ((pll->omega_prev <= M_PI && M_PI <= pll->omega && pll->omega != pll->omega_prev) || pll->omega_prev > pll->omega)
        pll->halfcycles_counter += 1;

    // Symbol rate detector (aka Zero-crossing detector) @ halfcycles_per_symbol * PI (symbol rate)
    if (pll->halfcycles_counter >= pll->halfcycles_per_symbol)
    {
        pll->halfcycles_counter = 0;
        pll->zero_crossing_flag = 1;
    }

    // Calculate complex output
    pll->nco_real = cosf(pll->omega);
    pll->nco_imag = -sinf(pll->omega);
}

/**
 * @brief Frees memory allocated by pll struct
 *
 * @param pll PLL's struct
 */
void pll_destroy(pll_s *pll)
{
    free(pll);
}
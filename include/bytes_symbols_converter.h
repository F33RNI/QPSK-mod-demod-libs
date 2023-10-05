/**
 * @file bytes_symbols_converter.c
 * @author Fern Lane
 * @brief Methods to convert array of bytes to symbols and back
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
#ifndef BYTES_SYMBOLS_CONVERTER_H__
#define BYTES_SYMBOLS_CONVERTER_H__

typedef struct
{
    uint8_t *symbols;
    uint32_t symbols_length;
} symbols_s;

uint32_t bytes_to_symbols_calculate_length(uint32_t bytes_length,
                                           uint32_t start_silence_length,
                                           uint32_t preamble_length,
                                           uint8_t add_iq_sync_byte,
                                           uint32_t end_silence_length);

uint8_t *bytes_to_symbols(uint8_t *bytes,
                          uint32_t bytes_length,
                          uint32_t start_silence_length,
                          uint32_t preamble_length,
                          uint8_t add_iq_sync_byte,
                          uint32_t end_silence_length);

uint8_t *symbols_to_bytes(uint8_t *symbols, uint32_t symbols_length);

symbols_s *symbols_remove_empty(uint8_t *symbols, uint32_t symbols_length);
#endif

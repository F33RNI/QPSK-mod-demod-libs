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
#include <stdlib.h>
#include <stdint.h>

#include "bytes_symbols_converter.h"

/**
 * @brief Calculates length of array of symbols (converted from bytes)
 *
 * @param bytes_length Length of bytes array (payload size in bytes, 0-N)
 * @param start_silence_length Add silence before preamble (length in symbols, 0-N)
 * @param preamble_length Add preamble (00) before sync byte (length in symbols, 0-N)
 * @param add_iq_sync_byte Add IQ (phase) sync byte (00011011) before payload (0 - false, 1 - true)
 * @param end_silence_length Add silence after payload (length in symbols, 0-N)
 * @return uint32_t start_silence_length +
 *                  preamble_length +
 *                  (add_iq_sync_byte ? 4 : 0) +
 *                  bytes_length * 4 +
 *                  end_silence_length
 */
uint32_t bytes_to_symbols_calculate_length(uint32_t bytes_length,
                                           uint32_t start_silence_length,
                                           uint32_t preamble_length,
                                           uint8_t add_iq_sync_byte,
                                           uint32_t end_silence_length)
{
    return (start_silence_length +
            preamble_length +
            (add_iq_sync_byte ? 4 : 0) +
            bytes_length * 4 +
            end_silence_length);
}

/**
 * @brief Converts array of bytes to symbols for future modulation
 *
 * @param bytes Array of bytes (payload) (use NULL if bytes_length is 0)
 * @param bytes_length Length of bytes array (payload size in bytes, 0-N)
 * @param start_silence_length Add silence before preamble (length in symbols, 0-N)
 * @param preamble_length Add preamble (00) before sync byte (length in symbols, 0-N)
 * @param add_iq_sync_byte Add IQ (phase) sync byte (00011011) before payload (0 - false, 1 - true)
 * @param end_silence_length Add silence after payload (length in symbols, 0-N)
 * @return uint8_t* Array of symbols. Total length will be: bytes_to_symbols_calculate_length()
 */
uint8_t *bytes_to_symbols(uint8_t *bytes,
                          uint32_t bytes_length,
                          uint32_t start_silence_length,
                          uint32_t preamble_length,
                          uint8_t add_iq_sync_byte,
                          uint32_t end_silence_length)
{
    // Allocate array of symbols. Each byte = 4 symbols (QPSK)
    uint8_t *symbols = malloc(bytes_to_symbols_calculate_length(bytes_length,
                                                                start_silence_length,
                                                                preamble_length,
                                                                add_iq_sync_byte,
                                                                end_silence_length) *
                              sizeof(uint8_t));
    // Fill start silence
    for (uint32_t i = 0; i < start_silence_length; i++)
        symbols[i] = 0xFF;

    // Fill preamble
    for (uint32_t i = 0; i < preamble_length; i++)
        symbols[start_silence_length + i] = 0;

    // Add IQ (initial phase) sync byte (0b00011011) (0 1 2 3)
    if (add_iq_sync_byte)
    {
        symbols[start_silence_length + preamble_length] = 0b00;
        symbols[start_silence_length + preamble_length + 1] = 0b01;
        symbols[start_silence_length + preamble_length + 2] = 0b10;
        symbols[start_silence_length + preamble_length + 3] = 0b11;
    }

    // Fill bytes
    uint32_t symbols_counter = 0;
    uint8_t current_byte = 0;
    for (uint32_t i = 0; i < bytes_length * 4; i++)
    {
        // Extract new byte and reset symbols (bit) counter
        if (symbols_counter == 0)
        {
            current_byte = bytes[i / 4];
            symbols_counter = 4;
        }
        symbols_counter--;

        // Fill symbols arrays
        symbols[start_silence_length + preamble_length + (add_iq_sync_byte ? 4 : 0) + i] =
            (current_byte >> (symbols_counter * 2)) & 0b11;
    }

    // Fill end silence
    for (uint32_t i = 0; i < end_silence_length; i++)
        symbols[start_silence_length + preamble_length + (add_iq_sync_byte ? 4 : 0) + bytes_length * 4 + i] = 0xFF;

    return symbols;
}

/**
 * @brief Converts array of symbols to array of bytes
 *
 * @param symbols Array of symbols
 * @param symbols_length Length of symbols array (number of symbols)
 * @return uint8_t* Array of bytes. Total length will be: symbols_length / 4
 */
uint8_t *symbols_to_bytes(uint8_t *symbols, uint32_t symbols_length)
{
    // Allocate array of bytes
    uint8_t *bytes = malloc(symbols_length / 4 * sizeof(uint8_t));

    uint8_t current_byte = 0;
    for (uint32_t i = 0; i < symbols_length; i++)
    {
        // Build from symbol bits
        current_byte |= (symbols[i] & 0b11) << (6 - (i % 4) * 2);
        if (i % 4 == 3)
        {
            bytes[i / 4] = current_byte;
            current_byte = 0;
        }
    }

    return bytes;
}

/**
 * @brief Removes 0xFF from array of symbols
 *
 * @param symbols Source array of symbols
 * @param symbols_length Length of source array (number of symbols)
 * @return symbols_s* Symbol's struct
 */
symbols_s *symbols_remove_empty(uint8_t *symbols, uint32_t symbols_length)
{
    // Allocate initial memory
    symbols_s *symbols_cleared = malloc(sizeof(symbols_s));
    symbols_cleared->symbols = malloc(0);
    symbols_cleared->symbols_length = 0;

    for (uint32_t i = 0; i < symbols_length; i++)
    {
        // Check if current symbol is not empty
        if (symbols[i] != 0xFF)
        {
            // Reallocate symbols array
            symbols_cleared->symbols = realloc(symbols_cleared->symbols,
                                               (symbols_cleared->symbols_length + 1) * sizeof(uint8_t));

            // Assign current symbol
            symbols_cleared->symbols[symbols_cleared->symbols_length] = symbols[i];

            // Increment size
            symbols_cleared->symbols_length++;
        }
    }

    return symbols_cleared;
}

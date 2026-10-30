#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include <cstddef>
#include <cstdint>

#include <mbedtls/pk.h>

namespace esphome::airplay {

static constexpr size_t RSA_SIGNATURE_SIZE = 256;

/// RSA operations of the AirPlay 1 (RAOP) handshake, using the well-known AirPort Express key.
class RAOPCrypto {
 public:
  RAOPCrypto();
  ~RAOPCrypto();
  RAOPCrypto(const RAOPCrypto &) = delete;
  RAOPCrypto &operator=(const RAOPCrypto &) = delete;

  /// @brief Parses the private key. Call once during setup.
  bool init();

  /// @brief Answers an "Apple-Challenge": signs challenge + local IP + MAC, zero padded to 32 bytes.
  /// @param output Room for RSA_SIGNATURE_SIZE bytes.
  bool sign_challenge(const uint8_t *data, size_t length, uint8_t *output);

  /// @brief Decrypts the RSA-OAEP encrypted AES key from the "rsaaeskey" SDP attribute.
  /// @param key Receives the 16 byte AES key.
  bool decrypt_aes_key(const uint8_t *data, size_t length, uint8_t *key);

 protected:
  mbedtls_pk_context pk_;
  bool initialized_{false};
};

/// @brief Decodes base64 text, with or without '=' padding. Stops at the first character outside the alphabet.
/// @return Number of bytes written, or 0 if the output buffer is too small.
size_t base64_decode_loose(const char *input, size_t input_length, uint8_t *output, size_t output_size);

/// @brief Encodes data as base64 without '=' padding, as AirPlay expects. The output is null terminated.
/// @return Length of the encoded text, or 0 if the output buffer is too small.
size_t base64_encode_unpadded(const uint8_t *input, size_t input_length, char *output, size_t output_size);

}  // namespace esphome::airplay

#endif  // USE_ESP32

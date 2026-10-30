#include "raop_crypto.h"

#include <cstring>

#include <mbedtls/rsa.h>

#ifdef ESP_PLATFORM
#include <esp_random.h>
#else
#include <random>
#endif

namespace esphome::airplay {

// Private key of the AirPort Express, published in 2011 and used by every open source AirPlay 1 receiver.
// AirPlay 1 senders check the "Apple-Challenge" signature and encrypt the session key with it.
static const char AIRPORT_PRIVATE_KEY[] =
    "-----BEGIN RSA PRIVATE KEY-----\n"
    "MIIEpQIBAAKCAQEA59dE8qLieItsH1WgjrcFRKj6eUWqi+bGLOX1HL3U3GhC/j0Qg90u3sG/1CUt\n"
    "wC5vOYvfDmFI6oSFXi5ELabWJmT2dKHzBJKa3k9ok+8t9ucRqMd6DZHJ2YCCLlDRKSKv6kDqnw4U\n"
    "wPdpOMXziC/AMj3Z/lUVX1G7WSHCAWKf1zNS1eLvqr+boEjXuBOitnZ/bDzPHrTOZz0Dew0uowxf\n"
    "/+sG+NCK3eQJVxqcaJ/vEHKIVd2M+5qL71yJQ+87X6oV3eaYvt3zWZYD6z5vYTcrtij2VZ9Zmni/\n"
    "UAaHqn9JdsBWLUEpVviYnhimNVvYFZeCXg/IdTQ+x4IRdiXNv5hEewIDAQABAoIBAQDl8Axy9XfW\n"
    "BLmkzkEiqoSwF0PsmVrPzH9KsnwLGH+QZlvjWd8SWYGN7u1507HvhF5N3drJoVU3O14nDY4TFQAa\n"
    "LlJ9VM35AApXaLyY1ERrN7u9ALKd2LUwYhM7Km539O4yUFYikE2nIPscEsA5ltpxOgUGCY7b7ez5\n"
    "NtD6nL1ZKauw7aNXmVAvmJTcuPxWmoktF3gDJKK2wxZuNGcJE0uFQEG4Z3BrWP7yoNuSK3dii2jm\n"
    "lpPHr0O/KnPQtzI3eguhe0TwUem/eYSdyzMyVx/YpwkzwtYL3sR5k0o9rKQLtvLzfAqdBxBurciz\n"
    "aaA/L0HIgAmOit1GJA2saMxTVPNhAoGBAPfgv1oeZxgxmotiCcMXFEQEWflzhWYTsXrhUIuz5jFu\n"
    "a39GLS99ZEErhLdrwj8rDDViRVJ5skOp9zFvlYAHs0xh92ji1E7V/ysnKBfsMrPkk5KSKPrnjndM\n"
    "oPdevWnVkgJ5jxFuNgxkOLMuG9i53B4yMvDTCRiIPMQ++N2iLDaRAoGBAO9v//mU8eVkQaoANf0Z\n"
    "oMjW8CN4xwWA2cSEIHkd9AfFkftuv8oyLDCG3ZAf0vrhrrtkrfa7ef+AUb69DNggq4mHQAYBp7L+\n"
    "k5DKzJrKuO0r+R0YbY9pZD1+/g9dVt91d6LQNepUE/yY2PP5CNoFmjedpLHMOPFdVgqDzDFxU8hL\n"
    "AoGBANDrr7xAJbqBjHVwIzQ4To9pb4BNeqDndk5Qe7fT3+/H1njGaC0/rXE0Qb7q5ySgnsCb3DvA\n"
    "cJyRM9SJ7OKlGt0FMSdJD5KG0XPIpAVNwgpXXH5MDJg09KHeh0kXo+QA6viFBi21y340NonnEfdf\n"
    "54PX4ZGS/Xac1UK+pLkBB+zRAoGAf0AY3H3qKS2lMEI4bzEFoHeK3G895pDaK3TFBVmD7fV0Zhov\n"
    "17fegFPMwOII8MisYm9ZfT2Z0s5Ro3s5rkt+nvLAdfC/PYPKzTLalpGSwomSNYJcB9HNMlmhkGzc\n"
    "1JnLYT4iyUyx6pcZBmCd8bD0iwY/FzcgNDaUmbX9+XDvRA0CgYEAkE7pIPlE71qvfJQgoA9em0gI\n"
    "LAuE4Pu13aKiJnfft7hIjbK+5kyb3TysZvoyDnb3HOKvInK7vXbKuU4ISgxB2bB3HcYzQMGsz1qJ\n"
    "2gG0N5hvJpzwwhbhXqFKA4zaaSrw622wDniAK5MlIE0tIAKKP4yxNGjoD2QYjhBGuhvkWKY=\n"
    "-----END RSA PRIVATE KEY-----\n";

static int random_bytes(void * /*context*/, unsigned char *output, size_t length) {
#ifdef ESP_PLATFORM
  esp_fill_random(output, length);
#else
  static std::random_device device;
  for (size_t i = 0; i < length; i++)
    output[i] = static_cast<unsigned char>(device());
#endif
  return 0;
}

RAOPCrypto::RAOPCrypto() { mbedtls_pk_init(&this->pk_); }

RAOPCrypto::~RAOPCrypto() { mbedtls_pk_free(&this->pk_); }

bool RAOPCrypto::init() {
  // The PEM parser needs the terminating null in the length
  if (mbedtls_pk_parse_key(&this->pk_, reinterpret_cast<const unsigned char *>(AIRPORT_PRIVATE_KEY),
                           sizeof(AIRPORT_PRIVATE_KEY), nullptr, 0, random_bytes, nullptr) != 0) {
    return false;
  }
  if (mbedtls_pk_get_type(&this->pk_) != MBEDTLS_PK_RSA)
    return false;
  // OAEP uses the hash configured here; the challenge signature uses PKCS#1 v1.5 explicitly
  if (mbedtls_rsa_set_padding(mbedtls_pk_rsa(this->pk_), MBEDTLS_RSA_PKCS_V21, MBEDTLS_MD_SHA1) != 0)
    return false;
  this->initialized_ = true;
  return true;
}

bool RAOPCrypto::sign_challenge(const uint8_t *data, size_t length, uint8_t *output) {
  static constexpr size_t MIN_LENGTH = 32;
  static constexpr size_t MAX_LENGTH = 64;
  if (!this->initialized_ || length > MAX_LENGTH)
    return false;
  uint8_t buffer[MAX_LENGTH]{};
  std::memcpy(buffer, data, length);
  if (length < MIN_LENGTH)
    length = MIN_LENGTH;
  // MBEDTLS_MD_NONE signs the raw bytes with PKCS#1 v1.5 type 1 padding
  return mbedtls_rsa_rsassa_pkcs1_v15_sign(mbedtls_pk_rsa(this->pk_), random_bytes, nullptr, MBEDTLS_MD_NONE,
                                           static_cast<unsigned int>(length), buffer, output) == 0;
}

bool RAOPCrypto::decrypt_aes_key(const uint8_t *data, size_t length, uint8_t *key) {
  static constexpr size_t AES_KEY_SIZE = 16;
  if (!this->initialized_ || length != RSA_SIGNATURE_SIZE)
    return false;
  uint8_t plain[RSA_SIGNATURE_SIZE];
  size_t plain_length = 0;
  if (mbedtls_rsa_rsaes_oaep_decrypt(mbedtls_pk_rsa(this->pk_), random_bytes, nullptr, nullptr, 0, &plain_length, data,
                                     plain, sizeof(plain)) != 0 ||
      plain_length != AES_KEY_SIZE) {
    return false;
  }
  std::memcpy(key, plain, AES_KEY_SIZE);
  return true;
}

static int8_t base64_value(char c) {
  if (c >= 'A' && c <= 'Z')
    return static_cast<int8_t>(c - 'A');
  if (c >= 'a' && c <= 'z')
    return static_cast<int8_t>(c - 'a' + 26);
  if (c >= '0' && c <= '9')
    return static_cast<int8_t>(c - '0' + 52);
  if (c == '+')
    return 62;
  if (c == '/')
    return 63;
  return -1;
}

size_t base64_decode_loose(const char *input, size_t input_length, uint8_t *output, size_t output_size) {
  uint32_t accumulator = 0;
  uint32_t bits = 0;
  size_t written = 0;
  for (size_t i = 0; i < input_length; i++) {
    const int8_t value = base64_value(input[i]);
    if (value < 0)
      break;
    accumulator = (accumulator << 6) | static_cast<uint32_t>(value);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      if (written >= output_size)
        return 0;
      output[written++] = static_cast<uint8_t>(accumulator >> bits);
    }
  }
  return written;
}

size_t base64_encode_unpadded(const uint8_t *input, size_t input_length, char *output, size_t output_size) {
  static constexpr char ALPHABET[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  const size_t encoded_length = (input_length * 4 + 2) / 3;
  if (encoded_length + 1 > output_size)
    return 0;
  uint32_t accumulator = 0;
  uint32_t bits = 0;
  size_t written = 0;
  for (size_t i = 0; i < input_length; i++) {
    accumulator = (accumulator << 8) | input[i];
    bits += 8;
    while (bits >= 6) {
      bits -= 6;
      output[written++] = ALPHABET[(accumulator >> bits) & 0x3f];
    }
  }
  if (bits > 0)
    output[written++] = ALPHABET[(accumulator << (6 - bits)) & 0x3f];
  output[written] = '\0';
  return written;
}

}  // namespace esphome::airplay

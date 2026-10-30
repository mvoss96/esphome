#include "alac_decoder.h"

#ifdef USE_ESP32

#include <algorithm>
#include <cstring>

namespace esphome::airplay {

// Constants of the adaptive Golomb-Rice coder (ag_dec.c in Apple's reference decoder)
static constexpr uint32_t QBSHIFT = 9;
static constexpr uint32_t QB = 1u << QBSHIFT;
static constexpr uint32_t MMULSHIFT = 2;
static constexpr uint32_t MDENSHIFT = QBSHIFT - MMULSHIFT - 1;
static constexpr uint32_t MOFF = 1u << (MDENSHIFT - 2);
static constexpr uint32_t BITOFF = 24;
static constexpr uint32_t N_MAX_MEAN_CLAMP = 0xffff;
static constexpr uint32_t N_MEAN_CLAMP_VAL = 0xffff;
static constexpr uint32_t MAX_PREFIX = 9;
static constexpr uint32_t ZERO_RUN_ESCAPE_BITS = 16;

static constexpr uint32_t MAX_COEFS = 32;

enum ElementType : uint8_t {
  ELEMENT_TYPE_SCE = 0,  // single channel element
  ELEMENT_TYPE_CPE = 1,  // channel pair element
  ELEMENT_TYPE_CCE = 2,  // coupling channel element (unsupported)
  ELEMENT_TYPE_LFE = 3,  // low frequency effects element, decoded like SCE
  ELEMENT_TYPE_DSE = 4,  // data stream element
  ELEMENT_TYPE_PCE = 5,  // program config element (unsupported)
  ELEMENT_TYPE_FIL = 6,  // fill element
  ELEMENT_TYPE_END = 7,
};

namespace {

/// Count of leading zero bits, defined as 32 for zero
inline uint32_t clz32(uint32_t value) { return value == 0 ? 32 : static_cast<uint32_t>(__builtin_clz(value)); }

inline int32_t sign_of(int32_t value) { return (value > 0) - (value < 0); }

/// Sign-extends the low `bits` bits of a value
inline int32_t sign_extend(int32_t value, uint32_t shift) {
  return static_cast<int32_t>(static_cast<uint32_t>(value) << shift) >> shift;
}

/// MSB-first bit reader. Reads past the end return zero bits and set the overrun flag.
class BitReader {
 public:
  BitReader(const uint8_t *data, size_t length) : data_(data), bit_length_(length * 8) {}

  /// Returns the next 32 bits without consuming them
  uint32_t peek32() const {
    size_t byte = this->position_ >> 3;
    uint64_t value = 0;
    for (size_t i = 0; i < 5; i++) {
      value <<= 8;
      if (byte + i < (this->bit_length_ >> 3))
        value |= this->data_[byte + i];
    }
    return static_cast<uint32_t>(value >> (8 - (this->position_ & 7)));
  }

  /// Reads up to 32 bits
  uint32_t read(uint32_t bits) {
    if (bits == 0)
      return 0;
    uint32_t value = this->peek32() >> (32 - bits);
    this->position_ += bits;
    return value;
  }

  void skip(size_t bits) { this->position_ += bits; }
  void byte_align() { this->position_ = (this->position_ + 7) & ~static_cast<size_t>(7); }
  bool overrun() const { return this->position_ > this->bit_length_; }

 protected:
  const uint8_t *data_;
  size_t bit_length_;
  size_t position_{0};
};

/// Reads one adaptive Golomb-Rice coded value (dyn_get and dyn_get_32bit in the reference decoder).
uint32_t read_rice(BitReader &reader, uint32_t m, uint32_t k, uint32_t escape_bits) {
  const uint32_t prefix = clz32(~reader.peek32());
  if (prefix >= MAX_PREFIX) {
    reader.skip(MAX_PREFIX);
    return reader.read(escape_bits);
  }
  reader.skip(prefix + 1);
  const uint32_t value = reader.peek32() >> (32 - k);
  if (value < 2) {
    reader.skip(k - 1);
    return prefix * m;
  }
  reader.skip(k);
  return prefix * m + value - 1;
}

/// Decodes the entropy coded prediction residuals of one channel (dyn_decomp in the reference decoder).
bool decompress(BitReader &reader, int32_t *out, uint32_t num_samples, uint32_t max_size, uint32_t pb, uint32_t mb0,
                uint32_t kb) {
  const uint32_t wb = (1u << kb) - 1;
  uint32_t mb = mb0;
  uint32_t zmode = 0;
  uint32_t count = 0;

  while (count < num_samples) {
    if (reader.overrun())
      return false;

    uint32_t k = std::min(31 - clz32((mb >> QBSHIFT) + 3), kb);
    uint32_t m = (1u << k) - 1;
    uint32_t n = read_rice(reader, m, k, max_size);

    // The least significant bit is the sign bit
    const uint32_t ndecode = n + zmode;
    const int32_t multiplier = -static_cast<int32_t>(ndecode & 1) | 1;
    out[count++] = static_cast<int32_t>((ndecode + 1) >> 1) * multiplier;

    mb = pb * (n + zmode) + mb - ((pb * mb) >> QBSHIFT);
    if (n > N_MAX_MEAN_CLAMP)
      mb = N_MEAN_CLAMP_VAL;

    zmode = 0;
    if (((mb << MMULSHIFT) < QB) && (count < num_samples)) {
      // Run of zeros
      zmode = 1;
      k = clz32(mb) - BITOFF + ((mb + MOFF) >> MDENSHIFT);
      const uint32_t mz = ((1u << k) - 1) & wb;
      n = read_rice(reader, mz, k, ZERO_RUN_ESCAPE_BITS);
      if (count + n > num_samples)
        return false;
      std::fill_n(out + count, n, 0);
      count += n;
      if (n >= 65535)
        zmode = 0;
      mb = 0;
    }
  }
  return !reader.overrun();
}

/// Runs the adaptive FIR predictor in reverse (unpc_block in the reference decoder). `in` and `out` may be equal.
void unpredict(const int32_t *in, int32_t *out, uint32_t num, int16_t *coefs, uint32_t num_active, uint32_t chan_bits,
               uint32_t den_shift) {
  const uint32_t chan_shift = 32 - chan_bits;
  const int32_t den_half = den_shift > 0 ? (1 << (den_shift - 1)) : 0;

  out[0] = in[0];
  if (num_active == 0) {
    if (num > 1 && in != out)
      std::memcpy(&out[1], &in[1], (num - 1) * sizeof(int32_t));
    return;
  }

  if (num_active == 31) {
    // First order difference only
    int32_t prev = out[0];
    for (uint32_t j = 1; j < num; j++) {
      prev = sign_extend(in[j] + prev, chan_shift);
      out[j] = prev;
    }
    return;
  }

  const uint32_t warmup = std::min(num_active + 1, num);
  for (uint32_t j = 1; j < warmup; j++)
    out[j] = sign_extend(in[j] + out[j - 1], chan_shift);

  const uint32_t lim = num_active + 1;
  for (uint32_t j = lim; j < num; j++) {
    const int32_t *pout = out + j - 1;
    const int32_t top = out[j - lim];

    int32_t sum = 0;
    for (uint32_t k = 0; k < num_active; k++)
      sum += coefs[k] * (pout[-static_cast<int32_t>(k)] - top);

    int32_t del = in[j];
    int32_t del0 = del;
    const int32_t sg = sign_of(del);
    del += top + ((sum + den_half) >> den_shift);
    out[j] = sign_extend(del, chan_shift);

    // Adapt the coefficients towards the sign of the residual
    if (sg > 0) {
      for (int32_t k = static_cast<int32_t>(num_active) - 1; k >= 0; k--) {
        const int32_t dd = top - pout[-k];
        const int32_t sgn = sign_of(dd);
        coefs[k] -= sgn;
        del0 -= (static_cast<int32_t>(num_active) - k) * ((sgn * dd) >> den_shift);
        if (del0 <= 0)
          break;
      }
    } else if (sg < 0) {
      for (int32_t k = static_cast<int32_t>(num_active) - 1; k >= 0; k--) {
        const int32_t dd = top - pout[-k];
        const int32_t sgn = sign_of(dd);
        coefs[k] += sgn;
        del0 -= (static_cast<int32_t>(num_active) - k) * ((-sgn * dd) >> den_shift);
        if (del0 >= 0)
          break;
      }
    }
  }
}

}  // namespace

bool ALACDecoder::init(uint32_t max_frame_length) {
  if (max_frame_length == 0)
    return false;
  this->predictor_ = std::make_unique<int32_t[]>(max_frame_length);
  this->mix_u_ = std::make_unique<int32_t[]>(max_frame_length);
  this->mix_v_ = std::make_unique<int32_t[]>(max_frame_length);
  this->max_frame_length_ = max_frame_length;
  return true;
}

bool ALACDecoder::configure(const ALACConfig &config) {
  this->configured_ = false;
  if (config.bit_depth != 16 || config.num_channels < 1 || config.num_channels > 2 || config.frame_length == 0 ||
      config.frame_length > this->max_frame_length_ || config.kb == 0 || config.kb > 31) {
    return false;
  }
  this->config_ = config;
  this->configured_ = true;
  return true;
}

int32_t ALACDecoder::decode(const uint8_t *data, size_t length, int16_t *output) {
  if (!this->configured_)
    return -1;

  BitReader reader(data, length);
  const uint32_t num_channels = this->config_.num_channels;
  uint32_t channel_index = 0;
  uint32_t num_samples = this->config_.frame_length;

  while (channel_index < num_channels) {
    if (reader.overrun())
      return -1;

    const uint32_t tag = reader.read(3);
    switch (tag) {
      case ELEMENT_TYPE_SCE:
      case ELEMENT_TYPE_LFE:
      case ELEMENT_TYPE_CPE: {
        const uint32_t element_channels = (tag == ELEMENT_TYPE_CPE) ? 2 : 1;
        if (channel_index + element_channels > num_channels)
          return -1;

        reader.skip(4);  // element instance tag
        if (reader.read(12) != 0)
          return -1;

        const uint32_t header = reader.read(4);
        const bool partial_frame = (header >> 3) != 0;
        const uint32_t bytes_shifted = (header >> 1) & 0x3;
        const bool escape = (header & 0x1) != 0;
        if (bytes_shifted == 3)
          return -1;

        num_samples = this->config_.frame_length;
        if (partial_frame) {
          num_samples = reader.read(16) << 16;
          num_samples |= reader.read(16);
          if (num_samples == 0 || num_samples > this->config_.frame_length)
            return -1;
        }

        int32_t *mix[2] = {this->mix_u_.get(), this->mix_v_.get()};
        uint32_t mix_bits = 0;
        int32_t mix_res = 0;

        if (!escape) {
          const uint32_t chan_bits = this->config_.bit_depth - (bytes_shifted * 8) + (element_channels - 1);
          mix_bits = reader.read(8);
          mix_res = static_cast<int8_t>(reader.read(8));

          uint32_t mode[2];
          uint32_t den_shift[2];
          uint32_t pb_factor[2];
          uint32_t num_coefs[2];
          int16_t coefs[2][MAX_COEFS];
          for (uint32_t ch = 0; ch < element_channels; ch++) {
            uint32_t byte = reader.read(8);
            mode[ch] = byte >> 4;
            den_shift[ch] = byte & 0xf;
            byte = reader.read(8);
            pb_factor[ch] = byte >> 5;
            num_coefs[ch] = byte & 0x1f;
            for (uint32_t i = 0; i < num_coefs[ch]; i++)
              coefs[ch][i] = static_cast<int16_t>(reader.read(16));
          }

          // Shifted-off low bits only exist for bit depths above 16; skip them like the reference decoder does
          if (bytes_shifted != 0)
            reader.skip(static_cast<size_t>(bytes_shifted) * 8 * element_channels * num_samples);

          for (uint32_t ch = 0; ch < element_channels; ch++) {
            int32_t *predictor = this->predictor_.get();
            if (!decompress(reader, predictor, num_samples, chan_bits, (this->config_.pb * pb_factor[ch]) / 4,
                            this->config_.mb, this->config_.kb)) {
              return -1;
            }
            if (mode[ch] != 0)
              unpredict(predictor, predictor, num_samples, nullptr, 31, chan_bits, 0);
            unpredict(predictor, mix[ch], num_samples, coefs[ch], num_coefs[ch], chan_bits, den_shift[ch]);
          }
        } else {
          // Uncompressed samples
          const uint32_t chan_bits = this->config_.bit_depth;
          const uint32_t shift = 32 - chan_bits;
          for (uint32_t i = 0; i < num_samples; i++) {
            for (uint32_t ch = 0; ch < element_channels; ch++)
              mix[ch][i] = sign_extend(static_cast<int32_t>(reader.read(chan_bits)), shift);
          }
        }

        if (reader.overrun())
          return -1;

        int16_t *out = output + channel_index;
        if (element_channels == 2) {
          const int32_t *u = mix[0];
          const int32_t *v = mix[1];
          for (uint32_t i = 0; i < num_samples; i++) {
            int32_t left = u[i];
            int32_t right = v[i];
            if (mix_res != 0) {
              // Matrixed stereo
              left = u[i] + v[i] - ((mix_res * v[i]) >> mix_bits);
              right = left - v[i];
            }
            out[i * num_channels] = static_cast<int16_t>(left);
            out[i * num_channels + 1] = static_cast<int16_t>(right);
          }
        } else {
          for (uint32_t i = 0; i < num_samples; i++)
            out[i * num_channels] = static_cast<int16_t>(mix[0][i]);
        }
        channel_index += element_channels;
        break;
      }

      case ELEMENT_TYPE_DSE: {
        reader.skip(4);  // element instance tag
        const bool byte_align = reader.read(1) != 0;
        uint32_t count = reader.read(8);
        if (count == 255)
          count += reader.read(8);
        if (byte_align)
          reader.byte_align();
        reader.skip(count * 8);
        break;
      }

      case ELEMENT_TYPE_FIL: {
        uint32_t count = reader.read(4);
        if (count == 15)
          count += reader.read(8) - 1;
        reader.skip(count * 8);
        break;
      }

      case ELEMENT_TYPE_END:
        if (channel_index == 0)
          return -1;
        return static_cast<int32_t>(num_samples);

      default:
        return -1;
    }
  }

  return static_cast<int32_t>(num_samples);
}

}  // namespace esphome::airplay

#endif  // USE_ESP32

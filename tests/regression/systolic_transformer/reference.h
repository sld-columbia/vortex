#ifndef _SYSTOLIC_TRANSFORMER_REFERENCE_H_
#define _SYSTOLIC_TRANSFORMER_REFERENCE_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "common.h"

namespace systolic_transformer {

/*
 * Host-side reference copied from systolic_programmer_view/pv_kernel.hpp.
 * Keep this logic structurally close to the programmer-view kernel so the
 * Vortex output is checked against the same algorithm the systolic team uses.
 */

inline uint32_t lcg_next(uint32_t* seed) {
  *seed = *seed * 1664525u + 1013904223u;
  return *seed;
}

inline TYPE next_random_value(uint32_t* seed) {
  uint32_t state = lcg_next(seed);
  return static_cast<TYPE>(state >> 8) / static_cast<TYPE>(1 << 24) - TYPE(0.5f);
}

inline void fill_random(std::vector<TYPE>& values, uint32_t* seed) {
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i] = next_random_value(seed);
  }
}

inline std::size_t input_a_elements(const kernel_arg_t& arg) {
  return static_cast<std::size_t>(arg.batch) * arg.M * arg.K;
}

inline std::size_t input_b_elements(const kernel_arg_t& arg) {
  return (arg.op == SYSTOLIC_OP_GEMM)
           ? static_cast<std::size_t>(arg.K) * arg.N
           : static_cast<std::size_t>(arg.batch) * arg.kv_len * arg.K;
}

inline std::size_t input_c_elements(const kernel_arg_t& arg) {
  if (arg.op == SYSTOLIC_OP_GEMM) {
    return systolic_has_flag(&arg, SYSTOLIC_FLAG_HAS_BIAS)
             ? static_cast<std::size_t>(arg.batch) * arg.M * arg.N
             : 0;
  }
  return static_cast<std::size_t>(arg.batch) * arg.kv_len * arg.K;
}

inline std::size_t output_elements(const kernel_arg_t& arg) {
  if (arg.op == SYSTOLIC_OP_GEMM) {
    return static_cast<std::size_t>(arg.batch) * arg.M * arg.N;
  }
  return static_cast<std::size_t>(arg.batch) * arg.M * arg.K;
}

inline TYPE gelu(TYPE x) {
  TYPE tanh_arg = TYPE(0.7978845608028654f) *
                  (x + TYPE(0.044715f) * x * x * x);
  return TYPE(0.5f) * x * (TYPE(1.0f) + std::tanh(tanh_arg));
}

inline std::vector<TYPE> run_reference(const std::vector<TYPE>& A,
                                       const std::vector<TYPE>& B,
                                       const std::vector<TYPE>& C,
                                       const kernel_arg_t& arg) {
  std::vector<TYPE> out(output_elements(arg), TYPE(0));

  if (arg.op == SYSTOLIC_OP_GEMM) {
    for (uint32_t batch = 0; batch < arg.batch; ++batch) {
      for (uint32_t row = 0; row < arg.M; ++row) {
        for (uint32_t col = 0; col < arg.N; ++col) {
          TYPE acc(0);
          for (uint32_t k = 0; k < arg.K; ++k) {
            acc += A[batch * arg.M * arg.K + row * arg.K + k]
                *  B[k * arg.N + col];
          }

          std::size_t out_index = batch * arg.M * arg.N + row * arg.N + col;
          if (systolic_has_flag(&arg, SYSTOLIC_FLAG_HAS_BIAS) && !C.empty()) {
            acc += C[out_index];
          }
          if (systolic_has_flag(&arg, SYSTOLIC_FLAG_DO_GELU)) {
            acc = gelu(acc);
          }
          out[out_index] = acc;
        }
      }
    }
    return out;
  }

  for (uint32_t batch = 0; batch < arg.batch; ++batch) {
    const TYPE* Q = A.data() + batch * arg.M * arg.K;
    const TYPE* KT = B.data() + batch * arg.kv_len * arg.K;
    const TYPE* V = C.data() + batch * arg.kv_len * arg.K;
    TYPE* O = out.data() + batch * arg.M * arg.K;

    std::vector<TYPE> scores(static_cast<std::size_t>(arg.M) * arg.kv_len, TYPE(0));
    std::vector<TYPE> weights(arg.kv_len, TYPE(0));

    for (uint32_t row = 0; row < arg.M; ++row) {
      for (uint32_t col = 0; col < arg.kv_len; ++col) {
        TYPE score(0);
        for (uint32_t k = 0; k < arg.K; ++k) {
          score += Q[row * arg.K + k] * KT[col * arg.K + k];
        }
        scores[row * arg.kv_len + col] = score;
      }
    }

    for (uint32_t row = 0; row < arg.M; ++row) {
      int mask_end = systolic_has_flag(&arg, SYSTOLIC_FLAG_CAUSAL_MASK)
                       ? static_cast<int>(std::min(row, arg.kv_len - 1))
                       : static_cast<int>(arg.kv_len - 1);

      if (mask_end < 0) {
        for (uint32_t j = 0; j < arg.K; ++j) {
          O[row * arg.K + j] = TYPE(0);
        }
        continue;
      }

      TYPE max_score = -INFINITY;
      for (int col = 0; col <= mask_end; ++col) {
        max_score = std::max(max_score, scores[row * arg.kv_len + col]);
      }

      TYPE sum(0);
      std::fill(weights.begin(), weights.end(), TYPE(0));
      for (uint32_t col = 0; col < arg.kv_len; ++col) {
        if (static_cast<int>(col) <= mask_end) {
          TYPE value = std::exp(scores[row * arg.kv_len + col] - max_score);
          weights[col] = value;
          sum += value;
        }
      }

      TYPE inv_sum = TYPE(1) / (sum + TYPE(1e-12f));
      for (uint32_t col = 0; col < arg.kv_len; ++col) {
        weights[col] *= inv_sum;
      }

      for (uint32_t j = 0; j < arg.K; ++j) {
        TYPE acc(0);
        for (int col = 0; col <= mask_end; ++col) {
          acc += weights[col] * V[col * arg.K + j];
        }
        O[row * arg.K + j] = acc;
      }
    }
  }

  return out;
}

} // namespace systolic_transformer

#endif

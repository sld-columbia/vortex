#include <math.h>

#include <vx_spawn.h>

#include "common.h"

/*
 * Device implementation of the programmer-view model from pv_kernel.hpp.
 * The math matches the host reference, but the outer loops are mapped onto
 * Vortex work-items so the regression runs through the normal runtime path.
 */

static inline TYPE dot_product(const TYPE* lhs, const TYPE* rhs, uint32_t count) {
  TYPE acc(0);
  for (uint32_t i = 0; i < count; ++i) {
    acc += lhs[i] * rhs[i];
  }
  return acc;
}

static inline TYPE clamp_to_tanh_domain(TYPE x) {
  return TYPE(0.5f) * (fabsf(x + TYPE(8.0f)) - fabsf(x - TYPE(8.0f)));
}

static inline TYPE tanh_via_exp(TYPE x) {
  TYPE clamped_x = clamp_to_tanh_domain(x);
  TYPE e = expf(TYPE(2.0f) * clamped_x);
  /*
   * Keep the ratio as numerator * reciprocal(denominator). On the affected
   * ESP/Vortex path this avoids the incorrect results we saw from direct
   * pairwise float division in the exp-based tanh reconstruction.
   */
  return (e - TYPE(1.0f)) * (TYPE(1.0f) / (e + TYPE(1.0f)));
}

static inline TYPE gelu_via_exp(TYPE x) {
  TYPE tanh_arg = TYPE(0.7978845608028654f) *
                  (x + TYPE(0.044715f) * x * x * x);
  TYPE tanh_val = tanh_via_exp(tanh_arg);
  return TYPE(0.5f) * x * (TYPE(1.0f) + tanh_val);
}

static void kernel_body(kernel_arg_t* __UNIFORM__ arg) {
  auto A = reinterpret_cast<const TYPE*>(arg->A_addr);
  auto B = reinterpret_cast<const TYPE*>(arg->B_addr);
  auto C = reinterpret_cast<const TYPE*>(arg->C_addr);
  auto O = reinterpret_cast<TYPE*>(arg->O_addr);

  if (arg->op == SYSTOLIC_OP_GEMM) {
    uint32_t col = blockIdx.x;
    /*
     * GEMM is launched as a 2D grid [N, batch*M]. Flattening batch and row
     * avoids the sparse correctness issues we saw with a 3D null-block launch
     * on the 1-core/4-warp/4-thread ESP configuration.
     */
    uint32_t row_batch = blockIdx.y;
    uint32_t batch = row_batch / arg->M;
    uint32_t row = row_batch - batch * arg->M;

    TYPE acc(0);
    const TYPE* a_row = A + batch * arg->M * arg->K + row * arg->K;
    for (uint32_t k = 0; k < arg->K; ++k) {
      acc += a_row[k] * B[k * arg->N + col];
    }

    uint64_t out_index = static_cast<uint64_t>(batch) * arg->M * arg->N
                       + static_cast<uint64_t>(row) * arg->N
                       + col;
    if (systolic_has_flag(arg, SYSTOLIC_FLAG_HAS_BIAS) && nullptr != C) {
      acc += C[out_index];
    }
    if (systolic_has_flag(arg, SYSTOLIC_FLAG_DO_GELU)) {
      acc = gelu_via_exp(acc);
    }
    O[out_index] = acc;
    return;
  }

  uint32_t row = blockIdx.x;
  uint32_t batch = blockIdx.y;

  const TYPE* Q = A + batch * arg->M * arg->K;
  const TYPE* KT = B + batch * arg->kv_len * arg->K;
  const TYPE* V = C + batch * arg->kv_len * arg->K;
  TYPE* out_row = O + batch * arg->M * arg->K + row * arg->K;

  int mask_end = systolic_has_flag(arg, SYSTOLIC_FLAG_CAUSAL_MASK)
                   ? ((row < arg->kv_len) ? static_cast<int>(row)
                                          : static_cast<int>(arg->kv_len - 1))
                   : static_cast<int>(arg->kv_len - 1);

  if (mask_end < 0) {
    for (uint32_t j = 0; j < arg->K; ++j) {
      out_row[j] = TYPE(0);
    }
    return;
  }

  const TYPE* q_row = Q + row * arg->K;
  /*
   * Attention keeps one Vortex task per output row. We recompute scores in
   * three passes (max, exp-sum, weighted sum) instead of storing a row-sized
   * temporary on device, which keeps the kernel simple and close to the
   * programmer-view algorithm.
   */
  TYPE max_score = -INFINITY;
  for (int col = 0; col <= mask_end; ++col) {
    TYPE score = dot_product(q_row, KT + col * arg->K, arg->K);
    if (score > max_score) {
      max_score = score;
    }
  }

  TYPE sum_exp(0);
  for (int col = 0; col <= mask_end; ++col) {
    TYPE score = dot_product(q_row, KT + col * arg->K, arg->K);
    sum_exp += expf(score - max_score);
  }

  TYPE inv_sum = TYPE(1) / (sum_exp + TYPE(1e-12f));

  for (uint32_t j = 0; j < arg->K; ++j) {
    out_row[j] = TYPE(0);
  }

  for (int col = 0; col <= mask_end; ++col) {
    TYPE score = dot_product(q_row, KT + col * arg->K, arg->K);
    TYPE weight = expf(score - max_score) * inv_sum;
    const TYPE* v_row = V + col * arg->K;
    for (uint32_t j = 0; j < arg->K; ++j) {
      out_row[j] += weight * v_row[j];
    }
  }
}

int main() {
  auto arg = (kernel_arg_t*)csr_read(VX_CSR_MSCRATCH);
  return vx_spawn_threads(arg->dimension, arg->grid_dim, nullptr,
                          (vx_kernel_func_cb)kernel_body, arg);
}

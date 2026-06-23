#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>

#ifndef TYPE
#define TYPE float
#endif

enum {
  SYSTOLIC_OP_GEMM = 0,
  SYSTOLIC_OP_ATTENTION = 1,
};

enum {
  /* GEMM-only modifiers copied from pv_kernel.hpp semantics. */
  SYSTOLIC_FLAG_HAS_BIAS    = 1u << 0,
  SYSTOLIC_FLAG_DO_GELU     = 1u << 1,
  /* ATTENTION mode: prefill-style causal mask when set, decode-style otherwise. */
  SYSTOLIC_FLAG_CAUSAL_MASK = 1u << 2,
};

typedef struct {
  /* Launch geometry used by vx_spawn_threads(). */
  uint32_t dimension;
  uint32_t grid_dim[3];
  /* Operation selector and shape copied from the programmer-view descriptor. */
  uint32_t op;
  uint32_t batch;
  uint32_t M;
  uint32_t K;
  uint32_t N;
  uint32_t kv_len;
  uint32_t flags;
  /* Device pointers for A/B/C/out buffers in Vortex global memory. */
  uint64_t A_addr;
  uint64_t B_addr;
  uint64_t C_addr;
  uint64_t O_addr;
} kernel_arg_t;

static inline int systolic_has_flag(const kernel_arg_t* arg, uint32_t flag) {
  return (arg->flags & flag) != 0;
}

static inline void systolic_prepare_launch(kernel_arg_t* arg) {
  if (arg->op == SYSTOLIC_OP_GEMM) {
    /*
     * GEMM uses one task per output element. Flatten [batch, row] into the
     * Y dimension so the kernel can recover both indices on device.
     */
    arg->dimension = 2;
    arg->grid_dim[0] = arg->N;
    arg->grid_dim[1] = arg->batch * arg->M;
    arg->grid_dim[2] = 1;
    return;
  }

  /*
   * Attention uses one task per output row [row, batch]. This keeps the
   * device launch geometry aligned with the programmer-view reference model.
   */
  arg->dimension = 2;
  arg->grid_dim[0] = arg->M;
  arg->grid_dim[1] = arg->batch;
  arg->grid_dim[2] = 1;
}

#endif

#ifndef _SYSTOLIC_TRANSFORMER_WORKLOADS_H_
#define _SYSTOLIC_TRANSFORMER_WORKLOADS_H_

#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include "common.h"

namespace systolic_transformer {

struct workload_model_spec_t {
  const char* name;
  uint32_t num_layers;
  uint32_t num_heads;
  uint32_t hidden_size;
  uint32_t head_dim;
  uint32_t intermediate_size;
};

struct workload_invocation_t {
  std::string name;
  kernel_arg_t arg;
};

inline const workload_model_spec_t& pythia_70m_spec() {
  static const workload_model_spec_t spec = {
    "pythia-70m",
    6,
    8,
    512,
    64,
    2048,
  };
  return spec;
}

inline const workload_model_spec_t& pythia_160m_spec() {
  static const workload_model_spec_t spec = {
    "pythia-160m",
    12,
    12,
    768,
    64,
    3072,
  };
  return spec;
}

inline const workload_model_spec_t* find_model_spec(const std::string& name) {
  if (name == pythia_70m_spec().name)
    return &pythia_70m_spec();
  if (name == pythia_160m_spec().name)
    return &pythia_160m_spec();
  return nullptr;
}

inline std::vector<workload_invocation_t> build_workload(
  const workload_model_spec_t& spec,
  uint32_t input_len,
  uint32_t output_len) {
  std::vector<workload_invocation_t> invocations;
  invocations.reserve(static_cast<std::size_t>(1 + output_len) *
                      spec.num_layers * 5);

  auto add_invocation =
    [&invocations](const std::string& name, const kernel_arg_t& arg) {
      invocations.push_back(workload_invocation_t{name, arg});
    };

  auto prefill_name = [](uint32_t layer, const char* suffix) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "prefill_l%02u_%s", layer, suffix);
    return std::string(buffer);
  };

  auto decode_name = [](uint32_t token, uint32_t layer, const char* suffix) {
    char buffer[64];
    std::snprintf(buffer,
                  sizeof(buffer),
                  "decode_t%03u_l%02u_%s",
                  token,
                  layer,
                  suffix);
    return std::string(buffer);
  };

  for (uint32_t layer = 0; layer < spec.num_layers; ++layer) {
    {
      kernel_arg_t arg = {};
      arg.op = SYSTOLIC_OP_GEMM;
      arg.batch = 1;
      arg.M = input_len;
      arg.K = spec.hidden_size;
      arg.N = 3 * spec.hidden_size;
      systolic_prepare_launch(&arg);
      add_invocation(prefill_name(layer, "qkv_proj"), arg);
    }

    {
      kernel_arg_t arg = {};
      arg.op = SYSTOLIC_OP_ATTENTION;
      arg.batch = spec.num_heads;
      arg.M = input_len;
      arg.K = spec.head_dim;
      arg.kv_len = input_len;
      arg.flags = SYSTOLIC_FLAG_CAUSAL_MASK;
      systolic_prepare_launch(&arg);
      add_invocation(prefill_name(layer, "flash_attn"), arg);
    }

    {
      kernel_arg_t arg = {};
      arg.op = SYSTOLIC_OP_GEMM;
      arg.batch = 1;
      arg.M = input_len;
      arg.K = spec.hidden_size;
      arg.N = spec.hidden_size;
      systolic_prepare_launch(&arg);
      add_invocation(prefill_name(layer, "o_proj"), arg);
    }

    {
      kernel_arg_t arg = {};
      arg.op = SYSTOLIC_OP_GEMM;
      arg.batch = 1;
      arg.M = input_len;
      arg.K = spec.hidden_size;
      arg.N = spec.intermediate_size;
      arg.flags = SYSTOLIC_FLAG_DO_GELU;
      systolic_prepare_launch(&arg);
      add_invocation(prefill_name(layer, "up_proj"), arg);
    }

    {
      kernel_arg_t arg = {};
      arg.op = SYSTOLIC_OP_GEMM;
      arg.batch = 1;
      arg.M = input_len;
      arg.K = spec.intermediate_size;
      arg.N = spec.hidden_size;
      systolic_prepare_launch(&arg);
      add_invocation(prefill_name(layer, "down_proj"), arg);
    }
  }

  for (uint32_t token = 0; token < output_len; ++token) {
    uint32_t context_len = input_len + token + 1;

    for (uint32_t layer = 0; layer < spec.num_layers; ++layer) {
      {
        kernel_arg_t arg = {};
        arg.op = SYSTOLIC_OP_GEMM;
        arg.batch = 1;
        arg.M = 1;
        arg.K = spec.hidden_size;
        arg.N = 3 * spec.hidden_size;
        systolic_prepare_launch(&arg);
        add_invocation(decode_name(token, layer, "qkv_proj"), arg);
      }

      {
        kernel_arg_t arg = {};
        arg.op = SYSTOLIC_OP_ATTENTION;
        arg.batch = spec.num_heads;
        arg.M = 1;
        arg.K = spec.head_dim;
        arg.kv_len = context_len;
        systolic_prepare_launch(&arg);
        add_invocation(decode_name(token, layer, "flash_attn"), arg);
      }

      {
        kernel_arg_t arg = {};
        arg.op = SYSTOLIC_OP_GEMM;
        arg.batch = 1;
        arg.M = 1;
        arg.K = spec.hidden_size;
        arg.N = spec.hidden_size;
        systolic_prepare_launch(&arg);
        add_invocation(decode_name(token, layer, "o_proj"), arg);
      }

      {
        kernel_arg_t arg = {};
        arg.op = SYSTOLIC_OP_GEMM;
        arg.batch = 1;
        arg.M = 1;
        arg.K = spec.hidden_size;
        arg.N = spec.intermediate_size;
        arg.flags = SYSTOLIC_FLAG_DO_GELU;
        systolic_prepare_launch(&arg);
        add_invocation(decode_name(token, layer, "up_proj"), arg);
      }

      {
        kernel_arg_t arg = {};
        arg.op = SYSTOLIC_OP_GEMM;
        arg.batch = 1;
        arg.M = 1;
        arg.K = spec.intermediate_size;
        arg.N = spec.hidden_size;
        systolic_prepare_launch(&arg);
        add_invocation(decode_name(token, layer, "down_proj"), arg);
      }
    }
  }

  return invocations;
}

} // namespace systolic_transformer

#endif

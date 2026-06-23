# systolic_transformer

## What this folder does

This regression ports the programmer-view floating-point transformer model used
by the systolic accelerator work onto Vortex so the same operations can be run
through the Vortex runtime and checked against a copied host reference.

The supported operations are:

* GEMM
* GEMM + GELU
* GEMM + bias + GELU
* Attention prefill with a causal mask
* Attention decode without a causal mask

The main files are:

* `reference.h`
  Host reference copied from the programmer-view model.
* `kernel.cpp`
  Vortex device kernel for GEMM/GELU and attention.
* `main.cpp`
  Linux-side launcher that allocates buffers, uploads tensors, runs the kernel,
  downloads outputs, and compares them against `reference.h`.
* `common.h`
  Shared ABI for operation selection, tensor shapes, flags, and buffer
  addresses.
* `workloads.h`
  Pythia-style prefill + decode workload builder used by workload mode.

## How the loops map onto Vortex

### GEMM

Programmer-view loop shape:

* output element `out[batch, row, col]`
* reduction over `k`
* optional bias
* optional GELU

Vortex mapping:

* one Vortex task computes one output element
* launch grid is `2D = [N, batch * M]`
* the kernel reconstructs:
  * `batch = row_batch / M`
  * `row = row_batch % M`
  * `col = blockIdx.x`

### Attention

Programmer-view loop shape:

1. compute one score row `Q * K^T`
2. apply a causal mask when requested
3. softmax the visible part of the row
4. compute the weighted sum over `V`

Vortex mapping:

* one Vortex task computes one output row
* launch grid is `2D = [M, batch]`
* each task performs three passes for that row:
  1. max score
  2. exponent sum
  3. weighted sum

## Run examples

Once staged into ESP Linux:

```bash
vortex-regression systolic_transformer -o gemm
vortex-regression systolic_transformer -o gemm_gelu -b 1 -m 64 -d 768 -n 3072
vortex-regression systolic_transformer -o gemm_bias_gelu -b 1 -m 64 -d 64 -n 64
vortex-regression systolic_transformer -o attn_prefill -b 1 -m 128 -d 64 -q 128
vortex-regression systolic_transformer -o attn_decode -b 1 -m 1 -d 64 -q 128
vortex-regression systolic_transformer -o gemm --no-verify --output-file single_run.txt
vortex-regression systolic_transformer --workload
vortex-regression systolic_transformer --model pythia-70m --input-len 64 --output-len 32
vortex-regression systolic_transformer --model pythia-70m --input-len 64 --output-len 32 --no-verify --output-file workload.txt
vortex-regression systolic_transformer --model pythia-160m --input-len 128 --output-len 16
vortex-regression systolic_transformer --model pythia-70m --input-len 64 --output-len 0 --dry-run
```

## Option mapping

### Single-kernel mode

* `-o`
  * `gemm`
  * `gemm_gelu`
  * `gemm_bias_gelu`
  * `attn_prefill`
  * `attn_decode`
* `-b`
  batch size
* `-m`
  GEMM rows / attention query length
* `-d`
  GEMM reduction dimension / attention head dimension
* `-n`
  GEMM output columns
* `-q`
  attention KV length
* `-s`
  deterministic random seed
* `-k`
  alternate `kernel.vxbin` path
* `--no-verify`
  skip CPU reference generation and output checking
* `--output-file`
  mirror the program's summary/table output to a file

### Workload mode

* `--workload`
  enable the workload runner using defaults
* `--model`
  `pythia-70m` or `pythia-160m`
* `--input-len`
  prefill token count
* `--output-len`
  decode token count
  `0` is allowed when you only want the prefill portion
* `--dry-run`
  list the generated kernel sequence without launching Vortex
* `--no-verify`
  skip CPU reference generation and output checking for faster runs
* `--output-file`
  mirror the program's summary/table output to a file
* `-s`
  deterministic random seed used across the whole workload
* `-k`
  alternate `kernel.vxbin` path

## Defaults

These are the defaults used by `main.cpp` before any CLI overrides:

* `gemm`
  `-b 2 -m 4 -d 8 -n 6`
* `gemm_gelu`
  `-b 1 -m 4 -d 8 -n 6`
* `gemm_bias_gelu`
  `-b 1 -m 4 -d 8 -n 6`
* `attn_prefill`
  `-b 2 -m 6 -d 8 -q 6`
* `attn_decode`
  `-b 2 -m 1 -d 8 -q 10`
* workload
  `--model pythia-70m --input-len 64 --output-len 32`

## Integration Notes

During ESP bring-up, two Vortex-side issues showed up in this regression:

* The runtime allocator needed to keep runtime-managed data buffers at or above
  `USER_BASE_ADDR`, even when the kernel image itself was linked at low
  addresses.
* The exp-based tanh reconstruction used by GELU,
  `(e - 1) / (e + 1)`, could produce incorrect results on the affected Vortex
  path when emitted as a direct pairwise divide.

The final fixes were:

* keep low-address kernel reservations from pulling normal allocations below
  the user allocation base
* evaluate the tanh ratio as `(e - 1) * (1 / (e + 1))` instead of a direct
  `num / den`

That keeps the math aligned with the original programmer-view algorithm while
avoiding the divide form that triggered the hardware/runtime issue in this
integration.

## Notes

* The attention implementation intentionally does not apply the usual
  `1 / sqrt(head_dim)` scaling because the source programmer-view model does
  not do that either.
* The host launcher checks Vortex output against the copied programmer-view
  reference unless `--no-verify` is passed.
* Workload mode reuses the same Vortex kernel for every generated invocation.
  The console table stays compact by showing per-kernel GPU timing, optional
  CPU-reference timing when verification is enabled, and error counts,
  followed by prefill/decode and GEMM/attention totals.
* `gemm_gelu` is useful for matching workload `up_proj` more closely than
  `gemm_bias_gelu`, because it applies GELU without adding a bias vector.
* When `--output-file` is used, the output file mirrors the launcher's normal
  console output and also captures the final runtime `PERF:` dump emitted at
  device shutdown.
* Long CPU reference, CPU verification, and device wait stages show a compact
  in-place progress ticker when running in an interactive terminal.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <streambuf>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <vortex.h>

#include "common.h"
#include "reference.h"
#include "workloads.h"

/*
 * Host launcher for the Vortex version of systolic_programmer_view/pv_kernel.hpp.
 * It supports both the original single-kernel regression modes and the newer
 * workload-style Pythia inference sweep copied from run_workload.cpp.
 */

#define RT_CHECK(_expr)                                         \
  do {                                                          \
    int _ret = (_expr);                                         \
    if (0 == _ret)                                              \
      break;                                                    \
    printf("Error: '%s' returned %d!\n", #_expr, (int)_ret);    \
    cleanup();                                                  \
    exit(-1);                                                   \
  } while (false)

enum workload_mode_t {
  MODE_GEMM = 0,
  MODE_GEMM_GELU,
  MODE_GEMM_BIAS_GELU,
  MODE_ATTN_PREFILL,
  MODE_ATTN_DECODE,
};

enum host_run_mode_t {
  HOST_RUN_SINGLE = 0,
  HOST_RUN_WORKLOAD,
};

struct host_options_t {
  host_options_t()
      : kernel_file("kernel.vxbin"),
        mode(MODE_GEMM),
        seed(50),
        batch(0),
        M(0),
        K(0),
        N(0),
        kv_len(0),
        batch_override(false),
        m_override(false),
        k_override(false),
        n_override(false),
        kv_override(false),
        run_mode(HOST_RUN_SINGLE),
        model_name("pythia-70m"),
        input_len(64),
        output_len(32),
        verify_output(true),
        dry_run(false),
        output_file(),
        single_mode_cli_seen(false),
        workload_cli_seen(false) {}

  const char* kernel_file;  /* Alternate kernel.vxbin path passed with -k. */
  workload_mode_t mode;     /* Which pv_kernel.hpp operation to run. */
  uint32_t seed;            /* Deterministic LCG seed for generated tensors. */
  uint32_t batch;           /* -b override: batch size for GEMM or ATTENTION. */
  uint32_t M;               /* -m override: GEMM rows / attention query length. */
  uint32_t K;               /* -d override: GEMM reduction dim / head dim. */
  uint32_t N;               /* -n override: GEMM output columns only. */
  uint32_t kv_len;          /* -q override: attention KV length only. */
  bool batch_override;
  bool m_override;
  bool k_override;
  bool n_override;
  bool kv_override;
  host_run_mode_t run_mode;
  std::string model_name;
  uint32_t input_len;
  uint32_t output_len;
  bool verify_output;
  bool dry_run;
  std::string output_file;
  bool single_mode_cli_seen;
  bool workload_cli_seen;
};

class TeeBuf : public std::streambuf {
public:
  TeeBuf(std::streambuf* primary, std::streambuf* secondary)
      : primary_(primary), secondary_(secondary) {}

protected:
  virtual int overflow(int ch) {
    if (traits_type::eq_int_type(ch, traits_type::eof()))
      return traits_type::not_eof(ch);

    const int primary_result = primary_->sputc(static_cast<char>(ch));
    const int secondary_result = secondary_->sputc(static_cast<char>(ch));
    if (traits_type::eq_int_type(primary_result, traits_type::eof()) ||
        traits_type::eq_int_type(secondary_result, traits_type::eof())) {
      return traits_type::eof();
    }

    return ch;
  }

  virtual std::streamsize xsputn(const char* s, std::streamsize count) {
    std::streamsize primary_result = primary_->sputn(s, count);
    std::streamsize secondary_result = secondary_->sputn(s, count);
    return std::min(primary_result, secondary_result);
  }

  virtual int sync() {
    int primary_result = primary_->pubsync();
    int secondary_result = secondary_->pubsync();
    return (0 == primary_result && 0 == secondary_result) ? 0 : -1;
  }

private:
  std::streambuf* primary_;
  std::streambuf* secondary_;
};

class ProgressTicker {
public:
  explicit ProgressTicker(const std::string& label,
                          uint64_t interval_seconds = 5)
      : label_(label),
        interval_seconds_(interval_seconds),
        enabled_((0 != interval_seconds) && (0 != ::isatty(::fileno(stdout)))),
        stop_(false),
        last_render_width_(0),
        worker_() {
    if (enabled_) {
      worker_ = std::thread(&ProgressTicker::run, this);
    }
  }

  ~ProgressTicker() {
    stop();
  }

  void stop() {
    bool already_stopped = stop_.exchange(true);
    if (worker_.joinable()) {
      worker_.join();
    }
    if (enabled_ && !already_stopped) {
      clear_line();
    }
  }

private:
  static std::mutex& output_mutex() {
    static std::mutex mutex;
    return mutex;
  }

  void render_line(const std::string& line) {
    std::lock_guard<std::mutex> lock(output_mutex());
    std::fputc('\r', stdout);
    std::fwrite(line.data(), 1, line.size(), stdout);
    if (last_render_width_ > line.size()) {
      std::string padding(last_render_width_ - line.size(), ' ');
      std::fwrite(padding.data(), 1, padding.size(), stdout);
    }
    std::fflush(stdout);
    last_render_width_ = line.size();
  }

  void clear_line() {
    if (0 == last_render_width_) {
      return;
    }

    std::lock_guard<std::mutex> lock(output_mutex());
    std::fputc('\r', stdout);
    std::string padding(last_render_width_, ' ');
    std::fwrite(padding.data(), 1, padding.size(), stdout);
    std::fputc('\r', stdout);
    std::fflush(stdout);
    last_render_width_ = 0;
  }

  void run() {
    if (!enabled_) {
      return;
    }

    static const char spinner_chars[] = {'|', '/', '-', '\\'};
    const auto started = std::chrono::steady_clock::now();
    std::size_t spinner_index = 0;

    while (!stop_.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      if (stop_.load()) {
        break;
      }

      uint64_t elapsed_seconds =
        static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - started)
            .count());
      if (elapsed_seconds < interval_seconds_) {
        continue;
      }

      std::ostringstream line;
      line << "[progress] " << spinner_chars[spinner_index]
           << " " << label_
           << " (" << elapsed_seconds << "s elapsed)";
      render_line(line.str());
      spinner_index = (spinner_index + 1) % (sizeof(spinner_chars) / sizeof(spinner_chars[0]));
    }
  }

  std::string label_;
  uint64_t interval_seconds_;
  bool enabled_;
  std::atomic<bool> stop_;
  std::size_t last_render_width_;
  std::thread worker_;
};

struct compare_stats_t {
  int errors;
  double max_abs_error;
  double max_rel_error;
};

struct invocation_result_t {
  double elapsed_ms;
  double cpu_reference_ms;
  compare_stats_t stats;
};

static host_options_t options;

static vx_device_h device = nullptr;
static vx_buffer_h A_buffer = nullptr;
static vx_buffer_h B_buffer = nullptr;
static vx_buffer_h C_buffer = nullptr;
static vx_buffer_h O_buffer = nullptr;
static vx_buffer_h krnl_buffer = nullptr;
static vx_buffer_h args_buffer = nullptr;
static std::ofstream output_file_stream;
static std::streambuf* saved_cout_buf = nullptr;
static std::streambuf* saved_cerr_buf = nullptr;
static TeeBuf* cout_tee_buf = nullptr;
static TeeBuf* cerr_tee_buf = nullptr;
static FILE* output_file_c = nullptr;

static void cleanup();
static void restore_output_streams();

static const char* mode_name(workload_mode_t mode) {
  switch (mode) {
  case MODE_GEMM:
    return "gemm";
  case MODE_GEMM_GELU:
    return "gemm_gelu";
  case MODE_GEMM_BIAS_GELU:
    return "gemm_bias_gelu";
  case MODE_ATTN_PREFILL:
    return "attn_prefill";
  case MODE_ATTN_DECODE:
    return "attn_decode";
  default:
    return "unknown";
  }
}

static std::string describe_arg(const kernel_arg_t& arg) {
  std::ostringstream stream;
  if (arg.op == SYSTOLIC_OP_GEMM) {
    stream << arg.batch << "x" << arg.M << "x" << arg.K << "->" << arg.N;
    if (systolic_has_flag(&arg, SYSTOLIC_FLAG_HAS_BIAS))
      stream << "+BIAS";
    if (systolic_has_flag(&arg, SYSTOLIC_FLAG_DO_GELU))
      stream << "+GELU";
  } else {
    stream << arg.batch << "x" << arg.M << "x" << arg.K
           << " kv=" << arg.kv_len
           << (systolic_has_flag(&arg, SYSTOLIC_FLAG_CAUSAL_MASK)
                 ? " causal"
                 : " full");
  }
  return stream.str();
}

static std::string fit_table_text(const std::string& value, std::size_t width) {
  if (value.size() <= width) {
    return value;
  }
  if (width <= 3) {
    return value.substr(0, width);
  }
  return value.substr(0, width - 3) + "...";
}

static void show_usage() {
  std::cout << "Systolic transformer regression for Vortex.\n";
  std::cout << "\nSingle-kernel mode:\n";
  std::cout << "  [-o op] [-b batch] [-m M] [-d K] [-n N] [-q kv_len] [-s seed] [-k kernel] [--no-verify] [--output-file path]\n";
  std::cout << "    -o op       gemm | gemm_gelu | gemm_bias_gelu | attn_prefill | attn_decode\n";
  std::cout << "    -b batch    batch size for all modes\n";
  std::cout << "    -m M        GEMM rows / attention query length\n";
  std::cout << "    -d K        GEMM reduction dim / attention head dim\n";
  std::cout << "    -n N        GEMM output columns only\n";
  std::cout << "    -q kv_len   attention KV length only\n";
  std::cout << "\nProgrammer-view workload mode:\n";
  std::cout << "  [--workload] [--model name] [--input-len N] [--output-len N] [--dry-run] [--no-verify] [--output-file path] [-s seed] [-k kernel]\n";
  std::cout << "    --model       pythia-70m | pythia-160m\n";
  std::cout << "    --input-len   prefill token count\n";
  std::cout << "    --output-len  autoregressive decode token count\n";
  std::cout << "    --dry-run     list kernels without launching Vortex\n";
  std::cout << "    --no-verify   skip CPU reference generation and output checking\n";
  std::cout << "    --output-file write the program summary/table output to a file too\n";
  std::cout << "\nShared options:\n";
  std::cout << "    -s seed     deterministic tensor seed\n";
  std::cout << "    -k kernel   path to kernel.vxbin\n";
  std::cout << "    -h          show this help\n";
  std::cout << "\nDefaults:\n";
  std::cout << "  gemm            -> -b 2 -m 4   -d 8  -n 6\n";
  std::cout << "  gemm_gelu       -> -b 1 -m 4   -d 8  -n 6\n";
  std::cout << "  gemm_bias_gelu  -> -b 1 -m 4   -d 8  -n 6\n";
  std::cout << "  attn_prefill    -> -b 2 -m 6   -d 8  -q 6\n";
  std::cout << "  attn_decode     -> -b 2 -m 1   -d 8  -q 10\n";
  std::cout << "  workload        -> --model pythia-70m --input-len 64 --output-len 32\n";
}

static workload_mode_t parse_mode(const char* name) {
  if (0 == std::strcmp(name, "gemm"))
    return MODE_GEMM;
  if (0 == std::strcmp(name, "gemm_gelu"))
    return MODE_GEMM_GELU;
  if (0 == std::strcmp(name, "gemm_bias_gelu"))
    return MODE_GEMM_BIAS_GELU;
  if (0 == std::strcmp(name, "attn_prefill"))
    return MODE_ATTN_PREFILL;
  if (0 == std::strcmp(name, "attn_decode"))
    return MODE_ATTN_DECODE;

  std::cerr << "Unknown op '" << name << "'" << std::endl;
  show_usage();
  exit(-1);
}

static uint32_t parse_u32_arg(const char* value, const char* name) {
  char* end = nullptr;
  unsigned long parsed = std::strtoul(value, &end, 0);
  if (end == value || *end != '\0') {
    std::cerr << "Invalid value for " << name << ": " << value << std::endl;
    exit(-1);
  }
  return static_cast<uint32_t>(parsed);
}

static void restore_output_streams() {
  if (saved_cout_buf) {
    std::cout.flush();
    std::cout.rdbuf(saved_cout_buf);
    saved_cout_buf = nullptr;
  }
  if (saved_cerr_buf) {
    std::cerr.flush();
    std::cerr.rdbuf(saved_cerr_buf);
    saved_cerr_buf = nullptr;
  }
  if (output_file_stream.is_open()) {
    output_file_stream.flush();
    output_file_stream.close();
  }
  if (output_file_c) {
    std::fflush(output_file_c);
    std::fclose(output_file_c);
    output_file_c = nullptr;
  }
}

static void setup_output_streams() {
  if (options.output_file.empty()) {
    return;
  }

  output_file_stream.open(options.output_file.c_str(),
                          std::ios::out | std::ios::trunc);
  if (!output_file_stream) {
    std::cerr << "Could not open output file: " << options.output_file
              << std::endl;
    exit(-1);
  }

  if (!saved_cout_buf) {
    saved_cout_buf = std::cout.rdbuf();
    cout_tee_buf = new TeeBuf(saved_cout_buf, output_file_stream.rdbuf());
    std::cout.rdbuf(cout_tee_buf);
  }

  if (!saved_cerr_buf) {
    saved_cerr_buf = std::cerr.rdbuf();
    cerr_tee_buf = new TeeBuf(saved_cerr_buf, output_file_stream.rdbuf());
    std::cerr.rdbuf(cerr_tee_buf);
  }

  output_file_c = std::fopen(options.output_file.c_str(), "a");
  if (nullptr == output_file_c) {
    std::cerr << "Could not open output file for append: "
              << options.output_file << std::endl;
    exit(-1);
  }

  std::atexit(restore_output_streams);
}

static void parse_args(int argc, char** argv) {
  enum {
    OPT_MODEL = 1000,
    OPT_INPUT_LEN,
    OPT_OUTPUT_LEN,
    OPT_DRY_RUN,
    OPT_WORKLOAD,
    OPT_NO_VERIFY,
    OPT_OUTPUT_FILE,
  };

  static const struct option long_options[] = {
    {"op", required_argument, nullptr, 'o'},
    {"batch", required_argument, nullptr, 'b'},
    {"rows", required_argument, nullptr, 'm'},
    {"head-dim", required_argument, nullptr, 'd'},
    {"cols", required_argument, nullptr, 'n'},
    {"kv-len", required_argument, nullptr, 'q'},
    {"seed", required_argument, nullptr, 's'},
    {"kernel", required_argument, nullptr, 'k'},
    {"model", required_argument, nullptr, OPT_MODEL},
    {"input-len", required_argument, nullptr, OPT_INPUT_LEN},
    {"output-len", required_argument, nullptr, OPT_OUTPUT_LEN},
    {"dry-run", no_argument, nullptr, OPT_DRY_RUN},
    {"workload", no_argument, nullptr, OPT_WORKLOAD},
    {"no-verify", no_argument, nullptr, OPT_NO_VERIFY},
    {"output-file", required_argument, nullptr, OPT_OUTPUT_FILE},
    {"help", no_argument, nullptr, 'h'},
    {nullptr, 0, nullptr, 0},
  };

  int option_index = 0;
  int c = 0;
  while ((c = getopt_long(argc,
                          argv,
                          "o:b:m:d:n:q:s:k:h?",
                          long_options,
                          &option_index)) != -1) {
    switch (c) {
    case 'o':
      options.mode = parse_mode(optarg);
      options.single_mode_cli_seen = true;
      break;
    case 'b':
      options.batch = parse_u32_arg(optarg, "batch");
      options.batch_override = true;
      options.single_mode_cli_seen = true;
      break;
    case 'm':
      options.M = parse_u32_arg(optarg, "M");
      options.m_override = true;
      options.single_mode_cli_seen = true;
      break;
    case 'd':
      options.K = parse_u32_arg(optarg, "K");
      options.k_override = true;
      options.single_mode_cli_seen = true;
      break;
    case 'n':
      options.N = parse_u32_arg(optarg, "N");
      options.n_override = true;
      options.single_mode_cli_seen = true;
      break;
    case 'q':
      options.kv_len = parse_u32_arg(optarg, "kv_len");
      options.kv_override = true;
      options.single_mode_cli_seen = true;
      break;
    case 's':
      options.seed = parse_u32_arg(optarg, "seed");
      break;
    case 'k':
      options.kernel_file = optarg;
      break;
    case OPT_MODEL:
      options.model_name = optarg;
      options.workload_cli_seen = true;
      break;
    case OPT_INPUT_LEN:
      options.input_len = parse_u32_arg(optarg, "input_len");
      options.workload_cli_seen = true;
      break;
    case OPT_OUTPUT_LEN:
      options.output_len = parse_u32_arg(optarg, "output_len");
      options.workload_cli_seen = true;
      break;
    case OPT_DRY_RUN:
      options.dry_run = true;
      options.workload_cli_seen = true;
      break;
    case OPT_WORKLOAD:
      options.workload_cli_seen = true;
      break;
    case OPT_NO_VERIFY:
      options.verify_output = false;
      break;
    case OPT_OUTPUT_FILE:
      options.output_file = optarg;
      break;
    case 'h':
    case '?':
      show_usage();
      exit(0);
    default:
      show_usage();
      exit(-1);
    }
  }

  if (optind != argc) {
    std::cerr << "Unexpected positional argument: " << argv[optind] << std::endl;
    show_usage();
    exit(-1);
  }

  if (options.workload_cli_seen) {
    if (options.single_mode_cli_seen) {
      std::cerr << "Single-kernel options (-o/-b/-m/-d/-n/-q) cannot be combined with workload mode."
                << std::endl;
      exit(-1);
    }
    options.run_mode = HOST_RUN_WORKLOAD;
  }
}

static kernel_arg_t build_single_mode_arg() {
  kernel_arg_t arg = {};

  switch (options.mode) {
  case MODE_GEMM:
    arg.op = SYSTOLIC_OP_GEMM;
    arg.batch = 2;
    arg.M = 4;
    arg.K = 8;
    arg.N = 6;
    break;
  case MODE_GEMM_GELU:
    arg.op = SYSTOLIC_OP_GEMM;
    arg.batch = 1;
    arg.M = 4;
    arg.K = 8;
    arg.N = 6;
    arg.flags = SYSTOLIC_FLAG_DO_GELU;
    break;
  case MODE_GEMM_BIAS_GELU:
    arg.op = SYSTOLIC_OP_GEMM;
    arg.batch = 1;
    arg.M = 4;
    arg.K = 8;
    arg.N = 6;
    arg.flags = SYSTOLIC_FLAG_HAS_BIAS | SYSTOLIC_FLAG_DO_GELU;
    break;
  case MODE_ATTN_PREFILL:
    arg.op = SYSTOLIC_OP_ATTENTION;
    arg.batch = 2;
    arg.M = 6;
    arg.K = 8;
    arg.kv_len = 6;
    arg.flags = SYSTOLIC_FLAG_CAUSAL_MASK;
    break;
  case MODE_ATTN_DECODE:
    arg.op = SYSTOLIC_OP_ATTENTION;
    arg.batch = 2;
    arg.M = 1;
    arg.K = 8;
    arg.kv_len = 10;
    break;
  }

  if (options.batch_override)
    arg.batch = options.batch;
  if (options.m_override)
    arg.M = options.M;
  if (options.k_override)
    arg.K = options.K;
  if (options.n_override)
    arg.N = options.N;
  if (options.kv_override)
    arg.kv_len = options.kv_len;

  systolic_prepare_launch(&arg);
  return arg;
}

static std::string format_output_coord(std::size_t index,
                                       const kernel_arg_t& arg) {
  std::ostringstream stream;

  if (arg.op == SYSTOLIC_OP_GEMM) {
    std::size_t batch_stride =
      static_cast<std::size_t>(arg.M) * static_cast<std::size_t>(arg.N);
    std::size_t batch = index / batch_stride;
    std::size_t batch_index = index - batch * batch_stride;
    std::size_t row = batch_index / arg.N;
    std::size_t col = batch_index - row * arg.N;
    stream << "batch=" << batch
           << ", row=" << row
           << ", col=" << col;
    return stream.str();
  }

  std::size_t batch_stride =
    static_cast<std::size_t>(arg.M) * static_cast<std::size_t>(arg.K);
  std::size_t batch = index / batch_stride;
  std::size_t batch_index = index - batch * batch_stride;
  std::size_t row = batch_index / arg.K;
  std::size_t channel = batch_index - row * arg.K;
  stream << "batch=" << batch
         << ", row=" << row
         << ", channel=" << channel;
  return stream.str();
}

static void validate_kernel_arg(const kernel_arg_t& arg, const char* context) {
  if (0 == arg.batch || 0 == arg.M) {
    std::cerr << context << ": batch and M must both be greater than zero"
              << std::endl;
    exit(-1);
  }

  if (arg.op == SYSTOLIC_OP_GEMM) {
    if (0 == arg.K || 0 == arg.N) {
      std::cerr << context << ": K and N must both be greater than zero for GEMM modes"
                << std::endl;
      exit(-1);
    }
    return;
  }

  if (arg.op != SYSTOLIC_OP_ATTENTION) {
    std::cerr << context << ": unsupported operation selector " << arg.op
              << std::endl;
    exit(-1);
  }

  if (0 == arg.kv_len) {
    std::cerr << context << ": kv_len must be greater than zero for attention modes"
              << std::endl;
    exit(-1);
  }
}

static void print_device_info(uint64_t num_cores,
                              uint64_t num_warps,
                              uint64_t num_threads) {
  std::cout << "open device connection" << std::endl;
  std::cout << "device: cores=" << num_cores
            << ", warps=" << num_warps
            << ", threads=" << num_threads << std::endl;
}

static void print_single_mode_config(const kernel_arg_t& arg) {
  std::cout << "operation: " << mode_name(options.mode) << std::endl;
  std::cout << "batch=" << arg.batch
            << ", M=" << arg.M;
  if (arg.op == SYSTOLIC_OP_GEMM) {
    std::cout << ", K=" << arg.K;
    std::cout << ", N=" << arg.N;
    std::cout << ", bias="
              << (systolic_has_flag(&arg, SYSTOLIC_FLAG_HAS_BIAS) ? "yes"
                                                                  : "no");
    std::cout << ", gelu="
              << (systolic_has_flag(&arg, SYSTOLIC_FLAG_DO_GELU) ? "yes"
                                                                 : "no");
  } else {
    std::cout << ", kv_len=" << arg.kv_len;
    std::cout << ", causal_mask="
              << (systolic_has_flag(&arg, SYSTOLIC_FLAG_CAUSAL_MASK) ? "yes"
                                                                     : "no");
  }
  std::cout << ", seed=" << options.seed << std::endl;
}

static compare_stats_t compare_outputs(const std::vector<TYPE>& actual,
                                       const std::vector<TYPE>& expected,
                                       const kernel_arg_t& arg) {
  const bool relaxed_tol =
    (arg.op == SYSTOLIC_OP_ATTENTION) ||
    systolic_has_flag(&arg, SYSTOLIC_FLAG_DO_GELU);

  const double abs_tol = relaxed_tol ? 2.5e-4 : 1.0e-4;
  const double rel_tol = relaxed_tol ? 1.0e-3 : 5.0e-4;

  compare_stats_t stats = {0, 0.0, 0.0};

  for (std::size_t i = 0; i < expected.size(); ++i) {
    double ref = expected[i];
    double got = actual[i];
    double abs_err = std::fabs(got - ref);
    double rel_err = abs_err / (std::fabs(ref) + 1.0e-12);

    if (abs_err > stats.max_abs_error)
      stats.max_abs_error = abs_err;
    if (rel_err > stats.max_rel_error)
      stats.max_rel_error = rel_err;

    double tol = abs_tol + rel_tol * std::fabs(ref);
    if (abs_err > tol) {
      if (stats.errors < 20) {
        std::cout << "*** error[" << i << "]"
                  << " (" << format_output_coord(i, arg) << ")";
        std::cout << ": expected=" << ref
                  << ", actual=" << got
                  << ", abs_err=" << abs_err
                  << ", tol=" << tol << std::endl;
      }
      ++stats.errors;
    }
  }

  return stats;
}

static void mirror_runtime_perf_to_file() {
  if (nullptr == output_file_c || nullptr == device) {
    return;
  }

  if (output_file_stream.is_open()) {
    output_file_stream.flush();
  }
  std::fflush(output_file_c);
  vx_dump_perf(device, output_file_c);
  std::fflush(output_file_c);
}

static void release_run_buffers() {
  if (A_buffer) {
    vx_mem_free(A_buffer);
    A_buffer = nullptr;
  }
  if (B_buffer) {
    vx_mem_free(B_buffer);
    B_buffer = nullptr;
  }
  if (C_buffer) {
    vx_mem_free(C_buffer);
    C_buffer = nullptr;
  }
  if (O_buffer) {
    vx_mem_free(O_buffer);
    O_buffer = nullptr;
  }
  if (args_buffer) {
    vx_mem_free(args_buffer);
    args_buffer = nullptr;
  }
}

static void cleanup() {
  release_run_buffers();
  if (krnl_buffer) {
    vx_mem_free(krnl_buffer);
    krnl_buffer = nullptr;
  }
  if (device) {
    vx_dev_close(device);
    device = nullptr;
  }
}

static void init_device(uint64_t* num_cores,
                        uint64_t* num_warps,
                        uint64_t* num_threads) {
  RT_CHECK(vx_dev_open(&device));
  RT_CHECK(vx_dev_caps(device, VX_CAPS_NUM_CORES, num_cores));
  RT_CHECK(vx_dev_caps(device, VX_CAPS_NUM_WARPS, num_warps));
  RT_CHECK(vx_dev_caps(device, VX_CAPS_NUM_THREADS, num_threads));
}

static void upload_program() {
  std::cout << "upload program" << std::endl;
  RT_CHECK(vx_upload_kernel_file(device, options.kernel_file, &krnl_buffer));
}

static invocation_result_t run_kernel_invocation(const kernel_arg_t& arg,
                                                 uint32_t* seed,
                                                 const std::string& invocation_name,
                                                 bool verbose) {
  release_run_buffers();

  kernel_arg_t device_arg = arg;
  std::size_t a_elems = systolic_transformer::input_a_elements(device_arg);
  std::size_t b_elems = systolic_transformer::input_b_elements(device_arg);
  std::size_t c_elems = systolic_transformer::input_c_elements(device_arg);
  std::size_t o_elems = systolic_transformer::output_elements(device_arg);

  std::vector<TYPE> h_A(a_elems);
  std::vector<TYPE> h_B(b_elems);
  std::vector<TYPE> h_C(c_elems);
  std::vector<TYPE> h_O(o_elems, TYPE(0));

  systolic_transformer::fill_random(h_A, seed);
  if (!h_B.empty()) {
    systolic_transformer::fill_random(h_B, seed);
  }
  if (!h_C.empty()) {
    systolic_transformer::fill_random(h_C, seed);
  }

  std::vector<TYPE> h_ref;
  double cpu_reference_ms = 0.0;
  if (options.verify_output && verbose) {
    std::cout << "compute host reference" << std::endl;
  }
  if (options.verify_output) {
    auto cpu_time_start = std::chrono::high_resolution_clock::now();
    {
      ProgressTicker progress("cpu reference: " + invocation_name);
      h_ref = systolic_transformer::run_reference(h_A, h_B, h_C, device_arg);
    }
    auto cpu_time_end = std::chrono::high_resolution_clock::now();
    cpu_reference_ms =
      std::chrono::duration_cast<std::chrono::microseconds>(
        cpu_time_end - cpu_time_start)
        .count() /
      1000.0;
  }

  if (verbose) {
    std::cout << "allocate device memory" << std::endl;
  }
  RT_CHECK(vx_mem_alloc(device, a_elems * sizeof(TYPE), VX_MEM_READ, &A_buffer));
  RT_CHECK(vx_mem_address(A_buffer, &device_arg.A_addr));
  if (!h_B.empty()) {
    RT_CHECK(vx_mem_alloc(device, b_elems * sizeof(TYPE), VX_MEM_READ, &B_buffer));
    RT_CHECK(vx_mem_address(B_buffer, &device_arg.B_addr));
  }
  if (!h_C.empty()) {
    RT_CHECK(vx_mem_alloc(device, c_elems * sizeof(TYPE), VX_MEM_READ, &C_buffer));
    RT_CHECK(vx_mem_address(C_buffer, &device_arg.C_addr));
  }
  RT_CHECK(vx_mem_alloc(device, o_elems * sizeof(TYPE), VX_MEM_WRITE, &O_buffer));
  RT_CHECK(vx_mem_address(O_buffer, &device_arg.O_addr));

  if (verbose) {
    std::cout << "A_addr=0x" << std::hex << device_arg.A_addr << std::endl;
    std::cout << "B_addr=0x" << std::hex << device_arg.B_addr << std::endl;
    std::cout << "C_addr=0x" << std::hex << device_arg.C_addr << std::endl;
    std::cout << "O_addr=0x" << std::hex << device_arg.O_addr << std::dec
              << std::endl;
    std::cout << "upload input buffers" << std::endl;
  }

  RT_CHECK(vx_copy_to_dev(A_buffer, h_A.data(), 0, a_elems * sizeof(TYPE)));
  if (!h_B.empty()) {
    RT_CHECK(vx_copy_to_dev(B_buffer, h_B.data(), 0, b_elems * sizeof(TYPE)));
  }
  if (!h_C.empty()) {
    RT_CHECK(vx_copy_to_dev(C_buffer, h_C.data(), 0, c_elems * sizeof(TYPE)));
  }

  if (verbose) {
    std::cout << "upload kernel argument" << std::endl;
  }
  RT_CHECK(vx_upload_bytes(device,
                           &device_arg,
                           sizeof(kernel_arg_t),
                           &args_buffer));

  auto time_start = std::chrono::high_resolution_clock::now();

  if (verbose) {
    std::cout << "start device" << std::endl;
  }
  RT_CHECK(vx_start(device, krnl_buffer, args_buffer));

  if (verbose) {
    std::cout << "wait for completion" << std::endl;
  }
  {
    ProgressTicker progress("device wait: " + invocation_name);
    RT_CHECK(vx_ready_wait(device, VX_MAX_TIMEOUT));
  }

  auto time_end = std::chrono::high_resolution_clock::now();
  double elapsed_ms =
    std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start)
      .count() /
    1000.0;

  if (verbose) {
    std::cout << "elapsed time: " << elapsed_ms << " ms" << std::endl;
    std::cout << "download output buffer" << std::endl;
  }
  RT_CHECK(vx_copy_from_dev(h_O.data(), O_buffer, 0, o_elems * sizeof(TYPE)));

  if (options.verify_output && verbose) {
    std::cout << "verify result" << std::endl;
  }
  compare_stats_t stats = {0, 0.0, 0.0};
  if (options.verify_output) {
    ProgressTicker progress("cpu verify: " + invocation_name);
    stats = compare_outputs(h_O, h_ref, device_arg);
  }

  release_run_buffers();

  invocation_result_t result = {elapsed_ms, cpu_reference_ms, stats};
  return result;
}

static int run_single_mode() {
  kernel_arg_t arg = build_single_mode_arg();
  validate_kernel_arg(arg, "single-kernel mode");

  uint64_t num_cores = 0;
  uint64_t num_warps = 0;
  uint64_t num_threads = 0;
  init_device(&num_cores, &num_warps, &num_threads);
  print_device_info(num_cores, num_warps, num_threads);
  upload_program();
  print_single_mode_config(arg);
  if (!options.verify_output) {
    std::cout << "verification: disabled" << std::endl;
  }

  uint32_t seed = options.seed;
  invocation_result_t result =
    run_kernel_invocation(arg, &seed, mode_name(options.mode), true);

  mirror_runtime_perf_to_file();
  cleanup();

  if (options.verify_output && result.stats.errors != 0) {
    std::cout << "Found " << result.stats.errors << " mismatches" << std::endl;
    std::cout << "max_abs_error=" << result.stats.max_abs_error
              << ", max_rel_error=" << result.stats.max_rel_error << std::endl;
    std::cout << "FAILED!" << std::endl;
    return result.stats.errors;
  }

  if (options.verify_output) {
    std::cout << "max_abs_error=" << result.stats.max_abs_error
              << ", max_rel_error=" << result.stats.max_rel_error << std::endl;
  } else {
    std::cout << "verification skipped" << std::endl;
  }
  std::cout << "PASSED!" << std::endl;
  return 0;
}

static int run_workload_mode() {
  const systolic_transformer::workload_model_spec_t* spec =
    systolic_transformer::find_model_spec(options.model_name);
  if (nullptr == spec) {
    std::cerr << "Unknown model '" << options.model_name
              << "'. Choose pythia-70m or pythia-160m." << std::endl;
    return -1;
  }

  if (0 == options.input_len) {
    std::cerr << "input_len must be greater than zero" << std::endl;
    return -1;
  }

  std::vector<systolic_transformer::workload_invocation_t> invocations =
    systolic_transformer::build_workload(*spec,
                                         options.input_len,
                                         options.output_len);

  std::cout << "model=" << spec->name
            << "  layers=" << spec->num_layers
            << "  heads=" << spec->num_heads
            << "  hidden=" << spec->hidden_size
            << "  intermediate=" << spec->intermediate_size << "\n"
            << "input_len=" << options.input_len
            << "  output_len=" << options.output_len
            << "  kernels=" << invocations.size()
            << (options.dry_run ? "  [dry-run]\n" : "\n")
            << std::endl;

  if (!options.dry_run) {
    uint64_t num_cores = 0;
    uint64_t num_warps = 0;
    uint64_t num_threads = 0;
    init_device(&num_cores, &num_warps, &num_threads);
    print_device_info(num_cores, num_warps, num_threads);
    upload_program();
    std::cout << "workload seed=" << options.seed << std::endl;
    if (!options.verify_output) {
      std::cout << "verification: disabled" << std::endl;
    }
  }

  const int width_index = 6;
  const int width_name = 26;
  const int width_shape = 24;
  const int width_cpu_ms = 12;
  const int width_gpu_ms = 12;
  const int width_err = 6;
  const bool show_cpu_reference_ms =
    !options.dry_run && options.verify_output;

  if (!options.dry_run) {
    std::cout << "GPU tests";
    if (show_cpu_reference_ms) {
      std::cout << " (device timing; CPU reference shown separately)";
    }
    std::cout << std::endl;
  }

  int table_width = width_index + 3 + width_name + 3 + width_shape;
  if (!options.dry_run) {
    if (show_cpu_reference_ms) {
      table_width += 3 + width_cpu_ms;
    }
    table_width += 3 + width_gpu_ms + 3 + width_err;
  }

  std::cout << std::left
            << std::setw(width_index) << "#"
            << " | " << std::setw(width_name) << "kernel"
            << " | " << std::setw(width_shape) << "shape";
  if (!options.dry_run) {
    if (show_cpu_reference_ms) {
      std::cout << " | " << std::setw(width_cpu_ms) << "cpu ref ms";
      std::cout << " | " << std::setw(width_gpu_ms) << "gpu ms";
    } else {
      std::cout << " | " << std::setw(width_gpu_ms) << "ms";
    }
    std::cout
              << " | " << std::setw(width_err) << "err";
  }
  std::cout << "\n"
            << std::string(table_width, '-')
            << std::endl;

  double total_prefill_ms = 0.0;
  double total_decode_ms = 0.0;
  double gemm_ms = 0.0;
  double attention_ms = 0.0;
  double total_max_abs_error = 0.0;
  double total_max_rel_error = 0.0;
  uint32_t seed = options.seed;

  for (std::size_t i = 0; i < invocations.size(); ++i) {
    const systolic_transformer::workload_invocation_t& invocation =
      invocations[i];
    validate_kernel_arg(invocation.arg, invocation.name.c_str());

    invocation_result_t result = {0.0, 0.0, {0, 0.0, 0.0}};
    if (!options.dry_run) {
      result =
        run_kernel_invocation(invocation.arg, &seed, invocation.name, false);

      bool is_prefill =
        invocation.name.compare(0, 7, "prefill") == 0;
      if (is_prefill) {
        total_prefill_ms += result.elapsed_ms;
      } else {
        total_decode_ms += result.elapsed_ms;
      }

      if (invocation.arg.op == SYSTOLIC_OP_GEMM) {
        gemm_ms += result.elapsed_ms;
      } else {
        attention_ms += result.elapsed_ms;
      }
      if (options.verify_output &&
          result.stats.max_abs_error > total_max_abs_error) {
        total_max_abs_error = result.stats.max_abs_error;
      }
      if (options.verify_output &&
          result.stats.max_rel_error > total_max_rel_error) {
        total_max_rel_error = result.stats.max_rel_error;
      }
    }

    std::cout << std::left
              << std::setw(width_index) << i
              << " | " << std::setw(width_name)
              << fit_table_text(invocation.name, width_name)
              << " | " << std::setw(width_shape)
              << fit_table_text(describe_arg(invocation.arg), width_shape);
    if (!options.dry_run) {
      std::cout << std::right
                << std::fixed << std::setprecision(3);
      if (show_cpu_reference_ms) {
        std::cout << " | " << std::setw(width_cpu_ms)
                  << result.cpu_reference_ms;
        std::cout << " | " << std::setw(width_gpu_ms)
                  << result.elapsed_ms;
      } else {
        std::cout << " | " << std::setw(width_gpu_ms)
                  << result.elapsed_ms;
      }
      if (options.verify_output) {
        std::cout << " | " << std::setw(width_err) << result.stats.errors;
      } else {
        std::cout << " | " << std::setw(width_err) << "skip";
      }
      std::cout << std::left;
    }
    std::cout << std::endl;

    if (!options.dry_run && options.verify_output && result.stats.errors != 0) {
      std::cout << "\nKernel '" << invocation.name << "' failed with "
                << result.stats.errors << " mismatches" << std::endl;
      std::cout << "max_abs_error=" << result.stats.max_abs_error
                << ", max_rel_error=" << result.stats.max_rel_error
                << std::endl;
      mirror_runtime_perf_to_file();
      cleanup();
      std::cout << "FAILED!" << std::endl;
      return result.stats.errors;
    }
  }

  if (!options.dry_run) {
    double total_ms = total_prefill_ms + total_decode_ms;
    const std::string separator(60, '-');
    std::cout << "\n" << separator << std::endl
              << std::fixed << std::setprecision(3)
              << "  prefill   : " << std::setw(10) << total_prefill_ms
              << " ms  (" << spec->num_layers * 5 << " kernels)\n"
              << "  decode    : " << std::setw(10) << total_decode_ms
              << " ms  (" << options.output_len * spec->num_layers * 5
              << " kernels, " << options.output_len << " tokens)\n"
              << "  total     : " << std::setw(10) << total_ms
              << " ms\n"
              << separator << std::endl
              << "  GEMM      : " << std::setw(10) << gemm_ms << " ms\n"
              << "  ATTENTION : " << std::setw(10) << attention_ms << " ms";
    if (options.verify_output) {
      std::cout << "\n"
                << "  max_abs_error=" << total_max_abs_error << "\n"
                << "  max_rel_error=" << total_max_rel_error;
    } else {
      std::cout << "\n"
                << "  verification skipped";
    }
    std::cout << std::endl;
    mirror_runtime_perf_to_file();
    cleanup();
    std::cout << "PASSED!" << std::endl;
  }

  return 0;
}

int main(int argc, char** argv) {
  parse_args(argc, argv);
  setup_output_streams();

  if (!options.output_file.empty()) {
    std::cout << "output_file=" << options.output_file << std::endl;
  }

  if (options.run_mode == HOST_RUN_WORKLOAD) {
    int result = run_workload_mode();
    restore_output_streams();
    return result;
  }

  int result = run_single_mode();
  restore_output_streams();
  return result;
}

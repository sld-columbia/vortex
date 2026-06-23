// Copyright © 2019-2023
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <common.h>

#include <gt_vortex_rtl.h>
#include <esp.h>
#include <bitmanip.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_map>

using namespace vortex;

namespace {

constexpr uint64_t kDefaultBaseAddr = 0xA5000000ull;
constexpr const char* kDefaultDev = "/dev/gt_vortex_rtl.0";
constexpr const char* kDefaultMemDev = "/dev/mem";
constexpr const char* kDefaultContigDev = "/dev/contig_alloc";

uint64_t parse_u64_env(const char* name, uint64_t fallback) {
  const char* value = getenv(name);
  if (value == nullptr || *value == '\0')
    return fallback;

  char* end = nullptr;
  errno = 0;
  uint64_t parsed = strtoull(value, &end, 0);
  if (errno != 0 || end == value) {
    std::cerr << "[VXDRV] invalid " << name << "='" << value << "', using 0x"
              << std::hex << fallback << std::dec << std::endl;
    return fallback;
  }
  return parsed;
}

std::string resolve_device_path(const char* env_name, const char* fallback) {
  const char* value = getenv(env_name);
  if (value == nullptr || *value == '\0') {
    return std::string(fallback);
  }
  if (value[0] == '/') {
    return std::string(value);
  }
  return std::string("/dev/") + value;
}

} // namespace

class vx_device {
public:
  vx_device()
    : allocator_(ALLOC_BASE_ADDR,
                 GLOBAL_MEM_SIZE - ALLOC_BASE_ADDR,
                 RAM_PAGE_SIZE,
                 CACHE_BLOCK_SIZE)
    , fd_(-1)
    , mem_fd_(-1)
    , contig_fd_(-1)
    , contig_req_(nullptr)
    , contig_chunk_size_(0)
    , base_addr_(kDefaultBaseAddr)
    , mpm_class_(0)
  {}

  ~vx_device() {
    free_contig();
    if (mem_fd_ >= 0) {
      close(mem_fd_);
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  int init() {
    base_addr_ = parse_u64_env("VORTEX_ESP_BASE_ADDR", kDefaultBaseAddr);

    std::string dev_path = resolve_device_path("VORTEX_ESP_DEV", kDefaultDev);
    fd_ = open(dev_path.c_str(), O_RDWR, 0);
    if (fd_ < 0) {
      std::perror("[VXDRV] open accelerator device failed");
      return -1;
    }

    std::string mem_dev = resolve_device_path("VORTEX_ESP_MEM_DEV", kDefaultMemDev);
    mem_fd_ = open(mem_dev.c_str(), O_RDWR | O_SYNC);
    if (mem_fd_ < 0) {
      std::perror("[VXDRV] open /dev/mem failed");
      close(fd_);
      fd_ = -1;
      return -1;
    }

    if (init_contig(RAM_PAGE_SIZE) != 0) {
      free_contig();
      close(mem_fd_);
      close(fd_);
      mem_fd_ = -1;
      fd_ = -1;
      return -1;
    }

    DBGPRINT("DEV_INIT: dev=%s, mem=%s, base=0x%" PRIx64 "\n",
             dev_path.c_str(), mem_dev.c_str(), base_addr_);

    return 0;
  }

  int get_caps(uint32_t caps_id, uint64_t *value) {
    uint64_t _value;
    switch (caps_id) {
    case VX_CAPS_VERSION:
      _value = IMPLEMENTATION_ID;
      break;
    case VX_CAPS_NUM_THREADS:
      _value = NUM_THREADS;
      break;
    case VX_CAPS_NUM_WARPS:
      _value = NUM_WARPS;
      break;
    case VX_CAPS_NUM_CORES:
      _value = NUM_CORES * NUM_CLUSTERS;
      break;
    case VX_CAPS_CACHE_LINE_SIZE:
      _value = CACHE_BLOCK_SIZE;
      break;
    case VX_CAPS_GLOBAL_MEM_SIZE:
      _value = GLOBAL_MEM_SIZE;
      break;
    case VX_CAPS_LOCAL_MEM_SIZE:
      _value = (1 << LMEM_LOG_SIZE);
      break;
    case VX_CAPS_ISA_FLAGS:
      _value = ((uint64_t(MISA_EXT))<<32) | ((log2floor(XLEN)-4) << 30) | MISA_STD;
      break;
    default:
      std::cout << "invalid caps id: " << caps_id << std::endl;
      std::abort();
      return -1;
    }
    *value = _value;
    return 0;
  }

  int mem_alloc(uint64_t size, int flags, uint64_t* dev_addr) {
    uint64_t addr;
    CHECK_ERR(allocator_.allocate(size, &addr), {
      return err;
    });
    CHECK_ERR(this->mem_access(addr, size, flags), {
      allocator_.release(addr);
      return err;
    });
    *dev_addr = addr;
    return 0;
  }

  int mem_reserve(uint64_t dev_addr, uint64_t size, int flags) {
    CHECK_ERR(allocator_.reserve(dev_addr, size), {
      return err;
    });
    CHECK_ERR(this->mem_access(dev_addr, size, flags), {
      allocator_.release(dev_addr);
      return err;
    });
    return 0;
  }

  int mem_free(uint64_t dev_addr) {
    return allocator_.release(dev_addr);
  }

  int mem_access(uint64_t dev_addr, uint64_t size, int /*flags*/) {
    if (size == 0)
      return 0;

    uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
    if (dev_addr + asize < dev_addr)
      return -1;
    if (dev_addr + asize > GLOBAL_MEM_SIZE)
      return -1;
    return 0;
  }

  int mem_info(uint64_t* mem_free, uint64_t* mem_used) const {
    if (mem_free)
      *mem_free = allocator_.free();
    if (mem_used)
      *mem_used = allocator_.allocated();
    return 0;
  }

  int upload(uint64_t dev_addr, const void* src, uint64_t size) {
    return memcpy_to_device(dev_addr, src, size);
  }

  int download(void* dst, uint64_t dev_addr, uint64_t size) {
    return memcpy_from_device(dst, dev_addr, size);
  }

  int start(uint64_t krnl_addr, uint64_t args_addr) {
    if (fd_ < 0)
      return -1;

    // Ensure host-side writes to shared memory are visible before launch.
    if (this->flush_cache() != 0) {
      return -1;
    }

    struct gt_vortex_rtl_access desc;
    std::memset(&desc, 0, sizeof(desc));

    desc.BASE_ADDR = static_cast<unsigned>(base_addr_ & 0xffffffffu);
    desc.START_VORTEX = 1;
    desc.STARTUP_ADDR0 = static_cast<unsigned>(krnl_addr & 0xffffffffu);
    desc.STARTUP_ADDR1 = static_cast<unsigned>(krnl_addr >> 32);
    desc.STARTUP_ARG0 = static_cast<unsigned>(args_addr & 0xffffffffu);
    desc.STARTUP_ARG1 = static_cast<unsigned>(args_addr >> 32);
    desc.MPM_CLASS = mpm_class_;

    desc.esp.contig = contig_req_ ? contig_req_->khandle : nullptr;
    desc.esp.run = 1;
    desc.esp.coherence = ACC_COH_NONE;
    desc.esp.p2p_store = 0;
    desc.esp.p2p_nsrcs = 0;
    desc.esp.p2p_mcast_dests = 0;
    desc.esp.ndev_yx_table = 0;
    desc.esp.footprint = 0;
    desc.esp.alloc_policy = CONTIG_ALLOC_PREFERRED;
    desc.esp.ddr_node = 0;
    desc.esp.in_place = 0;
    desc.esp.reuse_factor = 0;

    if (ioctl(fd_, GT_VORTEX_RTL_IOC_ACCESS, &desc) < 0) {
      std::perror("[VXDRV] ioctl GT_VORTEX_RTL_IOC_ACCESS failed");
      return -1;
    }

    mpm_cache_.clear();
    return 0;
  }

  int ready_wait(uint64_t /*timeout*/) {
    // The ESP access ioctl blocks until completion when run=1.
    // Flush/invalidate host-visible caches before user-space reads results.
    return this->flush_cache();
  }

  int dcr_write(uint32_t addr, uint32_t value) {
    dcrs_.write(addr, value);
    if (addr == VX_DCR_BASE_MPM_CLASS) {
      mpm_class_ = value;
    }
    return 0;
  }

  int dcr_read(uint32_t addr, uint32_t* value) const {
    return dcrs_.read(addr, value);
  }

  int mpm_query(uint32_t addr, uint32_t core_id, uint64_t* value) {
    uint32_t offset = addr - VX_CSR_MPM_BASE;
    if (offset > 31)
      return -1;
    if (mpm_cache_.count(core_id) == 0) {
      uint64_t mpm_mem_addr = IO_MPM_ADDR + core_id * 32 * sizeof(uint64_t);
      CHECK_ERR(this->download(mpm_cache_[core_id].data(), mpm_mem_addr, 32 * sizeof(uint64_t)), {
        return err;
      });
    }
    *value = mpm_cache_.at(core_id).at(offset);
    return 0;
  }

private:
  int flush_cache() {
    if (fd_ < 0)
      return -1;

    struct gt_vortex_rtl_access desc;
    std::memset(&desc, 0, sizeof(desc));
    desc.esp.coherence = ACC_COH_NONE;
    if (ioctl(fd_, ESP_IOC_FLUSH, &desc) < 0) {
      std::perror("[VXDRV] ioctl ESP_IOC_FLUSH failed");
      return -1;
    }
    return 0;
  }

  int memcpy_to_device(uint64_t dev_addr, const void* src, uint64_t size) {
    if (src == nullptr || size == 0)
      return -1;
    if (mem_fd_ < 0)
      return -1;
    if (this->mem_access(dev_addr, size, VX_MEM_WRITE) != 0)
      return -1;

    uint64_t phys_addr = base_addr_ + dev_addr;
    void* map_base = nullptr;
    uint64_t map_size = 0;
    uint64_t map_offset = 0;
    if (map_region(phys_addr, size, &map_base, &map_size, &map_offset) != 0)
      return -1;

    std::memcpy(reinterpret_cast<uint8_t*>(map_base) + map_offset, src, size);

    munmap(map_base, map_size);
    return 0;
  }

  int memcpy_from_device(void* dst, uint64_t dev_addr, uint64_t size) {
    if (dst == nullptr || size == 0)
      return -1;
    if (mem_fd_ < 0)
      return -1;
    if (this->mem_access(dev_addr, size, VX_MEM_READ) != 0)
      return -1;

    uint64_t phys_addr = base_addr_ + dev_addr;
    void* map_base = nullptr;
    uint64_t map_size = 0;
    uint64_t map_offset = 0;
    if (map_region(phys_addr, size, &map_base, &map_size, &map_offset) != 0)
      return -1;

    std::memcpy(dst, reinterpret_cast<uint8_t*>(map_base) + map_offset, size);

    munmap(map_base, map_size);
    return 0;
  }

  int map_region(uint64_t phys_addr, uint64_t size, void** map_base,
                 uint64_t* map_size, uint64_t* map_offset) {
    if (size == 0)
      return -1;

    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0)
      return -1;

    uint64_t page_mask = ~(uint64_t(page_size) - 1);
    uint64_t aligned_addr = phys_addr & page_mask;
    uint64_t offset = phys_addr - aligned_addr;
    uint64_t length = offset + size;

    void* mapped = mmap(nullptr, length, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd_, aligned_addr);
    if (mapped == MAP_FAILED) {
      std::perror("[VXDRV] mmap failed");
      return -1;
    }

    *map_base = mapped;
    *map_size = length;
    *map_offset = offset;

    return 0;
  }

  int init_contig(size_t size) {
    std::string contig_dev = resolve_device_path("VORTEX_ESP_CONTIG_DEV", kDefaultContigDev);
    contig_fd_ = open(contig_dev.c_str(), O_RDWR);
    if (contig_fd_ < 0) {
      std::perror("[VXDRV] open /dev/contig_alloc failed");
      return -1;
    }

    unsigned long chunk_log = 0;
    if (ioctl(contig_fd_, CONTIG_IOC_CHUNK_LOG, &chunk_log) < 0) {
      std::perror("[VXDRV] ioctl CONTIG_IOC_CHUNK_LOG failed");
      return -1;
    }
    contig_chunk_size_ = 1ul << chunk_log;
    unsigned long n_max = (size + contig_chunk_size_ - 1) / contig_chunk_size_;

    contig_req_ = static_cast<contig_alloc_req*>(std::calloc(1, sizeof(*contig_req_)));
    if (contig_req_ == nullptr)
      return -1;

    contig_req_->arr = static_cast<unsigned long*>(std::calloc(n_max, sizeof(*contig_req_->arr)));
    if (contig_req_->arr == nullptr)
      return -1;

    contig_req_->n_max = n_max;
    contig_req_->size = size;
    contig_req_->params.policy = CONTIG_ALLOC_PREFERRED;
    contig_req_->params.pol.first.ddr_node = 0;

    if (ioctl(contig_fd_, CONTIG_IOC_ALLOC, contig_req_) < 0) {
      std::perror("[VXDRV] ioctl CONTIG_IOC_ALLOC failed");
      return -1;
    }

    contig_req_->mm = mmap(nullptr, contig_req_->n * contig_chunk_size_, PROT_READ | PROT_WRITE,
                            MAP_SHARED, contig_fd_, contig_req_->arr[0]);
    if (contig_req_->mm == MAP_FAILED) {
      std::perror("[VXDRV] mmap contig buffer failed");
      return -1;
    }

    DBGPRINT("CONTIG_INIT: dev=%s, size=%zu, chunks=%u\n",
             contig_dev.c_str(), size, contig_req_->n);

    return 0;
  }

  void free_contig() {
    if (contig_req_) {
      if (contig_req_->mm && contig_req_->mm != MAP_FAILED && contig_chunk_size_ != 0) {
        munmap(contig_req_->mm, contig_req_->n * contig_chunk_size_);
      }
      if (contig_fd_ >= 0 && contig_req_->khandle != nullptr) {
        ioctl(contig_fd_, CONTIG_IOC_FREE, &contig_req_->khandle);
      }
      std::free(contig_req_->arr);
      std::free(contig_req_);
      contig_req_ = nullptr;
    }
    if (contig_fd_ >= 0) {
      close(contig_fd_);
      contig_fd_ = -1;
    }
  }

  MemoryAllocator allocator_;
  DeviceConfig dcrs_;
  int fd_;
  int mem_fd_;
  int contig_fd_;
  contig_alloc_req* contig_req_;
  unsigned long contig_chunk_size_;
  uint64_t base_addr_;
  uint32_t mpm_class_;
  std::unordered_map<uint32_t, std::array<uint64_t, 32>> mpm_cache_;
};

#include <callbacks.inc>

#include <stdint.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <unistd.h>
#include <assert.h>
#include <cmath>
#include <sstream>
#include <unordered_map>
#include <list>

#include "vx_utils.h"
#include "vx_malloc.h"
#include <vortex.h>
#include <VX_config.h>

#ifdef SCOPE
#include "vx_scope.h"
#endif

#include "libesp.h"
#include "esp.h"
#include "gt_vortex_rtl.h"

#define STATUS_STATE_BITS   8

#define VERSION 1
#define NUM_CORES 1
#define NUM_WARPS 2
#define NUM_THREADS 4

#define ESP_OFFSET 0x30000000
#define ESP_BASE_ADDR 0xB0000000
#define DEV_NAME "gt_vortex_rtl.0"
///////////////////////////////////////////////////////////////////////////////

class vx_device {
public:
    vx_device() 
        : mem_allocator(
            ALLOC_BASE_ADDR, 
            ALLOC_BASE_ADDR + LOCAL_MEM_SIZE,
            4096,            
            4096)
    {}
    
    ~vx_device() {}

    vortex::MemoryAllocator mem_allocator;
    unsigned version;
    unsigned num_cores;
    unsigned num_warps;
    unsigned num_threads;
    int fd;
    struct esp_access *esp_desc;
};

typedef struct vx_buffer_ {
    void* host_ptr;
    vx_device_h hdevice;
    uint64_t size;
} vx_buffer_t;

///////////////////////////////////////////////////////////////////////////////

#ifdef DUMP_PERF_STATS
class AutoPerfDump {
private:
    std::list<vx_device_h> devices_;

public:
    AutoPerfDump() {} 

    ~AutoPerfDump() {
        for (auto device : devices_) {
            vx_dump_perf(device, stdout);
        }
    }

    void add_device(vx_device_h device) {
        devices_.push_back(device);
    }

    void remove_device(vx_device_h device) {
        devices_.remove(device);
    }    
};

AutoPerfDump gAutoPerfDump;
#endif

///////////////////////////////////////////////////////////////////////////////

extern int vx_dev_caps(vx_device_h hdevice, uint32_t caps_id, uint64_t *value) {
    if (nullptr == hdevice)
        return -1;

    vx_device *device = ((vx_device*)hdevice);

    switch (caps_id) {
    case VX_CAPS_VERSION:
        *value = device->version;
        break;
    case VX_CAPS_MAX_CORES:
        *value = device->num_cores;
        break;
    case VX_CAPS_MAX_WARPS:
        *value = device->num_warps;
        break;
    case VX_CAPS_MAX_THREADS:
        *value = device->num_threads;
        break;
    case VX_CAPS_CACHE_LINE_SIZE:
        *value = CACHE_BLOCK_SIZE;
        break;
    case VX_CAPS_LOCAL_MEM_SIZE:
        *value = LOCAL_MEM_SIZE;
        break;
    case VX_CAPS_ALLOC_BASE_ADDR:
        *value = ALLOC_BASE_ADDR;
        break;
    case VX_CAPS_KERNEL_BASE_ADDR:
        *value = STARTUP_ADDR;
        break;
    default:
        fprintf(stderr, "[VXDRV] Error: invalid caps id: %d\n", caps_id);
        std::abort();
        return -1;
    }

    return 0;
}

extern int vx_dev_open(vx_device_h* hdevice) {
    if (nullptr == hdevice)
        return  -1;

    vx_device* device;
    char path[70];
    const char *prefix = "/dev/";

    if (strlen(DEV_NAME) > 64) {
	fprintf(stderr, "[VXDRV] device name %s exceeds maximum length of 64 characters\n", DEV_NAME);
        std::abort();
        return -1;
    }
     
    sprintf(path, "%s%s", prefix, DEV_NAME);

    // allocate device object
    device = new vx_device();
    if (nullptr == device) {
        return -1;
    }

    device->fd = open(path, O_RDWR, 0);

    if (device->fd < 0) {
	fprintf(stderr, "[VXDRV] fopen for device failed\n");
        std::abort();
        return -1;
    }

    device->version     = VERSION;
    device->num_cores   = NUM_CORES; 
    device->num_warps   = NUM_WARPS;
    device->num_threads = NUM_THREADS;
#ifndef NDEBUG    
    fprintf(stdout, "[VXDRV] DEVCAPS: version=%d, num_cores=%d, num_warps=%d, num_threads=%d\n", 
            device->version, device->num_cores, device->num_warps, device->num_threads);
#endif
    
#ifdef SCOPE
    {
        int ret = vx_scope_start(accel_handle, 0, -1);
        if (ret != 0) {
            close(device->fd);
	    return ret;
        }
    }
#endif    

    *hdevice = device;

#ifdef DUMP_PERF_STATS
    gAutoPerfDump.add_device(*hdevice);
#endif

    return 0;
}

extern int vx_dev_close(vx_device_h hdevice) {
    if (nullptr == hdevice)
        return -1;

    vx_device *device = ((vx_device*)hdevice);

#ifdef SCOPE
    vx_scope_stop(device->fpga);
#endif

#ifdef DUMP_PERF_STATS
    gAutoPerfDump.remove_device(hdevice);
    vx_dump_perf(hdevice, stdout);
#endif

    close(device->fd);

    delete device;

    return 0;
}

extern int vx_mem_alloc(vx_device_h hdevice, uint64_t size, uint64_t* dev_maddr) {
    if (nullptr == hdevice 
     || nullptr == dev_maddr
     || 0 >= size)
        return -1;

    vx_device *device = ((vx_device*)hdevice);
    return device->mem_allocator.allocate(size, dev_maddr);
}

extern int vx_mem_free(vx_device_h hdevice, uint64_t dev_maddr) {
    if (nullptr == hdevice)
        return -1;

    vx_device *device = ((vx_device*)hdevice);
    return device->mem_allocator.release(dev_maddr);
}

extern int vx_buf_alloc(vx_device_h hdevice, uint64_t size, vx_buffer_h* hbuffer) {
    void* host_ptr;
    vx_buffer_t* buffer;

    if (nullptr == hdevice
     || 0 >= size
     || nullptr == hbuffer)
        return -1;

    size_t asize = aligned_size(size, CACHE_BLOCK_SIZE);

    host_ptr = malloc(asize);
    if (host_ptr == nullptr) {
        return -1;
    }

    // allocate buffer object
    buffer = (vx_buffer_t*)malloc(sizeof(vx_buffer_t));
    if (nullptr == buffer) {
        free(host_ptr);
	return -1;
    }

    buffer->host_ptr = host_ptr;
    buffer->hdevice  = hdevice;
    buffer->size     = asize;

    *hbuffer = buffer;

    return 0;
}

extern void* vx_host_ptr(vx_buffer_h hbuffer) {
    if (nullptr == hbuffer)
        return nullptr;

    vx_buffer_t* buffer = ((vx_buffer_t*)hbuffer);
    return buffer->host_ptr;
}

extern int vx_buf_free(vx_buffer_h hbuffer) {
    if (nullptr == hbuffer)
        return -1;

    vx_buffer_t* buffer = ((vx_buffer_t*)hbuffer);

    free(buffer->host_ptr);

    return 0;
}

extern int vx_ready_wait(vx_device_h hdevice, uint64_t timeout) {
    if (nullptr == hdevice)
        return -1;

    vx_device *device = ((vx_device*)hdevice);

    ioctl(device->fd, ESP_IOC_WAIT, nullptr);  

    return 0;
}

extern int vx_copy_to_dev(vx_buffer_h hbuffer, uint64_t dev_maddr, uint64_t size, uint64_t src_offset) {
    if (nullptr == hbuffer 
     || 0 >= size)
        return -1;

    void *buf_ptr;
    void *host_ptr = vx_host_ptr(hbuffer);

    printf("size: %llu, dev_maddr: %llx, src_offset: %llx, dest_addr: %llx\n", size, dev_maddr, src_offset, ESP_OFFSET + dev_maddr);
    int fd = open("/dev/mem", O_RDWR);
    buf_ptr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, ESP_OFFSET + dev_maddr);
    close(fd);

    if (buf_ptr == MAP_FAILED)
	return -1;

    memcpy(buf_ptr, (void *) ((uint8_t *) host_ptr + src_offset), size); 

    return munmap(buf_ptr, size);
}

extern int vx_copy_from_dev(vx_buffer_h hbuffer, uint64_t dev_maddr, uint64_t size, uint64_t dest_offset) {
    if (nullptr == hbuffer 
     || 0 >= size)
        return -1;

    void *buf_ptr;
    void *host_ptr = vx_host_ptr(hbuffer);

    int fd = open("/dev/mem", O_RDWR);
    buf_ptr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, ESP_OFFSET + dev_maddr);
    close(fd);

    if (buf_ptr == MAP_FAILED)
	return -1;

    memcpy((void *) ((uint8_t *) host_ptr + dest_offset), buf_ptr, size); 

    return munmap(buf_ptr, size);
}

extern int vx_start(vx_device_h hdevice) {
    if (nullptr == hdevice)
        return -1;   

    vx_device *device = ((vx_device*)hdevice);
    struct gt_vortex_rtl_access vortex_desc;
    vortex_desc.VX_BUSY_INT = 0;
    vortex_desc.BASE_ADDR = ESP_OFFSET;
    vortex_desc.START_VORTEX = 1;
    vortex_desc.src_offset = 0;
    vortex_desc.dst_offset = 0;
    vortex_desc.esp.run = 1;
    vortex_desc.esp.p2p_store = 0;
    vortex_desc.esp.p2p_nsrcs = 0;
    vortex_desc.esp.coherence = ACC_COH_NONE;
    vortex_desc.esp.third_party = 1;


    ioctl(device->fd, GT_VORTEX_RTL_IOC_ACCESS, (struct esp_access *) &vortex_desc);

    return 0;
}

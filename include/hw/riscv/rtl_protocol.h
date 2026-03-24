/*
 * RTL-QEMU Communication Protocol
 *
 * This header defines the binary protocol used for communication between
 * the soc-simulator (RTL side) and QEMU (emulator side) over a Unix socket.
 *
 * Architecture:
 *   soc-simulator <--Unix Socket--> QEMU
 *
 * The RTL side drives the CPU core and memory. QEMU provides peripherals
 * (UART, NIC, block devices, etc.) and interrupt management.
 *
 * Message flow:
 *   1. soc-simulator connects to QEMU's Unix socket
 *   2. soc-simulator sends HELLO with CPU capabilities
 *   3. QEMU configures its device tree / address map accordingly
 *   4. Normal operation: MMIO forwarded to QEMU, DMA/IRQ from QEMU
 *
 * All multi-byte fields are little-endian.
 */

#ifndef RTL_PROTOCOL_H
#define RTL_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RTL_PROTOCOL_MAGIC   0x51454D55524C5400ULL  /* "QEMURTL\0" */
#define RTL_PROTOCOL_VERSION 3

/* Maximum data payload for DMA transfers (4KB per message) */
#define RTL_MAX_DMA_SIZE     4096

/* Maximum number of address ranges in HELLO */
#define RTL_MAX_ADDR_RANGES  16

/* Maximum number of IRQ lines */
#define RTL_MAX_IRQ_LINES    64

/*
 * Memory modes: determines where the CPU's main memory lives.
 *
 * RTL_MEMMODE_LOCAL:  Memory is in soc-simulator (current default).
 *   CPU AXI mem accesses are served by local axi4_mem.
 *   QEMU uses MEM_READ/MEM_WRITE to pre-load firmware into soc-sim memory.
 *   DMA from QEMU devices must go through the protocol to reach soc-sim.
 *
 * RTL_MEMMODE_QEMU:   Memory is in QEMU's address space.
 *   CPU AXI mem accesses are forwarded to QEMU via CPU_MEM_READ/CPU_MEM_WRITE.
 *   QEMU devices can DMA directly to their own RAM (no protocol needed).
 *   If the CPU lacks a coherent DMA port, the software must use CMO
 *   instructions (cbo.inval/cbo.flush/cbo.clean) to keep caches coherent.
 */
enum rtl_memory_mode {
    RTL_MEMMODE_LOCAL = 0,
    RTL_MEMMODE_QEMU  = 1,
};

/*
 * Capability flags reported in HELLO.
 */
enum rtl_cap_flags {
    RTL_CAP_COHERENT_DMA = (1 << 0),  /* CPU has a coherent DMA port
                                         (e.g. L2 frontend bus) */
};

/*
 * Message types
 */
enum rtl_msg_type {
    /* Connection setup */
    RTL_MSG_HELLO          = 0x01,  /* RTL -> QEMU: CPU info + address map */
    RTL_MSG_HELLO_ACK      = 0x02,  /* QEMU -> RTL: acknowledgement */

    /* MMIO (CPU accesses peripheral in QEMU) */
    RTL_MSG_MMIO_READ      = 0x10,  /* RTL -> QEMU: read request */
    RTL_MSG_MMIO_READ_RESP = 0x11,  /* QEMU -> RTL: read response */
    RTL_MSG_MMIO_WRITE     = 0x12,  /* RTL -> QEMU: write request */
    RTL_MSG_MMIO_WRITE_RESP= 0x13,  /* QEMU -> RTL: write response (ack) */

    /* DMA (QEMU device accesses memory in RTL via AXI slave - reserved) */
    RTL_MSG_DMA_READ       = 0x20,  /* QEMU -> RTL: read from RTL memory */
    RTL_MSG_DMA_READ_RESP  = 0x21,  /* RTL -> QEMU: read response with data */
    RTL_MSG_DMA_WRITE      = 0x22,  /* QEMU -> RTL: write to RTL memory */
    RTL_MSG_DMA_WRITE_RESP = 0x23,  /* RTL -> QEMU: write response (ack) */

    /* Interrupts */
    RTL_MSG_IRQ_UPDATE     = 0x30,  /* QEMU -> RTL: interrupt line change */

    /* Synchronization */
    RTL_MSG_SYNC           = 0x40,  /* RTL -> QEMU: clock sync / heartbeat */
    RTL_MSG_SYNC_ACK       = 0x41,  /* QEMU -> RTL: sync acknowledgement */

    /*
     * System memory access (pre-boot, bypasses cache safely because CPU
     * is not running yet).  Used by QEMU to load firmware / set up DTB
     * into the RTL side's DRAM before issuing CPU_START.
     */
    RTL_MSG_MEM_READ       = 0x50,  /* QEMU -> RTL: read system memory */
    RTL_MSG_MEM_READ_RESP  = 0x51,  /* RTL -> QEMU: read response + data */
    RTL_MSG_MEM_WRITE      = 0x52,  /* QEMU -> RTL: write system memory */
    RTL_MSG_MEM_WRITE_RESP = 0x53,  /* RTL -> QEMU: write acknowledgement */

    /* CPU control */
    RTL_MSG_CPU_START      = 0x60,  /* QEMU -> RTL: start RTL simulation */
    RTL_MSG_CPU_START_ACK  = 0x61,  /* RTL -> QEMU: start acknowledged */
    RTL_MSG_CPU_STOP       = 0x62,  /* QEMU -> RTL: stop/pause simulation */
    RTL_MSG_CPU_STOP_ACK   = 0x63,  /* RTL -> QEMU: stop acknowledged */
    RTL_MSG_CPU_STATUS     = 0x64,  /* QEMU -> RTL: query CPU status */
    RTL_MSG_CPU_STATUS_RESP= 0x65,  /* RTL -> QEMU: status response */

    /*
     * CPU memory access (QEMU-memory mode only).
     * In RTL_MEMMODE_QEMU the CPU's AXI memory-port transactions are
     * forwarded to QEMU so that the memory lives inside QEMU's address
     * space.  This allows QEMU devices to DMA without going through the
     * protocol when coherent DMA is not available.
     */
    RTL_MSG_CPU_MEM_READ       = 0x70,  /* RTL -> QEMU: CPU reads memory */
    RTL_MSG_CPU_MEM_READ_RESP  = 0x71,  /* QEMU -> RTL: read response + data */
    RTL_MSG_CPU_MEM_WRITE      = 0x72,  /* RTL -> QEMU: CPU writes memory */
    RTL_MSG_CPU_MEM_WRITE_RESP = 0x73,  /* QEMU -> RTL: write ack */

    /*
     * Debug / DMI interface.
     * QEMU sends DMI requests to the soc-simulator which drives them
     * on the Rocket-Chip debug port.  This enables GDB debugging of
     * the RTL CPU core.
     */
    RTL_MSG_DEBUG_DMI_REQ  = 0x80,  /* QEMU -> RTL: DMI read/write */
    RTL_MSG_DEBUG_DMI_RESP = 0x81,  /* RTL -> QEMU: DMI response */

    /* Control */
    RTL_MSG_SHUTDOWN       = 0xF0,  /* Either -> Either: terminate */
};

/*
 * Common message header (8 bytes)
 *
 * Every message starts with this header.
 */
struct rtl_msg_header {
    uint32_t type;       /* enum rtl_msg_type */
    uint32_t length;     /* total message length including header */
} __attribute__((packed));

/*
 * Address range descriptor for HELLO message
 */
struct rtl_addr_range {
    uint64_t base;       /* base address */
    uint64_t size;       /* size in bytes */
    uint32_t type;       /* 0=memory, 1=MMIO (forwarded to QEMU) */
    uint32_t flags;      /* reserved, must be 0 */
} __attribute__((packed));

/*
 * HELLO message: RTL -> QEMU
 *
 * Sent once after connection. Reports CPU capabilities.
 */
struct rtl_msg_hello {
    struct rtl_msg_header hdr;
    uint64_t magic;           /* RTL_PROTOCOL_MAGIC */
    uint32_t version;         /* RTL_PROTOCOL_VERSION */
    uint32_t num_cores;       /* number of CPU cores */
    uint32_t xlen;            /* XLEN: 32 or 64 */
    uint32_t num_irq_lines;   /* number of external interrupt lines */
    char     isa_string[64];  /* ISA string, e.g. "rv64imafdc" */
    uint32_t num_addr_ranges; /* number of address ranges following */
    uint32_t flags;           /* bitmask of rtl_cap_flags */
    uint32_t memory_mode;     /* enum rtl_memory_mode requested by RTL side */
    uint32_t reserved2;
    struct rtl_addr_range addr_ranges[RTL_MAX_ADDR_RANGES];
} __attribute__((packed));

/*
 * HELLO_ACK message: QEMU -> RTL
 */
struct rtl_msg_hello_ack {
    struct rtl_msg_header hdr;
    uint32_t status;       /* 0=OK, nonzero=error */
    uint32_t memory_mode;  /* confirmed enum rtl_memory_mode */
} __attribute__((packed));

/*
 * MMIO read request: RTL -> QEMU
 */
struct rtl_msg_mmio_read {
    struct rtl_msg_header hdr;
    uint64_t addr;       /* physical address */
    uint32_t size;       /* access size in bytes (1, 2, 4, 8) */
    uint32_t req_id;     /* request ID for matching response */
} __attribute__((packed));

/*
 * MMIO read response: QEMU -> RTL
 */
struct rtl_msg_mmio_read_resp {
    struct rtl_msg_header hdr;
    uint64_t data;       /* read data (zero-extended) */
    uint32_t req_id;     /* matching request ID */
    uint32_t status;     /* 0=OK, 1=error */
} __attribute__((packed));

/*
 * MMIO write request: RTL -> QEMU
 */
struct rtl_msg_mmio_write {
    struct rtl_msg_header hdr;
    uint64_t addr;       /* physical address */
    uint64_t data;       /* write data */
    uint32_t size;       /* access size in bytes (1, 2, 4, 8) */
    uint32_t req_id;     /* request ID for matching response */
} __attribute__((packed));

/*
 * MMIO write response: QEMU -> RTL
 */
struct rtl_msg_mmio_write_resp {
    struct rtl_msg_header hdr;
    uint32_t req_id;     /* matching request ID */
    uint32_t status;     /* 0=OK, 1=error */
} __attribute__((packed));

/*
 * DMA read request: QEMU -> RTL
 */
struct rtl_msg_dma_read {
    struct rtl_msg_header hdr;
    uint64_t addr;       /* physical address in RTL memory */
    uint32_t size;       /* size in bytes (up to RTL_MAX_DMA_SIZE) */
    uint32_t req_id;     /* request ID */
} __attribute__((packed));

/*
 * DMA read response: RTL -> QEMU
 *
 * Variable-length: header + data[size]
 */
struct rtl_msg_dma_read_resp {
    struct rtl_msg_header hdr;
    uint32_t req_id;     /* matching request ID */
    uint32_t status;     /* 0=OK, 1=error */
    uint8_t  data[];     /* response data (flexible array) */
} __attribute__((packed));

/*
 * DMA write request: QEMU -> RTL
 *
 * Variable-length: header + data[size]
 */
struct rtl_msg_dma_write {
    struct rtl_msg_header hdr;
    uint64_t addr;       /* physical address in RTL memory */
    uint32_t size;       /* size in bytes (up to RTL_MAX_DMA_SIZE) */
    uint32_t req_id;     /* request ID */
    uint8_t  data[];     /* write data (flexible array) */
} __attribute__((packed));

/*
 * DMA write response: RTL -> QEMU
 */
struct rtl_msg_dma_write_resp {
    struct rtl_msg_header hdr;
    uint32_t req_id;     /* matching request ID */
    uint32_t status;     /* 0=OK, 1=error */
} __attribute__((packed));

/*
 * IRQ update: QEMU -> RTL
 *
 * Carries a bitmask of all interrupt line levels.
 * Bit N = 1 means IRQ line N is asserted.
 */
struct rtl_msg_irq_update {
    struct rtl_msg_header hdr;
    uint64_t irq_levels; /* bitmask of IRQ line levels */
} __attribute__((packed));

/*
 * Sync message: RTL -> QEMU
 *
 * Sent periodically to synchronize virtual time.
 */
struct rtl_msg_sync {
    struct rtl_msg_header hdr;
    uint64_t tick_count;  /* current RTL tick count */
} __attribute__((packed));

/*
 * Sync acknowledgement: QEMU -> RTL
 */
struct rtl_msg_sync_ack {
    struct rtl_msg_header hdr;
    uint64_t tick_count;  /* acknowledged tick count */
} __attribute__((packed));

/*
 * System memory read request: QEMU -> RTL
 *
 * Used before CPU starts to read back memory contents.
 */
struct rtl_msg_mem_read {
    struct rtl_msg_header hdr;
    uint64_t addr;       /* physical address */
    uint32_t size;       /* bytes to read (up to RTL_MAX_DMA_SIZE) */
    uint32_t req_id;
} __attribute__((packed));

/*
 * System memory read response: RTL -> QEMU
 *
 * Variable-length: fixed header + data[size]
 */
struct rtl_msg_mem_read_resp {
    struct rtl_msg_header hdr;
    uint32_t req_id;
    uint32_t status;     /* 0=OK, 1=error */
    uint8_t  data[];     /* response data */
} __attribute__((packed));

/*
 * System memory write request: QEMU -> RTL
 *
 * Variable-length: fixed header + data[size]
 * Used before CPU starts to load firmware, DTB, etc.
 */
struct rtl_msg_mem_write {
    struct rtl_msg_header hdr;
    uint64_t addr;       /* physical address */
    uint32_t size;       /* bytes to write (up to RTL_MAX_DMA_SIZE) */
    uint32_t req_id;
    uint8_t  data[];     /* write data */
} __attribute__((packed));

/*
 * System memory write response: RTL -> QEMU
 */
struct rtl_msg_mem_write_resp {
    struct rtl_msg_header hdr;
    uint32_t req_id;
    uint32_t status;     /* 0=OK, 1=error */
} __attribute__((packed));

/*
 * CPU start: QEMU -> RTL
 *
 * Tells soc-simulator to begin clocking the RTL CPU.
 * The start_addr field can optionally override the reset vector.
 */
struct rtl_msg_cpu_start {
    struct rtl_msg_header hdr;
    uint64_t start_addr; /* 0 = use default reset vector */
} __attribute__((packed));

/*
 * CPU start acknowledgement: RTL -> QEMU
 */
struct rtl_msg_cpu_start_ack {
    struct rtl_msg_header hdr;
    uint32_t status;     /* 0=OK */
    uint32_t reserved;
} __attribute__((packed));

/*
 * CPU stop: QEMU -> RTL
 *
 * Tells soc-simulator to pause the RTL CPU.
 */
struct rtl_msg_cpu_stop {
    struct rtl_msg_header hdr;
    uint32_t reason;     /* 0=pause, 1=reset */
    uint32_t reserved;
} __attribute__((packed));

/*
 * CPU stop acknowledgement: RTL -> QEMU
 */
struct rtl_msg_cpu_stop_ack {
    struct rtl_msg_header hdr;
    uint32_t status;     /* 0=OK */
    uint32_t reserved;
} __attribute__((packed));

/* CPU status values */
enum rtl_cpu_status {
    RTL_CPU_STATUS_STOPPED = 0,
    RTL_CPU_STATUS_RUNNING = 1,
    RTL_CPU_STATUS_RESET   = 2,
};

/*
 * CPU status query: QEMU -> RTL
 */
struct rtl_msg_cpu_status {
    struct rtl_msg_header hdr;
} __attribute__((packed));

/*
 * CPU status response: RTL -> QEMU
 */
struct rtl_msg_cpu_status_resp {
    struct rtl_msg_header hdr;
    uint32_t status;     /* enum rtl_cpu_status */
    uint64_t tick_count; /* current tick */
    uint32_t reserved;
} __attribute__((packed));

/*
 * CPU memory read (QEMU-memory mode): RTL -> QEMU
 *
 * Sent when the CPU's AXI memory port issues a read and memory_mode
 * is RTL_MEMMODE_QEMU.  The payload describes a contiguous read.
 */
struct rtl_msg_cpu_mem_read {
    struct rtl_msg_header hdr;
    uint64_t addr;       /* physical address */
    uint32_t size;       /* bytes to read (up to RTL_MAX_DMA_SIZE) */
    uint32_t req_id;
} __attribute__((packed));

/*
 * CPU memory read response: QEMU -> RTL
 * Variable-length: fixed header + data[size]
 */
struct rtl_msg_cpu_mem_read_resp {
    struct rtl_msg_header hdr;
    uint32_t req_id;
    uint32_t status;     /* 0=OK, 1=error */
    uint8_t  data[];     /* response data */
} __attribute__((packed));

/*
 * CPU memory write (QEMU-memory mode): RTL -> QEMU
 * Variable-length: fixed header + data[size]
 */
struct rtl_msg_cpu_mem_write {
    struct rtl_msg_header hdr;
    uint64_t addr;       /* physical address */
    uint32_t size;       /* bytes to write (up to RTL_MAX_DMA_SIZE) */
    uint32_t req_id;
    uint8_t  data[];     /* write data */
} __attribute__((packed));

/*
 * CPU memory write response: QEMU -> RTL
 */
struct rtl_msg_cpu_mem_write_resp {
    struct rtl_msg_header hdr;
    uint32_t req_id;
    uint32_t status;     /* 0=OK, 1=error */
} __attribute__((packed));

/*
 * Debug DMI request: QEMU -> RTL
 *
 * Performs a single DMI register read or write on the Rocket-Chip
 * debug module.  Used by the GDB server in QEMU.
 */
struct rtl_msg_debug_dmi_req {
    struct rtl_msg_header hdr;
    uint32_t req_id;
    uint8_t  addr;       /* DMI address (7-bit) */
    uint8_t  op;         /* 1=read, 2=write */
    uint8_t  reserved[2];
    uint32_t data;       /* write data (ignored for reads) */
} __attribute__((packed));

/*
 * Debug DMI response: RTL -> QEMU
 */
struct rtl_msg_debug_dmi_resp {
    struct rtl_msg_header hdr;
    uint32_t req_id;
    uint32_t data;       /* read data */
    uint32_t status;     /* 0=success, nonzero=error */
} __attribute__((packed));

/*
 * Shutdown message
 */
struct rtl_msg_shutdown {
    struct rtl_msg_header hdr;
    uint32_t reason;     /* 0=normal, 1=error */
    uint32_t reserved;
} __attribute__((packed));

/*
 * Helper: compute message size for variable-length messages
 */
static inline uint32_t rtl_dma_read_resp_size(uint32_t data_size) {
    return (uint32_t)(sizeof(struct rtl_msg_dma_read_resp) + data_size);
}

static inline uint32_t rtl_dma_write_size(uint32_t data_size) {
    return (uint32_t)(sizeof(struct rtl_msg_dma_write) + data_size);
}

static inline uint32_t rtl_mem_read_resp_size(uint32_t data_size) {
    return (uint32_t)(sizeof(struct rtl_msg_mem_read_resp) + data_size);
}

static inline uint32_t rtl_mem_write_size(uint32_t data_size) {
    return (uint32_t)(sizeof(struct rtl_msg_mem_write) + data_size);
}

static inline uint32_t rtl_cpu_mem_read_resp_size(uint32_t data_size) {
    return (uint32_t)(sizeof(struct rtl_msg_cpu_mem_read_resp) + data_size);
}

static inline uint32_t rtl_cpu_mem_write_size(uint32_t data_size) {
    return (uint32_t)(sizeof(struct rtl_msg_cpu_mem_write) + data_size);
}

#ifdef __cplusplus
}
#endif

#endif /* RTL_PROTOCOL_H */

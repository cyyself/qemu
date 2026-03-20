/*
 * RTL Machine interface
 *
 * Copyright (c) 2024 qemu-system-rtl contributors
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 */

#ifndef HW_RISCV_RTL_H
#define HW_RISCV_RTL_H

#include "hw/core/boards.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define RTL_CPUS_MAX 1

#define TYPE_RTL_MACHINE MACHINE_TYPE_NAME("rtl")
typedef struct RTLMachineState RTLMachineState;
DECLARE_INSTANCE_CHECKER(RTLMachineState, RTL_MACHINE,
                         TYPE_RTL_MACHINE)

/*
 * Socket connection state for communication with soc-simulator.
 * The RTL machine acts as a client: soc-simulator listens on TCP,
 * QEMU connects to it.
 */
typedef struct RTLSockState {
    int conn_fd;        /* connected socket */
    bool connected;     /* true when connected to soc-simulator */

    /* CPU info from HELLO message */
    uint32_t num_cores;
    uint32_t xlen;
    uint32_t num_irq_lines;
    char isa_string[64];

    /* Capability flags and memory mode from HELLO */
    uint32_t flags;           /* bitmask of rtl_cap_flags */
    uint32_t memory_mode;     /* enum rtl_memory_mode (negotiated) */

    /* Address ranges from HELLO */
    uint64_t mmio_base;
    uint64_t mmio_size;
    uint64_t dram_base;
    uint64_t dram_size;
} RTLSockState;

struct RTLMachineState {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    RTLSockState sock;
    char *socket_path;

    /* MMIO proxy memory region */
    MemoryRegion mmio_proxy;

    /* DRAM backing region (used in QEMU memory mode) */
    MemoryRegion dram_region;
    bool dram_region_created;

    /* DMA proxy region (used in local memory mode with coherent DMA) */
    MemoryRegion dma_proxy;
    bool dma_proxy_created;

    /* IRQ tracking */
    uint64_t irq_levels;

    /* Request ID counters */
    uint32_t mem_req_id;
    uint32_t dma_req_id;

    /* RTL CPU state */
    bool rtl_cpu_started;
};

/* Memory map for peripherals provided by QEMU in the MMIO region */
enum {
    RTL_UART0,
    RTL_VIRTIO,
};

/* Default addresses within the MMIO region (0x60000000 base) */
#define RTL_UART0_ADDR      0x60100000ULL
#define RTL_UART0_SIZE      0x100ULL
#define RTL_UART0_IRQ       10

#define RTL_VIRTIO_ADDR     0x60200000ULL
#define RTL_VIRTIO_SIZE     0x1000ULL
#define RTL_VIRTIO_COUNT    8
#define RTL_VIRTIO_IRQ      1  /* 1 to 8 */

#endif /* HW_RISCV_RTL_H */

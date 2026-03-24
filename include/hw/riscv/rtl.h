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
    uint32_t dmi_req_id;

    /* RTL CPU state */
    bool rtl_cpu_started;

    /* GDB RSP server state */
    char *gdb_addr;          /* GDB listen address (host:port), NULL = disabled */
    int gdb_listen_fd;       /* GDB listening socket */
    int gdb_fd;              /* GDB connected client socket */
    bool gdb_connected;
    bool gdb_noack;          /* GDB no-ack mode */
    uint8_t gdb_rxbuf[4096]; /* receive buffer */
    size_t gdb_rxlen;        /* bytes in receive buffer */
    QEMUTimer *gdb_poll_timer;
    bool gdb_target_running;
    bool gdb_saved_dcsr_valid;
    uint64_t gdb_saved_dcsr;

    /* DMI response tracking for synchronous DMI from GDB context */
    bool dmi_resp_pending;
    uint32_t dmi_resp_req_id;
    uint32_t dmi_resp_data;
    uint32_t dmi_resp_status;

    /* DMA read response tracking for synchronous DMA proxy reads */
    bool dma_read_resp_pending;
    uint32_t dma_read_resp_req_id;
    uint32_t dma_read_resp_status;
    uint32_t dma_read_resp_size;
    uint8_t dma_read_resp_data[8];

    /* DMA write response tracking for synchronous DMA proxy writes */
    bool dma_write_resp_pending;
    uint32_t dma_write_resp_req_id;
    uint32_t dma_write_resp_status;
};

/* Memory map for peripherals provided by QEMU in the MMIO region */
enum {
    RTL_UART0,
    RTL_VIRTIO,
};

/* Default addresses within the MMIO region (0x60000000 base) */
#define RTL_UART0_ADDR      0x60100000ULL
#define RTL_UART0_SIZE      0x100ULL
#define RTL_UART0_IRQ       0  /* bit 0 of irq_levels -> PLIC IRQ 1 */

#define RTL_VIRTIO_ADDR     0x60200000ULL
#define RTL_VIRTIO_SIZE     0x1000ULL
#define RTL_VIRTIO_COUNT    8
#define RTL_VIRTIO_IRQ      1  /* bit 1+ of irq_levels -> PLIC IRQ 2+ */

/*
 * Rocket-Chip internal device addresses (fixed in RTL design).
 * Must match the Rocket-Chip configuration used to generate the Verilog.
 */
#define RTL_CLINT_ADDR      0x2000000ULL
#define RTL_CLINT_SIZE      0x10000ULL
#define RTL_PLIC_ADDR       0xc000000ULL
#define RTL_PLIC_SIZE       0x4000000ULL
#define RTL_PLIC_NDEV       2
#define RTL_PLIC_MAX_PRIO   3

/*
 * Firmware load offset from DRAM base.
 * A small trampoline at DRAM base sets a1 (FDT address) and jumps here.
 * OpenSBI should be built with FW_TEXT_START = dram_base + RTL_FW_OFFSET.
 */
#define RTL_FW_OFFSET       0x200000ULL

/*
 * Kernel load offset from DRAM base.
 * When -kernel is specified alongside firmware, the kernel Image is
 * loaded at dram_base + RTL_KERNEL_OFFSET and fw_dynamic_info.next_addr
 * is set accordingly.
 */
#define RTL_KERNEL_OFFSET   0x400000ULL

#endif /* HW_RISCV_RTL_H */

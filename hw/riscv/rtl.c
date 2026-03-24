/*
 * QEMU RISC-V RTL Machine
 *
 * Connects to an external RTL CPU simulator (soc-simulator) via TCP socket.
 * soc-simulator listens on a TCP port, QEMU connects to it.
 * QEMU provides peripheral devices (UART, VirtIO, etc.) and the RTL simulator
 * provides the actual CPU core. MMIO accesses from the CPU are forwarded to
 * QEMU, and QEMU can DMA into the RTL simulator's memory and send interrupts.
 *
 * Copyright (c) 2024 qemu-system-rtl contributors
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/char/serial-mm.h"
#include "hw/riscv/rtl.h"
#include "hw/riscv/boot.h"
#include "hw/riscv/boot_opensbi.h"
#include "hw/virtio/virtio-mmio.h"
#include "chardev/char.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "system/device_tree.h"
#include "qom/object.h"

#include <libfdt.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <poll.h>

/*
 * Include the shared protocol header.
 * It's in the top-level protocol/ directory.
 */
#include "hw/riscv/rtl_protocol.h"

/* ==================== Forward declarations ==================== */

static bool G_GNUC_UNUSED rtl_mem_write(RTLMachineState *s, uint64_t addr,
                                         const void *data, uint32_t size);
static bool rtl_cpu_start(RTLMachineState *s, uint64_t start_addr);
static bool rtl_dma_read(RTLMachineState *s, uint64_t addr,
                          void *data, uint32_t size);
static bool rtl_dma_write(RTLMachineState *s, uint64_t addr,
                           const void *data, uint32_t size);
static const MemoryRegionOps rtl_dma_proxy_ops;
static void rtl_handle_debug_dmi_resp(RTLMachineState *s,
                                       const struct rtl_msg_header *hdr);
static bool rtl_dispatch_message(RTLMachineState *s,
                                  const struct rtl_msg_header *hdr);

/* ==================== Socket helpers ==================== */

static bool rtl_send_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    while (len > 0) {
        ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            return false;
        }
        p += n;
        len -= n;
    }
    return true;
}

static bool rtl_recv_all(int fd, void *buf, size_t len)
{
    uint8_t *p = buf;
    while (len > 0) {
        ssize_t n = recv(fd, p, len, 0);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            return false;
        }
        p += n;
        len -= n;
    }
    return true;
}

/* ==================== Protocol message handlers ==================== */

/*
 * Handle MMIO read request from soc-simulator.
 * The RTL CPU is trying to read a peripheral register in QEMU.
 */
static void rtl_handle_mmio_read(RTLMachineState *s,
                                  const struct rtl_msg_header *hdr)
{
    struct rtl_msg_mmio_read req;
    struct rtl_msg_mmio_read_resp resp;
    MemTxResult result;

    req.hdr = *hdr;
    if (!rtl_recv_all(s->sock.conn_fd, ((uint8_t *)&req) + sizeof(*hdr),
                      sizeof(req) - sizeof(*hdr))) {
        return;
    }

    memset(&resp, 0, sizeof(resp));
    resp.hdr.type = RTL_MSG_MMIO_READ_RESP;
    resp.hdr.length = sizeof(resp);
    resp.req_id = req.req_id;

    /* Perform the MMIO read through QEMU's memory system */
    resp.data = 0;
    result = address_space_rw(&address_space_memory, req.addr,
                              MEMTXATTRS_UNSPECIFIED,
                              &resp.data, req.size, false);
    resp.status = (result == MEMTX_OK) ? 0 : 1;

    rtl_send_all(s->sock.conn_fd, &resp, sizeof(resp));
}

/*
 * Handle MMIO write request from soc-simulator.
 */
static void rtl_handle_mmio_write(RTLMachineState *s,
                                   const struct rtl_msg_header *hdr)
{
    struct rtl_msg_mmio_write req;
    struct rtl_msg_mmio_write_resp resp;
    MemTxResult result;

    req.hdr = *hdr;
    if (!rtl_recv_all(s->sock.conn_fd, ((uint8_t *)&req) + sizeof(*hdr),
                      sizeof(req) - sizeof(*hdr))) {
        return;
    }

    memset(&resp, 0, sizeof(resp));
    resp.hdr.type = RTL_MSG_MMIO_WRITE_RESP;
    resp.hdr.length = sizeof(resp);
    resp.req_id = req.req_id;

    /* Perform the MMIO write through QEMU's memory system */
    result = address_space_rw(&address_space_memory, req.addr,
                              MEMTXATTRS_UNSPECIFIED,
                              &req.data, req.size, true);
    resp.status = (result == MEMTX_OK) ? 0 : 1;

    rtl_send_all(s->sock.conn_fd, &resp, sizeof(resp));
}

/*
 * Handle DMA read response from soc-simulator.
 */
static void rtl_handle_dma_read_resp(RTLMachineState *s,
                                      const struct rtl_msg_header *hdr)
{
    struct rtl_msg_dma_read_resp resp_hdr;
    resp_hdr.hdr = *hdr;
    if (!rtl_recv_all(s->sock.conn_fd,
                      ((uint8_t *)&resp_hdr) + sizeof(*hdr),
                      sizeof(resp_hdr) - sizeof(*hdr))) {
        return;
    }

    /* Read data payload */
    uint32_t data_size = hdr->length - sizeof(resp_hdr);
    if (data_size > 0 && data_size <= RTL_MAX_DMA_SIZE) {
        uint8_t *buf = g_malloc(data_size);
        if (rtl_recv_all(s->sock.conn_fd, buf, data_size)) {
            if (s->dma_read_resp_pending &&
                resp_hdr.req_id == s->dma_read_resp_req_id) {
                s->dma_read_resp_status = resp_hdr.status;
                s->dma_read_resp_size = MIN(data_size,
                                            (uint32_t)sizeof(s->dma_read_resp_data));
                memcpy(s->dma_read_resp_data, buf, s->dma_read_resp_size);
                s->dma_read_resp_pending = false;
            }
            if (resp_hdr.status == 0) {
                info_report("rtl: DMA_READ_RESP req_id=%u size=%u OK",
                            resp_hdr.req_id, data_size);
            } else {
                warn_report("rtl: DMA_READ_RESP req_id=%u status=%u",
                            resp_hdr.req_id, resp_hdr.status);
            }
        }
        g_free(buf);
    }
}

/*
 * Handle DMA write response from soc-simulator.
 */
static void rtl_handle_dma_write_resp(RTLMachineState *s,
                                       const struct rtl_msg_header *hdr)
{
    struct rtl_msg_dma_write_resp resp;
    resp.hdr = *hdr;
    rtl_recv_all(s->sock.conn_fd,
                 ((uint8_t *)&resp) + sizeof(*hdr),
                 sizeof(resp) - sizeof(*hdr));
    if (s->dma_write_resp_pending &&
        resp.req_id == s->dma_write_resp_req_id) {
        s->dma_write_resp_status = resp.status;
        s->dma_write_resp_pending = false;
    }
    if (resp.status != 0) {
        warn_report("rtl: DMA_WRITE_RESP req_id=%u failed status=%u",
                    resp.req_id, resp.status);
    }
}

/*
 * Handle MEM_READ_RESP: response to our system memory read request.
 */
static void rtl_handle_mem_read_resp(RTLMachineState *s,
                                      const struct rtl_msg_header *hdr)
{
    struct rtl_msg_mem_read_resp resp_hdr;
    resp_hdr.hdr = *hdr;
    if (!rtl_recv_all(s->sock.conn_fd,
                      ((uint8_t *)&resp_hdr) + sizeof(*hdr),
                      sizeof(resp_hdr) - sizeof(*hdr))) {
        return;
    }

    uint32_t data_size = hdr->length - sizeof(resp_hdr);
    if (data_size > 0 && data_size <= RTL_MAX_DMA_SIZE) {
        uint8_t *buf = g_malloc(data_size);
        if (rtl_recv_all(s->sock.conn_fd, buf, data_size)) {
            /* MEM_READ response received */
        }
        g_free(buf);
    }
}

/*
 * Handle MEM_WRITE_RESP: response to our system memory write request.
 */
static void rtl_handle_mem_write_resp(RTLMachineState *s,
                                       const struct rtl_msg_header *hdr)
{
    struct rtl_msg_mem_write_resp resp;
    resp.hdr = *hdr;
    rtl_recv_all(s->sock.conn_fd,
                 ((uint8_t *)&resp) + sizeof(*hdr),
                 sizeof(resp) - sizeof(*hdr));
}

/*
 * Handle CPU_START_ACK: acknowledgement that CPU has started.
 */
static void rtl_handle_cpu_start_ack(RTLMachineState *s,
                                      const struct rtl_msg_header *hdr)
{
    struct rtl_msg_cpu_start_ack ack;
    ack.hdr = *hdr;
    rtl_recv_all(s->sock.conn_fd,
                 ((uint8_t *)&ack) + sizeof(*hdr),
                 sizeof(ack) - sizeof(*hdr));

    if (ack.status == 0) {
        s->rtl_cpu_started = true;
        info_report("rtl: CPU started");
    } else {
        error_report("rtl: CPU_START_ACK failed (status=%u)", ack.status);
    }
}

/*
 * Handle CPU_STOP_ACK.
 */
static void rtl_handle_cpu_stop_ack(RTLMachineState *s,
                                     const struct rtl_msg_header *hdr)
{
    struct rtl_msg_cpu_stop_ack ack;
    ack.hdr = *hdr;
    rtl_recv_all(s->sock.conn_fd,
                 ((uint8_t *)&ack) + sizeof(*hdr),
                 sizeof(ack) - sizeof(*hdr));

    if (ack.status == 0) {
        s->rtl_cpu_started = false;
        info_report("rtl: CPU stopped");
    }
}

/*
 * Handle CPU_STATUS_RESP.
 */
static void rtl_handle_cpu_status_resp(RTLMachineState *s,
                                        const struct rtl_msg_header *hdr)
{
    struct rtl_msg_cpu_status_resp resp;
    resp.hdr = *hdr;
    rtl_recv_all(s->sock.conn_fd,
                 ((uint8_t *)&resp) + sizeof(*hdr),
                 sizeof(resp) - sizeof(*hdr));

    info_report("rtl: CPU status=%u tick=%lu",
                resp.status, (unsigned long)resp.tick_count);
}

/*
 * Handle sync message from soc-simulator.
 */
static void rtl_handle_sync(RTLMachineState *s,
                             const struct rtl_msg_header *hdr)
{
    struct rtl_msg_sync msg;
    struct rtl_msg_sync_ack ack;

    msg.hdr = *hdr;
    if (!rtl_recv_all(s->sock.conn_fd,
                      ((uint8_t *)&msg) + sizeof(*hdr),
                      sizeof(msg) - sizeof(*hdr))) {
        return;
    }

    memset(&ack, 0, sizeof(ack));
    ack.hdr.type = RTL_MSG_SYNC_ACK;
    ack.hdr.length = sizeof(ack);
    ack.tick_count = msg.tick_count;

    rtl_send_all(s->sock.conn_fd, &ack, sizeof(ack));
}

/* ==================== CPU memory access handlers (QEMU memory mode) ====== */

/*
 * Handle CPU_MEM_READ: the RTL CPU is reading from QEMU-hosted memory.
 * Only valid in RTL_MEMMODE_QEMU.
 */
static void rtl_handle_cpu_mem_read(RTLMachineState *s,
                                     const struct rtl_msg_header *hdr)
{
    struct rtl_msg_cpu_mem_read req;
    req.hdr = *hdr;
    if (!rtl_recv_all(s->sock.conn_fd, ((uint8_t *)&req) + sizeof(*hdr),
                      sizeof(req) - sizeof(*hdr))) {
        return;
    }

    uint32_t resp_size = rtl_cpu_mem_read_resp_size(req.size);
    uint8_t *buf = g_malloc0(resp_size);
    struct rtl_msg_cpu_mem_read_resp *resp =
        (struct rtl_msg_cpu_mem_read_resp *)buf;

    resp->hdr.type = RTL_MSG_CPU_MEM_READ_RESP;
    resp->hdr.length = resp_size;
    resp->req_id = req.req_id;

    MemTxResult result = address_space_rw(&address_space_memory, req.addr,
                                           MEMTXATTRS_UNSPECIFIED,
                                           resp->data, req.size, false);
    resp->status = (result == MEMTX_OK) ? 0 : 1;

    rtl_send_all(s->sock.conn_fd, buf, resp_size);
    g_free(buf);
}

/*
 * Handle CPU_MEM_WRITE: the RTL CPU is writing to QEMU-hosted memory.
 * Only valid in RTL_MEMMODE_QEMU.
 */
static void rtl_handle_cpu_mem_write(RTLMachineState *s,
                                      const struct rtl_msg_header *hdr)
{
    struct rtl_msg_cpu_mem_write req_hdr;
    req_hdr.hdr = *hdr;
    if (!rtl_recv_all(s->sock.conn_fd, ((uint8_t *)&req_hdr) + sizeof(*hdr),
                      sizeof(req_hdr) - sizeof(*hdr))) {
        return;
    }

    uint8_t *data = NULL;
    if (req_hdr.size > 0 && req_hdr.size <= RTL_MAX_DMA_SIZE) {
        data = g_malloc(req_hdr.size);
        if (!rtl_recv_all(s->sock.conn_fd, data, req_hdr.size)) {
            g_free(data);
            return;
        }
    }

    struct rtl_msg_cpu_mem_write_resp resp;
    memset(&resp, 0, sizeof(resp));
    resp.hdr.type = RTL_MSG_CPU_MEM_WRITE_RESP;
    resp.hdr.length = sizeof(resp);
    resp.req_id = req_hdr.req_id;

    if (data) {
        MemTxResult result = address_space_rw(&address_space_memory,
                                               req_hdr.addr,
                                               MEMTXATTRS_UNSPECIFIED,
                                               data, req_hdr.size, true);
        resp.status = (result == MEMTX_OK) ? 0 : 1;
    } else {
        resp.status = 1;
    }

    rtl_send_all(s->sock.conn_fd, &resp, sizeof(resp));
    g_free(data);
}

/* ==================== Socket I/O handler ==================== */

/*
 * Dispatch a single protocol message.
 * Returns true on success, false if the connection was lost / should close.
 */
static bool rtl_dispatch_message(RTLMachineState *s,
                                  const struct rtl_msg_header *hdr)
{
    switch (hdr->type) {
    case RTL_MSG_MMIO_READ:
        rtl_handle_mmio_read(s, hdr);
        break;
    case RTL_MSG_MMIO_WRITE:
        rtl_handle_mmio_write(s, hdr);
        break;
    case RTL_MSG_DMA_READ_RESP:
        rtl_handle_dma_read_resp(s, hdr);
        break;
    case RTL_MSG_DMA_WRITE_RESP:
        rtl_handle_dma_write_resp(s, hdr);
        break;
    case RTL_MSG_MEM_READ_RESP:
        rtl_handle_mem_read_resp(s, hdr);
        break;
    case RTL_MSG_MEM_WRITE_RESP:
        rtl_handle_mem_write_resp(s, hdr);
        break;
    case RTL_MSG_CPU_START_ACK:
        rtl_handle_cpu_start_ack(s, hdr);
        break;
    case RTL_MSG_CPU_STOP_ACK:
        rtl_handle_cpu_stop_ack(s, hdr);
        break;
    case RTL_MSG_CPU_STATUS_RESP:
        rtl_handle_cpu_status_resp(s, hdr);
        break;
    case RTL_MSG_SYNC:
        rtl_handle_sync(s, hdr);
        break;
    case RTL_MSG_CPU_MEM_READ:
        rtl_handle_cpu_mem_read(s, hdr);
        break;
    case RTL_MSG_CPU_MEM_WRITE:
        rtl_handle_cpu_mem_write(s, hdr);
        break;
    case RTL_MSG_SHUTDOWN:
        info_report("rtl: soc-simulator sent shutdown");
        qemu_set_fd_handler(s->sock.conn_fd, NULL, NULL, NULL);
        close(s->sock.conn_fd);
        s->sock.conn_fd = -1;
        s->sock.connected = false;
        return false;
    case RTL_MSG_DEBUG_DMI_RESP:
        rtl_handle_debug_dmi_resp(s, hdr);
        break;
    default:
        warn_report("rtl: unknown message type 0x%x", hdr->type);
        if (hdr->length > sizeof(*hdr)) {
            size_t remaining = hdr->length - sizeof(*hdr);
            uint8_t skip[256];
            while (remaining > 0) {
                size_t chunk = MIN(remaining, sizeof(skip));
                if (!rtl_recv_all(s->sock.conn_fd, skip, chunk)) {
                    return false;
                }
                remaining -= chunk;
            }
        }
        break;
    }
    return true;
}

/*
 * Called by QEMU's main loop when data is available on the socket.
 */
static void rtl_sock_read_handler(void *opaque)
{
    RTLMachineState *s = opaque;
    struct rtl_msg_header hdr;

    if (!rtl_recv_all(s->sock.conn_fd, &hdr, sizeof(hdr))) {
        error_report("rtl: connection to soc-simulator lost");
        qemu_set_fd_handler(s->sock.conn_fd, NULL, NULL, NULL);
        close(s->sock.conn_fd);
        s->sock.conn_fd = -1;
        s->sock.connected = false;
        return;
    }

    rtl_dispatch_message(s, &hdr);
}

/* ==================== Connection acceptance ==================== */

/*
 * Handle HELLO message and complete the handshake.
 */
static bool rtl_handle_hello(RTLMachineState *s)
{
    struct rtl_msg_hello hello;
    struct rtl_msg_hello_ack ack;

    if (!rtl_recv_all(s->sock.conn_fd, &hello, sizeof(hello))) {
        error_report("rtl: failed to receive HELLO");
        return false;
    }

    if (hello.hdr.type != RTL_MSG_HELLO) {
        error_report("rtl: expected HELLO, got type 0x%x", hello.hdr.type);
        return false;
    }

    if (hello.magic != RTL_PROTOCOL_MAGIC) {
        error_report("rtl: bad protocol magic");
        return false;
    }

    if (hello.version != RTL_PROTOCOL_VERSION) {
        error_report("rtl: protocol version mismatch (got %u, expected %u)",
                     hello.version, RTL_PROTOCOL_VERSION);
        return false;
    }

    /* Store CPU info */
    s->sock.num_cores = hello.num_cores;
    s->sock.xlen = hello.xlen;
    s->sock.num_irq_lines = hello.num_irq_lines;
    memcpy(s->sock.isa_string, hello.isa_string, sizeof(s->sock.isa_string));
    s->sock.flags = hello.flags;
    s->sock.memory_mode = hello.memory_mode;

    /* Parse address ranges */
    for (uint32_t i = 0; i < hello.num_addr_ranges && i < RTL_MAX_ADDR_RANGES; i++) {
        if (hello.addr_ranges[i].type == 0) {
            /* Memory */
            s->sock.dram_base = hello.addr_ranges[i].base;
            s->sock.dram_size = hello.addr_ranges[i].size;
        } else if (hello.addr_ranges[i].type == 1) {
            /* MMIO */
            s->sock.mmio_base = hello.addr_ranges[i].base;
            s->sock.mmio_size = hello.addr_ranges[i].size;
        }
    }

    info_report("rtl: connected to soc-simulator");
    info_report("rtl:   cores=%u xlen=%u isa=%s irqs=%u",
                s->sock.num_cores, s->sock.xlen,
                s->sock.isa_string, s->sock.num_irq_lines);
    info_report("rtl:   dram=0x%lx+0x%lx  mmio=0x%lx+0x%lx",
                (unsigned long)s->sock.dram_base,
                (unsigned long)s->sock.dram_size,
                (unsigned long)s->sock.mmio_base,
                (unsigned long)s->sock.mmio_size);
    info_report("rtl:   flags=0x%x  memory_mode=%s  coherent_dma=%s",
                s->sock.flags,
                s->sock.memory_mode == RTL_MEMMODE_QEMU ? "qemu" : "local",
                (s->sock.flags & RTL_CAP_COHERENT_DMA) ? "yes" : "no");

    /* Send ACK */
    memset(&ack, 0, sizeof(ack));
    ack.hdr.type = RTL_MSG_HELLO_ACK;
    ack.hdr.length = sizeof(ack);
    ack.status = 0;
    ack.memory_mode = s->sock.memory_mode;

    if (!rtl_send_all(s->sock.conn_fd, &ack, sizeof(ack))) {
        error_report("rtl: failed to send HELLO_ACK");
        return false;
    }

    return true;
}

/*
 * Called when a new connection arrives on the listening socket.
 * NOTE: This is no longer used — QEMU now connects to soc-simulator.
 * Kept as a reference only.
 */
#if 0
static void rtl_accept_handler(void *opaque)
{
}
#endif

/*
 * Connect to soc-simulator, perform handshake, set up devices, and start CPU.
 * Called after peripheral devices are created in rtl_machine_init().
 */

/* ==================== FDT generation ==================== */

/*
 * Create a device tree for the RTL machine.
 * Called after HELLO handshake so CPU info from soc-simulator is available.
 * If the user provided -dtb, that file is loaded instead.
 */
static void rtl_create_fdt(RTLMachineState *s)
{
    MachineState *ms = MACHINE(s);
    void *fdt;
    uint32_t cpu_intc_phandle, plic_phandle;
    char *name;
    int fdt_alloc_size;

    if (ms->dtb) {
        int fdt_size;
        fdt = load_device_tree(ms->dtb, &fdt_size);
        if (!fdt) {
            error_report("rtl: failed to load DTB '%s'", ms->dtb);
            exit(1);
        }
        ms->fdt = fdt;
        return;
    }

    fdt = create_device_tree(&fdt_alloc_size);
    ms->fdt = fdt;

    /* Root */
    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 2);
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 2);
    qemu_fdt_setprop_string(fdt, "/", "compatible",
                            "freechips,rocketchip-unknown-dev");
    qemu_fdt_setprop_string(fdt, "/", "model",
                            "freechips,rocketchip-unknown");

    /* /chosen */
    qemu_fdt_add_subnode(fdt, "/chosen");
    if (ms->kernel_cmdline && ms->kernel_cmdline[0]) {
        qemu_fdt_setprop_string(fdt, "/chosen", "bootargs",
                                ms->kernel_cmdline);
    }

    /* /cpus */
    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0);
    qemu_fdt_setprop_cell(fdt, "/cpus", "timebase-frequency", 1000000);

    /* CPU node(s) */
    cpu_intc_phandle = qemu_fdt_alloc_phandle(fdt);
    for (uint32_t i = 0; i < s->sock.num_cores; i++) {
        char *intc_name;

        name = g_strdup_printf("/cpus/cpu@%u", i);
        qemu_fdt_add_subnode(fdt, name);
        qemu_fdt_setprop_cell(fdt, name, "clock-frequency", 0);
        qemu_fdt_setprop_string(fdt, name, "compatible", "riscv");
        qemu_fdt_setprop_string(fdt, name, "device_type", "cpu");
        qemu_fdt_setprop_cell(fdt, name, "reg", i);
        qemu_fdt_setprop_string(fdt, name, "riscv,isa", s->sock.isa_string);
        if (s->sock.xlen == 64) {
            qemu_fdt_setprop_string(fdt, name, "mmu-type", "riscv,sv39");
        } else {
            qemu_fdt_setprop_string(fdt, name, "mmu-type", "riscv,sv32");
        }
        qemu_fdt_setprop_string(fdt, name, "status", "okay");

        intc_name = g_strdup_printf("%s/interrupt-controller", name);
        qemu_fdt_add_subnode(fdt, intc_name);
        qemu_fdt_setprop_cell(fdt, intc_name, "#interrupt-cells", 1);
        qemu_fdt_setprop_string(fdt, intc_name, "compatible",
                                "riscv,cpu-intc");
        qemu_fdt_setprop(fdt, intc_name, "interrupt-controller", NULL, 0);
        qemu_fdt_setprop_cell(fdt, intc_name, "phandle", cpu_intc_phandle);
        g_free(intc_name);
        g_free(name);
    }

    /* /memory */
    name = g_strdup_printf("/memory@%lx", (unsigned long)s->sock.dram_base);
    qemu_fdt_add_subnode(fdt, name);
    qemu_fdt_setprop_string(fdt, name, "device_type", "memory");
    qemu_fdt_setprop_sized_cells(fdt, name, "reg",
                                 2, s->sock.dram_base,
                                 2, s->sock.dram_size);
    g_free(name);

    /* /soc */
    qemu_fdt_add_subnode(fdt, "/soc");
    qemu_fdt_setprop_cell(fdt, "/soc", "#address-cells", 2);
    qemu_fdt_setprop_cell(fdt, "/soc", "#size-cells", 2);
    qemu_fdt_setprop_string(fdt, "/soc", "compatible", "simple-bus");
    qemu_fdt_setprop(fdt, "/soc", "ranges", NULL, 0);

    /* CLINT */
    name = g_strdup_printf("/soc/clint@%lx", (unsigned long)RTL_CLINT_ADDR);
    qemu_fdt_add_subnode(fdt, name);
    qemu_fdt_setprop_string(fdt, name, "compatible", "riscv,clint0");
    qemu_fdt_setprop_sized_cells(fdt, name, "reg",
                                 2, RTL_CLINT_ADDR,
                                 2, RTL_CLINT_SIZE);
    qemu_fdt_setprop_cells(fdt, name, "interrupts-extended",
                           cpu_intc_phandle, 3,     /* MSIP */
                           cpu_intc_phandle, 7);    /* MTIP */
    g_free(name);

    /* PLIC */
    plic_phandle = qemu_fdt_alloc_phandle(fdt);
    name = g_strdup_printf("/soc/interrupt-controller@%lx",
                           (unsigned long)RTL_PLIC_ADDR);
    qemu_fdt_add_subnode(fdt, name);
    qemu_fdt_setprop_cell(fdt, name, "#interrupt-cells", 1);
    qemu_fdt_setprop_string(fdt, name, "compatible", "sifive,plic-1.0.0");
    qemu_fdt_setprop(fdt, name, "interrupt-controller", NULL, 0);
    qemu_fdt_setprop_sized_cells(fdt, name, "reg",
                                 2, RTL_PLIC_ADDR,
                                 2, RTL_PLIC_SIZE);
    qemu_fdt_setprop_cells(fdt, name, "interrupts-extended",
                           cpu_intc_phandle, 11,    /* M-mode external */
                           cpu_intc_phandle, 9);    /* S-mode external */
    qemu_fdt_setprop_cell(fdt, name, "riscv,max-priority", RTL_PLIC_MAX_PRIO);
    qemu_fdt_setprop_cell(fdt, name, "riscv,ndev", RTL_PLIC_NDEV);
    qemu_fdt_setprop_cell(fdt, name, "phandle", plic_phandle);
    g_free(name);

    /* UART (ns16550a) */
    name = g_strdup_printf("/soc/serial@%lx", (unsigned long)RTL_UART0_ADDR);
    qemu_fdt_add_subnode(fdt, name);
    qemu_fdt_setprop_string(fdt, name, "compatible", "ns16550a");
    qemu_fdt_setprop_sized_cells(fdt, name, "reg",
                                 2, RTL_UART0_ADDR,
                                 2, RTL_UART0_SIZE);
    qemu_fdt_setprop_cell(fdt, name, "reg-shift", 0);
    qemu_fdt_setprop_cell(fdt, name, "clock-frequency", 3686400);
    qemu_fdt_setprop_cell(fdt, name, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop_cell(fdt, name, "interrupts", RTL_UART0_IRQ + 1);
    qemu_fdt_setprop_string(fdt, "/chosen", "stdout-path", name);
    g_free(name);

    /* VirtIO MMIO devices - all share PLIC IRQ 2 (external interrupt bit 1) */
    {
        for (int i = RTL_VIRTIO_COUNT - 1; i >= 0; i--) {
            hwaddr addr = RTL_VIRTIO_ADDR + i * RTL_VIRTIO_SIZE;
            name = g_strdup_printf("/soc/virtio_mmio@%lx",
                                   (unsigned long)addr);
            qemu_fdt_add_subnode(fdt, name);
            qemu_fdt_setprop_string(fdt, name, "compatible", "virtio,mmio");
            qemu_fdt_setprop_sized_cells(fdt, name, "reg",
                                         2, addr,
                                         2, (uint64_t)RTL_VIRTIO_SIZE);
            qemu_fdt_setprop_cell(fdt, name, "interrupt-parent",
                                  plic_phandle);
            /* All VirtIO devices share PLIC source 2 (bit 1) */
            qemu_fdt_setprop_cell(fdt, name, "interrupts",
                                  RTL_VIRTIO_IRQ + 1);
            g_free(name);
        }
    }

    info_report("rtl: FDT generated (%d bytes) - %u cores, %s, "
                "DRAM 0x%lx+0x%lx",
                fdt_totalsize(fdt), s->sock.num_cores, s->sock.isa_string,
                (unsigned long)s->sock.dram_base,
                (unsigned long)s->sock.dram_size);
}

/* ==================== Memory write helper ==================== */

/*
 * Write data to the RTL CPU's memory.
 * Uses address_space_rw in QEMU memory mode, or MEM_WRITE in local mode.
 */
static bool rtl_write_memory(RTLMachineState *s, uint64_t addr,
                              const void *data, size_t size)
{
    const uint8_t *p = data;

    if (s->sock.memory_mode == RTL_MEMMODE_QEMU) {
        while (size > 0) {
            size_t chunk = MIN(size, 4096);
            address_space_rw(&address_space_memory, addr,
                             MEMTXATTRS_UNSPECIFIED,
                             (void *)p, (int)chunk, true);
            addr += chunk;
            p += chunk;
            size -= chunk;
        }
        return true;
    } else {
        while (size > 0) {
            uint32_t chunk = (uint32_t)MIN(size, RTL_MAX_DMA_SIZE);
            if (!rtl_mem_write(s, addr, p, chunk)) {
                return false;
            }
            addr += chunk;
            p += chunk;
            size -= chunk;
        }
        return true;
    }
}

/* ==================== BIOS / firmware loading ==================== */

/*
 * Load firmware with a boot trampoline and FDT.
 *
 * Firmware resolution uses the same logic as the virt machine:
 *   - No -bios:           load built-in OpenSBI fw_dynamic (default)
 *   - "-bios default":    same as above
 *   - "-bios none":       skip firmware loading
 *   - "-bios <file>":     load user-specified firmware
 *
 * Memory layout:
 *   dram_base + 0x0:       trampoline (sets a1=FDT, a2=fw_dyn_info, jumps)
 *   dram_base + 0x1000:    fw_dynamic_info struct
 *   dram_base + FW_OFFSET: firmware binary
 *   fdt_addr:              device tree blob (near end of DRAM)
 *
 * The Rocket-Chip bootrom jumps to dram_base with a0=mhartid.
 * The trampoline overrides a1 (FDT address), sets a2 to point at
 * fw_dynamic_info (for fw_dynamic), and jumps to the firmware.
 */
static void rtl_load_bios(RTLMachineState *s, const char *firmware_path)
{
    MachineState *ms = MACHINE(s);
    uint64_t dram_base = s->sock.dram_base;
    uint64_t dram_size = s->sock.dram_size;
    uint64_t fw_addr = dram_base + RTL_FW_OFFSET;
    uint64_t fdt_addr;
    uint64_t fwdyn_addr = dram_base + 0x1000; /* fw_dynamic_info location */
    int fdt_size;

    /* Generate FDT (or load user-provided DTB via -dtb) */
    rtl_create_fdt(s);
    fdt_pack(ms->fdt);
    fdt_size = fdt_totalsize(ms->fdt);

    /* Place FDT near end of DRAM, aligned to 2MB */
    fdt_addr = (dram_base + dram_size - fdt_size) & ~(0x200000ULL - 1);

    info_report("rtl: firmware at 0x%lx, FDT at 0x%lx (%d bytes)",
                (unsigned long)fw_addr, (unsigned long)fdt_addr, fdt_size);

    /*
     * Build boot trampoline (48 bytes = 6 insns + 3 dwords):
     *   auipc  t0, 0           ; t0 = PC (dram_base)
     *   ld     a1, 24(t0)      ; a1 = fdt_addr
     *   ld     a2, 32(t0)      ; a2 = fwdyn_addr  (fw_dynamic_info ptr)
     *   ld     t0, 40(t0)      ; t0 = fw_addr
     *   jr     t0              ; jump to firmware
     *   nop                    ; padding for alignment
     *   .dword fdt_addr        ; offset 24
     *   .dword fwdyn_addr      ; offset 32
     *   .dword fw_addr         ; offset 40
     */
    uint32_t trampoline[12] = {
        cpu_to_le32(0x00000297),                  /* auipc  t0, 0        */
        cpu_to_le32(0x0182b583),                  /* ld     a1, 24(t0)   */
        cpu_to_le32(0x0202b603),                  /* ld     a2, 32(t0)   */
        cpu_to_le32(0x0282b283),                  /* ld     t0, 40(t0)   */
        cpu_to_le32(0x00028067),                  /* jr     t0           */
        cpu_to_le32(0x00000013),                  /* nop                 */
        cpu_to_le32((uint32_t)(fdt_addr)),        /* fdt_addr low        */
        cpu_to_le32((uint32_t)(fdt_addr >> 32)),  /* fdt_addr high       */
        cpu_to_le32((uint32_t)(fwdyn_addr)),      /* fwdyn_addr low      */
        cpu_to_le32((uint32_t)(fwdyn_addr >> 32)),/* fwdyn_addr high     */
        cpu_to_le32((uint32_t)(fw_addr)),         /* fw_addr low         */
        cpu_to_le32((uint32_t)(fw_addr >> 32)),   /* fw_addr high        */
    };

    /* Write trampoline at DRAM base */
    if (!rtl_write_memory(s, dram_base, trampoline, sizeof(trampoline))) {
        error_report("rtl: failed to write boot trampoline");
        return;
    }

    /*
     * Write fw_dynamic_info struct at fwdyn_addr.
     * This tells OpenSBI fw_dynamic where to jump next (kernel).
     * If no -kernel is specified, next_addr = 0 (OpenSBI will hang
     * after init, which is fine for firmware-only testing).
     */
    {
        uint64_t kernel_entry = 0;
        uint64_t kernel_addr = dram_base + RTL_KERNEL_OFFSET;

        if (ms->kernel_filename) {
            kernel_entry = kernel_addr;
        }

        struct fw_dynamic_info64 fwdyn;
        memset(&fwdyn, 0, sizeof(fwdyn));
        fwdyn.magic     = cpu_to_le64(FW_DYNAMIC_INFO_MAGIC_VALUE);
        fwdyn.version   = cpu_to_le64(FW_DYNAMIC_INFO_VERSION);
        fwdyn.next_addr = cpu_to_le64(kernel_entry);
        fwdyn.next_mode = cpu_to_le64(FW_DYNAMIC_INFO_NEXT_MODE_S);
        fwdyn.options   = 0;
        fwdyn.boot_hart = 0;

        if (!rtl_write_memory(s, fwdyn_addr, &fwdyn, sizeof(fwdyn))) {
            error_report("rtl: failed to write fw_dynamic_info");
            return;
        }

        if (kernel_entry) {
            info_report("rtl: fw_dynamic next_addr = 0x%lx (S-mode kernel)",
                        (unsigned long)kernel_entry);
        }
    }

    /* Load firmware binary at dram_base + FW_OFFSET */
    {
        FILE *fp = fopen(firmware_path, "rb");
        if (!fp) {
            error_report("rtl: failed to open firmware '%s': %s",
                         firmware_path, strerror(errno));
            return;
        }

        uint8_t chunk[4096];
        uint64_t write_addr = fw_addr;
        size_t total = 0;
        size_t n;

        while ((n = fread(chunk, 1, sizeof(chunk), fp)) > 0) {
            if (!rtl_write_memory(s, write_addr, chunk, n)) {
                error_report("rtl: failed to write firmware at 0x%lx",
                             (unsigned long)write_addr);
                fclose(fp);
                return;
            }
            write_addr += n;
            total += n;
        }
        fclose(fp);

        info_report("rtl: loaded firmware '%s' (%zu bytes) at 0x%lx",
                    firmware_path, total, (unsigned long)fw_addr);
    }

    /* Load kernel Image at dram_base + KERNEL_OFFSET if -kernel is specified */
    if (ms->kernel_filename) {
        uint64_t kernel_addr = dram_base + RTL_KERNEL_OFFSET;
        FILE *fp = fopen(ms->kernel_filename, "rb");
        if (!fp) {
            error_report("rtl: failed to open kernel '%s': %s",
                         ms->kernel_filename, strerror(errno));
            return;
        }

        uint8_t chunk[4096];
        uint64_t write_addr = kernel_addr;
        size_t total = 0;
        size_t n;

        while ((n = fread(chunk, 1, sizeof(chunk), fp)) > 0) {
            if (!rtl_write_memory(s, write_addr, chunk, n)) {
                error_report("rtl: failed to write kernel at 0x%lx",
                             (unsigned long)write_addr);
                fclose(fp);
                return;
            }
            write_addr += n;
            total += n;
        }
        fclose(fp);

        info_report("rtl: loaded kernel '%s' (%zu bytes) at 0x%lx",
                    ms->kernel_filename, total, (unsigned long)kernel_addr);
    }

    /* Write FDT at computed address */
    if (!rtl_write_memory(s, fdt_addr, ms->fdt, fdt_size)) {
        error_report("rtl: failed to write FDT at 0x%lx",
                     (unsigned long)fdt_addr);
        return;
    }

    info_report("rtl: boot: trampoline@0x%lx -> fw@0x%lx, fdt@0x%lx",
                (unsigned long)dram_base, (unsigned long)fw_addr,
                (unsigned long)fdt_addr);
}

/* ==================== Connection and initialization ==================== */
static void rtl_connect_and_init(RTLMachineState *s)
{
    /* Parse host:port from socket_path */
    char host[256] = "127.0.0.1";
    uint16_t port = 2345;
    const char *colon = strrchr(s->socket_path, ':');
    if (colon) {
        size_t hlen = colon - s->socket_path;
        if (hlen >= sizeof(host)) hlen = sizeof(host) - 1;
        memcpy(host, s->socket_path, hlen);
        host[hlen] = '\0';
        port = (uint16_t)atoi(colon + 1);
    } else {
        port = (uint16_t)atoi(s->socket_path);
    }

    s->sock.conn_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (s->sock.conn_fd < 0) {
        error_report("rtl: socket() failed: %s", strerror(errno));
        exit(1);
    }

    struct sockaddr_in saddr;
    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &saddr.sin_addr) <= 0) {
        error_report("rtl: invalid address '%s'", host);
        exit(1);
    }

    info_report("rtl: connecting to soc-simulator at %s:%u...", host, port);

    if (connect(s->sock.conn_fd, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
        error_report("rtl: connect(%s:%u) failed: %s",
                     host, port, strerror(errno));
        exit(1);
    }

    /* Disable Nagle's algorithm for low latency */
    int flag = 1;
    setsockopt(s->sock.conn_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    info_report("rtl: connected to soc-simulator, waiting for HELLO...");

    /* Perform handshake (blocking - acceptable during setup) */
    if (!rtl_handle_hello(s)) {
        close(s->sock.conn_fd);
        s->sock.conn_fd = -1;
        error_report("rtl: handshake failed");
        exit(1);
    }

    s->sock.connected = true;

    /*
     * In QEMU memory mode, create a RAM MemoryRegion for the DRAM
     * address range.  This is the memory that the CPU will read/write
     * via CPU_MEM_READ/WRITE messages, and that QEMU devices can DMA
     * to directly.
     */
    if (s->sock.memory_mode == RTL_MEMMODE_QEMU &&
        s->sock.dram_size > 0 && !s->dram_region_created) {
        MemoryRegion *system_memory = get_system_memory();
        memory_region_init_ram(&s->dram_region, NULL,
                               "rtl-dram", s->sock.dram_size,
                               &error_fatal);
        memory_region_add_subregion(system_memory, s->sock.dram_base,
                                     &s->dram_region);
        s->dram_region_created = true;
        info_report("rtl: created DRAM region 0x%lx+0x%lx (QEMU memory mode)",
                    (unsigned long)s->sock.dram_base,
                    (unsigned long)s->sock.dram_size);
    }

    /*
     * In local memory mode with coherent DMA, create a DMA proxy region
     * so QEMU devices can DMA to the RTL CPU's cache-coherent memory.
     * The proxy forwards writes via DMA_WRITE through the L2 frontend bus.
     */
    if (s->sock.memory_mode == RTL_MEMMODE_LOCAL &&
        (s->sock.flags & RTL_CAP_COHERENT_DMA) &&
        s->sock.dram_size > 0 && !s->dma_proxy_created) {
        MemoryRegion *system_memory = get_system_memory();
        memory_region_init_io(&s->dma_proxy, NULL,
                              &rtl_dma_proxy_ops, s,
                              "rtl-dma-proxy", s->sock.dram_size);
        memory_region_add_subregion_overlap(system_memory, s->sock.dram_base,
                                            &s->dma_proxy, -1);
        s->dma_proxy_created = true;
        info_report("rtl: DMA proxy at 0x%lx+0x%lx (coherent DMA via L2)",
                    (unsigned long)s->sock.dram_base,
                    (unsigned long)s->sock.dram_size);
    }

    /* Register the fd handler for ongoing communication */
    qemu_set_fd_handler(s->sock.conn_fd, rtl_sock_read_handler, NULL, s);

    /*
     * Load firmware into memory before starting the CPU.
     *
     * Firmware resolution (same as virt machine):
     *   No -bios:        load built-in OpenSBI fw_dynamic (default)
     *   -bios default:   same as above
     *   -bios none:      skip firmware, fall through to -kernel path
     *   -bios <file>:    load user-specified firmware
     *
     * -kernel without firmware: load raw binary at DRAM base.
     */
    MachineState *machine = MACHINE(s);
    char *firmware_path = riscv_find_firmware(machine->firmware,
                                              RISCV64_BIOS_BIN);
    if (firmware_path) {
        rtl_load_bios(s, firmware_path);
        g_free(firmware_path);
    } else if (machine->kernel_filename) {
        if (s->sock.memory_mode == RTL_MEMMODE_QEMU) {
            /* QEMU memory mode: load directly into DRAM via address_space_rw.
             * Cannot use load_image_targphys() here because ROM blobs can only
             * be registered during machine init. */
            FILE *fp = fopen(machine->kernel_filename, "rb");
            if (!fp) {
                error_report("rtl: failed to open kernel '%s': %s",
                             machine->kernel_filename, strerror(errno));
            } else {
                uint8_t chunk[4096];
                uint64_t write_addr = s->sock.dram_base;
                size_t total = 0;
                size_t n;
                while ((n = fread(chunk, 1, sizeof(chunk), fp)) > 0) {
                    address_space_rw(&address_space_memory, write_addr,
                                      MEMTXATTRS_UNSPECIFIED,
                                      chunk, (int)n, true);
                    write_addr += n;
                    total += n;
                }
                fclose(fp);
                info_report("rtl: loaded kernel '%s' (%zu bytes) at 0x%lx",
                            machine->kernel_filename, total,
                            (unsigned long)s->sock.dram_base);
            }
        } else {
            /* Local memory mode: send firmware via MEM_WRITE to soc-sim.
             * Read the file and send it in chunks. */
            FILE *fp = fopen(machine->kernel_filename, "rb");
            if (fp) {
                uint8_t chunk[RTL_MAX_DMA_SIZE];
                uint64_t write_addr = s->sock.dram_base;
                size_t total = 0;
                size_t n;
                while ((n = fread(chunk, 1, sizeof(chunk), fp)) > 0) {
                    if (!rtl_mem_write(s, write_addr, chunk, (uint32_t)n)) {
                        error_report("rtl: MEM_WRITE failed at 0x%lx",
                                     (unsigned long)write_addr);
                        break;
                    }
                    write_addr += n;
                    total += n;
                }
                fclose(fp);
                info_report("rtl: loaded kernel '%s' (%zu bytes) at 0x%lx "
                            "via MEM_WRITE",
                            machine->kernel_filename, total,
                            (unsigned long)s->sock.dram_base);
            } else {
                error_report("rtl: failed to open kernel '%s': %s",
                             machine->kernel_filename, strerror(errno));
            }
        }
    }

    info_report("rtl: sending CPU_START...");
    if (!rtl_cpu_start(s, 0)) {
        error_report("rtl: failed to start CPU");
    }
}

/* ==================== Public API: system memory + CPU control ==================== */

/*
 * Write to soc-simulator's system memory (pre-boot).
 * Safe because CPU is not running — no cache coherency issues.
 */
static bool G_GNUC_UNUSED rtl_mem_write(RTLMachineState *s, uint64_t addr,
                                         const void *data, uint32_t size)
{
    if (!s->sock.connected) return false;

    uint32_t msg_size = rtl_mem_write_size(size);
    uint8_t *buf = g_malloc(msg_size);
    struct rtl_msg_mem_write *msg = (struct rtl_msg_mem_write *)buf;

    memset(msg, 0, sizeof(*msg));
    msg->hdr.type = RTL_MSG_MEM_WRITE;
    msg->hdr.length = msg_size;
    msg->addr = addr;
    msg->size = size;
    msg->req_id = s->mem_req_id++;
    memcpy(msg->data, data, size);

    bool ok = rtl_send_all(s->sock.conn_fd, buf, msg_size);
    g_free(buf);

    if (!ok) return false;

    /* Wait for response */
    struct rtl_msg_mem_write_resp resp;
    if (!rtl_recv_all(s->sock.conn_fd, &resp, sizeof(resp))) {
        return false;
    }

    return (resp.status == 0);
}

/*
 * Read from soc-simulator's system memory (pre-boot).
 */
static bool G_GNUC_UNUSED rtl_mem_read(RTLMachineState *s, uint64_t addr,
                                        void *data, uint32_t size)
{
    if (!s->sock.connected) return false;

    struct rtl_msg_mem_read req;
    memset(&req, 0, sizeof(req));
    req.hdr.type = RTL_MSG_MEM_READ;
    req.hdr.length = sizeof(req);
    req.addr = addr;
    req.size = size;
    req.req_id = s->mem_req_id++;

    if (!rtl_send_all(s->sock.conn_fd, &req, sizeof(req))) {
        return false;
    }

    /* Wait for response header */
    struct rtl_msg_header resp_hdr;
    if (!rtl_recv_all(s->sock.conn_fd, &resp_hdr, sizeof(resp_hdr))) {
        return false;
    }

    struct rtl_msg_mem_read_resp resp_fixed;
    resp_fixed.hdr = resp_hdr;
    if (!rtl_recv_all(s->sock.conn_fd,
                      ((uint8_t *)&resp_fixed) + sizeof(resp_hdr),
                      sizeof(resp_fixed) - sizeof(resp_hdr))) {
        return false;
    }

    if (resp_fixed.status != 0) return false;

    if (!rtl_recv_all(s->sock.conn_fd, data, size)) {
        return false;
    }

    return true;
}

/*
 * Send a DMA write to soc-simulator's memory via the L2 frontend bus.
 * Used by QEMU devices to DMA into the RTL CPU's cache-coherent memory.
 * This blocks by polling the socket directly until the matching response arrives.
 */
static bool rtl_dma_write(RTLMachineState *s, uint64_t addr,
                           const void *data, uint32_t size)
{
    struct pollfd pfd = {
        .fd = s->sock.conn_fd,
        .events = POLLIN,
    };
    int ret;

    if (!s->sock.connected) return false;
    if (size > RTL_MAX_DMA_SIZE) return false;

    uint32_t msg_size = (uint32_t)(sizeof(struct rtl_msg_dma_write) + size);
    uint8_t *buf = g_malloc(msg_size);
    struct rtl_msg_dma_write *msg = (struct rtl_msg_dma_write *)buf;

    memset(msg, 0, sizeof(*msg));
    msg->hdr.type = RTL_MSG_DMA_WRITE;
    msg->hdr.length = msg_size;
    msg->addr = addr;
    msg->size = size;
    msg->req_id = s->dma_req_id++;
    memcpy(msg->data, data, size);

    s->dma_write_resp_pending = true;
    s->dma_write_resp_req_id = msg->req_id;
    s->dma_write_resp_status = 1;

    bool ok = rtl_send_all(s->sock.conn_fd, buf, msg_size);
    g_free(buf);
    if (!ok) {
        s->dma_write_resp_pending = false;
        return false;
    }

    while (s->dma_write_resp_pending && s->sock.connected) {
        ret = poll(&pfd, 1, 30000);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            error_report("rtl: DMA write poll error: %s", strerror(errno));
            s->dma_write_resp_pending = false;
            return false;
        }
        if (ret == 0) {
            warn_report("rtl: DMA write timeout req_id=%u", msg->req_id);
            s->dma_write_resp_pending = false;
            return false;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            error_report("rtl: DMA write socket error");
            s->sock.connected = false;
            s->dma_write_resp_pending = false;
            return false;
        }
        if (pfd.revents & POLLIN) {
            struct rtl_msg_header hdr;
            if (!rtl_recv_all(s->sock.conn_fd, &hdr, sizeof(hdr))) {
                error_report("rtl: connection lost while waiting for DMA write");
                s->sock.connected = false;
                s->dma_write_resp_pending = false;
                return false;
            }
            rtl_dispatch_message(s, &hdr);
        }
    }

    return s->sock.connected && s->dma_write_resp_status == 0;
}

/*
 * Send a DMA read to soc-simulator's memory via the L2 frontend bus.
 * Used by QEMU devices to fetch guest memory (for example, virtio rings).
 * This blocks by polling the socket directly until the matching response arrives.
 */
static bool rtl_dma_read(RTLMachineState *s, uint64_t addr,
                          void *data, uint32_t size)
{
    struct rtl_msg_dma_read req;
    struct pollfd pfd = {
        .fd = s->sock.conn_fd,
        .events = POLLIN,
    };
    int ret;

    if (!s->sock.connected || size > sizeof(s->dma_read_resp_data)) {
        return false;
    }

    memset(&req, 0, sizeof(req));
    req.hdr.type = RTL_MSG_DMA_READ;
    req.hdr.length = sizeof(req);
    req.addr = addr;
    req.size = size;
    req.req_id = s->dma_req_id++;

    s->dma_read_resp_pending = true;
    s->dma_read_resp_req_id = req.req_id;
    s->dma_read_resp_status = 1;
    s->dma_read_resp_size = 0;

    if (!rtl_send_all(s->sock.conn_fd, &req, sizeof(req))) {
        s->dma_read_resp_pending = false;
        return false;
    }

    while (s->dma_read_resp_pending && s->sock.connected) {
        ret = poll(&pfd, 1, 30000);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            error_report("rtl: DMA read poll error: %s", strerror(errno));
            s->dma_read_resp_pending = false;
            return false;
        }
        if (ret == 0) {
            warn_report("rtl: DMA read timeout req_id=%u", req.req_id);
            s->dma_read_resp_pending = false;
            return false;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            error_report("rtl: DMA read socket error");
            s->sock.connected = false;
            s->dma_read_resp_pending = false;
            return false;
        }
        if (pfd.revents & POLLIN) {
            struct rtl_msg_header hdr;
            if (!rtl_recv_all(s->sock.conn_fd, &hdr, sizeof(hdr))) {
                error_report("rtl: connection lost while waiting for DMA read");
                s->sock.connected = false;
                s->dma_read_resp_pending = false;
                return false;
            }
            rtl_dispatch_message(s, &hdr);
        }
    }

    if (!s->sock.connected || s->dma_read_resp_status != 0 ||
        s->dma_read_resp_size < size) {
        return false;
    }

    memcpy(data, s->dma_read_resp_data, size);
    return true;
}

/*
 * DMA proxy MemoryRegion ops for local memory mode.
 * QEMU devices that DMA to the DRAM range hit these ops, which forward
 * the access to soc-simulator via DMA_WRITE / DMA_READ messages.
 * The data goes through the L2 frontend bus for cache coherency.
 */
static MemTxResult rtl_dma_proxy_read_with_attrs(void *opaque, hwaddr addr,
                                                  uint64_t *val, unsigned size,
                                                  MemTxAttrs attrs)
{
    RTLMachineState *s = opaque;
    uint64_t phys = s->sock.dram_base + addr;
    uint8_t data[sizeof(*val)] = {0};

    (void)attrs;
    *val = 0;
    if (!rtl_dma_read(s, phys, data, size)) {
        return MEMTX_ERROR;
    }
    memcpy(val, data, size);
    return MEMTX_OK;
}

static MemTxResult rtl_dma_proxy_write_with_attrs(void *opaque, hwaddr addr,
                                                   uint64_t val, unsigned size,
                                                   MemTxAttrs attrs)
{
    RTLMachineState *s = opaque;
    (void)attrs;
    uint64_t phys = s->sock.dram_base + addr;
    return rtl_dma_write(s, phys, &val, size) ? MEMTX_OK : MEMTX_ERROR;
}

static const MemoryRegionOps rtl_dma_proxy_ops = {
    .read_with_attrs = rtl_dma_proxy_read_with_attrs,
    .write_with_attrs = rtl_dma_proxy_write_with_attrs,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * Send CPU_START to soc-simulator.
 */
static bool rtl_cpu_start(RTLMachineState *s, uint64_t start_addr)
{
    if (!s->sock.connected) return false;

    struct rtl_msg_cpu_start msg;
    memset(&msg, 0, sizeof(msg));
    msg.hdr.type = RTL_MSG_CPU_START;
    msg.hdr.length = sizeof(msg);
    msg.start_addr = start_addr;

    if (!rtl_send_all(s->sock.conn_fd, &msg, sizeof(msg))) {
        return false;
    }

    /* Wait for ack */
    struct rtl_msg_cpu_start_ack ack;
    if (!rtl_recv_all(s->sock.conn_fd, &ack, sizeof(ack))) {
        return false;
    }

    if (ack.status == 0) {
        s->rtl_cpu_started = true;
        return true;
    }
    return false;
}

/*
 * Send CPU_STOP to soc-simulator.
 */
static bool G_GNUC_UNUSED rtl_cpu_stop(RTLMachineState *s, uint32_t reason)
{
    if (!s->sock.connected) return false;

    struct rtl_msg_cpu_stop msg;
    memset(&msg, 0, sizeof(msg));
    msg.hdr.type = RTL_MSG_CPU_STOP;
    msg.hdr.length = sizeof(msg);
    msg.reason = reason;

    if (!rtl_send_all(s->sock.conn_fd, &msg, sizeof(msg))) {
        return false;
    }

    struct rtl_msg_cpu_stop_ack ack;
    if (!rtl_recv_all(s->sock.conn_fd, &ack, sizeof(ack))) {
        return false;
    }

    if (ack.status == 0) {
        s->rtl_cpu_started = false;
        return true;
    }
    return false;
}

/*
 * Send CPU_STATUS query.
 */
static bool G_GNUC_UNUSED rtl_cpu_status(RTLMachineState *s,
                                          uint32_t *status,
                                          uint64_t *tick_count)
{
    if (!s->sock.connected) return false;

    struct rtl_msg_cpu_status msg;
    memset(&msg, 0, sizeof(msg));
    msg.hdr.type = RTL_MSG_CPU_STATUS;
    msg.hdr.length = sizeof(msg);

    if (!rtl_send_all(s->sock.conn_fd, &msg, sizeof(msg))) {
        return false;
    }

    struct rtl_msg_cpu_status_resp resp;
    if (!rtl_recv_all(s->sock.conn_fd, &resp, sizeof(resp))) {
        return false;
    }

    if (status) *status = resp.status;
    if (tick_count) *tick_count = resp.tick_count;
    return true;
}

/*
 * Send an interrupt level update to the soc-simulator.
 * The Rocket-Chip only has 2 external interrupt inputs:
 *   bit 0: UART (PLIC source 1)
 *   bit 1: shared VirtIO (PLIC source 2)
 * We track per-device levels in irq_levels and compute the effective
 * 2-bit value by ORing all VirtIO device lines together.
 */
static void rtl_update_irq(RTLMachineState *s, int irq, int level)
{
    if (!s->sock.connected) return;

    /* Track per-device level */
    if (level) {
        s->irq_levels |= (1ULL << irq);
    } else {
        s->irq_levels &= ~(1ULL << irq);
    }

    /* Compute effective IRQ levels for the 2-bit Rocket-Chip interrupt port:
     *   bit 0 = UART (line 0)
     *   bit 1 = OR of all VirtIO lines (lines 1..8) */
    uint64_t effective = 0;
    if (s->irq_levels & (1ULL << RTL_UART0_IRQ)) {
        effective |= 1;
    }
    uint64_t virtio_mask = ((1ULL << RTL_VIRTIO_COUNT) - 1) << RTL_VIRTIO_IRQ;
    if (s->irq_levels & virtio_mask) {
        effective |= 2;
    }

    struct rtl_msg_irq_update msg;
    memset(&msg, 0, sizeof(msg));
    msg.hdr.type = RTL_MSG_IRQ_UPDATE;
    msg.hdr.length = sizeof(msg);
    msg.irq_levels = effective;

    rtl_send_all(s->sock.conn_fd, &msg, sizeof(msg));
}

/* ==================== IRQ handler for QEMU devices ==================== */

/*
 * IRQ handler: called when a QEMU device changes its IRQ line.
 * We forward the change to the soc-simulator.
 */
static void rtl_irq_handler(void *opaque, int n, int level)
{
    RTLMachineState *s = opaque;
    rtl_update_irq(s, n, level);
}

/* ==================== Debug DMI helpers ==================== */

/*
 * Send a DMI request over the protocol to soc-simulator.
 * Returns a req_id that can be matched against the response.
 */
static uint32_t rtl_dmi_send(RTLMachineState *s, uint8_t addr,
                              uint8_t op, uint32_t data)
{
    struct rtl_msg_debug_dmi_req req;
    memset(&req, 0, sizeof(req));
    req.hdr.type = RTL_MSG_DEBUG_DMI_REQ;
    req.hdr.length = sizeof(req);
    req.req_id = s->dmi_req_id++;
    req.addr = addr;
    req.op = op;
    req.data = data;
    rtl_send_all(s->sock.conn_fd, &req, sizeof(req));
    return req.req_id;
}

/*
 * Handle DEBUG_DMI_RESP from soc-simulator.
 */
static void rtl_handle_debug_dmi_resp(RTLMachineState *s,
                                       const struct rtl_msg_header *hdr)
{
    struct rtl_msg_debug_dmi_resp resp;
    resp.hdr = *hdr;
    if (!rtl_recv_all(s->sock.conn_fd,
                      ((uint8_t *)&resp) + sizeof(*hdr),
                      sizeof(resp) - sizeof(*hdr))) {
        return;
    }
    s->dmi_resp_data = resp.data;
    s->dmi_resp_status = resp.status;
    s->dmi_resp_req_id = resp.req_id;
    s->dmi_resp_pending = false;
}

/*
 * Wait for a DMI response by directly polling the soc-simulator socket.
 * This avoids nesting main_loop_wait inside fd callbacks.
 * Returns the response status, or -1 on timeout/error.
 */
static int rtl_dmi_wait_response(RTLMachineState *s, int timeout_ms)
{
    struct pollfd pfd;
    pfd.fd = s->sock.conn_fd;
    pfd.events = POLLIN;

    while (s->dmi_resp_pending && s->sock.connected) {
        int ret = poll(&pfd, 1, timeout_ms);
        if (ret < 0) {
            if (errno == EINTR) continue;
            error_report("rtl-dmi: poll error: %s", strerror(errno));
            return -1;
        }
        if (ret == 0) {
            warn_report("rtl-dmi: timeout waiting for response (%d ms)",
                        timeout_ms);
            return -1;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            error_report("rtl-dmi: socket error during poll");
            s->sock.connected = false;
            return -1;
        }
        if (pfd.revents & POLLIN) {
            struct rtl_msg_header hdr;
            if (!rtl_recv_all(s->sock.conn_fd, &hdr, sizeof(hdr))) {
                error_report("rtl-dmi: connection lost while waiting");
                s->sock.connected = false;
                return -1;
            }
            rtl_dispatch_message(s, &hdr);
        }
    }
    if (!s->sock.connected) return -1;
    return s->dmi_resp_status;
}

/*
 * Perform a synchronous DMI write.
 * Blocks by polling the socket directly until the response arrives.
 */
static int rtl_dmi_write(RTLMachineState *s, uint8_t addr, uint32_t data)
{
    s->dmi_resp_pending = true;
    uint32_t id = rtl_dmi_send(s, addr, 2 /* DMI_OP_WRITE */, data);
    (void)id;
    return rtl_dmi_wait_response(s, 30000);
}

/*
 * Perform a synchronous DMI read.
 * Blocks by polling the socket directly until the response arrives.
 */
static int rtl_dmi_read(RTLMachineState *s, uint8_t addr, uint32_t *out)
{
    s->dmi_resp_pending = true;
    uint32_t id = rtl_dmi_send(s, addr, 1 /* DMI_OP_READ */, 0);
    (void)id;
    int rc = rtl_dmi_wait_response(s, 30000);
    if (out) *out = s->dmi_resp_data;
    return rc;
}

/* DMI register addresses (RISC-V Debug Spec 0.13) */
#define DMI_DATA0       0x04
#define DMI_DATA1       0x05
#define DMI_DMCONTROL   0x10
#define DMI_DMSTATUS    0x11
#define DMI_ABSTRACTCS  0x16
#define DMI_COMMAND     0x17
#define DMI_ABSTRACTAUTO 0x18
#define DMI_SBCS        0x38
#define DMI_SBADDR0     0x39
#define DMI_SBADDR1     0x3A
#define DMI_SBDATA0     0x3C
#define DMI_SBDATA1     0x3D
#define DMI_PROGBUF0    0x20
#define DMI_PROGBUF1    0x21

/* SBCS bits */
#define SBCS_SBACCESS_SHIFT   17
#define SBCS_SBREADONADDR     (1U << 20)
#define SBCS_SBBUSY           (1U << 21)
#define SBCS_SBERROR_MASK     (0x7U << 12)

/* DMCONTROL bits */
#define DMCONTROL_DMACTIVE   (1U << 0)
#define DMCONTROL_HALTREQ    (1U << 31)
#define DMCONTROL_RESUMEREQ  (1U << 30)

/* DMSTATUS bits */
#define DMSTATUS_ALLHALTED    (1U << 9)
#define DMSTATUS_ALLRUNNING   (1U << 11)
#define DMSTATUS_ALLRESUMEACK (1U << 17)

/* ABSTRACTCS bits */
#define ABSTRACTCS_BUSY       (1U << 12)
#define ABSTRACTCS_CMDERR     (0x7U << 8)

/* DCSR bits */
#define DCSR_STEP             (1U << 2)
#define DCSR_CAUSE_MASK       (0x7U << 6)
#define DCSR_EBREAKM          (1U << 15)

static void rtl_gdb_cancel_running(RTLMachineState *s);
static void rtl_gdb_disarm_swbreak(RTLMachineState *s);
static void rtl_gdb_poll_running(void *opaque);
static int rtl_gdb_continue(RTLMachineState *s);

static void rtl_gdb_cancel_running(RTLMachineState *s)
{
    s->gdb_target_running = false;
    if (s->gdb_poll_timer) {
        timer_del(s->gdb_poll_timer);
    }
}

/*
 * Halt the CPU via DMI.  Returns 0 on success.
 */
static int rtl_gdb_halt(RTLMachineState *s)
{
    uint32_t val;
    int rc;

    info_report("rtl-gdb: sending HALTREQ to debug module");
    rc = rtl_dmi_write(s, DMI_DMCONTROL, DMCONTROL_DMACTIVE | DMCONTROL_HALTREQ);
    if (rc) {
        warn_report("rtl-gdb: HALTREQ dmi_write failed (rc=%d)", rc);
        return -1;
    }
    info_report("rtl-gdb: HALTREQ sent, polling DMSTATUS...");

    for (int i = 0; i < 100; i++) {
        rc = rtl_dmi_read(s, DMI_DMSTATUS, &val);
        if (rc) {
            warn_report("rtl-gdb: DMSTATUS read failed (rc=%d, iter=%d)", rc, i);
            continue;
        }
        if (i < 5 || (i % 20 == 0)) {
            info_report("rtl-gdb: DMSTATUS[%d] = 0x%08x", i, val);
        }
        if (val & DMSTATUS_ALLHALTED) {
            /* Clear haltreq */
            rtl_dmi_write(s, DMI_DMCONTROL, DMCONTROL_DMACTIVE);
            rtl_gdb_cancel_running(s);
            rtl_gdb_disarm_swbreak(s);
            info_report("rtl-gdb: CPU halted successfully");
            return 0;
        }
    }
    warn_report("rtl-gdb: halt timeout, last DMSTATUS=0x%08x", val);
    return -1;
}

/*
 * Resume the CPU via DMI.  Returns 0 on success.
 */
static int rtl_gdb_resume(RTLMachineState *s)
{
    uint32_t val;
    int rc;

    rc = rtl_dmi_write(s, DMI_DMCONTROL, DMCONTROL_DMACTIVE | DMCONTROL_RESUMEREQ);
    if (rc) return -1;

    for (int i = 0; i < 100; i++) {
        rc = rtl_dmi_read(s, DMI_DMSTATUS, &val);
        if (rc) continue;
        if (val & DMSTATUS_ALLRESUMEACK) {
            rtl_dmi_write(s, DMI_DMCONTROL, DMCONTROL_DMACTIVE);
            return 0;
        }
    }
    return -1;
}

/* Forward declarations */
static int rtl_gdb_write_reg(RTLMachineState *s, uint16_t regno, uint64_t val);

/*
 * Wait for an abstract command to complete.
 */
static int rtl_dmi_wait_abstractcs(RTLMachineState *s)
{
    uint32_t val;
    for (int i = 0; i < 200; i++) {
        int rc = rtl_dmi_read(s, DMI_ABSTRACTCS, &val);
        if (rc) continue;
        if (!(val & ABSTRACTCS_BUSY)) {
            if (val & ABSTRACTCS_CMDERR) {
                /* Clear error */
                rtl_dmi_write(s, DMI_ABSTRACTCS, ABSTRACTCS_CMDERR);
                return (val >> 8) & 7; /* Return cmderr code (1-7) */
            }
            return 0;
        }
    }
    return -1;
}

/*
 * Read a 64-bit register via abstract access register command.
 * CPU must be halted. regno uses the Debug Spec encoding:
 *   0x0000-0x0FFF: CSRs
 *   0x1000-0x101F: GPRs (x0-x31)
 *   0x1020-0x103F: FPRs (f0-f31)
 */
static int rtl_gdb_read_reg_abstract(RTLMachineState *s, uint16_t regno,
                                      uint64_t *val)
{
    int rc;
    /* cmd: access register, aarsize=3(64-bit), transfer=1, write=0 */
    uint32_t cmd = (0U << 24) | (3U << 20) | (1U << 17) | (0U << 16) | regno;

    for (int retry = 0; retry < 3; retry++) {
        /* Clear any pending cmderr before issuing command */
        uint32_t acs;
        rc = rtl_dmi_read(s, DMI_ABSTRACTCS, &acs);
        if (rc == 0 && (acs & ABSTRACTCS_CMDERR)) {
            rtl_dmi_write(s, DMI_ABSTRACTCS, ABSTRACTCS_CMDERR);
        }
        /* Also wait if still busy from previous command */
        if (rc == 0 && (acs & ABSTRACTCS_BUSY)) {
            rc = rtl_dmi_wait_abstractcs(s);
            if (rc < 0) return -1;
        }

        rc = rtl_dmi_write(s, DMI_COMMAND, cmd);
        if (rc) return -1;
        rc = rtl_dmi_wait_abstractcs(s);
        if (rc == 0) {
            /* Success - read data */
            uint32_t lo, hi;
            rc = rtl_dmi_read(s, DMI_DATA0, &lo);
            if (rc) return -1;
            rc = rtl_dmi_read(s, DMI_DATA1, &hi);
            if (rc) return -1;
            *val = ((uint64_t)hi << 32) | lo;
            return 0;
        }
        if (rc == 1) {
            /* cmderr=1 (busy) - retry */
            continue;
        }
        /* Other cmderr (e.g. 2=not supported) - fail with code */
        return -rc;
    }
    return -1;
}

/*
 * Read a CSR via program buffer.
 * Used when abstract access doesn't support the register.
 * Uses s0 (x8) as scratch - saves and restores it.
 */
static int rtl_gdb_read_csr_progbuf(RTLMachineState *s, uint16_t csr_num,
                                     uint64_t *val)
{
    int rc;
    uint64_t saved_s0;

    /* Save s0 */
    rc = rtl_gdb_read_reg_abstract(s, 0x1008, &saved_s0); /* x8 = s0 */
    if (rc) return -1;

    /*
     * Write program buffer:
     *   progbuf0: csrr s0, <csr>  =  csrrs s0, <csr>, x0
     *     encoding: csr[11:0] | rs1(x0)[4:0] | funct3(010)[2:0] | rd(s0=x8)[4:0] | opcode(1110011)
     *   progbuf1: ebreak = 0x00100073
     */
    uint32_t csrr_insn = (csr_num << 20) | (0 << 15) | (2 << 12) | (8 << 7) | 0x73;
    rc = rtl_dmi_write(s, DMI_PROGBUF0, csrr_insn);
    if (rc) goto restore;
    rc = rtl_dmi_write(s, DMI_PROGBUF1, 0x00100073); /* ebreak */
    if (rc) goto restore;

    /* Execute program buffer: postexec=1, transfer=0 */
    uint32_t cmd = (0U << 24) | (3U << 20) | (1U << 18) | (0U << 17) | 0x1008;
    rc = rtl_dmi_write(s, DMI_COMMAND, cmd);
    if (rc) goto restore;
    rc = rtl_dmi_wait_abstractcs(s);
    if (rc) goto restore;

    /* Now read s0 which contains the CSR value */
    rc = rtl_gdb_read_reg_abstract(s, 0x1008, val);

restore:
    /* Restore s0 */
    rtl_gdb_write_reg(s, 0x1008, saved_s0);
    return rc;
}

/*
 * Read a register - tries abstract access first, falls back to program buffer
 * for CSRs when abstract access is not supported.
 */
static int rtl_gdb_read_reg(RTLMachineState *s, uint16_t regno, uint64_t *val)
{
    int rc = rtl_gdb_read_reg_abstract(s, regno, val);
    if (rc == -2 && regno < 0x1000) {
        /* cmderr=2 (not supported) for CSR - use program buffer */
        return rtl_gdb_read_csr_progbuf(s, regno, val);
    }
    return rc;
}

/*
 * Write a 64-bit register via abstract access register command.
 */
static int rtl_gdb_write_reg(RTLMachineState *s, uint16_t regno, uint64_t val)
{
    int rc;
    uint32_t cmd = (0U << 24) | (3U << 20) | (1U << 17) | (1U << 16) | regno;

    for (int retry = 0; retry < 3; retry++) {
        uint32_t acs;
        rc = rtl_dmi_read(s, DMI_ABSTRACTCS, &acs);
        if (rc == 0 && (acs & ABSTRACTCS_CMDERR)) {
            rtl_dmi_write(s, DMI_ABSTRACTCS, ABSTRACTCS_CMDERR);
        }
        if (rc == 0 && (acs & ABSTRACTCS_BUSY)) {
            rc = rtl_dmi_wait_abstractcs(s);
            if (rc < 0) return -1;
        }

        rc = rtl_dmi_write(s, DMI_DATA0, (uint32_t)(val & 0xFFFFFFFF));
        if (rc) return -1;
        rc = rtl_dmi_write(s, DMI_DATA1, (uint32_t)(val >> 32));
        if (rc) return -1;

        rc = rtl_dmi_write(s, DMI_COMMAND, cmd);
        if (rc) return -1;
        rc = rtl_dmi_wait_abstractcs(s);
        if (rc == 0) return 0;
        if (rc == 1) continue; /* cmderr=1 (busy) - retry */
        return -rc; /* return negative cmderr code */
    }
    return -1;
}

/*
 * Write a CSR via program buffer.
 * Used when abstract access doesn't support the register.
 */
static int rtl_gdb_write_csr_progbuf(RTLMachineState *s, uint16_t csr_num,
                                      uint64_t val)
{
    int rc;
    uint64_t saved_s0;

    /* Save s0 */
    rc = rtl_gdb_read_reg_abstract(s, 0x1008, &saved_s0);
    if (rc) return -1;

    /* Write the value to s0 */
    rc = rtl_gdb_write_reg(s, 0x1008, val);
    if (rc) goto restore;

    /*
     * Write program buffer:
     *   progbuf0: csrw <csr>, s0  =  csrrw x0, <csr>, s0
     *     encoding: csr[11:0] | rs1(s0=x8)[4:0] | funct3(001)[2:0] | rd(x0)[4:0] | opcode(1110011)
     *   progbuf1: ebreak = 0x00100073
     */
    uint32_t csrw_insn = (csr_num << 20) | (8 << 15) | (1 << 12) | (0 << 7) | 0x73;
    rc = rtl_dmi_write(s, DMI_PROGBUF0, csrw_insn);
    if (rc) goto restore;
    rc = rtl_dmi_write(s, DMI_PROGBUF1, 0x00100073); /* ebreak */
    if (rc) goto restore;

    /* Execute program buffer: postexec=1, transfer=0 */
    uint32_t exec_cmd = (0U << 24) | (3U << 20) | (1U << 18) | (0U << 17) | 0x1008;
    rc = rtl_dmi_write(s, DMI_COMMAND, exec_cmd);
    if (rc) goto restore;
    rc = rtl_dmi_wait_abstractcs(s);

restore:
    rtl_gdb_write_reg(s, 0x1008, saved_s0);
    return rc;
}

/*
 * Write a register with CSR fallback via program buffer.
 */
static int rtl_gdb_write_reg_ext(RTLMachineState *s, uint16_t regno,
                                  uint64_t val)
{
    int rc = rtl_gdb_write_reg(s, regno, val);
    if (rc == -2 && regno < 0x1000) {
        return rtl_gdb_write_csr_progbuf(s, regno, val);
    }
    return rc;
}

static void rtl_gdb_disarm_swbreak(RTLMachineState *s)
{
    if (!s->gdb_saved_dcsr_valid) {
        return;
    }

    rtl_gdb_write_reg_ext(s, 0x07b0, s->gdb_saved_dcsr);
    s->gdb_saved_dcsr_valid = false;
}

static int rtl_gdb_prepare_continue(RTLMachineState *s)
{
    uint64_t dcsr;
    int rc;

    if (s->gdb_saved_dcsr_valid) {
        return 0;
    }

    rc = rtl_gdb_read_reg(s, 0x07b0, &dcsr);
    if (rc) {
        return -1;
    }

    if (dcsr & DCSR_EBREAKM) {
        return 0;
    }

    s->gdb_saved_dcsr = dcsr;
    s->gdb_saved_dcsr_valid = true;

    dcsr |= DCSR_EBREAKM;
    rc = rtl_gdb_write_reg_ext(s, 0x07b0, dcsr);
    if (rc) {
        s->gdb_saved_dcsr_valid = false;
        return -1;
    }

    return 0;
}

static int rtl_gdb_continue(RTLMachineState *s)
{
    int rc;

    rc = rtl_gdb_prepare_continue(s);
    if (rc) {
        return -1;
    }

    rc = rtl_gdb_resume(s);
    if (rc) {
        rtl_gdb_disarm_swbreak(s);
        return -1;
    }

    s->gdb_target_running = true;
    timer_mod(s->gdb_poll_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 10);
    return 0;
}

/*
 * Read guest physical memory through the board memory map.
 *
 * This machine exposes DRAM either as normal QEMU RAM or as a DMA proxy
 * region backed by the soc-simulator. Using the physical address space here
 * keeps GDB memory access aligned with the rest of the co-simulation memory
 * model in both modes.
 */
static int rtl_gdb_read_mem(RTLMachineState *s, uint64_t addr,
                             uint8_t *buf, uint32_t len)
{
    MemTxResult result;

    result = address_space_rw(&address_space_memory, addr,
                              MEMTXATTRS_UNSPECIFIED,
                              buf, len, false);
    return (result == MEMTX_OK) ? 0 : -1;
}

/*
 * Write guest physical memory through the board memory map.
 */
static int rtl_gdb_write_mem(RTLMachineState *s, uint64_t addr,
                              const uint8_t *buf, uint32_t len)
{
    MemTxResult result;

    result = address_space_rw(&address_space_memory, addr,
                              MEMTXATTRS_UNSPECIFIED,
                              (uint8_t *)buf, len, true);
    return (result == MEMTX_OK) ? 0 : -1;
}

/*
 * Single-step the CPU: set DCSR.step, resume, wait for halt.
 */
static int rtl_gdb_step(RTLMachineState *s)
{
    uint64_t dcsr;
    int rc;

    /* Read DCSR (CSR 0x7b0, debug spec regno 0x07b0) */
    rc = rtl_gdb_read_reg(s, 0x07b0, &dcsr);
    if (rc) return -1;

    /* Set step bit */
    dcsr |= DCSR_STEP;
    rc = rtl_gdb_write_reg_ext(s, 0x07b0, dcsr);
    if (rc) return -1;

    /* Resume */
    rc = rtl_gdb_resume(s);
    if (rc) return -1;

    /* Wait for cputo halt again (single-step trap) */
    uint32_t dmstatus;
    for (int i = 0; i < 500; i++) {
        rc = rtl_dmi_read(s, DMI_DMSTATUS, &dmstatus);
        if (rc) continue;
        if (dmstatus & DMSTATUS_ALLHALTED) break;
    }

    /* Clear step bit */
    rc = rtl_gdb_read_reg(s, 0x07b0, &dcsr);
    if (!rc) {
        dcsr &= ~DCSR_STEP;
        rtl_gdb_write_reg_ext(s, 0x07b0, dcsr);
    }

    return (dmstatus & DMSTATUS_ALLHALTED) ? 0 : -1;
}

/* ==================== GDB RSP Server ==================== */

static void rtl_gdb_send_packet(RTLMachineState *s, const char *data)
{
    if (s->gdb_fd < 0) return;

    size_t len = strlen(data);
    /* $<data>#<checksum> */
    size_t pkt_len = 1 + len + 3; /* '$' + data + '#' + 2 hex */
    char *pkt = g_malloc(pkt_len + 1);
    uint8_t csum = 0;
    for (size_t i = 0; i < len; i++) {
        csum += (uint8_t)data[i];
    }
    snprintf(pkt, pkt_len + 1, "$%s#%02x", data, csum);
    send(s->gdb_fd, pkt, pkt_len, MSG_NOSIGNAL);
    g_free(pkt);
}

static void rtl_gdb_poll_running(void *opaque)
{
    RTLMachineState *s = opaque;
    uint32_t dmstatus;
    int rc;

    if (!s->gdb_connected || s->gdb_fd < 0 || !s->gdb_target_running) {
        return;
    }

    rc = rtl_dmi_read(s, DMI_DMSTATUS, &dmstatus);
    if (rc == 0 && (dmstatus & DMSTATUS_ALLHALTED)) {
        rtl_gdb_cancel_running(s);
        rtl_gdb_disarm_swbreak(s);
        rtl_gdb_send_packet(s, "S05");
        return;
    }

    timer_mod(s->gdb_poll_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 10);
}

static void rtl_gdb_send_empty(RTLMachineState *s)
{
    rtl_gdb_send_packet(s, "");
}

static void rtl_gdb_send_ok(RTLMachineState *s)
{
    rtl_gdb_send_packet(s, "OK");
}

static void rtl_gdb_send_error(RTLMachineState *s, uint8_t err)
{
    char buf[8];
    snprintf(buf, sizeof(buf), "E%02x", err);
    rtl_gdb_send_packet(s, buf);
}

/* Convert hex char to value. Returns -1 on invalid. */
static int hex_char(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Decode hex string to bytes. Returns decoded length. */
static size_t hex_decode(const char *hex, uint8_t *out, size_t max_out)
{
    size_t i = 0;
    while (hex[0] && hex[1] && i < max_out) {
        int hi = hex_char(hex[0]);
        int lo = hex_char(hex[1]);
        if (hi < 0 || lo < 0) break;
        out[i++] = (uint8_t)((hi << 4) | lo);
        hex += 2;
    }
    return i;
}

/* Encode bytes as hex string, null-terminated */
static void hex_encode(const uint8_t *data, size_t len, char *out)
{
    for (size_t i = 0; i < len; i++) {
        snprintf(out + i * 2, 3, "%02x", data[i]);
    }
    out[len * 2] = '\0';
}

/*
 * Handle a single GDB RSP command.
 * The 'cmd' string does not include the $...# framing.
 */
static void rtl_gdb_handle_command(RTLMachineState *s, const char *cmd)
{
    switch (cmd[0]) {
    case '?':  /* Stop reason */
        rtl_gdb_send_packet(s, "S05"); /* SIGTRAP */
        break;

    case 'g': { /* Read all registers (x0..x31 + PC = 33 regs, 64-bit each) */
        char buf[33 * 16 + 1];
        char *p = buf;
        for (int i = 0; i < 33; i++) {
            uint64_t val = 0;
            if (i < 32) {
                rtl_gdb_read_reg(s, 0x1000 + i, &val); /* GPR */
            } else {
                rtl_gdb_read_reg(s, 0x07b1, &val); /* DPC = PC when halted */
            }
            /* GDB expects little-endian hex */
            uint8_t bytes[8];
            memcpy(bytes, &val, 8);
            hex_encode(bytes, 8, p);
            p += 16;
        }
        *p = '\0';
        rtl_gdb_send_packet(s, buf);
        break;
    }

    case 'G': { /* Write all registers */
        const char *hex = cmd + 1;
        for (int i = 0; i < 33 && hex[0] && hex[1]; i++) {
            uint8_t bytes[8];
            size_t n = hex_decode(hex, bytes, 8);
            if (n < 8) break;
            uint64_t val;
            memcpy(&val, bytes, 8);
            if (i < 32) {
                rtl_gdb_write_reg(s, 0x1000 + i, val);
            } else {
                rtl_gdb_write_reg_ext(s, 0x07b1, val);
            }
            hex += 16;
        }
        rtl_gdb_send_ok(s);
        break;
    }

    case 'p': { /* Read single register */
        int regnum = (int)strtol(cmd + 1, NULL, 16);
        uint64_t val = 0;
        int rc;
        if (regnum < 32) {
            rc = rtl_gdb_read_reg(s, 0x1000 + regnum, &val);
        } else if (regnum == 32) {
            rc = rtl_gdb_read_reg(s, 0x07b1, &val); /* DPC/PC */
        } else {
            /* FPRs: regnum 33-64 -> Debug spec 0x1020 + (regnum-33) */
            rc = rtl_gdb_read_reg(s, 0x1020 + (regnum - 33), &val);
        }
        if (rc) {
            rtl_gdb_send_error(s, 1);
        } else {
            char buf[17];
            uint8_t bytes[8];
            memcpy(bytes, &val, 8);
            hex_encode(bytes, 8, buf);
            rtl_gdb_send_packet(s, buf);
        }
        break;
    }

    case 'P': { /* Write single register */
        char *eq = strchr(cmd + 1, '=');
        if (!eq) { rtl_gdb_send_error(s, 1); break; }
        int regnum = (int)strtol(cmd + 1, NULL, 16);
        uint8_t bytes[8] = {0};
        hex_decode(eq + 1, bytes, 8);
        uint64_t val;
        memcpy(&val, bytes, 8);
        int rc;
        if (regnum < 32) {
            rc = rtl_gdb_write_reg(s, 0x1000 + regnum, val);
        } else if (regnum == 32) {
            rc = rtl_gdb_write_reg_ext(s, 0x07b1, val);
        } else {
            rc = rtl_gdb_write_reg(s, 0x1020 + (regnum - 33), val);
        }
        if (rc) rtl_gdb_send_error(s, 1);
        else rtl_gdb_send_ok(s);
        break;
    }

    case 'm': { /* Read memory: m<addr>,<length> */
        uint64_t addr;
        uint32_t length;
        if (sscanf(cmd + 1, "%lx,%x", (unsigned long *)&addr, &length) != 2) {
            rtl_gdb_send_error(s, 1);
            break;
        }
        if (length > 4096) length = 4096;
        uint8_t *data = g_malloc(length);
        int rc = rtl_gdb_read_mem(s, addr, data, length);
        if (rc) {
            rtl_gdb_send_error(s, 1);
        } else {
            char *hex = g_malloc(length * 2 + 1);
            hex_encode(data, length, hex);
            rtl_gdb_send_packet(s, hex);
            g_free(hex);
        }
        g_free(data);
        break;
    }

    case 'M': { /* Write memory: M<addr>,<length>:<hex> */
        uint64_t addr;
        uint32_t length;
        const char *colon;
        if (sscanf(cmd + 1, "%lx,%x", (unsigned long *)&addr, &length) != 2) {
            rtl_gdb_send_error(s, 1);
            break;
        }
        colon = strchr(cmd, ':');
        if (!colon) { rtl_gdb_send_error(s, 1); break; }
        if (length > 4096) { rtl_gdb_send_error(s, 1); break; }
        uint8_t *data = g_malloc(length);
        hex_decode(colon + 1, data, length);
        int rc = rtl_gdb_write_mem(s, addr, data, length);
        g_free(data);
        if (rc) rtl_gdb_send_error(s, 1);
        else rtl_gdb_send_ok(s);
        break;
    }

    case 'c': /* Continue */
        if (rtl_gdb_continue(s) != 0) {
            rtl_gdb_send_error(s, 1);
        }
        /* We will send a stop reply when the CPU halts (e.g., breakpoint)
         * For now, just leave GDB waiting. The user can Ctrl-C to halt. */
        break;

    case 's': /* Single step */
        if (rtl_gdb_step(s) == 0) {
            rtl_gdb_send_packet(s, "S05"); /* SIGTRAP */
        } else {
            rtl_gdb_send_error(s, 1);
        }
        break;

    case 'D': /* Detach */
        rtl_gdb_cancel_running(s);
        rtl_gdb_disarm_swbreak(s);
        rtl_gdb_resume(s);
        rtl_gdb_send_ok(s);
        qemu_set_fd_handler(s->gdb_fd, NULL, NULL, NULL);
        close(s->gdb_fd);
        s->gdb_fd = -1;
        s->gdb_connected = false;
        info_report("rtl-gdb: client detached");
        break;

    case 'k': /* Kill */
        rtl_gdb_cancel_running(s);
        rtl_gdb_disarm_swbreak(s);
        qemu_set_fd_handler(s->gdb_fd, NULL, NULL, NULL);
        close(s->gdb_fd);
        s->gdb_fd = -1;
        s->gdb_connected = false;
        break;

    case 'q': /* Query */
        if (strncmp(cmd, "qSupported", 10) == 0) {
            rtl_gdb_send_packet(s,
                                "PacketSize=4096;QStartNoAckMode+;"
                                "vContSupported+");
        } else if (strcmp(cmd, "qAttached") == 0) {
            rtl_gdb_send_packet(s, "1"); /* attached to existing process */
        } else if (strcmp(cmd, "qC") == 0) {
            rtl_gdb_send_packet(s, "QC1"); /* current thread = 1 */
        } else if (strcmp(cmd, "qfThreadInfo") == 0) {
            rtl_gdb_send_packet(s, "m1"); /* thread 1 */
        } else if (strcmp(cmd, "qsThreadInfo") == 0) {
            rtl_gdb_send_packet(s, "l"); /* end of thread list */
        } else if (strncmp(cmd, "qXfer", 5) == 0) {
            /* Send target.xml for register layout */
            if (strncmp(cmd, "qXfer:features:read:target.xml:",
                        strlen("qXfer:features:read:target.xml:")) == 0) {
                const char *xml =
                    "l<?xml version=\"1.0\"?>"
                    "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">"
                    "<target version=\"1.0\">"
                    "<architecture>riscv:rv64</architecture>"
                    "<feature name=\"org.gnu.gdb.riscv.cpu\">";
                /* This is a simplified response - GDB will figure out regs */
                char buf[2048];
                int pos = 0;
                pos += snprintf(buf + pos, sizeof(buf) - pos, "%s", xml);
                for (int i = 0; i < 32; i++) {
                    pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "<reg name=\"x%d\" bitsize=\"64\" "
                        "type=\"uint64\" regnum=\"%d\"/>", i, i);
                }
                pos += snprintf(buf + pos, sizeof(buf) - pos,
                    "<reg name=\"pc\" bitsize=\"64\" "
                    "type=\"code_ptr\" regnum=\"32\"/>"
                    "</feature></target>");
                rtl_gdb_send_packet(s, buf);
            } else {
                rtl_gdb_send_empty(s);
            }
        } else {
            rtl_gdb_send_empty(s);
        }
        break;

    case 'Q': /* Set */
        if (strcmp(cmd, "QStartNoAckMode") == 0) {
            rtl_gdb_send_ok(s);
            s->gdb_noack = true;
        } else {
            rtl_gdb_send_empty(s);
        }
        break;

    case 'H': /* Set thread */
        rtl_gdb_send_ok(s);
        break;

    case 'T': /* Thread alive check */
        rtl_gdb_send_ok(s);
        break;

    case 'v':
        if (strncmp(cmd, "vMustReplyEmpty", 15) == 0) {
            rtl_gdb_send_empty(s);
        } else if (strncmp(cmd, "vCont?", 6) == 0) {
            rtl_gdb_send_packet(s, "vCont;c;s");
        } else if (strncmp(cmd, "vCont;c", 7) == 0) {
            if (rtl_gdb_continue(s) != 0) {
                rtl_gdb_send_error(s, 1);
            }
            /* Leave GDB waiting */
        } else if (strncmp(cmd, "vCont;s", 7) == 0) {
            if (rtl_gdb_step(s) == 0) {
                rtl_gdb_send_packet(s, "S05");
            } else {
                rtl_gdb_send_error(s, 1);
            }
        } else {
            rtl_gdb_send_empty(s);
        }
        break;

    case 'z': /* Remove breakpoint (not supported - use sw breakpoints) */
    case 'Z': /* Set breakpoint */
        rtl_gdb_send_empty(s); /* Not supported */
        break;

    default:
        rtl_gdb_send_empty(s);
        break;
    }
}

/*
 * Process data from the GDB client socket.
 * GDB RSP format: $<data>#<checksum> with optional +/- acks.
 */
static void rtl_gdb_read_handler(void *opaque)
{
    RTLMachineState *s = opaque;
    uint8_t buf[1024];

    ssize_t n = recv(s->gdb_fd, buf, sizeof(buf), 0);
    if (n <= 0) {
        bool was_running = s->gdb_target_running;

        info_report("rtl-gdb: client disconnected");
        rtl_gdb_cancel_running(s);
        if (!was_running) {
            rtl_gdb_disarm_swbreak(s);
        }
        qemu_set_fd_handler(s->gdb_fd, NULL, NULL, NULL);
        close(s->gdb_fd);
        s->gdb_fd = -1;
        s->gdb_connected = false;
        return;
    }

    /* Check for Ctrl-C (0x03) - halt request */
    for (ssize_t i = 0; i < n; i++) {
        if (buf[i] == 0x03) {
            info_report("rtl-gdb: received Ctrl-C, halting CPU");
            if (rtl_gdb_halt(s) == 0) {
                rtl_gdb_send_packet(s, "S02"); /* SIGINT */
            } else {
                rtl_gdb_send_packet(s, "S05"); /* SIGTRAP - best effort */
            }
            return;
        }
    }

    /* Append to receive buffer */
    size_t space = sizeof(s->gdb_rxbuf) - s->gdb_rxlen;
    if ((size_t)n > space) n = space;
    memcpy(s->gdb_rxbuf + s->gdb_rxlen, buf, n);
    s->gdb_rxlen += n;

    /* Process complete packets */
    while (s->gdb_rxlen > 0) {
        /* Skip leading +/- ack bytes */
        if (s->gdb_rxbuf[0] == '+' || s->gdb_rxbuf[0] == '-') {
            memmove(s->gdb_rxbuf, s->gdb_rxbuf + 1, --s->gdb_rxlen);
            continue;
        }

        /* Find packet: $<data>#<checksum> */
        if (s->gdb_rxbuf[0] != '$') {
            /* Skip junk */
            memmove(s->gdb_rxbuf, s->gdb_rxbuf + 1, --s->gdb_rxlen);
            continue;
        }

        /* Find '#' */
        uint8_t *hash = memchr(s->gdb_rxbuf + 1, '#', s->gdb_rxlen - 1);
        if (!hash) break; /* incomplete packet */

        size_t hash_pos = hash - s->gdb_rxbuf;
        if (hash_pos + 2 >= s->gdb_rxlen) break; /* need checksum bytes */

        /* Extract command (between $ and #) */
        size_t cmd_len = hash_pos - 1;
        char *cmd = g_malloc(cmd_len + 1);
        memcpy(cmd, s->gdb_rxbuf + 1, cmd_len);
        cmd[cmd_len] = '\0';

        /* Send ack if not in no-ack mode */
        if (!s->gdb_noack) {
            send(s->gdb_fd, "+", 1, MSG_NOSIGNAL);
        }

        /* Consume this packet from the buffer */
        size_t pkt_total = hash_pos + 3; /* $...#xx */
        memmove(s->gdb_rxbuf, s->gdb_rxbuf + pkt_total,
                s->gdb_rxlen - pkt_total);
        s->gdb_rxlen -= pkt_total;

        /* Process command */
        rtl_gdb_handle_command(s, cmd);
        g_free(cmd);
    }
}

/*
 * Accept a new GDB client connection.
 */
static void rtl_gdb_accept_handler(void *opaque)
{
    RTLMachineState *s = opaque;
    struct sockaddr_in peer;
    socklen_t plen = sizeof(peer);

    int fd = accept(s->gdb_listen_fd, (struct sockaddr *)&peer, &plen);
    if (fd < 0) return;

    if (s->gdb_connected) {
        /* Only one GDB at a time */
        close(fd);
        return;
    }

    int flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    s->gdb_fd = fd;
    s->gdb_connected = true;
    s->gdb_noack = false;
    s->gdb_rxlen = 0;
    s->gdb_target_running = false;

    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &peer.sin_addr, addr_str, sizeof(addr_str));
    info_report("rtl-gdb: client connected from %s:%u",
                addr_str, ntohs(peer.sin_port));

    /* Activate debug module */
    rtl_dmi_write(s, DMI_DMCONTROL, DMCONTROL_DMACTIVE);

    /* Halt the CPU so GDB can inspect state */
    if (rtl_gdb_halt(s) == 0) {
        rtl_gdb_disarm_swbreak(s);
        info_report("rtl-gdb: CPU halted for debug");
    } else {
        warn_report("rtl-gdb: failed to halt CPU");
    }

    qemu_set_fd_handler(fd, rtl_gdb_read_handler, NULL, s);
}

/*
 * Initialize the GDB RSP server.
 * Parse address string and start listening.
 */
static void rtl_gdb_server_init(RTLMachineState *s)
{
    if (!s->gdb_addr || strlen(s->gdb_addr) == 0) return;

    /* Parse host:port */
    char host[256] = "0.0.0.0";
    uint16_t port = 1234;
    const char *colon = strrchr(s->gdb_addr, ':');
    if (colon) {
        if (colon != s->gdb_addr) {
            size_t hlen = colon - s->gdb_addr;
            if (hlen >= sizeof(host)) hlen = sizeof(host) - 1;
            memcpy(host, s->gdb_addr, hlen);
            host[hlen] = '\0';
        }
        port = (uint16_t)atoi(colon + 1);
    } else {
        port = (uint16_t)atoi(s->gdb_addr);
    }

    s->gdb_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (s->gdb_listen_fd < 0) {
        error_report("rtl-gdb: socket() failed: %s", strerror(errno));
        return;
    }

    int optval = 1;
    setsockopt(s->gdb_listen_fd, SOL_SOCKET, SO_REUSEADDR,
               &optval, sizeof(optval));

    struct sockaddr_in saddr;
    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(port);
    inet_pton(AF_INET, host, &saddr.sin_addr);

    if (bind(s->gdb_listen_fd, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
        error_report("rtl-gdb: bind(%s:%u) failed: %s",
                     host, port, strerror(errno));
        close(s->gdb_listen_fd);
        s->gdb_listen_fd = -1;
        return;
    }

    if (listen(s->gdb_listen_fd, 1) < 0) {
        error_report("rtl-gdb: listen() failed: %s", strerror(errno));
        close(s->gdb_listen_fd);
        s->gdb_listen_fd = -1;
        return;
    }

    qemu_set_fd_handler(s->gdb_listen_fd, rtl_gdb_accept_handler, NULL, s);
    info_report("rtl-gdb: listening on %s:%u (connect with 'target remote %s:%u')",
                host, port, host, port);
}

/* ==================== MMIO proxy ==================== */

/*
 * The MMIO proxy region captures accesses that don't hit any device.
 * This shouldn't normally happen since devices are mapped in the MMIO space.
 * But it's here as a catch-all.
 */
static uint64_t rtl_mmio_proxy_read(void *opaque, hwaddr addr, unsigned size)
{
    /* Unhandled MMIO read - return 0 */
    return 0;
}

static void rtl_mmio_proxy_write(void *opaque, hwaddr addr,
                                  uint64_t val, unsigned size)
{
    /* Unhandled MMIO write - ignore */
}

static const MemoryRegionOps rtl_mmio_proxy_ops = {
    .read = rtl_mmio_proxy_read,
    .write = rtl_mmio_proxy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/* ==================== Machine initialization ==================== */

static void rtl_machine_init(MachineState *machine)
{
    RTLMachineState *s = RTL_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    int i;

    s->irq_levels = 0;
    s->mem_req_id = 0;
    s->dma_req_id = 0;
    s->rtl_cpu_started = false;
    s->dram_region_created = false;
    s->dma_proxy_created = false;

    /*
     * Create the MMIO proxy region covering the MMIO address space.
     * Use _overlap with low priority so that actual device regions
     * (UART, VirtIO) take precedence.
     */
    memory_region_init_io(&s->mmio_proxy, OBJECT(machine),
                          &rtl_mmio_proxy_ops, s,
                          "rtl-mmio-proxy", 0x20000000ULL);
    memory_region_add_subregion_overlap(system_memory, 0x60000000ULL,
                                        &s->mmio_proxy, -1);

    /*
     * Create IRQ sink: QEMU devices raise IRQs, we forward to soc-simulator.
     */
    qemu_irq *irqs = qemu_allocate_irqs(rtl_irq_handler, s, RTL_MAX_IRQ_LINES);

    /*
     * Create UART (16550 compatible) at the standard address.
     * This is the primary console device.
     */
    serial_mm_init(system_memory, RTL_UART0_ADDR, 0,
                   irqs[RTL_UART0_IRQ], 399193, serial_hd(0),
                   DEVICE_LITTLE_ENDIAN);

    /*
     * Create VirtIO MMIO devices for networking, block devices, etc.
     */
    for (i = 0; i < RTL_VIRTIO_COUNT; i++) {
        hwaddr virtio_addr = RTL_VIRTIO_ADDR + i * RTL_VIRTIO_SIZE;
        sysbus_create_simple("virtio-mmio", virtio_addr,
                             irqs[RTL_VIRTIO_IRQ + i]);
    }

    /*
     * Connect to soc-simulator via TCP.
     */
    if (!s->socket_path || strlen(s->socket_path) == 0) {
        s->socket_path = g_strdup("127.0.0.1:2345");
    }

    s->sock.conn_fd = -1;
    s->sock.connected = false;

    rtl_connect_and_init(s);

    /* Start GDB debug server if configured */
    s->gdb_listen_fd = -1;
    s->gdb_fd = -1;
    s->gdb_connected = false;
    s->gdb_noack = false;
    s->gdb_rxlen = 0;
    s->gdb_target_running = false;
    s->gdb_saved_dcsr_valid = false;
    s->dmi_req_id = 0;
    s->dmi_resp_pending = false;
    s->gdb_poll_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                     rtl_gdb_poll_running, s);
    rtl_gdb_server_init(s);

    info_report("rtl: UART at 0x%lx, VirtIO at 0x%lx (%d devices)",
                (unsigned long)RTL_UART0_ADDR,
                (unsigned long)RTL_VIRTIO_ADDR,
                RTL_VIRTIO_COUNT);
}

/* ==================== Machine properties ==================== */

static char *rtl_get_socket_path(Object *obj, Error **errp)
{
    RTLMachineState *s = RTL_MACHINE(obj);
    return g_strdup(s->socket_path);
}

static void rtl_set_socket_path(Object *obj, const char *value, Error **errp)
{
    RTLMachineState *s = RTL_MACHINE(obj);
    g_free(s->socket_path);
    s->socket_path = g_strdup(value);
}

static char *rtl_get_gdb_addr(Object *obj, Error **errp)
{
    RTLMachineState *s = RTL_MACHINE(obj);
    return g_strdup(s->gdb_addr ? s->gdb_addr : "");
}

static void rtl_set_gdb_addr(Object *obj, const char *value, Error **errp)
{
    RTLMachineState *s = RTL_MACHINE(obj);
    g_free(s->gdb_addr);
    s->gdb_addr = g_strdup(value);
}

static void rtl_machine_instance_init(Object *obj)
{
    RTLMachineState *s = RTL_MACHINE(obj);
    s->socket_path = g_strdup("127.0.0.1:2345");
}

static void rtl_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "RISC-V RTL co-simulation board";
    mc->init = rtl_machine_init;
    mc->max_cpus = 1;  /* Placeholder - actual CPU is in the RTL simulator */
    mc->default_cpus = 1;
    mc->min_cpus = 1;
    mc->no_cdrom = 1;

    object_class_property_add_str(oc, "rtl-sock",
                                  rtl_get_socket_path,
                                  rtl_set_socket_path);
    object_class_property_set_description(oc, "rtl-sock",
            "TCP address of soc-simulator to connect to (host:port) "
            "(default: 127.0.0.1:2345)");

    object_class_property_add_str(oc, "rtl-gdb",
                                  rtl_get_gdb_addr,
                                  rtl_set_gdb_addr);
    object_class_property_set_description(oc, "rtl-gdb",
            "GDB RSP server address for RTL CPU debug (host:port). "
            "Connect with: target remote <host>:<port>");
}

static const TypeInfo rtl_machine_typeinfo = {
    .name       = MACHINE_TYPE_NAME("rtl"),
    .parent     = TYPE_MACHINE,
    .class_init = rtl_machine_class_init,
    .instance_init = rtl_machine_instance_init,
    .instance_size = sizeof(RTLMachineState),
};

static void rtl_machine_init_register_types(void)
{
    type_register_static(&rtl_machine_typeinfo);
}

type_init(rtl_machine_init_register_types)

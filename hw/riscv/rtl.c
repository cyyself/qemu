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
static bool rtl_dma_write(RTLMachineState *s, uint64_t addr,
                           const void *data, uint32_t size);
static const MemoryRegionOps rtl_dma_proxy_ops;

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

    switch (hdr.type) {
    case RTL_MSG_MMIO_READ:
        rtl_handle_mmio_read(s, &hdr);
        break;
    case RTL_MSG_MMIO_WRITE:
        rtl_handle_mmio_write(s, &hdr);
        break;
    case RTL_MSG_DMA_READ_RESP:
        rtl_handle_dma_read_resp(s, &hdr);
        break;
    case RTL_MSG_DMA_WRITE_RESP:
        rtl_handle_dma_write_resp(s, &hdr);
        break;
    case RTL_MSG_MEM_READ_RESP:
        rtl_handle_mem_read_resp(s, &hdr);
        break;
    case RTL_MSG_MEM_WRITE_RESP:
        rtl_handle_mem_write_resp(s, &hdr);
        break;
    case RTL_MSG_CPU_START_ACK:
        rtl_handle_cpu_start_ack(s, &hdr);
        break;
    case RTL_MSG_CPU_STOP_ACK:
        rtl_handle_cpu_stop_ack(s, &hdr);
        break;
    case RTL_MSG_CPU_STATUS_RESP:
        rtl_handle_cpu_status_resp(s, &hdr);
        break;
    case RTL_MSG_SYNC:
        rtl_handle_sync(s, &hdr);
        break;
    case RTL_MSG_CPU_MEM_READ:
        rtl_handle_cpu_mem_read(s, &hdr);
        break;
    case RTL_MSG_CPU_MEM_WRITE:
        rtl_handle_cpu_mem_write(s, &hdr);
        break;
    case RTL_MSG_SHUTDOWN:
        info_report("rtl: soc-simulator sent shutdown");
        qemu_set_fd_handler(s->sock.conn_fd, NULL, NULL, NULL);
        close(s->sock.conn_fd);
        s->sock.conn_fd = -1;
        s->sock.connected = false;
        break;
    default:
        warn_report("rtl: unknown message type 0x%x", hdr.type);
        /* Skip remaining bytes */
        if (hdr.length > sizeof(hdr)) {
            size_t remaining = hdr.length - sizeof(hdr);
            uint8_t skip[256];
            while (remaining > 0) {
                size_t chunk = MIN(remaining, sizeof(skip));
                if (!rtl_recv_all(s->sock.conn_fd, skip, chunk)) break;
                remaining -= chunk;
            }
        }
        break;
    }
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

    /* VirtIO MMIO devices (only those with valid PLIC IRQs get FDT nodes) */
    {
        int virtio_fdt_count = MIN(RTL_VIRTIO_COUNT,
                                   (int)RTL_PLIC_NDEV - 1);
        for (int i = virtio_fdt_count - 1; i >= 0; i--) {
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
            qemu_fdt_setprop_cell(fdt, name, "interrupts",
                                  RTL_VIRTIO_IRQ + i + 1);
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
        if (ms->kernel_filename) {
            /* Kernel loaded after firmware */
            kernel_entry = fw_addr;  /* placeholder - not supported yet */
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
 * This is non-blocking: sends the request and processes the response
 * asynchronously via the fd_handler.
 */
static bool rtl_dma_write(RTLMachineState *s, uint64_t addr,
                           const void *data, uint32_t size)
{
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

    bool ok = rtl_send_all(s->sock.conn_fd, buf, msg_size);
    g_free(buf);
    return ok;
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
    (void)opaque; (void)addr; (void)attrs;
    /* DMA reads from devices are rare; return 0 for now */
    *val = 0;
    return MEMTX_OK;
}

static MemTxResult rtl_dma_proxy_write_with_attrs(void *opaque, hwaddr addr,
                                                   uint64_t val, unsigned size,
                                                   MemTxAttrs attrs)
{
    RTLMachineState *s = opaque;
    (void)attrs;
    uint64_t phys = s->sock.dram_base + addr;
    rtl_dma_write(s, phys, &val, size);
    return MEMTX_OK;
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
 */
static void rtl_update_irq(RTLMachineState *s, int irq, int level)
{
    if (!s->sock.connected) return;

    if (level) {
        s->irq_levels |= (1ULL << irq);
    } else {
        s->irq_levels &= ~(1ULL << irq);
    }

    struct rtl_msg_irq_update msg;
    memset(&msg, 0, sizeof(msg));
    msg.hdr.type = RTL_MSG_IRQ_UPDATE;
    msg.hdr.length = sizeof(msg);
    msg.irq_levels = s->irq_levels;

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

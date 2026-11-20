# Linux SPI/DMA Character Driver for High-Speed 16-Bit ADC (`adc16_dma`)

**Author:** Naman Bansal (`2025NS01191`)  
**Course:** ESZG547 – Device Drivers (BITS Pilani WILP)  
**Target Platform:** ARM Cortex-A53 SoC (e.g., NXP i.MX8M Plus) running Linux 6.x  

---

## Overview

This repository contains an out-of-tree Linux kernel character device driver (`adc16_dma.c`) designed for an external **16-bit, 4-channel, 1 MSPS Industrial Analog-to-Digital Converter (ADC)** interfaced over SPI with a hardware Data-Ready (`DRDY`) GPIO interrupt and a DMA controller.

Rather than blocking CPU cycles on per-sample SPI transactions, the driver exposes `/dev/adc0` and streams 8-byte sample frames (`4 channels × 16 bits`) directly into a **64 KB coherent DMA ring buffer** (`dma_alloc_coherent`), supporting both zero-copy user-space mapping (`mmap`) and event-driven reads (`poll` / `read`).

---

## Driver Architecture

```text
[External 16-Bit ADC] --(DRDY GPIO IRQ)--> adc_drdy_isr() [Top-Half]

        |                                        |
   (SPI RX FIFO)                     dmaengine_prep_slave_single()

        |                                        |
        v                                        v
  [DMA Controller] ------------> [64 KB Coherent DMA Ring Buffer]
                                                 |
                                    adc_dma_complete_cb() [Bottom-Half]
                                                 |
                                   wake_up_interruptible(&read_queue)
                                                 |
                         +-----------------------+-----------------------+

                         |                                               |
                         v                                               v
           Zero-Copy mmap() [Read-Only]                    Blocking/Non-Blocking read() & poll()

                         |                                               |
                         +-----------------------+-----------------------+
                                                 |
                                                 v
                                       [User-Space Application]
```

### Key Features
* **Device Tree Binding:** Matches `compatible = "vendor,adc16-dma"` via `struct spi_driver`, binding the RX DMA channel (`dmas`) and `DRDY` GPIO interrupt (`drdy-gpios`).
* **Two-Stage Interrupt & DMA Pipeline:**
  * **Top-Half ISR (`adc_drdy_isr`):** Triggers on falling-edge `DRDY` pulses and submits an asynchronous slave DMA descriptor (`DMA_DEV_TO_MEM`).
  * **DMA Completion Callback (`adc_dma_complete_cb`):** Updates circular buffer indices under `spin_lock_irqsave`, flags ring overruns (`-EOVERFLOW`), and wakes sleeping user-space readers.
* **User-Space Interface (`/dev/adc0`):**
  * `unlocked_ioctl`: Start acquisition (`ADC_IOC_START`), stop DMA & disable IRQ (`ADC_IOC_STOP`), and query running/overflow flags (`ADC_IOC_GET_STATUS`).
  * `mmap`: Maps the coherent DMA ring buffer read-only (`VM_WRITE` rejected with `-EPERM`) into user virtual memory via `dma_mmap_coherent()`.
  * `poll` & `read`: Integrates with `epoll`/`select` (`EPOLLIN | EPOLLRDNORM`) and supports both blocking (`wait_event_interruptible`) and `O_NONBLOCK` reads.

---

## Repository Structure

| File | Description |
| :--- | :--- |
| `adc16_dma.c` | Complete Linux kernel character + SPI/DMA driver implementation |
| `Makefile` | Out-of-tree Kbuild Makefile for compiling `adc16_dma.ko` |
| `.gitignore` | Ignores kernel build artifacts (`*.ko`, `*.o`, `*.mod*`, `*.symvers`, `*.order`) |
| `README.md` | Architecture overview, build instructions, and usage guide |

---

## Build & Verification Instructions

### 1. Prerequisites
Install kernel headers and build tools on your Linux environment:
```bash
sudo apt update && sudo apt install -y build-essential linux-headers-\$(uname -r) kmod
```

### 2. Compile the Kernel Module
```bash
make -C /lib/modules/\((uname -r)/build M=\)(pwd) modules
```

### 3. Inspect Module Metadata
```bash
/sbin/modinfo ./adc16_dma.ko
```

### 4. Load & Unload (On Target Hardware / VM with Root Privileges)
```bash
sudo insmod adc16_dma.ko
dmesg | tail -n 10
sudo rmmod adc16_dma
```

### 5. Clean Build Artifacts
```bash
make -C /lib/modules/\((uname -r)/build M=\)(pwd) clean
```


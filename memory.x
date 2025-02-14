MEMORY {
    STACK   (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
    RAM     (rwx) : ORIGIN = 0x20001000, LENGTH = 4K
    FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
}

_stack_start = ORIGIN(RAM);
_stack_end = ORIGIN(STACK);

.globl _version_info
_version_info:
    .word __u_boot_cmd_end-_start       /* length */
    .word 0                 /* crc */
    .ascii "BMGK"           /* magic value */
    .asciz U_BOOT_VERSION


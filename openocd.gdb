target extended-remote :4242

# print demangled symbols
set print asm-demangle on

# detect unhandled exceptions, hard faults and panics
break DefaultHandler
#break UserHardFault
break rust_begin_unwind

# monitor arm semihosting enable

# send captured ITM to the file itm.fifo
# (the microcontroller SWO pin must be connected to the programmer SWO pin)
# final number must match the core clock frequency at the time you want to
# record output
# monitor tpiu config internal itm.txt uart off 160000000

# # OR: make the microcontroller SWO pin output compatible with UART (8N1)
# # 16000000 must match the core clock frequency
# # 2000000 is the frequency of the SWO pin
# monitor tpiu config external uart off 16000000 2000000

# enable ITM port 0
# monitor itm port 0 on

# define some command to examine hardware registers on stm32f103rb
define can_mcr
    x /x 0x4000_6400
end

define can_msr
    x /x 0x4000_6404
end

define can_tsr
    x /x 0x4000_6408
end

define can_rf0r
    x /x 0x4000_640c
end

define can_rf1r
    x /x 0x4000_6410
end

define can_ier
    x /x 0x4000_6414
end

define can_esr
    x /x 0x4000_6418
end

define can_btr
    x /x 0x4000_641c
end

define gpioa
  x /7x 0x4001_0800
end

define gpiob
  x /7x 0x4001_0c00
end

define gpioc
  x /7x 0x4001_1000
end

define gpiod
  x /7x 0x4001_1400
end

define rcc_cr
  x /x 0x4001_1000
end

define rcc_cfgr
  x /x 0x4001_1004
end

define afio
  x /4x 0x4001_0000
end

define ahbenr
  x /x 0x4002_1014
end

define apb1enr
  x /x 0x4002_101c
end

define apb2enr
  x /x 0x4002_1018
end

define lilosdbg
  dump binary memory memory-image.bin 0x2000_0000 0x2000_5000
end

load

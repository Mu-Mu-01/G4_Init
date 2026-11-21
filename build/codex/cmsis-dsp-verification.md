# CMSIS-DSP 构建验证

| GNU Arm 预设 | Flash | RAM | ELF SHA-256 |
| --- | ---: | ---: | --- |
| Debug | 17420 B | 2056 B | 6e7dafb64f7b0bbd18f9fe85b25d445b2e9b367e6cf1cbf62ac5df25bf3c5055 |
| Release | 9668 B | 2056 B | 56b1809d767de93a47dee447f82163e35e456afcbf46c20be5ba671ff4481cca |

两种预设均生成 ELF 和 MAP，无构建警告。ELF 属性均为 Thumb-2、单精度 VFPv4-D16 和浮点寄存器传参。独立链接检查在两种预设的 DSP 静态库中均解析到 arm_sin_f32、arm_hanning_f32 和 cosf。

两个固件的 .bss 均占 504 B。Debug 最大的三个 .bss 对象为 hadc1（108 B）、hdma_adc1（96 B）和 hdma_dac1_ch1（96 B）。RAM 占用含堆栈保留空间。

验证范围为编译和链接。ST Arm Clang 构建及实板运行尚未验证。

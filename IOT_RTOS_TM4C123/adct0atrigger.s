; generated by ARM C/C++ Compiler, 5.03 [Build 76]
; commandline ArmCC [--c99 --split_sections --debug -c -S -o.\adct0atrigger.s --depend=.\adct0atrigger.d --cpu=Cortex-M4 --apcs=interwork -O0 -I.. -I..\..\.. -IC:\Keil\ARM\RV31\INC -IC:\Keil\ARM\CMSIS\Include -IC:\Keil\ARM\Inc\Luminary -Drvmdk -DPART_LM4F120H5QR --signed_chars --omf_browse=.\adct0atrigger.crf ADCT0ATrigger.c]
        THUMB
        REQUIRE8
        PRESERVE8

        AREA ||i.ADC0Seq3_Handler||, CODE, READONLY, ALIGN=2

ADC0Seq3_Handler PROC
        PUSH     {r4,lr}
        MOVS     r0,#8
        LDR      r1,|L0.28|
        STR      r0,[r1,#0xc]
        LDR      r0,|L0.32|
        LDR      r0,[r0,#0]
        LDR      r1,|L0.36|
        STR      r0,[r1,#0]  ; ADCvalue
        MOV      r0,r1
        LDR      r0,[r0,#0]  ; ADCvalue
        LDR      r1,|L0.40|
        LDR      r1,[r1,#0]  ; ProducerTask
        BLX      r1
        POP      {r4,pc}
        ENDP

|L0.28|
        DCD      0x40038000
|L0.32|
        DCD      0x400380a8
|L0.36|
        DCD      ADCvalue
|L0.40|
        DCD      ProducerTask

        AREA ||i.ADC0_InitSWTriggerSeq2||, CODE, READONLY, ALIGN=2

ADC0_InitSWTriggerSeq2 PROC
        PUSH     {r3,lr}
        CMP      r0,#0xc
        BCS      |L1.82|
        TBB      [pc,r0]
        DCB      0x06,0x07
        DCB      0x08,0x09,0x12,0x13
        DCB      0x14,0x15,0x0a,0x0b
        DCB      0x1c,0x1d
        NOP      
        NOP      
        NOP      
        NOP      
        NOP      
        LDR      r1,|L1.968|
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x10
        LDR      r2,|L1.968|
        STR      r1,[r2,#0]
        B        |L1.84|
        NOP      
        NOP      
        NOP      
        LDR      r1,|L1.968|
        LDR      r1,[r1,#0]
        ORR      r1,r1,#8
        LDR      r2,|L1.968|
        STR      r1,[r2,#0]
        B        |L1.84|
        NOP      
        LDR      r1,|L1.968|
        LDR      r1,[r1,#0]
        ORR      r1,r1,#2
        LDR      r2,|L1.968|
        STR      r1,[r2,#0]
        B        |L1.84|
|L1.82|
        POP      {r3,pc}
|L1.84|
        NOP      
        LDR      r1,|L1.968|
        LDR      r1,[r1,#0]
        STR      r1,[sp,#0]
        LDR      r1,|L1.968|
        LDR      r1,[r1,#0]
        STR      r1,[sp,#0]
        CMP      r0,#0xc
        BCS      |L1.174|
        TBB      [pc,r0]
        DCB      0x06,0x23
        DCB      0x40,0x5d,0x7a,0x97
        DCB      0xb4,0xd1,0xf2,0xf0
        DCB      0xef,0xee
        LDR      r1,|L1.972|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#8
        LDR      r2,|L1.972|
        STR      r1,[r2,#0]
        LDR      r1,|L1.972|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#8
        LDR      r2,|L1.972|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#8
        LDR      r2,|L1.976|
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#8
        LDR      r2,|L1.976|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
|L1.174|
        B        |L1.822|
        LDR      r1,|L1.972|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#4
        LDR      r2,|L1.972|
        STR      r1,[r2,#0]
        LDR      r1,|L1.972|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#4
        LDR      r2,|L1.972|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#4
        LDR      r2,|L1.976|
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#4
        LDR      r2,|L1.976|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
        B        |L1.822|
        LDR      r1,|L1.972|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#2
        LDR      r2,|L1.972|
        STR      r1,[r2,#0]
        LDR      r1,|L1.972|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#2
        LDR      r2,|L1.972|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#2
        LDR      r2,|L1.976|
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#2
        LDR      r2,|L1.976|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
        B        |L1.822|
        LDR      r1,|L1.972|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#1
        LDR      r2,|L1.972|
        STR      r1,[r2,#0]
        LDR      r1,|L1.972|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#1
        LDR      r2,|L1.972|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#1
        LDR      r2,|L1.976|
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#1
        LDR      r2,|L1.976|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
        B        |L1.822|
        LDR      r1,|L1.980|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#8
        LDR      r2,|L1.980|
        STR      r1,[r2,#0]
        LDR      r1,|L1.980|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#8
        LDR      r2,|L1.980|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.984|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#8
        LDR      r2,|L1.984|
        STR      r1,[r2,#0]
        LDR      r1,|L1.984|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#8
        LDR      r2,|L1.984|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
        B        |L1.822|
        LDR      r1,|L1.980|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#4
        LDR      r2,|L1.980|
        STR      r1,[r2,#0]
        LDR      r1,|L1.980|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#4
        LDR      r2,|L1.980|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.984|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#4
        LDR      r2,|L1.984|
        STR      r1,[r2,#0]
        LDR      r1,|L1.984|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#4
        LDR      r2,|L1.984|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
        B        |L1.822|
        LDR      r1,|L1.980|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#2
        LDR      r2,|L1.980|
        STR      r1,[r2,#0]
        LDR      r1,|L1.980|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#2
        LDR      r2,|L1.980|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.984|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#2
        LDR      r2,|L1.984|
        STR      r1,[r2,#0]
        LDR      r1,|L1.984|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#2
        LDR      r2,|L1.984|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
        B        |L1.822|
        LDR      r1,|L1.980|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#1
        LDR      r2,|L1.980|
        STR      r1,[r2,#0]
        LDR      r1,|L1.980|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#1
        LDR      r2,|L1.980|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.984|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#1
        LDR      r2,|L1.984|
        STR      r1,[r2,#0]
        LDR      r1,|L1.984|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#1
        LDR      r2,|L1.984|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
        B        |L1.822|
        B        |L1.764|
        B        |L1.706|
        B        |L1.648|
        B        |L1.590|
|L1.590|
        LDR      r1,|L1.972|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0x20
        LDR      r2,|L1.972|
        STR      r1,[r2,#0]
        LDR      r1,|L1.972|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x20
        LDR      r2,|L1.972|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0x20
        LDR      r2,|L1.976|
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x20
        LDR      r2,|L1.976|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
        B        |L1.822|
|L1.648|
        LDR      r1,|L1.972|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0x10
        LDR      r2,|L1.972|
        STR      r1,[r2,#0]
        LDR      r1,|L1.972|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x10
        LDR      r2,|L1.972|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0x10
        LDR      r2,|L1.976|
        STR      r1,[r2,#0]
        LDR      r1,|L1.976|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x10
        LDR      r2,|L1.976|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
        B        |L1.822|
|L1.706|
        LDR      r1,|L1.988|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0x10
        LDR      r2,|L1.988|
        STR      r1,[r2,#0]
        LDR      r1,|L1.988|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x10
        LDR      r2,|L1.988|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.992|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0x10
        LDR      r2,|L1.992|
        STR      r1,[r2,#0]
        LDR      r1,|L1.992|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x10
        LDR      r2,|L1.992|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
        B        |L1.822|
|L1.764|
        LDR      r1,|L1.988|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0x20
        LDR      r2,|L1.988|
        STR      r1,[r2,#0]
        LDR      r1,|L1.988|
        ADDS     r1,r1,#0x20
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x20
        LDR      r2,|L1.988|
        ADDS     r2,r2,#0x20
        STR      r1,[r2,#0]
        LDR      r1,|L1.992|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0x20
        LDR      r2,|L1.992|
        STR      r1,[r2,#0]
        LDR      r1,|L1.992|
        ADDS     r1,r1,#0xc
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x20
        LDR      r2,|L1.992|
        ADDS     r2,r2,#0xc
        STR      r1,[r2,#0]
        NOP      
|L1.822|
        NOP      
        LDR      r1,|L1.968|
        ADDS     r1,r1,#0x30
        LDR      r1,[r1,#0]
        ORR      r1,r1,#1
        LDR      r2,|L1.968|
        ADDS     r2,r2,#0x30
        STR      r1,[r2,#0]
        LDR      r1,|L1.968|
        LDR      r1,[r1,#0]
        STR      r1,[sp,#0]
        LDR      r1,|L1.968|
        LDR      r1,[r1,#0]
        STR      r1,[sp,#0]
        LDR      r1,|L1.968|
        LDR      r1,[r1,#0]
        STR      r1,[sp,#0]
        LDR      r1,|L1.996|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0xf
        LDR      r2,|L1.996|
        STR      r1,[r2,#0]
        MOV      r1,r2
        LDR      r1,[r1,#0]
        ORR      r1,r1,#1
        STR      r1,[r2,#0]
        MOV      r1,#0x3210
        LDR      r2,|L1.1000|
        STR      r1,[r2,#0x20]
        MOV      r1,r2
        LDR      r1,[r1,#0]
        BIC      r1,r1,#4
        STR      r1,[r2,#0]
        MOV      r1,r2
        LDR      r1,[r1,#0x14]
        BIC      r1,r1,#0xf00
        STR      r1,[r2,#0x14]
        LDR      r1,|L1.1004|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0xf
        STR      r1,[r2,#0x80]
        MOV      r1,r2
        LDR      r1,[r1,#0x80]
        ADD      r1,r1,r0
        LDR      r2,|L1.1004|
        STR      r1,[r2,#0]
        MOV      r1,#6
        LDR      r2,|L1.1000|
        STR      r1,[r2,#0x84]
        MOV      r1,r2
        LDR      r1,[r1,#8]
        BIC      r1,r1,#4
        STR      r1,[r2,#8]
        MOV      r1,r2
        LDR      r1,[r1,#0]
        ORR      r1,r1,#4
        STR      r1,[r2,#0]
        NOP      
        B        |L1.82|
        ENDP

        DCW      0x0000
|L1.968|
        DCD      0x400fe608
|L1.972|
        DCD      0x40024400
|L1.976|
        DCD      0x4002451c
|L1.980|
        DCD      0x40007400
|L1.984|
        DCD      0x4000751c
|L1.988|
        DCD      0x40005400
|L1.992|
        DCD      0x4000551c
|L1.996|
        DCD      0x40038fc4
|L1.1000|
        DCD      0x40038000
|L1.1004|
        DCD      0x40038080

        AREA ||i.ADC0_InitTimer0ATriggerSeq3||, CODE, READONLY, ALIGN=2

ADC0_InitTimer0ATriggerSeq3 PROC
        PUSH     {r3-r5,lr}
        MOV      r4,r0
        MOV      r5,r1
        CMP      r4,#0xc
        BCS      |L2.86|
        TBB      [pc,r4]
        DCB      0x06,0x07
        DCB      0x08,0x09,0x12,0x13
        DCB      0x14,0x15,0x0a,0x0b
        DCB      0x1c,0x1d
        NOP      
        NOP      
        NOP      
        NOP      
        NOP      
        LDR      r0,|L2.1036|
        LDR      r0,[r0,#0]
        ORR      r0,r0,#0x10
        LDR      r1,|L2.1036|
        STR      r0,[r1,#0]
        B        |L2.88|
        NOP      
        NOP      
        NOP      
        LDR      r0,|L2.1036|
        LDR      r0,[r0,#0]
        ORR      r0,r0,#8
        LDR      r1,|L2.1036|
        STR      r0,[r1,#0]
        B        |L2.88|
        NOP      
        LDR      r0,|L2.1036|
        LDR      r0,[r0,#0]
        ORR      r0,r0,#2
        LDR      r1,|L2.1036|
        STR      r0,[r1,#0]
        B        |L2.88|
|L2.86|
        POP      {r3-r5,pc}
|L2.88|
        NOP      
        LDR      r0,|L2.1036|
        LDR      r0,[r0,#0]
        STR      r0,[sp,#0]
        LDR      r0,|L2.1036|
        LDR      r0,[r0,#0]
        STR      r0,[sp,#0]
        CMP      r4,#0xc
        BCS      |L2.178|
        TBB      [pc,r4]
        DCB      0x06,0x23
        DCB      0x40,0x5d,0x7a,0x97
        DCB      0xb4,0xd1,0xf2,0xf0
        DCB      0xef,0xee
        LDR      r0,|L2.1040|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#8
        LDR      r1,|L2.1040|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1040|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#8
        LDR      r1,|L2.1040|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#8
        LDR      r1,|L2.1044|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#8
        LDR      r1,|L2.1044|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
|L2.178|
        B        |L2.826|
        LDR      r0,|L2.1040|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#4
        LDR      r1,|L2.1040|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1040|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#4
        LDR      r1,|L2.1040|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#4
        LDR      r1,|L2.1044|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#4
        LDR      r1,|L2.1044|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        B        |L2.826|
        LDR      r0,|L2.1040|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#2
        LDR      r1,|L2.1040|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1040|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#2
        LDR      r1,|L2.1040|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#2
        LDR      r1,|L2.1044|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#2
        LDR      r1,|L2.1044|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        B        |L2.826|
        LDR      r0,|L2.1040|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#1
        LDR      r1,|L2.1040|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1040|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#1
        LDR      r1,|L2.1040|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#1
        LDR      r1,|L2.1044|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#1
        LDR      r1,|L2.1044|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        B        |L2.826|
        LDR      r0,|L2.1048|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#8
        LDR      r1,|L2.1048|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1048|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#8
        LDR      r1,|L2.1048|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1052|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#8
        LDR      r1,|L2.1052|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1052|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#8
        LDR      r1,|L2.1052|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        B        |L2.826|
        LDR      r0,|L2.1048|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#4
        LDR      r1,|L2.1048|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1048|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#4
        LDR      r1,|L2.1048|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1052|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#4
        LDR      r1,|L2.1052|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1052|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#4
        LDR      r1,|L2.1052|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        B        |L2.826|
        LDR      r0,|L2.1048|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#2
        LDR      r1,|L2.1048|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1048|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#2
        LDR      r1,|L2.1048|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1052|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#2
        LDR      r1,|L2.1052|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1052|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#2
        LDR      r1,|L2.1052|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        B        |L2.826|
        LDR      r0,|L2.1048|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#1
        LDR      r1,|L2.1048|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1048|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#1
        LDR      r1,|L2.1048|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1052|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#1
        LDR      r1,|L2.1052|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1052|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#1
        LDR      r1,|L2.1052|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        B        |L2.826|
        B        |L2.768|
        B        |L2.710|
        B        |L2.652|
        B        |L2.594|
|L2.594|
        LDR      r0,|L2.1040|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#0x20
        LDR      r1,|L2.1040|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1040|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#0x20
        LDR      r1,|L2.1040|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#0x20
        LDR      r1,|L2.1044|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#0x20
        LDR      r1,|L2.1044|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        B        |L2.826|
|L2.652|
        LDR      r0,|L2.1040|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#0x10
        LDR      r1,|L2.1040|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1040|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#0x10
        LDR      r1,|L2.1040|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#0x10
        LDR      r1,|L2.1044|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1044|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#0x10
        LDR      r1,|L2.1044|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        B        |L2.826|
|L2.710|
        LDR      r0,|L2.1056|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#0x10
        LDR      r1,|L2.1056|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1056|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#0x10
        LDR      r1,|L2.1056|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1060|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#0x10
        LDR      r1,|L2.1060|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1060|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#0x10
        LDR      r1,|L2.1060|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        B        |L2.826|
|L2.768|
        LDR      r0,|L2.1056|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#0x20
        LDR      r1,|L2.1056|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1056|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#0x20
        LDR      r1,|L2.1056|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L2.1060|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#0x20
        LDR      r1,|L2.1060|
        STR      r0,[r1,#0]
        LDR      r0,|L2.1060|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#0x20
        LDR      r1,|L2.1060|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        NOP      
|L2.826|
        NOP      
        BL       DisableInterrupts
        LDR      r0,|L2.1036|
        ADDS     r0,r0,#0x30
        LDR      r0,[r0,#0]
        ORR      r0,r0,#1
        LDR      r1,|L2.1036|
        ADDS     r1,r1,#0x30
        STR      r0,[r1,#0]
        LDR      r0,|L2.1036|
        SUBS     r0,r0,#4
        LDR      r0,[r0,#0]
        STR      r0,[sp,#0]
        LDR      r0,|L2.1036|
        SUBS     r0,r0,#4
        LDR      r0,[r0,#0]
        STR      r0,[sp,#0]
        LDR      r0,|L2.1036|
        SUBS     r0,r0,#4
        LDR      r0,[r0,#0]
        ORR      r0,r0,#1
        LDR      r1,|L2.1036|
        SUBS     r1,r1,#4
        STR      r0,[r1,#0]
        MOV      r0,r1
        LDR      r0,[r0,#0]
        STR      r0,[sp,#0]
        MOVS     r0,#0
        LDR      r1,|L2.1064|
        STR      r0,[r1,#0xc]
        MOV      r0,r1
        LDR      r0,[r0,#0xc]
        ORR      r0,r0,#0x20
        STR      r0,[r1,#0xc]
        MOVS     r0,#0
        STR      r0,[r1,#0]
        MOVS     r0,#2
        STR      r0,[r1,#4]
        MOVS     r0,#0
        STR      r0,[r1,#0x38]
        SUBS     r0,r5,#1
        STR      r0,[r1,#0x28]
        MOVS     r0,#0
        STR      r0,[r1,#0x18]
        MOV      r0,r1
        LDR      r0,[r0,#0xc]
        ORR      r0,r0,#1
        STR      r0,[r1,#0xc]
        MOVS     r0,#1
        LDR      r1,|L2.1068|
        STR      r0,[r1,#0]
        MOV      r0,#0x3210
        LDR      r1,|L2.1072|
        STR      r0,[r1,#0x20]
        MOV      r0,r1
        LDR      r0,[r0,#0]
        BIC      r0,r0,#8
        STR      r0,[r1,#0]
        MOV      r0,r1
        LDR      r0,[r0,#0x14]
        BIC      r0,r0,#0xf000
        ADD      r0,r0,#0x5000
        STR      r0,[r1,#0x14]
        LDR      r0,|L2.1076|
        STR      r4,[r0,#0]
        MOV      r0,#6
        STR      r0,[r1,#0xa4]
        MOV      r0,r1
        LDR      r0,[r0,#8]
        ORR      r0,r0,#8
        STR      r0,[r1,#8]
        MOV      r0,r1
        LDR      r0,[r0,#0]
        ORR      r0,r0,#8
        STR      r0,[r1,#0]
        LDR      r0,|L2.1080|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#0xff00
        ORR      r0,r0,#0x4000
        LDR      r1,|L2.1080|
        STR      r0,[r1,#0]
        MOV      r0,#0x20000
        LDR      r1,|L2.1084|
        STR      r0,[r1,#0]
        BL       EnableInterrupts
        NOP      
        B        |L2.86|
        DCW      0x0000
|L2.1036|
        DCD      0x400fe608
|L2.1040|
        DCD      0x40024400
|L2.1044|
        DCD      0x4002451c
|L2.1048|
        DCD      0x40007400
|L2.1052|
        DCD      0x4000751c
|L2.1056|
        DCD      0x40005400
|L2.1060|
        DCD      0x4000551c
|L2.1064|
        DCD      0x40030000
|L2.1068|
        DCD      0x40038fc4
|L2.1072|
        DCD      0x40038000
|L2.1076|
        DCD      0x400380a0
|L2.1080|
        DCD      0xe000e410
|L2.1084|
        DCD      0xe000e100
        ENDP


        AREA ||i.ADC0_InitTimer0ATriggerSeq3PD3||, CODE, READONLY, ALIGN=2

ADC0_InitTimer0ATriggerSeq3PD3 PROC
        PUSH     {r3-r5,lr}
        MOV      r4,r0
        LDR      r0,|L3.268|
        LDR      r0,[r0,#0]
        ORR      r0,r0,#1
        LDR      r1,|L3.268|
        STR      r0,[r1,#0]
        LDR      r0,|L3.268|
        SUBS     r0,r0,#0x30
        LDR      r0,[r0,#0]
        ORR      r0,r0,#8
        LDR      r1,|L3.268|
        SUBS     r1,r1,#0x30
        STR      r0,[r1,#0]
        MOV      r0,r1
        LDR      r0,[r0,#0]
        STR      r0,[sp,#0]
        LDR      r0,|L3.272|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#8
        LDR      r1,|L3.272|
        STR      r0,[r1,#0]
        LDR      r0,|L3.272|
        ADDS     r0,r0,#0x20
        LDR      r0,[r0,#0]
        ORR      r0,r0,#8
        LDR      r1,|L3.272|
        ADDS     r1,r1,#0x20
        STR      r0,[r1,#0]
        LDR      r0,|L3.276|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#8
        LDR      r1,|L3.276|
        STR      r0,[r1,#0]
        LDR      r0,|L3.276|
        ADDS     r0,r0,#0xc
        LDR      r0,[r0,#0]
        ORR      r0,r0,#8
        LDR      r1,|L3.276|
        ADDS     r1,r1,#0xc
        STR      r0,[r1,#0]
        MOVS     r0,#1
        LDR      r1,|L3.280|
        STR      r0,[r1,#0]
        MOV      r0,#0x3210
        LDR      r1,|L3.284|
        STR      r0,[r1,#0x20]
        LDR      r0,|L3.268|
        SUBS     r0,r0,#0x34
        LDR      r0,[r0,#0]
        ORR      r0,r0,#1
        LDR      r1,|L3.268|
        SUBS     r1,r1,#0x34
        STR      r0,[r1,#0]
        ADDS     r0,r1,#4
        LDR      r0,[r0,#0]
        STR      r0,[sp,#0]
        MOVS     r0,#0
        LDR      r1,|L3.288|
        STR      r0,[r1,#0xc]
        MOV      r0,r1
        LDR      r0,[r0,#0xc]
        ORR      r0,r0,#0x20
        STR      r0,[r1,#0xc]
        MOVS     r0,#0
        STR      r0,[r1,#0]
        MOVS     r0,#2
        STR      r0,[r1,#4]
        MOVS     r0,#0
        STR      r0,[r1,#0x38]
        SUBS     r0,r4,#1
        STR      r0,[r1,#0x28]
        MOVS     r0,#0
        STR      r0,[r1,#0x18]
        MOV      r0,r1
        LDR      r0,[r0,#0xc]
        ORR      r0,r0,#1
        STR      r0,[r1,#0xc]
        LDR      r0,|L3.284|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#8
        LDR      r1,|L3.284|
        STR      r0,[r1,#0]
        MOV      r0,r1
        LDR      r0,[r0,#0x14]
        BIC      r0,r0,#0xf000
        ADD      r0,r0,#0x5000
        STR      r0,[r1,#0x14]
        MOVS     r0,#4
        LDR      r1,|L3.292|
        STR      r0,[r1,#0]
        MOV      r0,#6
        LDR      r1,|L3.284|
        STR      r0,[r1,#0xa4]
        MOV      r0,r1
        LDR      r0,[r0,#8]
        ORR      r0,r0,#8
        STR      r0,[r1,#8]
        MOV      r0,r1
        LDR      r0,[r0,#0]
        ORR      r0,r0,#8
        STR      r0,[r1,#0]
        LDR      r0,|L3.296|
        LDR      r0,[r0,#0]
        BIC      r0,r0,#0xff00
        ORR      r0,r0,#0x8000
        LDR      r1,|L3.296|
        STR      r0,[r1,#0]
        MOV      r0,#0x20000
        LDR      r1,|L3.300|
        STR      r0,[r1,#0]
        BL       EnableInterrupts
        POP      {r3-r5,pc}
        ENDP

|L3.268|
        DCD      0x400fe638
|L3.272|
        DCD      0x40007400
|L3.276|
        DCD      0x4000751c
|L3.280|
        DCD      0x40038fc4
|L3.284|
        DCD      0x40038000
|L3.288|
        DCD      0x40030000
|L3.292|
        DCD      0x400380a0
|L3.296|
        DCD      0xe000e410
|L3.300|
        DCD      0xe000e100

        AREA ||i.ADC_Collect||, CODE, READONLY, ALIGN=2

ADC_Collect PROC
        PUSH     {r4-r8,lr}
        MOV      r4,r0
        MOV      r5,r1
        MOV      r6,r2
        LDR      r0,|L4.40|
        STR      r4,[r0,#0]  ; TimerChannelNumber
        LDR      r0,|L4.44|
        STR      r6,[r0,#0]  ; ProducerTask
        LDR      r0,|L4.48|
        UDIV     r7,r0,r5
        MOV      r1,r7
        MOV      r0,r4
        BL       ADC0_InitTimer0ATriggerSeq3
        MOVS     r0,#0
        POP      {r4-r8,pc}
        ENDP

        DCW      0x0000
|L4.40|
        DCD      TimerChannelNumber
|L4.44|
        DCD      ProducerTask
|L4.48|
        DCD      0x04c4b400

        AREA ||i.ADC_In||, CODE, READONLY, ALIGN=2

ADC_In PROC
        MOVS     r1,#4
        LDR      r2,|L5.36|
        STR      r1,[r2,#0x28]
        NOP      
|L5.8|
        LDR      r1,|L5.36|
        LDR      r1,[r1,#4]
        TST      r1,#4
        BEQ      |L5.8|
        LDR      r1,|L5.40|
        LDR      r1,[r1,#0]
        UBFX     r0,r1,#0,#12
        MOV      r1,#4
        LDR      r2,|L5.36|
        STR      r1,[r2,#0xc]
        BX       lr
        ENDP

|L5.36|
        DCD      0x40038000
|L5.40|
        DCD      0x40038088

        AREA ||i.ADC_Init||, CODE, READONLY, ALIGN=2

ADC_Init PROC
        PUSH     {r4,lr}
        MOV      r4,r0
        MOV      r0,r4
        BL       ADC0_InitSWTriggerSeq2
        LDR      r0,|L6.16|
        STR      r4,[r0,#0]  ; ChannelNumber
        POP      {r4,pc}
        ENDP

|L6.16|
        DCD      ChannelNumber

        AREA ||.arm_vfe_header||, DATA, READONLY, NOALLOC, ALIGN=2

        DCD      0x00000000

        AREA ||.data||, DATA, ALIGN=2

ChannelNumber
        DCD      0xffffffff
NumberSamples
        DCD      0x00000000
TimerChannelNumber
        DCD      0xffffffff
BufferName
        DCD      0x00000000
ProducerTask
        DCD      0x00000000
ADCvalue
        DCD      0x00000000

        EXPORT ADC0Seq3_Handler [CODE]
        EXPORT ADC0_InitSWTriggerSeq2 [CODE]
        EXPORT ADC0_InitTimer0ATriggerSeq3 [CODE]
        EXPORT ADC0_InitTimer0ATriggerSeq3PD3 [CODE]
        EXPORT ADC_Collect [CODE]
        EXPORT ADC_In [CODE]
        EXPORT ADC_Init [CODE]
        EXPORT ChannelNumber [DATA,SIZE=4]
        EXPORT NumberSamples [DATA,SIZE=4]
        EXPORT TimerChannelNumber [DATA,SIZE=4]
        EXPORT BufferName [DATA,SIZE=4]
        EXPORT ProducerTask [DATA,SIZE=4]
        EXPORT ADCvalue [DATA,SIZE=4]

        IMPORT ||Lib$$Request$$armlib|| [CODE,WEAK]
        IMPORT DisableInterrupts [CODE]
        IMPORT EnableInterrupts [CODE]

        ATTR FILESCOPE
        ATTR SETVALUE Tag_ABI_PCS_wchar_t,2
        ATTR SETVALUE Tag_ABI_enum_size,1
        ATTR SETVALUE Tag_ABI_optimization_goals,6
        ATTR SETSTRING Tag_conformance,"2.06"
        ATTR SETVALUE AV,18,1

        ASSERT {ENDIAN} = "little"
        ASSERT {INTER} = {TRUE}
        ASSERT {ROPI} = {FALSE}
        ASSERT {RWPI} = {FALSE}
        ASSERT {IEEE_FULL} = {FALSE}
        ASSERT {IEEE_PART} = {FALSE}
        ASSERT {IEEE_JAVA} = {FALSE}
        END

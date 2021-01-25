-stack           0x00010000//0x00000800      /* Stack Size */
-heap            0x01000000//0x00000800      /* Heap Size */




/* MODULE MEM */
MEMORY {
    VECS:        o = 0xFFFF0000  l = 0x00000080
    ARMRAM:      o = 0xFFFF0080  l = 0x00001f80 	//
    L2RAM:       o = 0x11800000  l = 0x00040000
    SHAREDRAM:   o = 0x800000FF  l = 0x0001FF00 	//SHAREDRAM L3RAM //Actual size 20000 but 256 bytes for uboot
    SDRAM:       o = 0xC0000000  l = 0x6000000 		//96MB (32MB Left for UBOOT)
}

SECTIONS
{
  	  vectors           :> L2RAM
     .cinit             :> SDRAM		//SHAREDRAM        
     .text              :> SDRAM		//SHAREDRAM 
     .stack             :> L2RAM		//SHAREDRAM        
     .bss               :> SDRAM		//L2RAM
     .const             :> SDRAM		//L2RAM
     .data              :> L2RAM
     .far               :> SDRAM		//SDRAM
     .switch            :> SDRAM		//L2RAM
     .sysmem            :> SDRAM		//L2RAM
     .tables            :> L2RAM
     .cio               :> SDRAM		//L2RAM
}

SECTIONS
{
    sdram_data: 		load >> SDRAM 
    sharedram_data: 	load >> SHAREDRAM
    l2ram_data:			load >> L2RAM
}

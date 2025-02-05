////////////////////////////////////////////
//                                        //
//        LRU replacement policy          //
//     Jinchun Kim, cienlux@tamu.edu      //
//                                        //
////////////////////////////////////////////

// #include "../inc/champsim_crc2.h"

#define NUM_CORE 1
#define LLC_SETS NUM_CORE*1024 // SM: need to change it based on L2 size
#define LLC_WAYS 8 // // SM: need to change it based on L2 size

//SM
#define PREFETCH  2
#define WRITEBACK 3
#define NUM_TYPES 4
//
uint32_t lru[LLC_SETS][LLC_WAYS];

// initialize replacement state
void LRUInitReplacementState()
{
    cout << "Initialize LRU replacement state" << endl;

    for (int i=0; i<LLC_SETS; i++) {
        for (int j=0; j<LLC_WAYS; j++) {
            lru[i][j] = j;
        }
    }
}

// find replacement victim
// return value should be 0 ~ 15 or 16 (bypass)
uint32_t LRUGetVictimInSet (uint32_t cpu, uint32_t set, uint64_t current_set, uint64_t PC, uint64_t paddr, uint32_t type)
{
    for (int i=0; i<LLC_WAYS; i++)
        if (lru[set][i] == (LLC_WAYS-1))
            return i;

    return 0;
}

// called on every cache hit and cache fill
void LRUUpdateReplacementState (uint32_t cpu, uint32_t set, uint32_t way, uint64_t paddr, uint64_t PC, uint64_t victim_addr, uint32_t type, uint8_t hit)
{
    // update lru replacement state
    for (uint32_t i=0; i<LLC_WAYS; i++) {
        if (lru[set][i] < lru[set][way]) {
            lru[set][i]++;

            if (lru[set][i] == LLC_WAYS)
                assert(0);
        }
    }
    lru[set][way] = 0; // promote to the MRU position
}

// use this function to print out your own stats on every heartbeat 
void LRUPrintStats_Heartbeat()
{

}

// use this function to print out your own stats at the end of simulation
void LRUPrintStats()
{

}

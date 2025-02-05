/*  Hawkeye with Belady's Algorithm Replacement Policy
    Code for Hawkeye configurations of 1 and 2 in Champsim */

// #include "../inc/champsim_crc2.h"
#include <map>
#include <math.h>
#include <cinttypes>  // For PRIu64
#include <fstream>
#define NUM_CORE 1
#define LLC_SETS NUM_CORE*1024 // SM: need to change it based on L2 size
#define LLC_WAYS 8 // SM: need to change it based on L2 size

//3-bit RRIP counter
#define MAXRRIP 7
uint32_t rrip[LLC_SETS][LLC_WAYS];

#include "hawkeye_predictor.h"
#include "optgen.h"
#include "helper_function.h"
#define PREFETCH  2
#define WRITEBACK 3
#define NUM_TYPES 4

std::ofstream output("output.txt", std::ios::out);

//Hawkeye predictors for demand and prefetch requests
Hawkeye_Predictor* predictor_demand;    //2K entries, 5-bit counter per each entry
Hawkeye_Predictor* predictor_prefetch;  //2K entries, 5-bit counter per each entry

OPTgen optgen_occup_vector[LLC_SETS];   //64 vecotrs, 128 entries each

//Prefectching
bool prefetching[LLC_SETS][LLC_WAYS];

//Sampler components tracking cache history
#define SAMPLER_ENTRIES 2800
#define SAMPLER_HIST 8
#define SAMPLER_SETS (SAMPLER_ENTRIES/SAMPLER_HIST)
vector<map<uint64_t, HISTORY>> cache_history_sampler;  //2800 entries, 4-bytes per each entry
uint64_t sample_signature[LLC_SETS][LLC_WAYS];

//History time
#define TIMER_SIZE 1024
uint64_t set_timer[LLC_SETS];   //64 sets, where 1 timer is used for every set

//Mathmatical functions needed for sampling set
#define bitmask(l) (((l) == 64) ? (unsigned long long)(-1LL) : ((1LL << (l))-1LL))
#define bits(x, i, l) (((x) >> (i)) & bitmask(l))
#define SAMPLED_SET(set) (bits(set, 0 , 6) == bits(set, ((unsigned long long)log2(LLC_SETS) - 6), 6) )  //Helper function to sample 64 sets for each core



uint64_t count_cache_averse_evictions = 0;
uint64_t count_evictions = 0;

uint64_t count_pred_as_hit_and_hit = 0;
uint64_t count_pred_as_hit_but_miss = 0;
uint64_t count_pred_as_miss_but_hit = 0;
uint64_t count_pred_as_miss_and_miss = 0;

// Initialize replacement state
void InitReplacementState()
{
    cout << "Initialize Hawkeye replacement policy state" << endl;

    for (int i=0; i<LLC_SETS; i++) {
        for (int j=0; j<LLC_WAYS; j++) {
            rrip[i][j] = MAXRRIP;
            sample_signature[i][j] = 0;
            prefetching[i][j] = false;
        }
        set_timer[i] = 0;
        optgen_occup_vector[i].init(LLC_WAYS-2);
    }

    cache_history_sampler.resize(SAMPLER_SETS);
    for(int i = 0; i < SAMPLER_SETS; i++){
        cache_history_sampler[i].clear();
    }

    predictor_prefetch = new Hawkeye_Predictor();
    predictor_demand = new Hawkeye_Predictor();

    cout << "Finished initializing Hawkeye replacement policy state" << endl;
}

// Find replacement victim
// Return value should be 0 ~ 15 or 16 (bypass)
uint32_t GetVictimInSet (uint32_t cpu, uint32_t set, uint64_t current_set, uint64_t PC, uint64_t paddr, uint32_t type)
{


    //     // Print the function arguments
    // cout << "Function GetVictimInSet called with arguments:"
    //      << "  cpu: " << cpu << ", "
    //      << "  set: " << set << ", "
    //      << "  current_set: " << current_set <<", "
    //      << "  PC: " << PC << ", "
    //      << "  paddr: " << paddr << ", "
    //      << "  type: " << type << endl;



    // if (output.is_open()) {
    // // Output the frequency data
    // output << "output file is open:" << std::endl;
    // output << "Function GetVictimInSet called with arguments:"
    //     << "  cpu: " << cpu << ", "
    //     << "  set: " << set << ", "
    //     << "  current_set: " << current_set <<", "
    //     << "  PC: " << PC << ", "
    //     << "  paddr: " << paddr << ", "
    //     << "  type: " << type << endl;
    // } else {
    //     std::cerr << "Unable to open the output file for writing." << std::endl;
    // }

    // cout << "rrip for set: " << set << " ,for finding the victimway" << endl;
    // for (uint32_t i = 0; i < LLC_WAYS; i++) {
    //     cout << rrip[set][i] << " ";
    // }
    // cout << endl;



    count_evictions++;
    //Find the line with RRPV of 7 in that set
    for(uint32_t i = 0; i < LLC_WAYS; i++){
        if(rrip[set][i] == MAXRRIP){
            count_cache_averse_evictions++;
            // output << "0x" << paddr << " -> VictimWay: " << i << ", set: " << set << std::endl;
            return i;
        }
    }

    //If no RRPV of 7, then we find next highest RRPV value (oldest cache-friendly line)
    uint32_t max_rrpv = 0;
    int32_t victim = -1;
    for(uint32_t i = 0; i < LLC_WAYS; i++){
        if(rrip[set][i] >= max_rrpv){
            max_rrpv = rrip[set][i];
            victim = i;
        }
    }

    //Asserting that LRU victim is not -1
    //Predictor will be trained negaively on evictions
    if(SAMPLED_SET(set)){
        if(prefetching[set][victim]){
            predictor_prefetch->decrease(sample_signature[set][victim]);
        }
        else{
            predictor_demand->decrease(sample_signature[set][victim]);
        }
    }
    // output << "0x" << paddr << " -> VictimWay: " << victim << ", set: " << set << std::endl;
    return victim;
}

//Helper function for "UpdateReplacementState" to update cache history
void update_cache_history(unsigned int sample_set, unsigned int currentVal){
    for(map<uint64_t, HISTORY>::iterator it = cache_history_sampler[sample_set].begin(); it != cache_history_sampler[sample_set].end(); it++){
        if((it->second).lru < currentVal){
            (it->second).lru++;
        }
    }

}

// Called on every cache hit and cache fill
void UpdateReplacementState (uint32_t cpu, uint32_t set, uint32_t way, uint64_t paddr, uint64_t PC, uint64_t victim_addr, uint32_t type, uint8_t hit)
{

    // // // Print the function arguments
    // cout << "Function UpdateReplacementState called with arguments:"
    //      << "  set: " << set << ", "
    //      << "  way: " << way << ", "
    //      << "  paddr: " << paddr << ", "
    //      << "  PC: " << PC << ", "
    //      << "  victim_addr: " << victim_addr << ", "
    //      << "  type: " << type << ", "  
    //      << "  hit: " << static_cast<int>(hit) << endl;    // type cast for correct printing

    // // // write the arguments
    // if (output.is_open()) {
    //     output << "Function UpdateReplacementState called with arguments: "
    //          << "cpu: " << cpu << ", "
    //          << "set: " << set << ", "
    //          << "way: " << way << ", "
    //          << "paddr: " << paddr << ", "
    //          << "PC: " << PC << ", "
    //          << "victim_addr: " << victim_addr << ", "
    //          << "type: " << type << ", "  
    //          << "hit: " << static_cast<int>(hit) << std::endl;  
    //     // output.close();
    // } else {
    //     std::cerr << "Unable to open file for writing." << std::endl;
    // }


    paddr = (paddr >> 6) << 6;

    //Ignore all types that are writebacks
    if(type == WRITEBACK){
        return;
    }

    if(type == PREFETCH){
        if(!hit){
            prefetching[set][way] = true;
        }
    }
    else{
        prefetching[set][way] = false;
    }

    //Only if we are using sampling sets for OPTgen
    if(SAMPLED_SET(set)){
        uint64_t currentVal = set_timer[set] % OPTGEN_SIZE;
        uint64_t sample_tag = CRC(paddr >> 12) % 256;
        uint32_t sample_set = (paddr >> 6) % SAMPLER_SETS;
        // std::cout << "Set " << set << " is sampled." << std::endl;


        //If line has been used before, ignoring prefetching (demand access operation)
        if((type != PREFETCH) && (cache_history_sampler[sample_set].find(sample_tag) != cache_history_sampler[sample_set].end())){
            unsigned int current_time = set_timer[set];
            if(current_time < cache_history_sampler[sample_set][sample_tag].previousVal){
                current_time += TIMER_SIZE;
            }
            uint64_t previousVal = cache_history_sampler[sample_set][sample_tag].previousVal % OPTGEN_SIZE;
            bool isWrap = (current_time - cache_history_sampler[sample_set][sample_tag].previousVal) > OPTGEN_SIZE;

            //Train predictor positvely for last PC value that was prefetched
            if(!isWrap && optgen_occup_vector[set].is_cache(currentVal, previousVal)){
                if(cache_history_sampler[sample_set][sample_tag].prefetching){
                    predictor_prefetch->increase(cache_history_sampler[sample_set][sample_tag].PCval);
                }
                else{
                    predictor_demand->increase(cache_history_sampler[sample_set][sample_tag].PCval);
                }
                //SM-OPTGen decision
                output << "is_cache_result: " << "true"
                << ", set: " << set
                << ", hit: " << static_cast<int>(hit) << std::endl;
            }
            //Train predictor negatively since OPT did not cache this line
            else{
                if(cache_history_sampler[sample_set][sample_tag].prefetching){
                    predictor_prefetch->decrease(cache_history_sampler[sample_set][sample_tag].PCval);
                }
                else{
                    predictor_demand->decrease(cache_history_sampler[sample_set][sample_tag].PCval);
                }
                //SM-OPTGen decision
                output << "is_cache_result: " << "false"
                << ", set: " << set
                << ", hit: " << static_cast<int>(hit) << std::endl;
            }
            
            optgen_occup_vector[set].set_access(currentVal);
            //Update cache history
            update_cache_history(sample_set, cache_history_sampler[sample_set][sample_tag].lru);

            //Mark prefetching as false since demand access
            cache_history_sampler[sample_set][sample_tag].prefetching = false;
        }
        //If line has not been used before, mark as prefetch or demand
        else if(cache_history_sampler[sample_set].find(sample_tag) == cache_history_sampler[sample_set].end()){
            //If sampling, find victim from cache
            if(cache_history_sampler[sample_set].size() == SAMPLER_HIST){
                //Replace the element in the cache history
                uint64_t addr_val = 0;
                for(map<uint64_t, HISTORY>::iterator it = cache_history_sampler[sample_set].begin(); it != cache_history_sampler[sample_set].end(); it++){
                    if((it->second).lru == (SAMPLER_HIST-1)){
                        addr_val = it->first;
                        break;
                    }
                }
                cache_history_sampler[sample_set].erase(addr_val);
            }

            //Create new entry
            cache_history_sampler[sample_set][sample_tag].init();
            //If preftech, mark it as a prefetching or if not, just set the demand access
            if(type == PREFETCH){
                cache_history_sampler[sample_set][sample_tag].set_prefetch();
                optgen_occup_vector[set].set_prefetch(currentVal);
            }
            else{
                optgen_occup_vector[set].set_access(currentVal);
            }

            //Update cache history
            update_cache_history(sample_set, SAMPLER_HIST-1);
        }
        //If line is neither of the two above options, then it is a prefetch line
        else{
            uint64_t previousVal = cache_history_sampler[sample_set][sample_tag].previousVal % OPTGEN_SIZE;
            if(set_timer[set] - cache_history_sampler[sample_set][sample_tag].previousVal < 5*NUM_CORE){
                if(optgen_occup_vector[set].is_cache(currentVal, previousVal)){
                    if(cache_history_sampler[sample_set][sample_tag].prefetching){
                        predictor_prefetch->increase(cache_history_sampler[sample_set][sample_tag].PCval);
                    }
                    else{
                        predictor_demand->increase(cache_history_sampler[sample_set][sample_tag].PCval);
                    }
                }
            }
            cache_history_sampler[sample_set][sample_tag].set_prefetch();
            optgen_occup_vector[set].set_prefetch(currentVal);
            //Update cache history
            update_cache_history(sample_set, cache_history_sampler[sample_set][sample_tag].lru);

        }   
        //Update the sample with time and PC
        cache_history_sampler[sample_set][sample_tag].update(set_timer[set], PC);
        cache_history_sampler[sample_set][sample_tag].lru = 0;
        set_timer[set] = (set_timer[set] + 1) % TIMER_SIZE;
    }

    // DN: stats
    if (predictor_demand->get_prediction(PC) && hit){  // prediction worked
        count_pred_as_hit_and_hit++;
        // cout<< "count_pred_as_hit_and_hit: " << count_pred_as_hit_and_hit << endl;
    }else if (predictor_demand->get_prediction(PC) && !hit){ // misspredicted as hit
        count_pred_as_hit_but_miss++;
        // cout<< "count_pred_as_hit_but_miss: " << count_pred_as_hit_but_miss << endl;
    }else if (!predictor_demand->get_prediction(PC) && hit){ // misspredicted as miss
        count_pred_as_miss_but_hit++;
        // cout<< "count_pred_as_miss_but_hit: " << count_pred_as_miss_but_hit << endl;
    }else{
        count_pred_as_miss_and_miss++;
        // cout<< "count_pred_as_miss_and_miss: " << count_pred_as_miss_and_miss << endl;
    }

    //Retrieve Hawkeye's prediction for line
    bool prediction = predictor_demand->get_prediction(PC);
    if(type == PREFETCH){
        prediction = predictor_prefetch->get_prediction(PC);
    }
    
    // std::cout << "prediction: " << predictor_demand->get_prediction(PC)
    //         << " for Hashed PC: " << static_cast<uint64_t>(CRC(PC) % 2048)
    //         << ", set: " << set
    //         << ", hit: " << static_cast<int>(hit) << std::endl;

//  if (output.is_open()) {
    // output << "prediction: " << predictor_demand->get_prediction(PC)
    // << " for Hashed PC: " << static_cast<uint64_t>(CRC(PC) % 1024)
    // << ", set: " << set
    // << ", hit: " << static_cast<int>(hit) << std::endl;
    //     // output.close();
    // } else {
    //     std::cerr << "Unable to open file for writing." << std::endl;
    // }


    sample_signature[set][way] = PC;
    //Fix RRIP counters with correct RRPVs and age accordingly
    if(!prediction){
        rrip[set][way] = MAXRRIP;
    }
    else{
        rrip[set][way] = 0;
        if(!hit){
            //Verifying RRPV of lines has not saturated
            bool isMaxVal = false;
            for(uint32_t i = 0; i < LLC_WAYS; i++){
                if(rrip[set][i] == MAXRRIP-1){
                    isMaxVal = true;
                }
            }

            //Aging cache-friendly lines that have not saturated
            for(uint32_t i = 0; i < LLC_WAYS; i++){
                if(!isMaxVal && rrip[set][i] < MAXRRIP-1){
                    rrip[set][i]++;
                }
            }
        }
        rrip[set][way] = 0;
    }

}

// Use this function to print out your own stats on every heartbeat 
void PrintStats_Heartbeat()
{
    int hits = 0;
    int access = 0;
    for(int i = 0; i < LLC_SETS; i++){
        hits += optgen_occup_vector[i].get_optgen_hits();
        access += optgen_occup_vector[i].access;
    }
//SM
   static uint64_t last_access_count = 0; // To track previous accesses
    if (access - last_access_count >= 10000) {
        printf("Final OPTGen Hits: %d\n", hits);
        printf("Final OPTGen Access: %d\n", access);
        last_access_count = access;

        if (access > 0) {
            printf("Final OPTGEN Hit Rate: %.2f %%\n", 100.0 * ((double)hits / (double)access));
        } else {
            printf("Final OPTGEN Hit Rate: Undefined (No accesses recorded)\n");
        }
    }


    // printf("Final OPTGen Hits: %d\n", hits);
    // printf("Final OPTGen Access: %d\n", access);

    // if (access > 0) {
    //     printf("Final OPTGEN Hit Rate: %.2f %%\n", 100.0 * ((double)hits / (double)access));
    // } else {
    //     printf("Final OPTGEN Hit Rate: Undefined (No accesses recorded)\n");
    // }

   
    // output<< "Cache-averse Evitions Rate: " << 100 * ( (double)count_cache_averse_evictions/(double)count_evictions )<< endl;

    // output<< endl;
    // int total_pred = count_pred_as_hit_and_hit + count_pred_as_hit_but_miss + count_pred_as_miss_but_hit + count_pred_as_miss_and_miss;
    // output<< "Correctly predicted as hit: " << 100 * ( (double)count_pred_as_hit_and_hit/(double)total_pred )<< endl;
    // output<< "Predicted as hit but miss : " << 100 * ( (double)count_pred_as_hit_but_miss/(double)total_pred )<< endl;
    // output<< "Predicted as miss but hit : " << 100 * ( (double)count_pred_as_miss_but_hit/(double)total_pred )<< endl;
    // output<< "Predicted as miss and miss: " << 100 * ( (double)count_pred_as_miss_and_miss/(double)total_pred )<< endl;

    // output<< endl;
    // if ((count_pred_as_hit_and_hit+count_pred_as_miss_but_hit) != 0){
    // output<< "Predicted hits :  " << 100 * ( (double)count_pred_as_hit_and_hit/(double)(count_pred_as_hit_and_hit+count_pred_as_miss_but_hit)) << endl;
    // } else{
    // output<< "Predicted hits :  " << (double)100 << endl;    
    // }
    // output<< "False positives:  " << 100 * ( (double)count_pred_as_hit_but_miss/(double)(count_pred_as_hit_and_hit+count_pred_as_hit_but_miss)) << endl;
    // output<< "Hit rate (no WB): " << 100 * ( (double)((count_pred_as_hit_and_hit+count_pred_as_miss_but_hit))/(double)(total_pred)) << endl;
}

// Use this function to print out your own stats at the end of simulation
void PrintStats()
{
    // int hits = 0;
    // int access = 0;
    // for(int i = 0; i < LLC_SETS; i++){
    //     hits += optgen_occup_vector[i].get_optgen_hits();
    //     access += optgen_occup_vector[i].access;
    // }

    // cout<< "Final OPTGen Hits: " << hits << endl;
    // cout<< "Final OPTGen Access: " << access << endl;
    // cout<< "Final OPTGEN Hit Rate: " << 100 * ( (double)hits/(double)access )<< endl;

    // output<< endl;
    // int total_pred = count_pred_as_hit_and_hit + count_pred_as_hit_but_miss + count_pred_as_miss_but_hit + count_pred_as_miss_and_miss;
    // output<< "Correctly predicted as hit: " << 100 * ( (double)count_pred_as_hit_and_hit/(double)total_pred )<< endl;
    // output<< "Predicted as hit but miss : " << 100 * ( (double)count_pred_as_hit_but_miss/(double)total_pred )<< endl;
    // output<< "Predicted as miss but hit : " << 100 * ( (double)count_pred_as_miss_but_hit/(double)total_pred )<< endl;
    // output<< "Predicted as miss and miss: " << 100 * ( (double)count_pred_as_miss_and_miss/(double)total_pred )<< endl;

    // output<< endl;
    // if ((count_pred_as_hit_and_hit+count_pred_as_miss_but_hit) != 0){
    // output<< "Predicted hits :  " << 100 * ( (double)count_pred_as_hit_and_hit/(double)(count_pred_as_hit_and_hit+count_pred_as_miss_but_hit)) << endl;
    // } else{
    // output<< "Predicted hits :  " << (double)100 << endl;    
    // }
    // output<< "False positives:  " << 100 * ( (double)count_pred_as_hit_but_miss/(double)(count_pred_as_hit_and_hit+count_pred_as_hit_but_miss)) << endl;
    // output<< "Hit rate (no WB): " << 100 * ( (double)((count_pred_as_hit_and_hit+count_pred_as_miss_but_hit))/(double)(total_pred)) << endl;

    // output << "Total accesses: " << total_pred << endl;
    // output << "Total hits    : "     << count_pred_as_hit_and_hit+count_pred_as_miss_but_hit << endl;

}
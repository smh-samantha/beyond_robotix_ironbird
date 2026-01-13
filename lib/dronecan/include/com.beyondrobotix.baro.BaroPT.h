
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>




#define COM_BEYONDROBOTIX_BARO_BAROPT_MAX_SIZE 9
#define COM_BEYONDROBOTIX_BARO_BAROPT_SIGNATURE (0x5EA3647FF1A27A7BULL)

#define COM_BEYONDROBOTIX_BARO_BAROPT_ID 20060





#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class com_beyondrobotix_baro_BaroPT_cxx_iface;
#endif


struct com_beyondrobotix_baro_BaroPT {

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = com_beyondrobotix_baro_BaroPT_cxx_iface;
#endif




    uint8_t sensor_id;



    float pressure_pa;



    float temperature_c;



};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t com_beyondrobotix_baro_BaroPT_encode(struct com_beyondrobotix_baro_BaroPT* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool com_beyondrobotix_baro_BaroPT_decode(const CanardRxTransfer* transfer, struct com_beyondrobotix_baro_BaroPT* msg);

#if defined(CANARD_DSDLC_INTERNAL)

static inline void _com_beyondrobotix_baro_BaroPT_encode(uint8_t* buffer, uint32_t* bit_ofs, struct com_beyondrobotix_baro_BaroPT* msg, bool tao);
static inline bool _com_beyondrobotix_baro_BaroPT_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct com_beyondrobotix_baro_BaroPT* msg, bool tao);
void _com_beyondrobotix_baro_BaroPT_encode(uint8_t* buffer, uint32_t* bit_ofs, struct com_beyondrobotix_baro_BaroPT* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->sensor_id);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->pressure_pa);

    *bit_ofs += 32;






    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->temperature_c);

    *bit_ofs += 32;





}

/*
 decode com_beyondrobotix_baro_BaroPT, return true on failure, false on success
*/
bool _com_beyondrobotix_baro_BaroPT_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct com_beyondrobotix_baro_BaroPT* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;





    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->sensor_id);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->pressure_pa);

    *bit_ofs += 32;







    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->temperature_c);

    *bit_ofs += 32;





    return false; /* success */

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct com_beyondrobotix_baro_BaroPT sample_com_beyondrobotix_baro_BaroPT_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>


BROADCAST_MESSAGE_CXX_IFACE(com_beyondrobotix_baro_BaroPT, COM_BEYONDROBOTIX_BARO_BAROPT_ID, COM_BEYONDROBOTIX_BARO_BAROPT_SIGNATURE, COM_BEYONDROBOTIX_BARO_BAROPT_MAX_SIZE);


#endif
#endif

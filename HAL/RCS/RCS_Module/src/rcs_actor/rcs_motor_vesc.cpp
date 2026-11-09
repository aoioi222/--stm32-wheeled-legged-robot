#include "rcs_motor_vesc.h"

rcs_motor_vesc::rcs_motor_vesc(uint16_t id)
{
    this->id = id;
}

rcs_err_t rcs_motor_vesc::set_port(rcs_can_t can)
{
    if (this->can != NULL){
        this->can = can;
        vesc_id.push_back(this->id);
        vesc_can.push_back(can);
        return RCS_OK;
    }
    return RCS_ERR_MULTIPLE_USED;
}

rcs_err_t rcs_motor_vesc::update_current(float current)
{

}
/** ************************************************************************
 * @file
 * @copyright DAKO-CZ, a.s.
 * @copyright Mechatronics
 * @copyright Josefa Da�ka 1956
 * @copyright 538 43 T�emo�nice, Czech Republic
 *
 * $HeadURL$
 * $Revision$    01.00
 * $Date$        11.11.2019
 * $Author$      V.Stritesky
 *
 * @addtogroup mov_avg
 * @{
 *
 * @brief    EEPROM One-Wire
 * ************************************************************************* */

#define __MOV_AVG_INTERNAL_H__
#include "mov_avg.h"

/* =========================================================================
 *                    Definition of external objects
 * ========================================================================= */

/* =========================================================================
 *     Definition of the module global variables shared only within the module
 * ========================================================================= */

/* =========================================================================
 *   Definition of global functions or procedures that is adjustable also
 *                            from other modules
 * ========================================================================= */

/** *************************************************************************
 * @brief   Moving average structure (s_MOV_AVG_32) initialization.
 * @param   [in]   s_MOV_AVG_32 pointer to avg structure
 *                 uint32_t pointer to buffer for calculation
 *                 uint32_t length of buffer
 * @param   [out]  n/a
 * @param   [err]  n/a
 *
 * @return  n/a
 * ************************************************************************** */
void xv_mov_avg_init(s_MOV_AVG_32* avg_struct, uint32_t* buffer, uint32_t buf_len)
{
    avg_struct->average = 0;
    avg_struct->buffer = buffer;
    avg_struct->init_index = 0;
    avg_struct->last_index = 0;
    avg_struct->buf_len = buf_len;
    avg_struct->sum = 0;

    for(uint32_t i = 0; i < avg_struct->buf_len; i++)
    {
        avg_struct->buffer[i] = 0;
    }
}

/** *************************************************************************
 * @brief   Same as @ref xv_mov_avg_init, but skip init phase, filling buffer
 *          by init value.
 * @param   [in]   s_MOV_AVG_32 pointer to avg structure
 *                 uint32_t pointer to buffer for calculation
 *                 uint32_t length of buffer
 * @param   [out]  n/a
 * @param   [err]  n/a
 *
 * @return  n/a
 * ************************************************************************** */
void xv_mov_avg_init_val(s_MOV_AVG_32* avg_struct, uint32_t* buffer, uint32_t buf_len, uint32_t init_val)
{
    avg_struct->average = init_val;
    avg_struct->buffer = buffer;
    avg_struct->init_index = buf_len;
    avg_struct->last_index = 0;
    avg_struct->buf_len = buf_len;
    avg_struct->sum = buf_len*init_val;

    //fill buffer by init value
    for(uint32_t i = 0; i < avg_struct->buf_len; i++)
    {
        avg_struct->buffer[i] = init_val;
    }

}

/** *************************************************************************
 * @brief   Adds new value to moving average calculation.
 * @param   [in]   s_MOV_AVG_32 pointer to avg structure
 *                 uint32_t new value to average calculation
 * @param   [out]  n/a
 * @param   [err]  n/a
 *
 * @return  Calculated average
 * ************************************************************************** */
uint32_t xu32_mov_avg_add(s_MOV_AVG_32* avg_struct, uint32_t new_value)
{
    //init sequence
    if(avg_struct->init_index < avg_struct->buf_len)
    {
        avg_struct->buffer[avg_struct->init_index] = new_value;
        avg_struct->sum += new_value;
        avg_struct->last_index = avg_struct->init_index;

        avg_struct->init_index++;
        avg_struct->average = avg_struct->sum/avg_struct->init_index;
    }
    //moving average
    else
    {
        if(++avg_struct->last_index >= avg_struct->buf_len)
        {
            //avg_struct->last_index -= avg_struct->buf_len;
            avg_struct->last_index = 0;
        }
        avg_struct->sum -= avg_struct->buffer[avg_struct->last_index];
        avg_struct->buffer[avg_struct->last_index] = new_value;
        avg_struct->sum += new_value;
        avg_struct->average = avg_struct->sum/avg_struct->buf_len;
    }

    return avg_struct->average;
}

/** *************************************************************************
 * @brief   Returns last calculated average
 * @param   [in]   Average struct (s_MOV_AVG_32) pointer
 * @param   [out]  n/a
 * @param   [err]  n/a
 *
 * @return  Average value
 * ************************************************************************** */
uint32_t xu32_mov_avg_get_avg(s_MOV_AVG_32* avg_struct)
{
    return avg_struct->average;
}

/* =========================================================================
*    Definition of local functions and procedures that is callable only
*                              from the module
* ========================================================================= */

/* =========================================================================
*                    Definition of local Interrupt handlers
* ========================================================================= */

/** @} */

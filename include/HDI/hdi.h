/******************************************************************************
 *
 *   Copyright (C) 2007
 *
 *   GETRAG
 *   Getriebe- und Zahnradfabri
 *   Hermann Hagenmeyer
 *   GmbH & Cie KG
 *
 *******************************************************************************
 *
 * File: hdi.h
 *
 * Real-Time Workshop code generated for Simulink model hdi.
 *
 * Model version                        : 1.716
 * Real-Time Workshop file version      : 8.6 (R2014a) 27-Dec-2013
 * Real-Time Workshop file generated on : Tue Mar 27 10:13:16 2018
 * TLC version                          : 8.6 (Jan 30 2014)
 * C/C++ source code generated on       : Tue Mar 27 10:13:16 2018
 *
 ******************************************************************************/

#ifndef RTW_HEADER_hdi_h_
#define RTW_HEADER_hdi_h_
#include <stddef.h>
#ifndef hdi_COMMON_INCLUDES_
# define hdi_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* hdi_COMMON_INCLUDES_ */

#include "hdi_types.h"
#include "stm32f4xx_hal.h"
/* Macros for accessing real-time model data structure */
#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((void*) 0)
#endif

/* Exported data define */

/* Definition for custom storage class: Define */
#define HDI_DUMMY                      0

/* Block states (auto storage) for system '<S1>/Subsystem' */
typedef struct {
  uint16_T Delay_DSTATE;               /* '<S3>/Delay' */
} rtDW_time_comparator_function;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  rtDW_time_comparator_function Subsystem;/* '<S1>/Subsystem' */
} D_Work_hdi;

/* Invariant block signals for system '<S1>/Subsystem' */
typedef struct {
  const uint16_T DataTypeConversion;   /* '<S3>/Data Type Conversion' */
} rtC_time_comparator_function;

/* Invariant block signals (auto storage) */
typedef struct {
  rtC_time_comparator_function Subsystem;/* '<S1>/Subsystem' */
} ConstBlockIO_hdi;

/* Block states (auto storage) */
extern D_Work_hdi hdi_DWork;
extern const ConstBlockIO_hdi hdi_ConstB;/* constant block i/o */

/* Model entry point functions */
extern void hdi_initialize(void);
extern void hdi_step(void);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'hdi'
 * '<S1>'   : 'hdi/hdi'
 * '<S2>'   : 'hdi/hdi/Subsystem'
 * '<S3>'   : 'hdi/hdi/Subsystem/time_comparator'
 */
#endif                                 /* RTW_HEADER_hdi_h_ */

/*======================== TOOL VERSION INFORMATION ==========================*
 * MATLAB 8.3 (R2014a)27-Dec-2013                                             *
 * Simulink 8.3 (R2014a)27-Dec-2013                                           *
 * Simulink Coder 8.6 (R2014a)27-Dec-2013                                     *
 * Embedded Coder 6.6 (R2014a)27-Dec-2013                                     *
 * Stateflow 8.3 (R2014a)27-Dec-2013                                          *
 * Fixed-Point Designer 4.2 (R2014a)27-Dec-2013                               *
 *============================================================================*/

/*======================= LICENSE IN USE INFORMATION =========================*
 * matlab                                                                     *
 * matlab_coder                                                               *
 * real-time_workshop                                                         *
 * rtw_embedded_coder                                                         *
 * simulink                                                                   *
 * stateflow                                                                  *
 *============================================================================*/

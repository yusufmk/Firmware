//
// Prerelease License - for engineering feedback and testing purposes
// only. Not for sale.
//
// File: TestConnExample.cpp
//
// Code generated for Simulink model 'TestConnExample'.
//
// Model version                  : 1.86
// Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
// C/C++ source code generated on : Fri Jan 11 08:30:05 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "TestConnExample.h"
#include "TestConnExample_private.h"

// Block signals (default storage)
B_TestConnExample_T TestConnExample_B;

// Block states (default storage)
DW_TestConnExample_T TestConnExample_DW;

// Real-time model
RT_MODEL_TestConnExample_T TestConnExample_M_;
RT_MODEL_TestConnExample_T *const TestConnExample_M = &TestConnExample_M_;

// Forward declaration for local functions
static void TestConnExample_nullAssignment(real_T x_data[], int32_T x_size[2],
  const boolean_T idx_data[], const int32_T idx_size[2]);
static void TestConnExam_SystemCore_release(const
  px4_internal_block_PX4SCIRead_T *obj);
static void TestConnExamp_SystemCore_delete(const
  px4_internal_block_PX4SCIRead_T *obj);
static void matlabCodegenHandle_matlabCodeg(px4_internal_block_PX4SCIRead_T *obj);
static void TestConnEx_SystemCore_release_g(const
  px4_internal_block_PX4SCIWrit_T *obj);
static void TestConnExa_SystemCore_delete_g(const
  px4_internal_block_PX4SCIWrit_T *obj);
static void matlabCodegenHandle_matlabCod_g(px4_internal_block_PX4SCIWrit_T *obj);
static void TestConn_SystemCore_release_gyo(const
  px4_internal_block_Subscriber_T *obj);
static void TestConnE_SystemCore_delete_gyo(const
  px4_internal_block_Subscriber_T *obj);
static void matlabCodegenHandle_matlabC_gyo(px4_internal_block_Subscriber_T *obj);

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void TestConnExample_nullAssignment(real_T x_data[], int32_T x_size[2],
  const boolean_T idx_data[], const int32_T idx_size[2])
{
  int32_T nxout;
  int32_T k0;
  int32_T k;
  nxout = 0;
  for (k0 = 0; k0 < idx_size[1]; k0++) {
    nxout += idx_data[k0];
  }

  nxout = x_size[1] - nxout;
  k0 = -1;
  for (k = 0; k < x_size[1]; k++) {
    if ((k + 1 > idx_size[1]) || (!idx_data[k])) {
      k0++;
      x_data[k0] = x_data[k];
    }
  }

  if (1 > nxout) {
    x_size[1] = 0;
  } else {
    x_size[1] = nxout;
  }
}

static void TestConnExam_SystemCore_release(const
  px4_internal_block_PX4SCIRead_T *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    MW_SCI_Close(obj->MW_SCIHANDLE);
  }
}

static void TestConnExamp_SystemCore_delete(const
  px4_internal_block_PX4SCIRead_T *obj)
{
  TestConnExam_SystemCore_release(obj);
}

static void matlabCodegenHandle_matlabCodeg(px4_internal_block_PX4SCIRead_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    TestConnExamp_SystemCore_delete(obj);
  }
}

static void TestConnEx_SystemCore_release_g(const
  px4_internal_block_PX4SCIWrit_T *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    MW_SCI_Close(obj->MW_SCIHANDLE);
  }
}

static void TestConnExa_SystemCore_delete_g(const
  px4_internal_block_PX4SCIWrit_T *obj)
{
  TestConnEx_SystemCore_release_g(obj);
}

static void matlabCodegenHandle_matlabCod_g(px4_internal_block_PX4SCIWrit_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    TestConnExa_SystemCore_delete_g(obj);
  }
}

static void TestConn_SystemCore_release_gyo(const
  px4_internal_block_Subscriber_T *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    uORB_read_terminate(&obj->eventStructObj);
  }
}

static void TestConnE_SystemCore_delete_gyo(const
  px4_internal_block_Subscriber_T *obj)
{
  TestConn_SystemCore_release_gyo(obj);
}

static void matlabCodegenHandle_matlabC_gyo(px4_internal_block_Subscriber_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    TestConnE_SystemCore_delete_gyo(obj);
  }
}

// Model step function
void TestConnExample_step(void)
{
  uint8_T sizeptr;
  boolean_T packetfound;
  int32_T idx;
  int32_T b_ii;
  boolean_T p;
  boolean_T b_p;
  uint8_T TxDataLocChar[2];
  int32_T rtb_en;
  int32_T loop_ub;
  int32_T tmp_size[2];
  int32_T start_size[2];
  real_T tmp;
  boolean_T exitg1;

  // MATLABSystem: '<Root>/Serial Receive1' incorporates:
  //   DataTypeConversion: '<S1>/Data Type Conversion'
  //   Inport: '<S9>/In1'
  //   MATLAB Function: '<S1>/MATLAB Function'
  //   MATLABSystem: '<S8>/SourceBlock'

  if (TestConnExample_DW.obj_p.SampleTime !=
      TestConnExample_P.SerialReceive1_SampleTime) {
    TestConnExample_DW.obj_p.SampleTime =
      TestConnExample_P.SerialReceive1_SampleTime;
  }

  MW_SCI_GetDataBytesAvailable(TestConnExample_DW.obj_p.MW_SCIHANDLE, false,
    &sizeptr, 5);
  if (sizeptr >= 10U) {
    MW_SCI_Receive(TestConnExample_DW.obj_p.MW_SCIHANDLE,
                   TestConnExample_B.RxDataLocChar, 10U);
    memcpy((void *)&TestConnExample_B.RxData[0], (void *)
           &TestConnExample_B.RxDataLocChar[0], (uint32_T)((size_t)10 * sizeof
            (uint8_T)));

    // Outputs for Enabled SubSystem: '<Root>/Subsystem2' incorporates:
    //   EnablePort: '<S1>/Enable'

    // MATLAB Function: '<S1>/MATLAB Function'
    rtb_en = 3;
    TestConnExample_B.packet_data[0] = 0U;
    TestConnExample_B.packet_data[1] = 0U;
    TestConnExample_B.packet_data[2] = 0U;
    packetfound = false;
    for (b_ii = 0; b_ii < 10; b_ii++) {
      TestConnExample_B.x[b_ii] = (TestConnExample_B.RxData[b_ii] == 7);
    }

    idx = 0;
    b_ii = 0;
    exitg1 = false;
    while ((!exitg1) && (b_ii < 10)) {
      if (TestConnExample_B.x[b_ii]) {
        idx++;
        TestConnExample_B.ii_data[idx - 1] = static_cast<int8_T>((b_ii + 1));
        if (idx >= 10) {
          exitg1 = true;
        } else {
          b_ii++;
        }
      } else {
        b_ii++;
      }
    }

    if (1 > idx) {
      idx = 0;
    }

    TestConnExample_B.start_size[0] = 1;
    TestConnExample_B.start_size[1] = idx;
    loop_ub = idx - 1;
    for (b_ii = 0; b_ii <= loop_ub; b_ii++) {
      TestConnExample_B.start_data[b_ii] = TestConnExample_B.ii_data[b_ii];
    }

    start_size[0] = 1;
    start_size[1] = idx;
    for (b_ii = 0; b_ii < idx; b_ii++) {
      TestConnExample_B.x[b_ii] = ((TestConnExample_B.start_data[b_ii] + 2.0) -
        1.0 > 10.0);
    }

    TestConnExample_nullAssignment(TestConnExample_B.start_data,
      TestConnExample_B.start_size, TestConnExample_B.x, start_size);

    // MATLAB Function: '<S1>/MATLAB Function'
    tmp_size[0] = 1;
    tmp_size[1] = TestConnExample_B.start_size[1];
    loop_ub = TestConnExample_B.start_size[0] * TestConnExample_B.start_size[1];
    for (b_ii = 0; b_ii < loop_ub; b_ii++) {
      TestConnExample_B.x[b_ii] = (TestConnExample_B.RxData[static_cast<int32_T>
        (((TestConnExample_B.start_data[b_ii] + 2.0) - 1.0)) - 1] != 7);
    }

    TestConnExample_nullAssignment(TestConnExample_B.start_data,
      TestConnExample_B.start_size, TestConnExample_B.x, tmp_size);

    // MATLAB Function: '<S1>/MATLAB Function'
    if (TestConnExample_B.start_size[1] != 0) {
      b_ii = 0;
      exitg1 = false;
      while ((!exitg1) && (b_ii <= TestConnExample_B.start_size[1] - 1)) {
        p = false;
        b_p = true;
        tmp = TestConnExample_B.start_data[1 + b_ii];
        if (!(tmp - TestConnExample_B.start_data[b_ii] == 5.0)) {
          b_p = false;
        }

        if (b_p) {
          p = true;
        }

        if (p) {
          if (TestConnExample_B.start_data[b_ii] + 2.0 > tmp - 1.0) {
            idx = 1;
            rtb_en = 0;
          } else {
            idx = static_cast<int32_T>((TestConnExample_B.start_data[b_ii] + 2.0));
            rtb_en = static_cast<int32_T>((TestConnExample_B.start_data[1 + b_ii]
              - 1.0));
          }

          loop_ub = rtb_en - idx;
          rtb_en = loop_ub + 1;
          for (b_ii = 0; b_ii <= loop_ub; b_ii++) {
            TestConnExample_B.packet_data[b_ii] = TestConnExample_B.RxData[(idx
              + b_ii) - 1];
          }

          packetfound = true;
          exitg1 = true;
        } else {
          b_ii++;
        }
      }
    }

    if (packetfound) {
      packetfound = false;
      p = false;
      if (rtb_en == 3) {
        p = true;
      }

      if (p && (rtb_en != 0)) {
        rtb_en = 0;
        exitg1 = false;
        while ((!exitg1) && (rtb_en < 3)) {
          if (TestConnExample_B.packet_data[rtb_en] != 9) {
            p = false;
            exitg1 = true;
          } else {
            rtb_en++;
          }
        }
      }

      if (p) {
        packetfound = true;
      }

      rtb_en = packetfound;
    } else {
      rtb_en = 0;
    }

    // Outputs for Enabled SubSystem: '<S1>/Subsystem' incorporates:
    //   EnablePort: '<S3>/Enable'

    if (static_cast<int32_T>(fmod((real_T)rtb_en, 256.0)) > 0) {
      // S-Function (fcncallgen): '<S3>/Function-Call Generator'
      for (idx = 0; idx < 10; idx++) {
        // Outputs for Function Call SubSystem: '<S3>/Subsystem'
        // S-Function (fcncallgen): '<S4>/Function-Call Generator' incorporates:
        //   SubSystem: '<S4>/Header Subsystem'

        // MATLABSystem: '<S6>/Serial Transmit' incorporates:
        //   Constant: '<S6>/Header'

        memcpy((void *)&TxDataLocChar[0], (void *)
               &TestConnExample_P.Header_Value[0], (uint32_T)((size_t)2 * sizeof
                (uint8_T)));
        MW_SCI_Transmit(TestConnExample_DW.obj_l.MW_SCIHANDLE, TxDataLocChar, 2U);

        // S-Function (fcncallgen): '<S4>/Function-Call Generator' incorporates:
        //   SubSystem: '<S4>/Data - Subsystem1'

        // MATLABSystem: '<S8>/SourceBlock'
        packetfound = uORB_read_step(TestConnExample_DW.obj.orbMetadataObj,
          &TestConnExample_DW.obj.eventStructObj,
          &TestConnExample_B.b_varargout_2, false, 1.0);

        // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
        //   EnablePort: '<S9>/Enable'

        if (packetfound) {
          TestConnExample_B.In1 = TestConnExample_B.b_varargout_2;
        }

        // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'

        // SignalConversion: '<S5>/TmpSignal ConversionAtSerial TransmitInport1' incorporates:
        //   Inport: '<S9>/In1'
        //   MATLABSystem: '<S8>/SourceBlock'

        TestConnExample_B.TmpSignalConversionAtSerial[0] =
          TestConnExample_B.In1.x;
        TestConnExample_B.TmpSignalConversionAtSerial[1] =
          TestConnExample_B.In1.y;
        TestConnExample_B.TmpSignalConversionAtSerial[2] =
          TestConnExample_B.In1.z;

        // MATLABSystem: '<S5>/Serial Transmit'
        memcpy((void *)&TestConnExample_B.TxDataLocChar[0], (void *)
               &TestConnExample_B.TmpSignalConversionAtSerial[0], (uint32_T)
               ((size_t)12 * sizeof(uint8_T)));
        MW_SCI_Transmit(TestConnExample_DW.obj_e.MW_SCIHANDLE,
                        TestConnExample_B.TxDataLocChar, 12U);

        // End of Outputs for S-Function (fcncallgen): '<S4>/Function-Call Generator' 
        // End of Outputs for SubSystem: '<S3>/Subsystem'
      }

      // End of Outputs for S-Function (fcncallgen): '<S3>/Function-Call Generator' 
    }

    // End of Outputs for SubSystem: '<S1>/Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subsystem2'
  }

  // End of MATLABSystem: '<Root>/Serial Receive1'
}

// Model initialize function
void TestConnExample_initialize(void)
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus(TestConnExample_M, (NULL));

  // block I/O
  (void) memset(((void *) &TestConnExample_B), 0,
                sizeof(B_TestConnExample_T));

  // states (dwork)
  (void) memset((void *)&TestConnExample_DW, 0,
                sizeof(DW_TestConnExample_T));

  {
    px4_internal_block_PX4SCIRead_T *obj;
    uint32_T RxPinLoc;
    uint32_T TxPinLoc;
    MW_SCI_StopBits_Type StopBitsValue;
    MW_SCI_Parity_Type ParityValue;
    MW_SCI_HardwareFlowControl_Type HardwareFlowControlValue;
    px4_internal_block_PX4SCIWrit_T *obj_0;
    static const char_T tmp[12] = { '/', 'd', 'e', 'v', '/', 't', 't', 'y', 'A',
      'C', 'M', '0' };

    int32_T i;

    // Start for MATLABSystem: '<Root>/Serial Receive1' incorporates:
    //   MATLABSystem: '<S5>/Serial Transmit'
    //   MATLABSystem: '<S6>/Serial Transmit'

    TestConnExample_DW.obj_p.matlabCodegenIsDeleted = true;
    TestConnExample_DW.obj_p.isInitialized = 0;
    TestConnExample_DW.obj_p.matlabCodegenIsDeleted = false;
    TestConnExample_DW.obj_p.SampleTime =
      TestConnExample_P.SerialReceive1_SampleTime;
    obj = &TestConnExample_DW.obj_p;
    TestConnExample_DW.obj_p.isSetupComplete = false;
    TestConnExample_DW.obj_p.isInitialized = 1;
    RxPinLoc = MW_UNDEFINED_VALUE;
    TxPinLoc = MW_UNDEFINED_VALUE;
    for (i = 0; i < 12; i++) {
      // Start for Enabled SubSystem: '<Root>/Subsystem2'
      // Start for Enabled SubSystem: '<S1>/Subsystem'
      // Start for S-Function (fcncallgen): '<S3>/Function-Call Generator' incorporates:
      //   SubSystem: '<S3>/Subsystem'

      // Start for S-Function (fcncallgen): '<S4>/Function-Call Generator' incorporates:
      //   SubSystem: '<S4>/Data - Subsystem1'

      // Start for S-Function (fcncallgen): '<S4>/Function-Call Generator' incorporates:
      //   SubSystem: '<S4>/Header Subsystem'

      TestConnExample_B.SCIModuleLoc_tmp[i] = tmp[i];

      // End of Start for S-Function (fcncallgen): '<S4>/Function-Call Generator' 
      // End of Start for S-Function (fcncallgen): '<S3>/Function-Call Generator' 
      // End of Start for SubSystem: '<S1>/Subsystem'
      // End of Start for SubSystem: '<Root>/Subsystem2'
    }

    // Start for Enabled SubSystem: '<Root>/Subsystem2'
    // Start for Enabled SubSystem: '<S1>/Subsystem'
    // Start for S-Function (fcncallgen): '<S3>/Function-Call Generator' incorporates:
    //   SubSystem: '<S3>/Subsystem'

    // Start for S-Function (fcncallgen): '<S4>/Function-Call Generator' incorporates:
    //   SubSystem: '<S4>/Data - Subsystem1'

    // Start for S-Function (fcncallgen): '<S4>/Function-Call Generator' incorporates:
    //   SubSystem: '<S4>/Header Subsystem'

    TestConnExample_B.SCIModuleLoc_tmp[12] = '\x00';

    // End of Start for S-Function (fcncallgen): '<S4>/Function-Call Generator'
    // End of Start for S-Function (fcncallgen): '<S3>/Function-Call Generator'
    // End of Start for SubSystem: '<S1>/Subsystem'
    // End of Start for SubSystem: '<Root>/Subsystem2'
    obj->MW_SCIHANDLE = MW_SCI_Open(TestConnExample_B.SCIModuleLoc_tmp, true,
      RxPinLoc, TxPinLoc);
    MW_SCI_SetBaudrate(TestConnExample_DW.obj_p.MW_SCIHANDLE, 9600U);
    StopBitsValue = MW_SCI_STOPBITS_1;
    ParityValue = MW_SCI_PARITY_NONE;
    MW_SCI_SetFrameFormat(TestConnExample_DW.obj_p.MW_SCIHANDLE, 8, ParityValue,
                          StopBitsValue);
    RxPinLoc = MW_UNDEFINED_VALUE;
    TxPinLoc = MW_UNDEFINED_VALUE;
    HardwareFlowControlValue = MW_SCI_FLOWCONTROL_RTS_CTS;
    MW_SCI_ConfigureHardwareFlowControl(TestConnExample_DW.obj_p.MW_SCIHANDLE,
      HardwareFlowControlValue, RxPinLoc, TxPinLoc);
    TestConnExample_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/Serial Receive1'

    // Start for Enabled SubSystem: '<Root>/Subsystem2'
    // Start for Enabled SubSystem: '<S1>/Subsystem'
    // Start for S-Function (fcncallgen): '<S3>/Function-Call Generator' incorporates:
    //   SubSystem: '<S3>/Subsystem'

    // Start for S-Function (fcncallgen): '<S4>/Function-Call Generator' incorporates:
    //   SubSystem: '<S4>/Header Subsystem'

    // Start for MATLABSystem: '<S6>/Serial Transmit'
    TestConnExample_DW.obj_l.matlabCodegenIsDeleted = true;
    TestConnExample_DW.obj_l.isInitialized = 0;
    TestConnExample_DW.obj_l.matlabCodegenIsDeleted = false;
    obj_0 = &TestConnExample_DW.obj_l;
    TestConnExample_DW.obj_l.isSetupComplete = false;
    TestConnExample_DW.obj_l.isInitialized = 1;
    RxPinLoc = MW_UNDEFINED_VALUE;
    TxPinLoc = MW_UNDEFINED_VALUE;
    obj_0->MW_SCIHANDLE = MW_SCI_Open(TestConnExample_B.SCIModuleLoc_tmp, true,
      RxPinLoc, TxPinLoc);
    MW_SCI_SetBaudrate(TestConnExample_DW.obj_l.MW_SCIHANDLE, 9600U);
    StopBitsValue = MW_SCI_STOPBITS_1;
    ParityValue = MW_SCI_PARITY_NONE;
    MW_SCI_SetFrameFormat(TestConnExample_DW.obj_l.MW_SCIHANDLE, 8, ParityValue,
                          StopBitsValue);
    TestConnExample_DW.obj_l.isSetupComplete = true;

    // Start for S-Function (fcncallgen): '<S4>/Function-Call Generator' incorporates:
    //   SubSystem: '<S4>/Data - Subsystem1'

    // Start for MATLABSystem: '<S8>/SourceBlock'
    TestConnExample_DW.obj.matlabCodegenIsDeleted = true;
    TestConnExample_DW.obj.isInitialized = 0;
    TestConnExample_DW.obj.ticksUntilNextHit = 0.0;
    TestConnExample_DW.obj.matlabCodegenIsDeleted = false;
    TestConnExample_DW.obj.isSetupComplete = false;
    TestConnExample_DW.obj.isInitialized = 1;
    TestConnExample_DW.obj.orbMetadataObj = ORB_ID(sensor_accel);
    uORB_read_initialize(TestConnExample_DW.obj.orbMetadataObj,
                         &TestConnExample_DW.obj.eventStructObj);
    TestConnExample_DW.obj.isSetupComplete = true;

    // Start for MATLABSystem: '<S5>/Serial Transmit'
    TestConnExample_DW.obj_e.matlabCodegenIsDeleted = true;
    TestConnExample_DW.obj_e.isInitialized = 0;
    TestConnExample_DW.obj_e.matlabCodegenIsDeleted = false;
    obj_0 = &TestConnExample_DW.obj_e;
    TestConnExample_DW.obj_e.isSetupComplete = false;
    TestConnExample_DW.obj_e.isInitialized = 1;
    RxPinLoc = MW_UNDEFINED_VALUE;
    TxPinLoc = MW_UNDEFINED_VALUE;
    obj_0->MW_SCIHANDLE = MW_SCI_Open(TestConnExample_B.SCIModuleLoc_tmp, true,
      RxPinLoc, TxPinLoc);
    MW_SCI_SetBaudrate(TestConnExample_DW.obj_e.MW_SCIHANDLE, 9600U);
    StopBitsValue = MW_SCI_STOPBITS_1;
    ParityValue = MW_SCI_PARITY_NONE;
    MW_SCI_SetFrameFormat(TestConnExample_DW.obj_e.MW_SCIHANDLE, 8, ParityValue,
                          StopBitsValue);
    TestConnExample_DW.obj_e.isSetupComplete = true;

    // End of Start for S-Function (fcncallgen): '<S4>/Function-Call Generator'
    // End of Start for S-Function (fcncallgen): '<S3>/Function-Call Generator'
    // End of Start for SubSystem: '<S1>/Subsystem'
    // End of Start for SubSystem: '<Root>/Subsystem2'

    // SystemInitialize for Enabled SubSystem: '<Root>/Subsystem2'
    // SystemInitialize for Enabled SubSystem: '<S1>/Subsystem'
    // SystemInitialize for S-Function (fcncallgen): '<S3>/Function-Call Generator' incorporates:
    //   SubSystem: '<S3>/Subsystem'

    // SystemInitialize for S-Function (fcncallgen): '<S4>/Function-Call Generator' incorporates:
    //   SubSystem: '<S4>/Data - Subsystem1'

    // SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S9>/Out1'
    TestConnExample_B.In1 = TestConnExample_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem'
    // End of SystemInitialize for S-Function (fcncallgen): '<S4>/Function-Call Generator' 
    // End of SystemInitialize for S-Function (fcncallgen): '<S3>/Function-Call Generator' 
    // End of SystemInitialize for SubSystem: '<S1>/Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subsystem2'
  }
}

// Model terminate function
void TestConnExample_terminate(void)
{
  // Terminate for MATLABSystem: '<Root>/Serial Receive1'
  matlabCodegenHandle_matlabCodeg(&TestConnExample_DW.obj_p);

  // Terminate for Enabled SubSystem: '<Root>/Subsystem2'
  // Terminate for Enabled SubSystem: '<S1>/Subsystem'
  // Terminate for S-Function (fcncallgen): '<S3>/Function-Call Generator' incorporates:
  //   SubSystem: '<S3>/Subsystem'

  // Terminate for S-Function (fcncallgen): '<S4>/Function-Call Generator' incorporates:
  //   SubSystem: '<S4>/Header Subsystem'

  // Terminate for MATLABSystem: '<S6>/Serial Transmit'
  matlabCodegenHandle_matlabCod_g(&TestConnExample_DW.obj_l);

  // Terminate for S-Function (fcncallgen): '<S4>/Function-Call Generator' incorporates:
  //   SubSystem: '<S4>/Data - Subsystem1'

  // Terminate for MATLABSystem: '<S8>/SourceBlock'
  matlabCodegenHandle_matlabC_gyo(&TestConnExample_DW.obj);

  // Terminate for MATLABSystem: '<S5>/Serial Transmit'
  matlabCodegenHandle_matlabCod_g(&TestConnExample_DW.obj_e);

  // End of Terminate for S-Function (fcncallgen): '<S4>/Function-Call Generator' 
  // End of Terminate for S-Function (fcncallgen): '<S3>/Function-Call Generator' 
  // End of Terminate for SubSystem: '<S1>/Subsystem'
  // End of Terminate for SubSystem: '<Root>/Subsystem2'
}

//
// File trailer for generated code.
//
// [EOF]
//

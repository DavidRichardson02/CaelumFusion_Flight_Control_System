`ifndef FLIGHT_VIZ_BUNDLE_DEFS_VH
`define FLIGHT_VIZ_BUNDLE_DEFS_VH

//==============================================================================
// flight_viz_bundle_defs.vh
//------------------------------------------------------------------------------
// Canonical visualization bundle field map.
//
// NOTES
//   - Total width is fixed at 1152 bits.
//   - Legacy fields keep their original 479:0 positions.
//   - New authority/model fields grow upward from bit 480.
//   - Navigation/wind estimator fields grow upward from bit 640.
//   - PMON1 power-monitor fields grow upward from bit 800.
//   - Future sensor-extension evidence grows upward from bit 880.
//   - All producers and consumers must use only these macros.
//   - The schema field is intentionally 4 bits wide in this revision to avoid
//     overlap with the existing 32-bit build_id field.
//==============================================================================

`define VIZ_BUNDLE_W 1152

//------------------------------------------------------------------------------
// Future sensor-extension evidence and black-box logging summary
//------------------------------------------------------------------------------
`define VIZ_EXT_VALID_BIT             1151
`define VIZ_EXT_STATUS_MSB            1150
`define VIZ_EXT_STATUS_LSB            1143
`define VIZ_EXT_PRESENT_MSB           1142
`define VIZ_EXT_PRESENT_LSB           1127
`define VIZ_EXT_FAULT_MSB             1126
`define VIZ_EXT_FAULT_LSB             1111
`define VIZ_EXT_MAG_DELTA_L1_MSB      1110
`define VIZ_EXT_MAG_DELTA_L1_LSB      1095
`define VIZ_EXT_MAG_NORM0_MSB         1094
`define VIZ_EXT_MAG_NORM0_LSB         1079
`define VIZ_EXT_MAG_NORM1_MSB         1078
`define VIZ_EXT_MAG_NORM1_LSB         1063
`define VIZ_EXT_RNG_HEIGHT_CM_MSB     1062
`define VIZ_EXT_RNG_HEIGHT_CM_LSB     1047
`define VIZ_EXT_AIR_DP_PA_MSB         1046
`define VIZ_EXT_AIR_DP_PA_LSB         1031
`define VIZ_EXT_AIR_SPEED_CMS_MSB     1030
`define VIZ_EXT_AIR_SPEED_CMS_LSB     1015
`define VIZ_EXT_ENV_TEMP_CDEG_MSB     1014
`define VIZ_EXT_ENV_TEMP_CDEG_LSB     999
`define VIZ_EXT_ENV_RH_CENTI_MSB      998
`define VIZ_EXT_ENV_RH_CENTI_LSB      983
`define VIZ_EXT_SUN_LUMA_MSB          982
`define VIZ_EXT_SUN_LUMA_LSB          967
`define VIZ_EXT_FLOW_DX_MSB           966
`define VIZ_EXT_FLOW_DX_LSB           951
`define VIZ_EXT_FLOW_DY_MSB           950
`define VIZ_EXT_FLOW_DY_LSB           935
`define VIZ_EXT_LOG_SEQ_MSB           934
`define VIZ_EXT_LOG_SEQ_LSB           919
`define VIZ_EXT_LOG_DROP_MSB          918
`define VIZ_EXT_LOG_DROP_LSB          903
`define VIZ_EXT_MAX_AGE_MS_MSB        902
`define VIZ_EXT_MAX_AGE_MS_LSB        887
`define VIZ_EXT_MAG_SEQ_ALIGNED_BIT   886
`define VIZ_EXT_MAG_DISAGREE_BIT      885
`define VIZ_EXT_MAG_SECTOR_DELTA_MSB  884
`define VIZ_EXT_MAG_SECTOR_DELTA_LSB  881
`define VIZ_EXT_MAG_SOURCE_SYNTH_BIT  880

//------------------------------------------------------------------------------
// PMON1 / ADM1191 power monitor evidence
//------------------------------------------------------------------------------
`define VIZ_PWR_VALID_BIT         879
`define VIZ_PWR_STATUS_MSB        878
`define VIZ_PWR_STATUS_LSB        871
`define VIZ_PWR_SEQ_MSB           870
`define VIZ_PWR_SEQ_LSB           855
`define VIZ_PWR_AGE_MS_MSB        854
`define VIZ_PWR_AGE_MS_LSB        839
`define VIZ_PWR_VOLT_CODE_MSB     838
`define VIZ_PWR_VOLT_CODE_LSB     827
`define VIZ_PWR_CURR_CODE_MSB     826
`define VIZ_PWR_CURR_CODE_LSB     815
`define VIZ_PWR_ALERT_MSB         814
`define VIZ_PWR_ALERT_LSB         807
`define VIZ_PWR_RSVD_MSB          806
`define VIZ_PWR_RSVD_LSB          800

//------------------------------------------------------------------------------
// Navigation / wind estimator evidence
//------------------------------------------------------------------------------
`define VIZ_NAV_VALID_BIT          799
`define VIZ_NAV_STATUS_MSB         798
`define VIZ_NAV_STATUS_LSB         791
`define VIZ_NAV_FLAGS_MSB          790
`define VIZ_NAV_FLAGS_LSB          783
`define VIZ_NAV_DOWNRANGE_M_MSB    782
`define VIZ_NAV_DOWNRANGE_M_LSB    767
`define VIZ_NAV_CROSSRANGE_M_MSB   766
`define VIZ_NAV_CROSSRANGE_M_LSB   751
`define VIZ_WIND_VALID_BIT         750
`define VIZ_WIND_STATUS_MSB        749
`define VIZ_WIND_STATUS_LSB        742
`define VIZ_WIND_X_CMS_MSB         741
`define VIZ_WIND_X_CMS_LSB         726
`define VIZ_WIND_Y_CMS_MSB         725
`define VIZ_WIND_Y_CMS_LSB         710
`define VIZ_WIND_Z_CMS_MSB         709
`define VIZ_WIND_Z_CMS_LSB         694
`define VIZ_NAV_AGE_MS_MSB         693
`define VIZ_NAV_AGE_MS_LSB         678
`define VIZ_WIND_AGE_MS_MSB        677
`define VIZ_WIND_AGE_MS_LSB        662
`define VIZ_NAV_WIND_RSVD_MSB      661
`define VIZ_NAV_WIND_RSVD_LSB      640

//------------------------------------------------------------------------------
// Apogee authority / drag-servo policy
//------------------------------------------------------------------------------
`define VIZ_AUTH_VALID_BIT          639
`define VIZ_AUTH_STATUS_MSB         638
`define VIZ_AUTH_STATUS_LSB         631
`define VIZ_AUTH_FLAGS_MSB          630
`define VIZ_AUTH_FLAGS_LSB          623
`define VIZ_AUTH_TARGET_CM_MSB      622
`define VIZ_AUTH_TARGET_CM_LSB      591
`define VIZ_AUTH_PRED_NO_CM_MSB     590
`define VIZ_AUTH_PRED_NO_CM_LSB     559
`define VIZ_AUTH_PRED_FULL_CM_MSB   558
`define VIZ_AUTH_PRED_FULL_CM_LSB   527
`define VIZ_AUTH_UNC_CM_MSB         526
`define VIZ_AUTH_UNC_CM_LSB         511
`define VIZ_AUTH_CMD_U8_MSB         510
`define VIZ_AUTH_CMD_U8_LSB         503
`define VIZ_AUTH_SERVO_US_MSB       502
`define VIZ_AUTH_SERVO_US_LSB       491
`define VIZ_AUTH_PHASE_MSB          490
`define VIZ_AUTH_PHASE_LSB          487
`define VIZ_AUTH_GATE_MSB           486
`define VIZ_AUTH_GATE_LSB           480

`define VIZ_AUTH_FLG_INPUT_OK_BIT   0
`define VIZ_AUTH_FLG_ASCENDING_BIT  1
`define VIZ_AUTH_FLG_NO_HIGH_BIT    2
`define VIZ_AUTH_FLG_REACHABLE_BIT  3
`define VIZ_AUTH_FLG_CMD_NONZERO_BIT 4
`define VIZ_AUTH_FLG_CMD_SAT_BIT    5
`define VIZ_AUTH_FLG_UNC_CAP_BIT    6
`define VIZ_AUTH_FLG_ACT_SAFE_BIT   7

`define VIZ_AUTH_PHASE_UNKNOWN      4'h0
`define VIZ_AUTH_PHASE_IDLE         4'h1
`define VIZ_AUTH_PHASE_BOOST        4'h2
`define VIZ_AUTH_PHASE_COAST        4'h3
`define VIZ_AUTH_PHASE_BRAKE        4'h4
`define VIZ_AUTH_PHASE_DESCENT      4'h5

`define VIZ_AUTH_GATE_SAFETY_RUNTIME_OK_BIT 0
`define VIZ_AUTH_GATE_SAFETY_ALLOWS_BIT     1
`define VIZ_AUTH_GATE_POLICY_ENABLE_BIT     2
`define VIZ_AUTH_GATE_SOFTWARE_ARMED_BIT    3
`define VIZ_AUTH_GATE_ACTUATOR_ACTIVE_BIT   4
`define VIZ_AUTH_GATE_EXTERNAL_PHASE_BIT    5
`define VIZ_AUTH_GATE_LOCAL_PHASE_BIT       6

//------------------------------------------------------------------------------
// Raw BMP
//------------------------------------------------------------------------------
`define VIZ_BMP_SEQ_MSB           479
`define VIZ_BMP_SEQ_LSB           464
`define VIZ_BMP_VALID_BIT         463
`define VIZ_BMP_STATUS_MSB        462
`define VIZ_BMP_STATUS_LSB        455
`define VIZ_BMP_AGE_MS_MSB        454
`define VIZ_BMP_AGE_MS_LSB        439

//------------------------------------------------------------------------------
// Raw ACC
//------------------------------------------------------------------------------
`define VIZ_ACC_SEQ_MSB           438
`define VIZ_ACC_SEQ_LSB           423
`define VIZ_ACC_VALID_BIT         422
`define VIZ_ACC_STATUS_MSB        421
`define VIZ_ACC_STATUS_LSB        414
`define VIZ_ACC_AGE_MS_MSB        413
`define VIZ_ACC_AGE_MS_LSB        398

//------------------------------------------------------------------------------
// Raw MAG
//------------------------------------------------------------------------------
`define VIZ_MAG_SEQ_MSB           397
`define VIZ_MAG_SEQ_LSB           382
`define VIZ_MAG_VALID_BIT         381
`define VIZ_MAG_STATUS_MSB        380
`define VIZ_MAG_STATUS_LSB        373
`define VIZ_MAG_AGE_MS_MSB        372
`define VIZ_MAG_AGE_MS_LSB        357

//------------------------------------------------------------------------------
// Derived control
//------------------------------------------------------------------------------
`define VIZ_DER_VALID_BIT         356
`define VIZ_DER_STATUS_MSB        355
`define VIZ_DER_STATUS_LSB        348
`define VIZ_DER_ALT_FRESH_BIT     347
`define VIZ_DER_VSPD_FRESH_BIT    346
`define VIZ_DER_ROLL_FRESH_BIT    345
`define VIZ_DER_HEAD_FRESH_BIT    344

//------------------------------------------------------------------------------
// Derived provenance
//------------------------------------------------------------------------------
`define VIZ_DER_BMP_SEQ_REF_MSB   343
`define VIZ_DER_BMP_SEQ_REF_LSB   328
`define VIZ_DER_ACC_SEQ_REF_MSB   327
`define VIZ_DER_ACC_SEQ_REF_LSB   312
`define VIZ_DER_MAG_SEQ_REF_MSB   311
`define VIZ_DER_MAG_SEQ_REF_LSB   296

`define VIZ_DER_BMP_AGE_MS_MSB    295
`define VIZ_DER_BMP_AGE_MS_LSB    280
`define VIZ_DER_ACC_AGE_MS_MSB    279
`define VIZ_DER_ACC_AGE_MS_LSB    264
`define VIZ_DER_MAG_AGE_MS_MSB    263
`define VIZ_DER_MAG_AGE_MS_LSB    248

`define VIZ_DER_BMP_VALID_REF_BIT 247
`define VIZ_DER_ACC_VALID_REF_BIT 246
`define VIZ_DER_MAG_VALID_REF_BIT 245

//------------------------------------------------------------------------------
// Derived numerics
//------------------------------------------------------------------------------
`define VIZ_DER_ALT_CM_MSB        244
`define VIZ_DER_ALT_CM_LSB        213
`define VIZ_DER_VSPD_CMS_MSB      212
`define VIZ_DER_VSPD_CMS_LSB      181
`define VIZ_DER_ROLL_MDEG_MSB     180
`define VIZ_DER_ROLL_MDEG_LSB     149
`define VIZ_DER_HEAD_MDEG_MSB     148
`define VIZ_DER_HEAD_MDEG_LSB     117

//------------------------------------------------------------------------------
// Health / metadata
//------------------------------------------------------------------------------
`define VIZ_I2C_NACK_MSB          116
`define VIZ_I2C_NACK_LSB          101
`define VIZ_I2C_TMO_MSB           100
`define VIZ_I2C_TMO_LSB           85
`define VIZ_TXN_RATE_MSB          84
`define VIZ_TXN_RATE_LSB          69
`define VIZ_CDC_UPD_MSB           68
`define VIZ_CDC_UPD_LSB           37
`define VIZ_BUILD_ID_MSB          36
`define VIZ_BUILD_ID_LSB          5
`define VIZ_SCHEMA_MSB            4
`define VIZ_SCHEMA_LSB            1

//------------------------------------------------------------------------------
// Reserved
//------------------------------------------------------------------------------
`define VIZ_RSVD_MSB              0
`define VIZ_RSVD_LSB              0

`endif

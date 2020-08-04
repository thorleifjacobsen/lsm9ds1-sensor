/*
  LSM9DS1.js

  A Node.js I2C module for the ST LSM9DS1 3D accelerometer, 3D gyroscope and 3D magnometer.

*/

'use strict';

module.exports = class LSM9DS1 {

  constructor(options) {
    const i2c = require('i2c-bus');

    this.i2cBusNo = (options && options.hasOwnProperty('i2cBusNo')) ? options.i2cBusNo : 1;    
    this.i2cBus = i2c.openSync(this.i2cBusNo);
    this.i2cAccelGyroAddress = (options && options.hasOwnProperty('i2cAccelGyroAddress')) ? options.i2cAccelGyroAddress : 0x6B;
    this.i2cMagAddress = (options && options.hasOwnProperty('i2cMagAddress')) ? options.i2cMagAddress : 0x1E;

    this.gyro = {
      enabled: true,
      enableX: true,
      enableY: true,
      enableZ: true,
      scale: 245,
      sampleRate: 6,
      bandwidth: 0,
      lowPowerEnable: false,
      HPFEnable: false,
      HPFCutoff: 0,
      flipX: false,
      flipY: false,
      flipZ: false,
      orientation: 0,
      latchInterrupt: true
    }

    this.accel = {
      enabled: true,
      enableX: true,
      enableY: true,
      enableZ: true,
      scale: 2,
      sampleRate: 6,
      bandwidth: -1,
      highResEnable : false,
      highResBandwidth : 0
    }

    this.mag = {
      enabled: true,
      scale: 4,
      sampleRate: 7,
      tempCompensationEnable: false,
      XYPerformance: 3,
      ZPerformance: 3,
      lowPowerEnable: false,
      operatingMode : 0
    }

    this.temp = {
      enabled: true
    }

    this.gBias = [0,0,0];
    this.aBias = [0,0,0];
    this.mBias = [0,0,0];
    this.gBiasRaw = [0,0,0];
    this.aBiasRaw = [0,0,0];
    this.nBiasRaw = [0,0,0];

    this._autoCalc = false;
  }

  init() {
    return new Promise((resolve, reject) => {

      // Make sure constrinas are within the allowed values and
      // Once we have the scale values, we can calculate the resolution
      // of each sensor. That's what these functions are for. One for each sensor
      this.constrainScalesAndCalculateResolutions();

      try {
        // Verify Magnometer Chip ID
        this.i2cBus.writeByteSync(this.i2cMagAddress, LSM9DS1.WHO_AM_I_M(), 0);
        let mChipId = this.i2cBus.readByteSync(this.i2cMagAddress, LSM9DS1.WHO_AM_I_M());
        if(mChipId !== LSM9DS1.WHO_AM_I_M_RSP()) {
          return reject(`Unexpected LSM9DS1 Magnometer chip ID: 0x${mChipId.toString(16)}`);
        }

        // Verify Accel & Gyro Chip ID
        this.i2cBus.writeByteSync(this.i2cAccelGyroAddress, LSM9DS1.WHO_AM_I_XG(), 0);
        let xgChipId = this.i2cBus.readByteSync(this.i2cAccelGyroAddress, LSM9DS1.WHO_AM_I_XG());
        if(xgChipId !== LSM9DS1.WHO_AM_I_AG_RSP()) {
          return reject(`Unexpected LSM9DS1 Accel/Gyro chip ID: 0x${xgChipId.toString(16)}`);
        }
      } catch(err) {
        return reject(err); 
      };

      try {
        // Gyro initialization stuff:
        this.initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.
        
        // Accelerometer initialization stuff:
        this.initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
        
        // Magnetometer initialization stuff:
        this.initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.
      } catch(err) {
        reject(err);
      }

      resolve();
    })
  }
  
  initGyro() {
    let tempRegValue = 0;

    // CTRL_REG1_G (Default value: 0x00)
    // [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
    // ODR_G[2:0] - Output data rate selection
    // FS_G[1:0] - Gyroscope full-scale selection
    // BW_G[1:0] - Gyroscope bandwidth selection

    // To disable gyro, set sample rate bits to 0. We'll only set sample
    // rate if the gyro is enabled.
    if (this.gyro.enabled) {
      tempRegValue = (this.gyro.sampleRate & 0x07) << 5;
    }

    switch (this.gyro.scale) {
      case 500: tempRegValue |= (0x1 << 3); break;
      case 2000: tempRegValue |= (0x3 << 3); break;
      // Otherwise we'll set it to 245 dps (0x0 << 4)
    }
    tempRegValue |= (this.gyro.bandwidth & 0x3);
    this.i2cBus.writeByteSync(this.i2cAccelGyroAddress, LSM9DS1.CTRL_REG1_G(), tempRegValue);

    // CTRL_REG2_G (Default value: 0x00)
    // [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
    // INT_SEL[1:0] - INT selection configuration
    // OUT_SEL[1:0] - Out selection configuration
    this.i2cBus.writeByteSync(this.i2cAccelGyroAddress, LSM9DS1.CTRL_REG2_G(), 0x00);

    // CTRL_REG3_G (Default value: 0x00)
    // [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
    // LP_mode - Low-power mode enable (0: disabled, 1: enabled)
    // HP_EN - HPF enable (0:disabled, 1: enabled)
    // HPCF_G[3:0] - HPF cutoff frequency
    tempRegValue = this.gyro.lowPowerEnable ? (1<<7) : 0;
    if (this.gyro.HPFEnable) {
      tempRegValue |= (1<<6) | (this.gyro.HPFCutoff & 0x0F);
    }
    this.i2cBus.writeByteSync(this.i2cAccelGyroAddress, LSM9DS1.CTRL_REG3_G(), tempRegValue);

    // CTRL_REG4 (Default value: 0x38)
    // [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
    // Zen_G - Z-axis output enable (0:disable, 1:enable)
    // Yen_G - Y-axis output enable (0:disable, 1:enable)
    // Xen_G - X-axis output enable (0:disable, 1:enable)
    // LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
    // 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
    tempRegValue = 0;
    if (this.gyro.enableZ) tempRegValue |= (1<<5);
    if (this.gyro.enableY) tempRegValue |= (1<<4);
    if (this.gyro.enableX) tempRegValue |= (1<<3);
    if (this.gyro.latchInterrupt) tempRegValue |= (1<<1);
    this.i2cBus.writeByteSync(this.i2cAccelGyroAddress, LSM9DS1.CTRL_REG4(), tempRegValue);

    // ORIENT_CFG_G (Default value: 0x00)
    // [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
    // SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
    // Orient [2:0] - Directional user orientation selection
    tempRegValue = 0;
    if (this.gyro.flipX) tempRegValue |= (1<<5);
    if (this.gyro.flipY) tempRegValue |= (1<<4);
    if (this.gyro.flipZ) tempRegValue |= (1<<3);
    this.i2cBus.writeByteSync(this.i2cAccelGyroAddress, LSM9DS1.ORIENT_CFG_G(), tempRegValue);
  }

  initAccel() {
    let tempRegValue = 0;
	
    //	CTRL_REG5_XL (0x1F) (Default value: 0x38)
    //	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
    //	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
    //		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
    //	Zen_XL - Z-axis output enabled
    //	Yen_XL - Y-axis output enabled
    //	Xen_XL - X-axis output enabled
    if (this.accel.enableZ) tempRegValue |= (1<<5);
    if (this.accel.enableY) tempRegValue |= (1<<4);
    if (this.accel.enableX) tempRegValue |= (1<<3);
    
    this.i2cBus.writeByteSync(this.i2cAccelGyroAddress, LSM9DS1.CTRL_REG5_XL(), tempRegValue);
    
    // CTRL_REG6_XL (0x20) (Default value: 0x00)
    // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
    // ODR_XL[2:0] - Output data rate & power mode selection
    // FS_XL[1:0] - Full-scale selection
    // BW_SCAL_ODR - Bandwidth selection
    // BW_XL[1:0] - Anti-aliasing filter bandwidth selection
    tempRegValue = 0;
    // To disable the accel, set the sampleRate bits to 0.
    if (this.accel.enabled) {
      tempRegValue |= (this.accel.sampleRate & 0x07) << 5;
    }
    switch (this.accel.scale)     {
      case 4: tempRegValue |= (0x2 << 3); break;
      case 8: tempRegValue |= (0x3 << 3); break;
      case 16: tempRegValue |= (0x1 << 3); break;
      // Otherwise it'll be set to 2g (0x0 << 3)
    }
    if (this.accel.bandwidth >= 0) {
      tempRegValue |= (1<<2); // Set BW_SCAL_ODR
      tempRegValue |= (this.accel.bandwidth & 0x03);
    }
    this.i2cBus.writeByteSync(this.i2cAccelGyroAddress, LSM9DS1.CTRL_REG6_XL(), tempRegValue);

    // CTRL_REG7_XL (0x21) (Default value: 0x00)
    // [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
    // HR - High resolution mode (0: disable, 1: enable)
    // DCF[1:0] - Digital filter cutoff frequency
    // FDS - Filtered data selection
    // HPIS1 - HPF enabled for interrupt function
    tempRegValue = 0;
    if (this.accel.highResEnable) {
      tempRegValue |= (1<<7); // Set HR bit
      tempRegValue |= (this.accel.highResBandwidth & 0x3) << 5;
    }
    this.i2cBus.writeByteSync(this.i2cAccelGyroAddress, LSM9DS1.CTRL_REG7_XL(), tempRegValue);
  }

  initMag() {
    let tempRegValue = 0;
	
    // CTRL_REG1_M (Default value: 0x10)
    // [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
    // TEMP_COMP - Temperature compensation
    // OM[1:0] - X & Y axes op mode selection
    //	00:low-power, 01:medium performance
    //	10: high performance, 11:ultra-high performance
    // DO[2:0] - Output data rate selection
    // ST - Self-test enable
    if (this.mag.tempCompensationEnable) tempRegValue |= (1<<7);
    tempRegValue |= (this.mag.XYPerformance & 0x3) << 5;
    tempRegValue |= (this.mag.sampleRate & 0x7) << 2;
    this.i2cBus.writeByteSync(this.i2cMagAddress, LSM9DS1.CTRL_REG1_M(), tempRegValue);

    // CTRL_REG2_M (Default value 0x00)
    // [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
    // FS[1:0] - Full-scale configuration
    // REBOOT - Reboot memory content (0:normal, 1:reboot)
    // SOFT_RST - Reset config and user registers (0:default, 1:reset)
    tempRegValue = 0;
    switch (this.mag.scale) {
      case 8: tempRegValue |= (0x1 << 5); break;
      case 12: tempRegValue |= (0x2 << 5); break;
      case 16: tempRegValue |= (0x3 << 5); break;
      // Otherwise we'll default to 4 gauss (00)
    }
    this.i2cBus.writeByteSync(this.i2cMagAddress, LSM9DS1.CTRL_REG2_M(), tempRegValue);

    // CTRL_REG3_M (Default value: 0x03)
    // [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
    // I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
    // LP - Low-power mode cofiguration (1:enable)
    // SIM - SPI mode selection (0:write-only, 1:read/write enable)
    // MD[1:0] - Operating mode
    //	00:continuous conversion, 01:single-conversion,
    //  10,11: Power-down
    tempRegValue = 0;
    if (this.mag.lowPowerEnable) tempRegValue |= (1<<5);
    tempRegValue |= (this.mag.operatingMode & 0x3);
    this.i2cBus.writeByteSync(this.i2cMagAddress, LSM9DS1.CTRL_REG3_M(), tempRegValue);

    // CTRL_REG4_M (Default value: 0x00)
    // [0][0][0][0][OMZ1][OMZ0][BLE][0]
    // OMZ[1:0] - Z-axis operative mode selection
    //	00:low-power mode, 01:medium performance
    //	10:high performance, 10:ultra-high performance
    // BLE - Big/little endian data
    tempRegValue = 0;
    tempRegValue = (this.mag.ZPerformance & 0x3) << 2;
    this.i2cBus.writeByteSync(this.i2cMagAddress, LSM9DS1.CTRL_REG4_M(), tempRegValue);

    // CTRL_REG5_M (Default value: 0x00)
    // [0][BDU][0][0][0][0][0][0]
    // BDU - Block data update for magnetic data
    //	0:continuous, 1:not updated until MSB/LSB are read
    tempRegValue = 0;
    this.i2cBus.writeByteSync(this.i2cMagAddress, LSM9DS1.CTRL_REG5_M(), tempRegValue);
  }

  accelAvailable() {
    try {
      const status = this.i2cBus.readByteSync(this.i2cAccelGyroAddress, LSM9DS1.STATUS_REG_1());
      return (status & (1<<0));
    } catch(err) { return false; }
  }

  gyroAvailable() {
    try {
      const status = this.i2cBus.readByteSync(this.i2cAccelGyroAddress, LSM9DS1.STATUS_REG_1());
      return ((status & (1<<1)) >> 1);
    } catch(err) { return false; }
  }

  tempAvailable() {
    try {
      const status = this.i2cBus.readByteSync(this.i2cAccelGyroAddress, LSM9DS1.STATUS_REG_1());
      return ((status & (1<<2)) >> 2);
    } catch(err) { return false; }
  }

  magAvailable(axis) {
    try {
      const status = this.i2cBus.readByteSync(this.i2cMagAddress, LSM9DS1.STATUS_REG_M());
      return ((status & (1<<axis)) >> axis);
    } catch(err) { return false; }
  }

  readGyro() {
    try {
      const temp = new Buffer(6); // We'll read six bytes from the gyro into temp
      if (this.i2cBus.readI2cBlockSync(this.i2cAccelGyroAddress, LSM9DS1.OUT_X_L_G(), 6, temp)) {
        this.gyro.x = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
        this.gyro.y = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
        this.gyro.z = (temp[5] << 8) | temp[4]; // Store z-axis values into gz

        if(this.gyro.x > 32768) { this.gyro.x -= 65536; }
        if(this.gyro.y > 32768) { this.gyro.y -= 65536; }
        if(this.gyro.z > 32768) { this.gyro.z -= 65536; }

        if (this._autoCalc) {
          /*gx -= gBiasRaw[X_AXIS];
          gy -= gBiasRaw[Y_AXIS];
          gz -= gBiasRaw[Z_AXIS];*/
        }
      }
    } catch(err) { return false; }
  }

  readMag() {
    try {
      const temp = new Buffer(6); // We'll read six bytes from the mag into temp	
      if (this.i2cBus.readI2cBlockSync(this.i2cMagAddress, LSM9DS1.OUT_X_L_M(), 6, temp)) {
        this.mag.x = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
        this.mag.y = (temp[3] << 8) | temp[2]; // Store y-axis values into my
        this.mag.z = (temp[5] << 8) | temp[4]; // Store z-axis values into mz

        if(this.mag.x > 32768) { this.mag.x -= 65536; }
        if(this.mag.y > 32768) { this.mag.y -= 65536; }
        if(this.mag.z > 32768) { this.mag.z -= 65536; }

      }
    } catch(err) { return false; }
  }

  readAccel() {
    try {
      const temp = new Buffer(6); // We'll read six bytes from the accelerometer into temp	
      if (this.i2cBus.readI2cBlockSync(this.i2cAccelGyroAddress, LSM9DS1.OUT_X_L_XL(), 6, temp)) {
        this.accel.x = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
        this.accel.y = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
        this.accel.z = (temp[5] << 8) | temp[4]; // Store z-axis values into az

        if(this.accel.x > 32768) { this.accel.x -= 65536; }
        if(this.accel.y > 32768) { this.accel.y -= 65536; }
        if(this.accel.z > 32768) { this.accel.z -= 65536; }

        if (this._autoCalc) {
        /* ax -= aBiasRaw[X_AXIS];
          ay -= aBiasRaw[Y_AXIS];
          az -= aBiasRaw[Z_AXIS];*/
        }
      }
    } catch(err) { return false; }
  }

  readTemp() {
    try {
      const temp = new Buffer(2); // We'll read two bytes from the temperature sensor into temp	
      if (this.i2cBus.readI2cBlockSync(this.i2cMagAddress, LSM9DS1.OUT_TEMP_L(), 2, temp)) {
        let offset = 25;  // Per datasheet sensor outputs 0 typically @ 25 degrees centigrade
        this.temp.value = offset + (((temp[1] << 8) | temp[0]) >> 8);
      }
    } catch(err) { return false; }
  }

  calcGyro(value) {
    return value * this.gyro.resolution;
  }

  calcAccel(value) {
    return value * this.accel.resolution;
  }

  calcMag(value) {
    return value * this.mag.resolution;
  }

  enableFIFO(enable) {
    try {
      let temp = this.i2cBus.readByteSync(this.i2cAccelGyroAddress, LSM9DS1.CTRL_REG9());
      if (enable) temp |= (1<<1);
      else temp &= ~(1<<1);
      this.i2cBus.writeByteSync(this.i2cAccelGyroAddress, LSM9DS1.CTRL_REG9(), temp);
      return true;
    } catch(err) {
      return false; 
    }
  }
  
  setFIFO(fifoMode, fifoThs) {
    try {
      // Limit threshold - 0x1F (31) is the maximum. If more than that was asked
      // limit it to the maximum.
      let threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
      let temp = ((fifoMode & 0x7) << 5) | (threshold & 0x1F);
      this.i2cBus.writeByteSync(this.i2cAccelGyroAddress, LSM9DS1.FIFO_CTRL(), temp);
    } catch(err) {
      return false; 
    }
  }

  constrainScalesAndCalculateResolutions() {
    // Sensor Sensitivity Cosntants
    // Values set according to the typical specifications provided in
    // table 3 of the LSM9DS1 datasheet. (pg 12)
    const gyroSens  = { "245": 0.00875, "500": 0.0175, "2000": 0.07 };
    const accelSens = { "2": 0.000061, "4": 0.000122, "8": 0.000244, "16": 0.000732 };
    const magSens   = { "4": 0.00014, "8": 0.00029, "12": 0.00043, "16": 0.00058 };

    // Constrain scales
    if(!Object.keys(gyroSens).includes(this.gyro.scale.toString()))   { this.gyro.scale = 245; }
    if(!Object.keys(accelSens).includes(this.accel.scale.toString())) { this.accel.scale = 2;  }
    if(!Object.keys(magSens).includes(this.mag.scale.toString()))     { this.mag.scale = 4;    }

    // Set resolutions
    
    this.gyro.resolution  = gyroSens[this.gyro.scale];   // Calculate DPS / ADC tick
    this.accel.resolution = accelSens[this.accel.scale]; // Calculate g / ADC tick
    this.mag.resolution   = magSens[this.mag.scale];     // Calculate Gs / ADC tick
  }

  /////////////////////////////////////////
  // LSM9DS1 Accel/Gyro (XL/G) Registers //
  /////////////////////////////////////////
  static ACT_THS() { return 0x04; }
  static ACT_DUR() { return 0x05; }
  static INT_GEN_CFG_XL() { return 0x06; }
  static INT_GEN_THS_X_XL() { return 0x07; }
  static INT_GEN_THS_Y_XL() { return 0x08; }
  static INT_GEN_THS_Z_XL() { return 0x09; }
  static INT_GEN_DUR_XL() { return 0x0A; }
  static REFERENCE_G() { return 0x0B; }
  static INT1_CTRL() { return 0x0C; }
  static INT2_CTRL() { return 0x0D; }
  static WHO_AM_I_XG() { return 0x0F; }
  static CTRL_REG1_G() { return 0x10; }
  static CTRL_REG2_G() { return 0x11; }
  static CTRL_REG3_G() { return 0x12; }
  static ORIENT_CFG_G() { return 0x13; }
  static INT_GEN_SRC_G() { return 0x14; }
  static OUT_TEMP_L() { return 0x15; }
  static OUT_TEMP_H() { return 0x16; }
  static STATUS_REG_0() { return 0x17; }
  static OUT_X_L_G() { return 0x18; }
  static OUT_X_H_G() { return 0x19; }
  static OUT_Y_L_G() { return 0x1A; }
  static OUT_Y_H_G() { return 0x1B; }
  static OUT_Z_L_G() { return 0x1C; }
  static OUT_Z_H_G() { return 0x1D; }
  static CTRL_REG4() { return 0x1E; }
  static CTRL_REG5_XL() { return 0x1F; }
  static CTRL_REG6_XL() { return 0x20; }
  static CTRL_REG7_XL() { return 0x21; }
  static CTRL_REG8() { return 0x22; }
  static CTRL_REG9() { return 0x23; }
  static CTRL_REG10() { return 0x24; }
  static INT_GEN_SRC_XL() { return 0x26; }
  static STATUS_REG_1() { return 0x27; }
  static OUT_X_L_XL() { return 0x28; }
  static OUT_X_H_XL() { return 0x29; }
  static OUT_Y_L_XL() { return 0x2A; }
  static OUT_Y_H_XL() { return 0x2B; }
  static OUT_Z_L_XL() { return 0x2C; }
  static OUT_Z_H_XL() { return 0x2D; }
  static FIFO_CTRL() { return 0x2E; }
  static FIFO_SRC() { return 0x2F; }
  static INT_GEN_CFG_G() { return 0x30; }
  static INT_GEN_THS_XH_G() { return 0x31; }
  static INT_GEN_THS_XL_G() { return 0x32; }
  static INT_GEN_THS_YH_G() { return 0x33; }
  static INT_GEN_THS_YL_G() { return 0x34; }
  static INT_GEN_THS_ZH_G() { return 0x35; }
  static INT_GEN_THS_ZL_G() { return 0x36; }
  static INT_GEN_DUR_G() { return 0x37; }

  ///////////////////////////////
  // LSM9DS1 Magneto Registers //
  ///////////////////////////////
  static OFFSET_X_REG_L_M() { return 0x05; }
  static OFFSET_X_REG_H_M() { return 0x06; }
  static OFFSET_Y_REG_L_M() { return 0x07; }
  static OFFSET_Y_REG_H_M() { return 0x08; }
  static OFFSET_Z_REG_L_M() { return 0x09; }
  static OFFSET_Z_REG_H_M() { return 0x0A; }
  static WHO_AM_I_M() { return 0x0F; }
  static CTRL_REG1_M() { return 0x20; }
  static CTRL_REG2_M() { return 0x21; }
  static CTRL_REG3_M() { return 0x22; }
  static CTRL_REG4_M() { return 0x23; }
  static CTRL_REG5_M() { return 0x24; }
  static STATUS_REG_M() { return 0x27; }
  static OUT_X_L_M() { return 0x28; }
  static OUT_X_H_M() { return 0x29; }
  static OUT_Y_L_M() { return 0x2A; }
  static OUT_Y_H_M() { return 0x2B; }
  static OUT_Z_L_M() { return 0x2C; }
  static OUT_Z_H_M() { return 0x2D; }
  static INT_CFG_M() { return 0x30; }
  static INT_SRC_M() { return 0x31; }
  static INT_THS_L_M() { return 0x32; }
  static INT_THS_H_M() { return 0x33; }

  ////////////////////////////////
  // LSM9DS1 WHO_AM_I Responses //
  ////////////////////////////////
  static WHO_AM_I_AG_RSP() { return 0x68; }
  static WHO_AM_I_M_RSP() { return 0x3D; }

}








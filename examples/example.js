const LSM9DS1 = require('../LSM9DS1.js')

const imu = new LSM9DS1()
imu.init().then(() => {
  console.log("Initialized LSM9DS1 Successfully\n")

  imu.calibrate()

  console.log("Calibration complete\n")

  imu.readGyro()
  imu.readAccel()
  imu.readMag()

  console.log(`Gyro raw values: x=${imu.gyro.x.toFixed(2)}, y=${imu.gyro.y.toFixed(2)}, z=${imu.gyro.z.toFixed(2)}`)
  console.log(`Accel raw values: x=${imu.accel.x.toFixed(2)}, y=${imu.accel.y.toFixed(2)}, z=${imu.accel.z.toFixed(2)}`)
  console.log(`Mag raw values: x=${imu.mag.x.toFixed(2)}, y=${imu.mag.y.toFixed(2)}, z=${imu.mag.z.toFixed(2)}`)

  setInterval(() => {
    imu.readGyro()
    imu.readAccel()
    imu.readMag()

    // Calculate roll and pitch
    const Xa = imu.accel.x
    const Ya = imu.accel.y
    const Za = imu.accel.z
    const Xm = imu.mag.x
    const Ym = imu.mag.y
    const Zm = imu.mag.z
    let Phi, Theta, Psi, Xh, Yh

    // roll: Rotation around the X-axis. -180 <= roll <= 180
    // a positive roll angle is defined to be a clockwise rotation about the positive X-axis
    //
    //                    y
    //      roll = atan2(---)
    //                    z
    //
    // where:  y, z are returned value from accelerometer sensor
    Phi = Math.atan2(Ya, Za);

    // pitch: Rotation around the Y-axis. -180 <= roll <= 180
    // a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis
    //
    //                                 -x
    //      pitch = atan(-------------------------------)
    //                    y * sin(roll) + z * cos(roll)
    //
    // where:  x, y, z are returned value from accelerometer sensor
    var tmp = Ya * Math.sin(Phi) + Za * Math.cos(Phi);
    if (tmp == 0) Theta = Xa > 0 ? (Math.PI / 2) : (-Math.PI / 2);
    else Theta = Math.atan(-Xa / tmp);

    // heading: Rotation around the Z-axis. -180 <= roll <= 180
    // a positive heading angle is defined to be a clockwise rotation about the positive Z-axis
    //
    //                                       z * sin(roll) - y * cos(roll)                           < Yh
    //   heading = atan(--------------------------------------------------------------------------)
    //                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z * sin(pitch) * cos(roll))  < Xh
    //
    // where:  x, y, z are returned value from magnetometer sensor
    Yh = Zm * Math.sin(Phi) - Ym * Math.cos(Phi);
    Xh = Xm * Math.cos(Theta) +
      Ym * Math.sin(Theta) * Math.sin(Phi) +
      Zm * Math.sin(Theta) * Math.cos(Phi);
    Psi = Math.atan2(-Yh, Xh);

    // Convert angular data to degree
    Phi = Phi * 180 / Math.PI;
    Theta = Theta * 180 / Math.PI;
    Psi = Psi * 180 / Math.PI;
    if (Psi < 0) Psi += 360;

    console.log(`Roll: ${Phi.toFixed(2)} Pitch: ${Theta.toFixed(2)} Yaw/Heading: ${Psi.toFixed(2)}`)

  }, 500)
})
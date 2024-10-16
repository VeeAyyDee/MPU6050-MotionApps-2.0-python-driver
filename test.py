from MPU6050_6Axis_MotionApps20 import MPU6050

devStatus = 0
MPUIntStatus = 0
packetSize = 0
DMPReady = False

mpu = MPU6050(0x69)

print("Initializing I2C devices...")
mpu.initialize()

print("Testing MPU6050 connection...")
if not mpu.testConnection():
    print("MPU6050 connection failed")
else:
    print("MPU6050 connection successful")


print("Initializing DMP...")
devStatus = mpu.dmpInitialize()


mpu.setXGyroOffset(0)
mpu.setYGyroOffset(0)
mpu.setZGyroOffset(0)
mpu.setXAccelOffset(0)
mpu.setYAccelOffset(0)
mpu.setZAccelOffset(0)



if devStatus == 0:
    mpu.CalibrateAccel(6)  # Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6)
    print("These are the Active offsets: ")
    mpu.PrintActiveOffsets()
    print("Enabling DMP...")  #Turning ON DMP
    mpu.setDMPEnabled(True)

    #Enable Arduino interrupt detection
    MPUIntStatus = mpu.getIntStatus()

    # Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    print("DMP ready! Waiting for first interrupt...")
    DMPReady = True
    packetSize = mpu.dmpGetFIFOPacketSize(); #Get expected DMP packet size for later comparison

else:
    print("DMP Initialization failed (code", devStatus, ")")


while True:
    data = mpu.getMotion6()
    print(f' AX: {data[0]:+2.3f} AY: {data[1]:+2.3f} AZ: {data[2]:+2.3f} GX: {data[3]:+3.3f} GY: {data[4]:+3.3f} GZ: {data[5]:+3.3f}')
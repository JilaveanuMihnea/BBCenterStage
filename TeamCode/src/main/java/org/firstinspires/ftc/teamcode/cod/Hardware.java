package org.firstinspires.ftc.teamcode.cod;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    public boolean isAuto = false;
    public DcMotor[] motor = new DcMotor[4];
    public DcMotor sliderMotor, tagaMotor;

    public Servo clawServoRight, clawServoLeft, clawServoHold, droneServo, droneAdjustServo;

    public AnalogInput analogInput;

    public BNO055IMU imu;
    public BNO055IMU.Parameters parameters;

    private HardwareMap hardwareMap = null;

    public Hardware(HardwareMap hdMap, boolean auto) {
        this.isAuto = auto;
        initializare(hdMap);
    }


    private DcMotor setDefaultStateMotor(DcMotor motor, String nume, DcMotorSimple.Direction direction) {
        motor = hardwareMap.get(DcMotor.class, nume);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
        return motor;
    }

    private void initializare(HardwareMap hdMap) {
        this.hardwareMap = hdMap;

        initMotors();
        initServos();
        initSenzors();
        initAnal();
        initImu();
    }

    private void initSenzors()
    {

    }
    private void initMotors() {
        motor[0] = setDefaultStateMotor(motor[0], "mFrontRight", DcMotorSimple.Direction.FORWARD); //0
        motor[1] = setDefaultStateMotor(motor[1], "mFrontLeft", DcMotorSimple.Direction.FORWARD); //1
        motor[2] = setDefaultStateMotor(motor[2], "mBackRight", DcMotorSimple.Direction.FORWARD); //2
        motor[3] = setDefaultStateMotor(motor[3], "mBackLeft", DcMotorSimple.Direction.FORWARD); //3

        sliderMotor = hardwareMap.get(DcMotor.class, "sliderMotor");
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tagaMotor = hardwareMap.get(DcMotor.class, "tagaMotor");
        tagaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tagaMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tagaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initServos(){
        clawServoRight = hardwareMap.get(Servo.class, "clawServoRight");
        clawServoLeft = hardwareMap.get(Servo.class, "clawServoLeft");
        clawServoHold = hardwareMap.get(Servo.class, "clawServoHold");
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        droneAdjustServo = hardwareMap.get(Servo.class, "droneAdjustServo");
    }

    private void initAnal(){
        analogInput = hardwareMap.get(AnalogInput.class, "myanaloginput");
    }

    private void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

}

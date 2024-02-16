package org.firstinspires.ftc.teamcode.cod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Vali", group="Linear Opmode")
public class Vali extends LinearOpMode {


    public DcMotor[] motor = new DcMotor[4];

    public DcMotor mSlider;

    public Servo  sClaw, sBeam;


    private float posL = 0;
    private float posR = 0;

    boolean openMouth = false;



    @Override
    public void runOpMode() throws InterruptedException {
        motor[0] = hardwareMap.get(DcMotor.class, "mFrontRight"); //0
        motor[1] = hardwareMap.get(DcMotor.class, "mFrontLeft"); //1
        motor[2] = hardwareMap.get(DcMotor.class, "mBackRight"); //2
        motor[3] = hardwareMap.get(DcMotor.class, "mBackLeft");

        mSlider = hardwareMap.get(DcMotor.class, "mSlider");

        sBeam = hardwareMap.get(Servo.class, "sBeam");
        sClaw = hardwareMap.get(Servo.class, "sClaw");

        waitForStart();

        while(opModeIsActive())
        {
            float forward = gamepad1.left_stick_y;
            float strafe = -gamepad1.left_stick_x;
            float rotation = gamepad1.right_stick_x;

            movement(forward, strafe, rotation);

            militaru(gamepad1.right_trigger, gamepad1.left_trigger);
            beam(gamepad1.right_bumper, gamepad1.left_bumper);
            claw(gamepad1.a, gamepad1.b);


        }

    }

    private void movement( float forward, float strafe, float rotation){
        // power applied to the robot wheel by wheel
        double[] power;
            power = new double[4];
            rotation *= -1;
            power[0] = (-forward -strafe + rotation) * 0.7f;   //+
            power[1] = (-forward - strafe - rotation) * 0.7f;   //-
            power[2] = (-forward + strafe + rotation) * 0.7f;   //-
            power[3] = (+forward - strafe + rotation) * 0.7f;
        //+
        // applying the power
        for (int i = 0; i < 4; i++) {
            motor[i].setPower(power[i]);
        }
    }
    private void militaru(float button, float button1){
        mSlider.setPower(button * 0.7f - button1 * 0.4f);
    }

    private void beam(boolean button1, boolean button2){
        if(button1) {
            sBeam.setPosition(0.55f);
        }
        else if (button2) {
            sBeam.setPosition(0.2f);
        }
    }

    private void claw(boolean button1, boolean button2){
        if(button1){
            sClaw.setPosition(0);
        }
        else if(button2){
            sClaw.setPosition(1);
        }
    }

}
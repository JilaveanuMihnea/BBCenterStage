package org.firstinspires.ftc.teamcode.cod;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Testare Glisiera", group="Version1")
public class Testare extends OpMode {
    private Hardware hardware;

    private ElapsedTime elapsedTime;

    private boolean clawStateRight = false;
    private boolean clawStateLeft = false;

    private boolean droneState = false;


    private boolean resetA = false;
    private boolean resetB = false;
    private double lastTimeA;
    private double lastTimeB;

    private double lastTimedrone;

    private boolean resetdrone = false;
    public void init(){
        hardware = new Hardware(hardwareMap, false);
        hardware.clawServoHold.setPosition(0.15f);
        hardware.clawServoRight.setPosition(Spec.OPENED_POS_RIGHT);
        hardware.clawServoLeft.setPosition(Spec.OPENED_POS_LEFT);
        elapsedTime = new ElapsedTime();
    }
    public void loop(){
        float forward = gamepad1.left_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float rotation = gamepad1.right_stick_x;
        boolean dronelnch = gamepad1.left_bumper;

        movement(forward, strafe, rotation);
        sliderManual(gamepad2.right_trigger, gamepad2.left_trigger);
        openClaws(gamepad2.a, gamepad2.b);
        tagaManual(gamepad2.right_bumper, gamepad2.left_bumper);
        drone(dronelnch);
        timers();


    }
    private void sliderManual(float up, float down){
        if(hardware.sliderMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
            hardware.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // todo testeaza daca se opreste glisiera jos
        hardware.sliderMotor.setPower(up*Spec.SLIDER_SPEED_UP-down*Spec.SLIDER_SPEED_DOWN*1);
    }

    private void openClaws(boolean button1, boolean button2) {
        if (!resetA && button1) {


            if (clawStateLeft) hardware.clawServoLeft.setPosition(Spec.OPENED_POS_LEFT);
            else hardware.clawServoLeft.setPosition(Spec.CLOSED_POS_LEFT);

            resetA = true;
            clawStateLeft = !clawStateLeft;
            lastTimeA = elapsedTime.milliseconds();
        }
        if (!resetB && button2) {
            if (clawStateRight) hardware.clawServoRight.setPosition(Spec.OPENED_POS_RIGHT);
            else hardware.clawServoRight.setPosition(Spec.CLOSED_POS_RIGHT);

            resetB = true;
            clawStateRight = !clawStateRight;
            lastTimeB = elapsedTime.milliseconds();
        }
    }
    private void tagaManual(boolean up, boolean down){
        if(hardware.tagaMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
            hardware.tagaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if(up) hardware.tagaMotor.setPower(Spec.TAGA_SPEED);
        else if (down) hardware.tagaMotor.setPower(-Spec.TAGA_SPEED);
        else hardware.tagaMotor.setPower(0);

    }
    private void movement( float forward, float strafe, float rotation){
        // power applied to the robot wheel by wheel
        double[] power = new double[4];
        rotation *= -1;
        power[0] = (-forward + strafe + rotation) * Spec.MOVEMENT_SPEED;   //+
        power[1] = (+forward + strafe + rotation) * Spec.MOVEMENT_SPEED;   //-
        power[2] = (-forward - strafe + rotation) * Spec.MOVEMENT_SPEED;   //-
        power[3] = (+forward - strafe + rotation) * Spec.MOVEMENT_SPEED;   //+
        // applying the power
        for (int i = 0; i < 4; i++) {
            hardware.motor[i].setPower(power[i]);
        }
    }
    private void drone(boolean button){
        //todo find values
        if (!resetdrone && button) {
            if (droneState) hardware.droneServo.setPosition(1f);
            else hardware.droneServo.setPosition(0f);
            if(droneState) hardware.droneAdjustServo.setPosition(Spec.DRONE_ADJUST);
            else hardware.droneAdjustServo.setPosition(0.3f);

            resetdrone = true;
            droneState = !droneState;
            lastTimedrone = elapsedTime.milliseconds();
        }

    }
    private void timers() {
        if(resetA && elapsedTime.milliseconds() > lastTimeA + Spec.BUTTON_DELAY) resetA = false;
        if(resetB && elapsedTime.milliseconds() > lastTimeB + Spec.BUTTON_DELAY) resetB = false;
        if(resetdrone && elapsedTime.milliseconds() > lastTimedrone + Spec.BUTTON_DELAY) resetdrone = false;
    }
}

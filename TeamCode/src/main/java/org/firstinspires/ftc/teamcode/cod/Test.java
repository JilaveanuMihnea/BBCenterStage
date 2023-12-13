package org.firstinspires.ftc.teamcode.cod;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test", group="Version1")
public class Test extends OpMode {

    private Hardware hardware;

    private ElapsedTime elapsedTime;

    private boolean clawStateRight = false;
    private boolean clawStateLeft = false;

    private boolean resetA = false;
    private boolean resetB = false;

    private double lastTimeA;
    private double lastTimeB;

    @Override
    public void init() {

        hardware = new Hardware(hardwareMap, false);
        elapsedTime = new ElapsedTime();

    }

    @Override
    public void loop() {
        openClaws(gamepad1.a, gamepad1.b);
        timers();
        float forward = gamepad1.left_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float rotation = gamepad1.right_stick_x;


        movement(forward, strafe, rotation);
        sliderManual(gamepad2.right_trigger, gamepad2.left_trigger);
        tagaManual(gamepad2.right_bumper, gamepad2.left_bumper);

        telemetry.addData("Servo angle",hardware.analogInput.getVoltage() / 3.3 * 360);
        telemetry.update();
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

    private void openClaws(boolean button1, boolean button2) {
        if (!resetA && button1) {
            if (clawStateLeft) hardware.clawServoLeft.setPosition(Spec.CLOSED_POS_LEFT);
            else hardware.clawServoLeft.setPosition(Spec.OPENED_POS_LEFT);

            resetA = true;
            clawStateLeft = !clawStateLeft;
            lastTimeA = elapsedTime.milliseconds();
        }
        if (!resetB && button2) {
            if (clawStateRight) hardware.clawServoRight.setPosition(Spec.CLOSED_POS_RIGHT);
            else hardware.clawServoRight.setPosition(Spec.OPENED_POS_RIGHT);

            resetB = true;
            clawStateRight = !clawStateRight;
            lastTimeB = elapsedTime.milliseconds();
        }
    }

    private void sliderManual(float up, float down){
        hardware.sliderMotor.setPower((up-down)*Spec.SLIDER_SPEED);
    }

    private void sliderAuto(){

    }

    private void tagaManual(boolean up, boolean down){
        hardware.tagaMotor.setPower(((up?1:0)-(down?1:0))*Spec.TAGA_SPEED);
    }

    private void tagaAuto(){

    }

    private void clawHold(){

    }

    private void clawMonkey(){

    }

    private void drone(){

    }

    private void pull(){

    }


    private void timers() {
        if(resetA && elapsedTime.milliseconds() > lastTimeA + Spec.BUTTON_DELAY) resetA = false;
        if(resetB && elapsedTime.milliseconds() > lastTimeB + Spec.BUTTON_DELAY) resetB = false;
    }
}
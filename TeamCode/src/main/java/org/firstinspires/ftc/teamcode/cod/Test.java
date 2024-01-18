package org.firstinspires.ftc.teamcode.cod;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOP", group="Version1")
public class Test extends OpMode {

    private Hardware hardware;

    private ElapsedTime elapsedTime;

    private boolean clawStateRight = false;
    private boolean clawStateLeft = false;
    private boolean clawStateHold = false;
    private boolean droneState = false;
    private boolean droneLaunchState = false;
    private boolean droneAdjustState = false;

    private boolean resetA = false;
    private boolean resetB = false;
    private boolean resetsq = false;
    private boolean resetdrone = false;
    private boolean resetLB = false;
    private boolean resetRB = false;

    private double tmr = 0;

    private double lastTimeA;
    private double lastTimeB;
    private double lastTimesq;
    private double lastTimedrone;
    private double lastTimeLB;
    private double lastTimeRB;


    private enum STATE_MACHINE
    {
        DRIVER_CONTROLLED,
        PIXEL_GRAB,
        PIXEL_PLACE
    }

    private enum GRAB_MOVEMENTS{
        EXTEND,
        CLAW,
        RETRACT
    }

    private enum PLACE_MOVEMENTS{
        RAISE,
        CLAW,
        LOWER
    }

    private STATE_MACHINE currentState = STATE_MACHINE.DRIVER_CONTROLLED;
    private GRAB_MOVEMENTS grabState = GRAB_MOVEMENTS.EXTEND;
    private PLACE_MOVEMENTS placeState = PLACE_MOVEMENTS.RAISE;

    @Override
    public void init() {

        hardware = new Hardware(hardwareMap, false);
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
        hardware.clawServoRight.setPosition(Spec.OPENED_POS_RIGHT);
        hardware.clawServoLeft.setPosition(Spec.OPENED_POS_LEFT);
        hardware.droneServo.setPosition(Spec.DRONE_HOLD);

        elapsedTime = new ElapsedTime();

    }

    @Override
    public void loop() {
        //binds

        // china hack controls
//        float forward = gamepad1.left_stick_y;
//        float strafe = -gamepad1.right_stick_x;
//        float lrotation = gamepad1.left_trigger;
//        float rrotation = gamepad1.right_trigger;
//        float rotation = rrotation-lrotation;
        //gp1 - movement
        float forward = gamepad1.left_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float rotation = gamepad1.right_stick_x;

        //gp1 - drone
        boolean dronelnch = gamepad1.a;
        boolean droneadj = gamepad1.y;

        //gp1 - hang
        float hangUp = gamepad1.right_trigger;
        float hangDown = gamepad1.left_trigger;

        //gp2 - manual
        float sliderUp = gamepad2.right_trigger;
        float sliderDown = gamepad2.left_trigger;
        boolean tagaUp = gamepad2.right_bumper;
        boolean tagaDown = gamepad2.left_bumper;
        boolean clawLeft = gamepad2.x;
        boolean clawRight = gamepad2.b;

        //gp2 - auto
        boolean grabsqc = gamepad2.dpad_down;
        boolean placesqc = gamepad2.dpad_up;

        movement(forward, strafe, rotation);
        timers();
        tagaManual(tagaUp, tagaDown);
        openClaws(clawLeft, clawRight);
        drone(dronelnch, droneadj);
        clawHold();
        hang(hangUp, hangDown);

        switch(currentState){
            case DRIVER_CONTROLLED:
                sliderManual(sliderUp, sliderDown);
                tagaManual(tagaUp, tagaDown);
                openClaws(clawLeft, clawRight);
                if(grabsqc){
                    grabState = GRAB_MOVEMENTS.EXTEND;
                    currentState = STATE_MACHINE.PIXEL_GRAB;
                }
                if(placesqc){
                    placeState = PLACE_MOVEMENTS.RAISE;
                    currentState = STATE_MACHINE.PIXEL_PLACE;
                }
                break;

            case PIXEL_GRAB:
                switch(grabState){
                    case EXTEND:
                        sliderAuto(Spec.SLIDER_TICK_GRAB);
                        if(hardware.sliderMotor.getCurrentPosition()>Spec.SLIDER_TICK_GRAB-50){
                            grabState = GRAB_MOVEMENTS.CLAW;

                        }
                        break;

                    case CLAW:
                        openClaws(clawLeft, clawRight);
                        sliderManual(sliderUp, sliderDown);
                        //todo inverseaza daca inchis = false state
                        //delay?
                        if(clawStateLeft && clawStateRight ){
                            telemetry.addData("aaaaaa", 1);
                            grabState = GRAB_MOVEMENTS.RETRACT;
                        }
                        break;

                    case RETRACT:
                        hardware.clawServoLeft.setPosition(Spec.CLOSED_POS_LEFT);
                        hardware.clawServoRight.setPosition(Spec.CLOSED_POS_RIGHT);
                        sliderAuto(0);
                        if(hardware.sliderMotor.getCurrentPosition()<50){
                            currentState = STATE_MACHINE.DRIVER_CONTROLLED;
                        }
                        break;
                }
                break;

            case PIXEL_PLACE:
                switch(placeState){
                    case RAISE:
                        tagaAuto(Spec.TAGA_TICK_60DEG);
                        if(!hardware.tagaMotor.isBusy()){
                            //add slider raise autolevel ?
                            tmr = 0;


                            placeState = PLACE_MOVEMENTS.CLAW;

                        }
                        break;

                    case CLAW:
//                        hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);
                        sliderManual(sliderUp, sliderDown);
                        openClaws(clawLeft, clawRight);
                        tagaManual(tagaUp, tagaDown);
                        if(gamepad2.y){
                            placeState = PLACE_MOVEMENTS.LOWER;
                        }
                        break;

                    case LOWER:
//                        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                        sliderAuto(0);

                        if(Math.abs(hardware.sliderMotor.getCurrentPosition()) < Spec.TAGA_THRESHOLD_LOWER){
                            tagaAuto(50);

                        }else{
                            tagaAuto(Spec.TAGA_TICK_60DEG);
                        }
                        if(Math.abs(hardware.tagaMotor.getCurrentPosition()) < 50){
                            currentState = STATE_MACHINE.DRIVER_CONTROLLED;
                        }
                        break;
                }
                break;
        }


        telemetry.addData("state", currentState);
        telemetry.addData("place", placeState);
        telemetry.addData("TAGA",hardware.tagaMotor.getCurrentPosition());
        telemetry.addData("SLIDER", hardware.sliderMotor.getCurrentPosition());
//        telemetry.addData("dsLeft", hardware.dsLeft.getDistance(DistanceUnit.CM));
//        telemetry.addData("dsRight", hardware.dsRight.getDistance(DistanceUnit.CM));
        telemetry.addData("Claw Pos",hardware.analogInput.getVoltage()/3.3 * 360);
        telemetry.update();
    }

    private void movement( float forward, float strafe, float rotation){
        // power applied to the robot wheel by wheel
        if(hardware.dsLeft.getDistance(DistanceUnit.CM)<5 || hardware.dsRight.getDistance(DistanceUnit.CM)<5){
            gamepad1.rumble(100);
//            forward = Math.min(0, forward);
        }
        double[] power;
        if(gamepad1.right_bumper){
            power = new double[4];
            rotation *= -1;
            power[0] = (-forward + strafe + rotation) * Spec.MOVEMENT_SPEED * 0.4f;   //+
            power[1] = (+forward + strafe + rotation) * Spec.MOVEMENT_SPEED * 0.4f;   //-
            power[2] = (-forward - strafe + rotation) * Spec.MOVEMENT_SPEED * 0.4f;   //-
            power[3] = (+forward - strafe + rotation) * Spec.MOVEMENT_SPEED * 0.4f;   //+
            // applying the power
        }
        else {
            power = new double[4];
            rotation *= -1;
            power[0] = (-forward + strafe + rotation) * Spec.MOVEMENT_SPEED;   //+
            power[1] = (+forward + strafe + rotation) * Spec.MOVEMENT_SPEED;   //-
            power[2] = (-forward - strafe + rotation) * Spec.MOVEMENT_SPEED;   //-
            power[3] = (+forward - strafe + rotation) * Spec.MOVEMENT_SPEED;
        }//+
        // applying the power
        for (int i = 0; i < 4; i++) {
            hardware.motor[i].setPower(power[i]);
        }
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

    private void sliderManual(float up, float down){
        if(hardware.sliderMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
            hardware.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        hardware.sliderMotor.setPower(up*Spec.SLIDER_SPEED_UP*(hardware.sliderMotor.getCurrentPosition()>1550?0:1)-down*Spec.SLIDER_SPEED_DOWN*(hardware.sliderMotor.getCurrentPosition()<50?0:1));
    }

    private void sliderBun(float up, float down){
        if(hardware.sliderMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !hardware.sliderMotor.isBusy()){
            hardware.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // todo testeaza daca se opreste glisiera jos
        hardware.sliderMotor.setPower(up*Spec.SLIDER_SPEED_UP-down*Spec.SLIDER_SPEED_DOWN*1);
    }

    private void sliderAuto(int ticks){
        hardware.sliderMotor.setTargetPosition(ticks);
        hardware.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(ticks>hardware.sliderMotor.getCurrentPosition())
            hardware.sliderMotor.setPower(Spec.SLIDER_SPEED_UP);
        else
            hardware.sliderMotor.setPower(Spec.SLIDER_SPEED_DOWN);
    }


    private void tagaManual(boolean up, boolean down){
        if(hardware.tagaMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
            if(!hardware.tagaMotor.isBusy())
                hardware.tagaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if(up) hardware.tagaMotor.setPower(Spec.TAGA_SPEED);
        else if (down) hardware.tagaMotor.setPower(-Spec.TAGA_SPEED);
        else{
            int tagaPos = hardware.tagaMotor.getCurrentPosition();
            if(tagaPos > 840) hardware.tagaMotor.setPower(-0.001f);
            else if(tagaPos <= 840) hardware.tagaMotor.setPower(0.001f);
            else hardware.tagaMotor.setPower(0f);
        }

    }
    private void tagaAuto(int ticks){
        hardware.tagaMotor.setPower(0f);
        hardware.tagaMotor.setTargetPosition(ticks);
        hardware.tagaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.tagaMotor.setPower(Spec.TAGA_SPEED+0.3);
    }

    private void clawHold(){
        //todo align claw with backdrop angle
        int armPos = hardware.tagaMotor.getCurrentPosition();
        if(armPos<=840){
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
        }else{
            hardware.clawServoHold.setPosition(Spec.HOLD_PLACE+((armPos-840)/8.33-30)/355);
        }
    }

    private void hang(float button1, float button2){
        if(button1 > 0) {
            hardware.servoTest.setPower(0.5f);
            hardware.servoTest2.setPower(-0.5f);
        }
        else
        if(button2 > 0) {
            hardware.servoTest.setPower(-0.5f);
            hardware.servoTest2.setPower(0.5f);
        }
        else {
            hardware.servoTest.setPower(0);
            hardware.servoTest2.setPower(0);
        }
    }

    private void drone(boolean drnlnch, boolean raise){
        //todo find values
        if (!resetLB && drnlnch) {


            if (droneLaunchState) hardware.droneServo.setPosition(Spec.DRONE_HOLD);
            else hardware.droneServo.setPosition(Spec.DRONE_LAUNCH);

            resetLB = true;
            droneLaunchState = !droneLaunchState;
            lastTimeLB = elapsedTime.milliseconds();
        }

        if (!resetRB && raise) {


            if (droneAdjustState) hardware.droneAdjustServo.setPosition(Spec.DRONE_ADJUST);
            else hardware.droneAdjustServo.setPosition(0.15f);

            resetRB = true;
            droneAdjustState = !droneAdjustState;
            lastTimeRB = elapsedTime.milliseconds();
        }

    }

    private void timers() {
        if(resetA && elapsedTime.milliseconds() > lastTimeA + Spec.BUTTON_DELAY) resetA = false;
        if(resetB && elapsedTime.milliseconds() > lastTimeB + Spec.BUTTON_DELAY) resetB = false;
        if(resetsq && elapsedTime.milliseconds() > lastTimesq + Spec.BUTTON_DELAY) resetsq = false;
        if(resetdrone && elapsedTime.milliseconds() > lastTimedrone + Spec.BUTTON_DELAY) resetdrone = false;
        if(resetLB && elapsedTime.milliseconds() > lastTimeLB + Spec.BUTTON_DELAY)  resetLB = false;
        if(resetRB && elapsedTime.milliseconds() > lastTimeRB + Spec.BUTTON_DELAY)  resetRB = false;
    }
}
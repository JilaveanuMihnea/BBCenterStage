package org.firstinspires.ftc.teamcode.cod;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test", group="Version1")
public class Test extends OpMode {

    private Hardware hardware;

    private ElapsedTime elapsedTime;

    private boolean clawStateRight = false;
    private boolean clawStateLeft = false;
    private boolean clawStateHold = false;

    private boolean resetA = false;
    private boolean resetB = false;
    private boolean resetsq = false;

    private double lastTimeA;
    private double lastTimeB;
    private double lastTimesq;


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
        //hardware.tagaMotor.setTargetPosition(650);
       // hardware.tagaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //hardware.tagaMotor.setPower(0.4f);
        elapsedTime = new ElapsedTime();

    }

    @Override
    public void loop() {
        //binds
        float forward = gamepad1.left_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float rotation = gamepad1.right_stick_x;
        boolean dronelnch = gamepad1.left_bumper;

        float sliderUp = gamepad2.right_trigger;
        float sliderDown = gamepad2.left_trigger;
        boolean tagaUp = gamepad2.right_bumper;
        boolean tagaDown = gamepad2.left_bumper;
        boolean clawLeft = gamepad2.a;
        boolean clawRight = gamepad2.b;

        boolean grabsqc = gamepad2.dpad_down;
        boolean placesqc = gamepad2.dpad_up;

        movement(forward, strafe, rotation);
        timers();


        switch(currentState){
            case DRIVER_CONTROLLED:
                sliderManual(sliderUp, sliderDown);
                tagaManual(tagaUp, tagaDown);
                openClaws(clawLeft, clawRight);
                drone(dronelnch);
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
                        if(clawStateLeft && clawStateRight){
                            grabState = GRAB_MOVEMENTS.RETRACT;
                        }
                        break;

                    case RETRACT:
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
                        if(Math.abs(hardware.tagaMotor.getCurrentPosition()) > Spec.SLIDER_THRESHOLD_RAISE){
                            //add slider raise autolevel ?
                            placeState = PLACE_MOVEMENTS.CLAW;
                        }
                        break;

                    case CLAW:
                        sliderManual(sliderUp, sliderDown);
                        openClaws(clawLeft, clawRight);
                        //todo inverseaza
                        //delay?
                        if(!clawStateLeft && !clawStateRight){
                            placeState = PLACE_MOVEMENTS.LOWER;
                        }
                        break;

                    case LOWER:
                        sliderAuto(0);
                        if(Math.abs(hardware.sliderMotor.getCurrentPosition()) < Spec.TAGA_THRESHOLD_LOWER){
                            tagaAuto(0);

                        }
                        if(Math.abs(hardware.tagaMotor.getCurrentPosition()) < 50){
                            currentState = STATE_MACHINE.DRIVER_CONTROLLED;
                        }
                        break;
                }
                break;
        }

//        clawMonkey(gamepad2.x);


        telemetry.addData("TAGA",hardware.tagaMotor.getCurrentPosition());
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
        if(hardware.sliderMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
            hardware.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // todo testeaza daca se opreste glisiera jos
        hardware.sliderMotor.setPower(up*Spec.SLIDER_SPEED_UP-down*Spec.SLIDER_SPEED_DOWN*(hardware.sliderMotor.getCurrentPosition()<50?0:1));
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
            hardware.tagaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if(up) hardware.tagaMotor.setPower(Spec.TAGA_SPEED);
        else if (down) hardware.tagaMotor.setPower(-Spec.TAGA_SPEED);
        else hardware.tagaMotor.setPower(0);

    }
    private void tagaAuto(int ticks){
        hardware.tagaMotor.setTargetPosition(ticks);
        hardware.tagaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.tagaMotor.setPower(Spec.TAGA_SPEED);
    }


    private void clawHold(){
        //todo align claw with backdrop angle
        //nu acum
    }

    private void clawMonkey(boolean button){
        if (!resetsq && button) {
            if (clawStateHold) hardware.clawServoHold.setPosition(0.8f);
            else hardware.clawServoHold.setPosition(0.7f);

            resetsq = true;
            clawStateHold = !clawStateHold;
            lastTimesq = elapsedTime.milliseconds();
        }
    }

    private void drone(boolean button){
        //todo find values
        if(button){
           hardware.droneAdjustServo.setPosition(Spec.DRONE_ADJUST);
            //double tmr = elapsedTime.milliseconds();
            //if(tmr+400 < elapsedTime.milliseconds()){
                hardware.droneServo.setPosition(Spec.DRONE_LAUNCH);
           // }
        }

    }

    private void pull(boolean button){

    }


    private void timers() {
        if(resetA && elapsedTime.milliseconds() > lastTimeA + Spec.BUTTON_DELAY) resetA = false;
        if(resetB && elapsedTime.milliseconds() > lastTimeB + Spec.BUTTON_DELAY) resetB = false;
        if(resetsq && elapsedTime.milliseconds() > lastTimesq + Spec.BUTTON_DELAY) resetsq = false;
    }
}
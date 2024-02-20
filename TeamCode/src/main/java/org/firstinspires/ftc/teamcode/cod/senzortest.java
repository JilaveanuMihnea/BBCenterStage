package org.firstinspires.ftc.teamcode.cod;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="hangTest", group="Version1")
public class senzortest extends OpMode{
    private Hardware hardware;

    private ElapsedTime elapsedTime;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap, false);
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void loop() {
//        telemetry.addData("left", hardware.dsLeft.getDistance(DistanceUnit.CM));
//        telemetry.addData("left", hardware.dsLeft.getDistance(DistanceUnit.CM));
//        telemetry.update();
//        if(hardware.dsLeft.getDistance(DistanceUnit.CM)<5 || hardware.dsRight.getDistance(DistanceUnit.CM)<5){
//            hardware.clawServoHold.setPosition(0f);
//        }else{
//            hardware.clawServoHold.setPosition(1f);
//        }
        if(gamepad1.right_trigger > 0){
            hardware.servoHangLeft.setPower(1f);
            hardware.servoHangRight.setPower(-1f);
        } else if (gamepad1.left_trigger > 0) {
            hardware.servoHangLeft.setPower(-1f);
            hardware.servoHangRight.setPower(1f);
        }
        else {
            hardware.servoHangLeft.setPower(0f);
            hardware.servoHangRight.setPower(0f);
        }

        }
    }



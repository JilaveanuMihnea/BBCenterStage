package org.firstinspires.ftc.teamcode.cod;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Testare Glisiera", group="Version1")
public class Testare extends OpMode {
    private Hardware hardware;
    public void init(){
        hardware = new Hardware(hardwareMap, false);
    }
    public void loop(){
        sliderManual(gamepad2.right_trigger, gamepad2.left_trigger);
    }
    private void sliderManual(float up, float down){
        if(hardware.sliderMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
            hardware.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // todo testeaza daca se opreste glisiera jos
        hardware.sliderMotor.setPower(up*Spec.SLIDER_SPEED_UP-down*Spec.SLIDER_SPEED_DOWN*1);
    }
}

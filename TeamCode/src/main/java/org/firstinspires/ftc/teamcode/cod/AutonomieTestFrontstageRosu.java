package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Rosu Test Frontstage", group="Autonomy")
public class AutonomieTestFrontstageRosu extends ScheletRosu{
    @Override
    public void runOpMode(){
        init_auto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        while(!isStarted() && !isStopRequested()){
            telemetryTfod();
        }

        java.util.List<Recognition> currentRecognitions = tfod.getRecognitions();

        sleep(300);
        placesqc();
    }
}

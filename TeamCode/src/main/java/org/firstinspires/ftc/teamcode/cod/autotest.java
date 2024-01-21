package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: autotest", group="Autonomy")
public class autotest extends ScheletRosu {
    ElapsedTime elapsedTime;

    boolean edgepark = true;
    float xpark = 4f; //52 -> center, 4 -> edge

    @Override
    public void runOpMode() {
        if(edgepark){
            xpark = 4f;
        }else{
            xpark = 50f;
        }
        init_auto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj_test, traj_1;
        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPos);

        traj_test = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(18, 0, Math.toRadians(0)))
                .addTemporalMarker(0.1, () -> {
                            sliderAuto(1000);
                        }
                )
                .build();
        traj_1 = drive.trajectoryBuilder(traj_test.end())
                .lineToLinearHeading(new Pose2d(26,-30,Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(Spec.TAGA_PE_SPATE,0.7f);

                })
                .build();

        while(!isStarted() && !isStopRequested()){
            telemetryTfod();
        }
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN+0.02f);
        clawOpenBoth();
        tagaAuto(78, 1f);
        sleep(100);
       // tagaFixativ();
        sleep(30000);
        java.util.List<Recognition> currentRecognitions = tfod.getRecognitions();
        sleep(300);
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);

        drive.followTrajectory(traj_test);
        sleep(500);
        clawOpenLeft();
        sleep(500);
        drive.followTrajectory(traj_1);
        sleep(500);
        clawOpenRight();
        sleep(500);
        tagaAuto(100,0.5f);
        sleep(1000);
        sliderAuto(0);
        sleep(10000);
    }
}

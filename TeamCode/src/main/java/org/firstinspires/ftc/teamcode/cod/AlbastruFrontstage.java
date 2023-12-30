package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Albastru Frontstage", group="Autonomy")
public class AlbastruFrontstage extends ScheletAlbastru {

    ElapsedTime elapsedTime;
    @Override
    public void runOpMode() {
        init_auto();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1_0, traj1_1, traj1_2, traj1_3, traj1_4, traj1_5, traj2_0, traj2_1, trajtranz, traj2_2, traj2_3, traj2_4, traj3_0, traj3_1, traj3_2, traj3_3, traj3_4;

        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPos);


        //case 1 traj
        traj1_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(1, -4, Math.toRadians(0)))
                .build();

        traj1_1 = drive.trajectoryBuilder(traj1_0.end())
                .lineToLinearHeading(new Pose2d(27, -4, Math.toRadians(0)))
                .build();

        traj1_2 = drive.trajectoryBuilder(traj1_1.end())
                .lineToLinearHeading(new Pose2d(26, -5.2, Math.toRadians(90)))
                .build();

        traj1_3 = drive.trajectoryBuilder(traj1_2.end())
                .lineToLinearHeading(new Pose2d(54, -4, Math.toRadians(90)))
                .build();

        //case 2 traj
        traj2_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(1, 4, Math.toRadians(0)))
                .build();

        traj2_1 = drive.trajectoryBuilder(traj2_0.end())
                .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(0)))
                .build();

        trajtranz = drive.trajectoryBuilder(traj2_1.end())

                .lineToLinearHeading(new Pose2d(60, 0, Math.toRadians(90)))
                .build();

        traj2_2 = drive.trajectoryBuilder(trajtranz.end())
                .lineToLinearHeading(new Pose2d(54, -88, Math.toRadians(90)))
                .build();

        traj2_3 = drive.trajectoryBuilder(traj2_2.end())
                .lineToLinearHeading(new Pose2d(35, -88, Math.toRadians(90)))
                .build();



        //case 3 traj
        traj3_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(1, -4, Math.toRadians(0)))
                .build();

        traj3_1 = drive.trajectoryBuilder(traj3_0.end())
                .lineToLinearHeading(new Pose2d(27, -4, Math.toRadians(0)))
                .build();

        traj3_2 = drive.trajectoryBuilder(traj3_1.end())
                .lineToLinearHeading(new Pose2d(26, -4, Math.toRadians(-90)))
                .build();

        traj3_3 = drive.trajectoryBuilder(traj3_2.end())
                .lineToLinearHeading(new Pose2d(54, -4, Math.toRadians(90)))
                .build();

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while(!isStarted() && !isStopRequested()){
            telemetryTfod();
        }

        java.util.List<Recognition> currentRecognitions = tfod.getRecognitions();
        if(currentRecognitions.size()==0){
            drive.followTrajectory(traj1_0);
            sleep(300);
            drive.followTrajectory(traj1_1);
            sleep(300);
            drive.followTrajectory(traj1_2);
            sleep(300);
            clawOpenLeft();
            sleep(100);
            hardware.clawServoHold.setPosition(0.7f);
            sleep(350);

        }else if((currentRecognitions.get(0).getLeft() + currentRecognitions.get(0).getRight())/2 < 650){
            drive.followTrajectory(traj2_0);
            sleep(300);
            drive.followTrajectory(traj2_1);
            sleep(300);
            drive.turn(Math.toRadians(190));
            sleep(300);
            clawOpenLeft();
            sleep(100);
            hardware.clawServoHold.setPosition(0.7f);
            sleep(300);

            drive.followTrajectory(traj2_2);
            sleep(300);
            drive.followTrajectory(traj2_3);

        }else{
            drive.followTrajectory(traj3_0);
            sleep(300);
            drive.followTrajectory(traj3_1);
            sleep(300);
            drive.followTrajectory(traj3_2);
            sleep(300);
            clawOpenLeft();
            sleep(100);
            hardware.clawServoHold.setPosition(0.7f);
            sleep(350);

        }



        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());


        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        sleep(4000);

    }
}

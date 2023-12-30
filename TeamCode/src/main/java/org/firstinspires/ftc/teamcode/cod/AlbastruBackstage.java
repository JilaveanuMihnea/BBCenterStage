package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Albastru Backstage", group="Autonomy")
public class AlbastruBackstage extends ScheletAlbastru {

    ElapsedTime elapsedTime;
    @Override
    public void runOpMode() {
        init_auto();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1_0, traj1_1, traj1_2, traj1_3, traj1_4;
        Trajectory traj3_0, traj3_1, traj3_2, traj3_3, traj3_4, traj3_5;
        Trajectory traj2_0, traj2_1, traj2_2, traj2_3, traj2_4, trajtranz;

        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPos);

        //caz ajutor os mor@"D"DD:D
        traj1_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(1, -4, Math.toRadians(0)))
                .build();

        traj1_1 = drive.trajectoryBuilder(traj1_0.end())
                .lineToLinearHeading(new Pose2d(27, -4, Math.toRadians(-90)))
                .build();

        traj1_2 = drive.trajectoryBuilder(traj1_1.end())
                .lineToLinearHeading(new Pose2d(26, -5.2, Math.toRadians(-90)))
                .build();

        traj1_3 = drive.trajectoryBuilder(traj1_2.end())
                .lineToLinearHeading(new Pose2d(54, -4, Math.toRadians(-90)))
                .build();

        //caz mijcij
        traj2_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(1, -4, Math.toRadians(0)))
                .build();

        traj2_1 = drive.trajectoryBuilder(traj2_0.end())
                .lineToLinearHeading(new Pose2d(27, -4, Math.toRadians(0)))
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
        //caz ahhHAHAHAHA
        traj3_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(1, -4, Math.toRadians(0)))
                .build();

        traj3_1 = drive.trajectoryBuilder(traj3_0.end())
                .lineToLinearHeading(new Pose2d(27, -4, Math.toRadians(0)))
                .build();

        traj3_2 = drive.trajectoryBuilder(traj3_1.end())
                .lineToLinearHeading(new Pose2d(27, -4, Math.toRadians(-90)))
                .build();

        traj3_3 = drive.trajectoryBuilder(traj3_2.end())
                .lineToLinearHeading(new Pose2d(27, 30, Math.toRadians(-90)))
                .build();

        traj3_4 = drive.trajectoryBuilder(traj3_3.end())
                .lineToLinearHeading(new Pose2d(22, 30, Math.toRadians(-90)))
                .build();

        traj3_5 = drive.trajectoryBuilder(traj3_4.end())
                .lineToLinearHeading(new Pose2d(22, 44, Math.toRadians(-90)))
                .build();

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while(!isStarted() && !isStopRequested()){
            telemetryTfod();
        }

        java.util.List<Recognition> currentRecognitions = tfod.getRecognitions();

        sleep(300);
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);

        if(currentRecognitions.size()==0){//caz gen spre backdrop
            drive.followTrajectory(traj1_0);
            sleep(300);
            clawOpenRight();
            sleep(100);
            hardware.clawServoHold.setPosition(0.7f);
            sleep(350);
            drive.followTrajectory(traj1_1);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            sleep(700);
            placesqc();
            sleep(300);
            /*drive.followTrajectory(traj1_2);
            sleep(300);
            drive.followTrajectory(traj1_3);*/

        }else if((currentRecognitions.get(0).getLeft()+currentRecognitions.get(0).getRight())/2<650){//caz mijloc
            drive.followTrajectory(traj2_0);
            drive.followTrajectory(traj2_1);
            sleep(150);
            clawOpenRight();
            sleep(100);
            hardware.clawServoHold.setPosition(0.7f);
            sleep(350);
            drive.followTrajectory(traj2_2);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            sleep(700);
            placesqc();
            clawOpenRight();
//            sleep(300);
//            drive.followTrajectory(traj2_3);
//            sleep(300);
//            drive.followTrajectory(traj2_4);

        }else{
            drive.followTrajectory(traj3_0);
            drive.followTrajectory(traj3_1);
            sleep(250);
            drive.followTrajectory(traj3_2);
            sleep(250);
            clawOpenRight();
            sleep(100);
            hardware.clawServoHold.setPosition(0.7f);
            sleep(350);
            drive.followTrajectory(traj3_3);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            sleep(700);
            placesqc();
            clawOpenRight();
//            sleep(300);
//            drive.followTrajectory(traj3_4);
//            sleep(300);
//            drive.followTrajectory(traj3_5);
        }

        hardware.clawServoHold.setPosition(0);
        sleep(1000);



        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

    }
}

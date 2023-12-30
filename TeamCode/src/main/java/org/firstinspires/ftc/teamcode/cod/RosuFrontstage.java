package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Test", group="Autonomy")
public class RosuFrontstage extends ScheletRosu{
    ElapsedTime elapsedTime;
    @Override
    public void runOpMode(){
        init_auto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj_ajutor, traj0, traj1_0, traj1_2, traj1_3, traj1_4, traj1_5, traj2_0, traj2_1, traj2_2, traj2_3, traj2_4, traj2_5, traj3_0, traj3_1, traj3_2, traj3_3, traj3_4;

        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPos);

        traj0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .build();

        traj1_0 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(32, -1, Math.toRadians(90)))
                .build();

        traj2_1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(90)))
                .build();

        traj2_2 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(26, -80, Math.toRadians(0)))
                .build();

        traj2_3 = drive.trajectoryBuilder(traj2_2.end())
                .lineToLinearHeading(new Pose2d(22, -80, Math.toRadians(90)))
                .build();

        traj_ajutor = drive.trajectoryBuilder(traj2_3.end())
                .lineToLinearHeading(new Pose2d(23, -80, Math.toRadians(90)))
                .build();
        traj2_4 = drive.trajectoryBuilder(traj_ajutor.end())
                .lineToLinearHeading(new Pose2d(49, -82, Math.toRadians(90)))
                .build();

        traj2_5 = drive.trajectoryBuilder(traj2_4.end())
                .lineToLinearHeading(new Pose2d(46,-97, Math.toRadians(90)))
                .build();
        traj3_0 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(30 , -2 ,Math.toRadians(-90)))
                .build();


        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while(!isStarted() && !isStopRequested()){
            telemetryTfod();
        }

        java.util.List<Recognition> currentRecognitions = tfod.getRecognitions();
        sleep(300);
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);

        if(currentRecognitions.size()==0){//caz gen sttanga
            drive.followTrajectory(traj0);
            sleep(300);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            drive.followTrajectory(traj1_0);
            sleep(300);
            clawOpenLeft();
            sleep(100);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
//            placesqc();
//            sleep(300);
           /*drive.followTrajectory(traj1_2);
            sleep(300);
            drive.followTrajectory(traj1_3);*/

        }else if((currentRecognitions.get(0).getLeft()+currentRecognitions.get(0).getRight())/2<650){//caz mijloc
            drive.followTrajectory(traj0);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            clawOpenLeft();
            sleep(300);
            hardware.clawServoHold.setPosition(0.47f);
            hardware.clawServoLeft.setPosition(Spec.CLOSED_POS_LEFT);
           // drive.followTrajectory(traj2_1);
            sleep(200);
            drive.followTrajectory(traj2_2);
            sleep(200);
            drive.followTrajectory(traj2_3);
            sleep(200);
            drive.followTrajectory(traj_ajutor);
            sleep(300);
            placesqc();
            sleep(200);
            drive.followTrajectory(traj2_4);
            sleep(200);
            drive.followTrajectory(traj2_5);

//            hardware.clawServoHold.setPosition(0.7f);
//            sleep(350);
//            drive.followTrajectory(traj2_2);
//            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
//            sleep(700);
//            placesqc();
//            clawOpenRight();
//            sleep(300);
//            drive.followTrajectory(traj2_3);
//            sleep(300);
//            drive.followTrajectory(traj2_4);

        }else{                                        // dreapta
            drive.followTrajectory(traj0);
            sleep(300);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            drive.followTrajectory(traj3_0);
            sleep(250);
            clawOpenLeft();
            sleep(100);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
//            drive.followTrajectory(traj3_2);
//            sleep(250);
//            clawOpenRight();
//            sleep(100);
//            hardware.clawServoHold.setPosition(0.7f);
//            sleep(350);
//            drive.followTrajectory(traj3_3);
//            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
//            sleep(700);
//            placesqc();
//            clawOpenRight();
//            sleep(300);
//            drive.followTrajectory(traj3_4);
//            sleep(300);
//            drive.followTrajectory(traj3_5);
        }

//        drive.followTrajectory(traj1_0);
//        drive.followTrajectory(traj1_1);

    }
}

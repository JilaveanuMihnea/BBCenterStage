package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Albastru backstage", group="Autonomy")
public class AlbastruBackstage extends ScheletAlbastru {
    ElapsedTime elapsedTime;

    boolean edgepark = true;
    float xpark = 4f; //52 -> center, 4 -> edge

    @Override
    public void runOpMode() {
        if(edgepark){
            xpark = 3f;
        }else{
            xpark = 47f;
        }
        init_auto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1_0, traj1_1,traj1_2, traj1_3, traj1_4, traj1_5, traj1_6,
                traj2_0, traj2_1, traj2_2, traj2_3, traj2_4, traj2_5, traj2_6,
                traj3_0, traj3_1, traj3_2, traj3_3, traj3_4;

        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPos);

        //dreapta
        traj1_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .build();

        traj1_1 = drive.trajectoryBuilder(traj1_0.end())
                .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(-90)))
                .build();

        traj1_2 = drive.trajectoryBuilder(traj1_1.end())
                .lineToLinearHeading(new Pose2d(33,32.5,Math.toRadians(-90)))
                .build();

        traj1_3 = drive.trajectoryBuilder(traj1_2.end())
                .lineToLinearHeading(new Pose2d(xpark, 32.5, Math.toRadians(-90)))
                .build();

        traj1_4 = drive.trajectoryBuilder(traj1_3.end())
                .lineToLinearHeading(new Pose2d(xpark, 46, Math.toRadians(-90)))
                .build();

        //mijloc
        traj2_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .build();

        traj2_1 = drive.trajectoryBuilder(traj2_0.end())
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                .build();

        traj2_2 = drive.trajectoryBuilder(new Pose2d(20, 0, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(-90)))
                .build();

        traj2_3 = drive.trajectoryBuilder(traj2_2.end())
                .lineToLinearHeading(new Pose2d(27.5, 32.5, Math.toRadians(-90)))
                .build();

        traj2_4 = drive.trajectoryBuilder(traj2_3.end())
                .lineToLinearHeading(new Pose2d(xpark, 32.5, Math.toRadians(-90)))
                .build();

        traj2_5 = drive.trajectoryBuilder(traj2_4.end())
                .lineToLinearHeading(new Pose2d(xpark, 46, Math.toRadians(-90)))
                .build();

        //stanga
        traj3_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(20, 12.5, Math.toRadians(0)))
                .build();

        traj3_1 = drive.trajectoryBuilder(traj3_0.end())
                .lineToLinearHeading(new Pose2d(17, 32.5, Math.toRadians(0)))
                .build();

        traj3_2 = drive.trajectoryBuilder(new Pose2d(17, 32.5, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(19.5, 32.5, Math.toRadians(-90)))
                .build();

        traj3_3 = drive.trajectoryBuilder(traj3_2.end())
                .lineToLinearHeading(new Pose2d(xpark, 32.5, Math.toRadians(-90)))
                .build();

        traj3_4 = drive.trajectoryBuilder(traj3_3.end())
                .lineToLinearHeading(new Pose2d(xpark, 46, Math.toRadians(-90)))
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
            drive.followTrajectory(traj3_0);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            clawOpenRight();
            sleep(200);
            hardware.clawServoHold.setPosition(0.45f);
            clawOpenRight();
            drive.followTrajectory(traj3_1);
            sleep(50);
            drive.turn(Math.toRadians(-90));
            sleep(50);
            drive.followTrajectory(traj3_2);
            sleep(200);
            placesqc(Spec.TAGA_PE_SPATE);
            sleep(50);
            drive.followTrajectory(traj3_3);
            sleep(50);
            drive.followTrajectory(traj3_4);
        }else if((currentRecognitions.get(0).getLeft()+currentRecognitions.get(0).getRight())/2<650){//caz mijloc
            drive.followTrajectory(traj2_0);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            clawOpenRight();
            sleep(200);
            hardware.clawServoHold.setPosition(0.45f);
            clawOpenRight();
            drive.followTrajectory(traj2_1);
            sleep(50);
            drive.turn(Math.toRadians(-90));
            sleep(50);
            drive.followTrajectory(traj2_2);
            sleep(50);
            drive.followTrajectory(traj2_3);
            sleep(200);
            placesqc(Spec.TAGA_PE_SPATE);
            hardware.clawServoHold.setPosition(0.45f);
            sleep(100);
            drive.followTrajectory(traj2_4);
            sleep(50);
            drive.followTrajectory(traj2_5);

        }else{                                        // dreapta
            drive.followTrajectory(traj1_0);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            drive.followTrajectory(traj1_1);
            sleep(200);
            clawOpenRight();
            sleep(100);
            hardware.clawServoHold.setPosition(0.45f);
            clawOpenRight();
            sleep(50);
            drive.followTrajectory(traj1_2);
            sleep(200);
            placesqc(Spec.TAGA_PE_SPATE);
            sleep(100);
            drive.followTrajectory(traj1_3);
            sleep(50);
            drive.followTrajectory(traj1_4);
        }

    }
}

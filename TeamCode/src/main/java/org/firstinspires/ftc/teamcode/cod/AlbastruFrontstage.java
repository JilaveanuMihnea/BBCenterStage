package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Albastru frontstage", group="Autonomy")
public class AlbastruFrontstage extends ScheletAlbastru{
    ElapsedTime elapsedTime;
    boolean edgepark = false ;
    float xpark;
    @Override
    public void runOpMode(){
        init_auto();
        if(edgepark){
            xpark = 4f;
        }else{
            xpark = 49.5f;
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1_0, traj1_1,traj1_2, traj1_3, traj1_4, traj1_5, traj1_6,
                traj2_0, traj2_1, traj2_2, traj2_3, traj2_4, traj2_5, traj2_6,
                traj3_0, traj3_1, traj3_2, traj3_3, traj3_4, traj3_5, traj3_6, traj3_7;

        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPos);
        // dreapta

        traj1_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .build();

        traj1_1 = drive.trajectoryBuilder(traj1_0.end())
                .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(-90)))
                .build();

        traj1_2 = drive.trajectoryBuilder(traj1_1.end())
                .lineToLinearHeading(new Pose2d(52.5, 0.5, Math.toRadians(-90)))
                .build();

        traj1_3 = drive.trajectoryBuilder(traj1_2.end())
                .lineToLinearHeading(new Pose2d(52.5, 80, Math.toRadians(-90)))
                .build();

        traj1_4 = drive.trajectoryBuilder(traj1_3.end())
                .lineToLinearHeading(new Pose2d(33,79,Math.toRadians(-90)))
                .build();

        traj1_5 = drive.trajectoryBuilder(traj1_4.end())
                .lineToLinearHeading(new Pose2d(xpark, 80, Math.toRadians(-90)))
                .build();

        traj1_6 = drive.trajectoryBuilder(traj1_5.end())
                .lineToLinearHeading(new Pose2d(xpark, 92, Math.toRadians(-90)))
                .build();

        // mijloc
        traj2_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .build();

        traj2_1 = drive.trajectoryBuilder(traj2_0.end())
                .lineToLinearHeading(new Pose2d(20, -2, Math.toRadians(0)))
                .build();

        traj2_2 = drive.trajectoryBuilder(new Pose2d(20, -2, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(26, -2, Math.toRadians(-90)))
                .build();

        traj2_3 = drive.trajectoryBuilder(traj2_2.end())
                .lineToLinearHeading(new Pose2d(26, 80, Math.toRadians(-90)))
                .build();

        traj2_4 = drive.trajectoryBuilder(traj2_3.end()) //stanga-dreapta pe backdrop
                .lineToLinearHeading(new Pose2d(28, 80, Math.toRadians(-90)))
                .build();

        traj2_5 = drive.trajectoryBuilder(traj2_4.end())
                .lineToLinearHeading(new Pose2d(xpark, 80.5, Math.toRadians(-90)))
                .build();

        traj2_6 = drive.trajectoryBuilder(traj2_5.end())
                .lineToLinearHeading(new Pose2d(xpark,92, Math.toRadians(-90)))
                .build();

        // stang
        traj3_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .build();

        traj3_1 = drive.trajectoryBuilder(traj3_0.end())
                .lineToLinearHeading(new Pose2d(27 , 0 ,Math.toRadians(90)))
                .build();

        traj3_2 = drive.trajectoryBuilder(traj3_1.end())
                .lineToLinearHeading(new Pose2d(27, -6, Math.toRadians(90)))
                .build();

        traj3_3 = drive.trajectoryBuilder(traj3_2.end())
                .lineToLinearHeading(new Pose2d(53, -6, Math.toRadians(90)))
                .build();

        traj3_4 = drive.trajectoryBuilder(new Pose2d(53, -6, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(54, 80.5, Math.toRadians(-90)))
                .build();

        traj3_5 = drive.trajectoryBuilder(traj3_4.end())
                .lineToLinearHeading(new Pose2d(22, 79, Math.toRadians(-90)))
                .build();

        traj3_6 = drive.trajectoryBuilder(traj3_5.end())
                .lineToLinearHeading(new Pose2d(xpark, 79, Math.toRadians(-90)))
                .build();

        traj3_7 = drive.trajectoryBuilder(traj3_6.end())
                .lineToLinearHeading(new Pose2d(xpark+3, 92, Math.toRadians(-90)))
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
            sleep(50);
            drive.followTrajectory(traj3_1);
            sleep(200);
            clawOpenRight();
            sleep(100);
            hardware.clawServoHold.setPosition(0.45f);
            clawOpenRight();
            drive.followTrajectory(traj3_2);
            drive.followTrajectory(traj3_3);
            sleep(6000);
            drive.turn(Math.toRadians(180));
            sleep(300);
            drive.followTrajectory(traj3_4);
            sleep(50);
            drive.followTrajectory(traj3_5);
            sleep(200);
            placesqc(Spec.TAGA_PE_SPATE);
            sleep(200);
            drive.followTrajectory(traj3_6);
            sleep(50);
            drive.followTrajectory(traj3_7);


        }else if((currentRecognitions.get(0).getLeft()+currentRecognitions.get(0).getRight())/2<650){//caz mijloc
            drive.followTrajectory(traj2_0);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            clawOpenRight();
            sleep(200);
            hardware.clawServoHold.setPosition(0.45f);
            clawOpenRight();
            drive.followTrajectory(traj2_1);
            sleep(100);
            drive.turn(Math.toRadians(-90));
            sleep(100);
            drive.followTrajectory(traj2_2);
            sleep(8500);
            drive.followTrajectory(traj2_3);
            sleep(300);
            drive.followTrajectory(traj2_4);
            sleep(100);
            placesqc(Spec.TAGA_PE_SPATE);
            sleep(200);
            hardware.clawServoHold.setPosition(0.45f);
            drive.followTrajectory(traj2_5);
            sleep(200);
            drive.followTrajectory(traj2_6);


        }else{                                        // dreapta
            drive.followTrajectory(traj1_0);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            sleep(50);
            drive.followTrajectory(traj1_1);
            sleep(200);
            clawOpenRight();
            sleep(100);
            hardware.clawServoHold.setPosition(0.45f);
            clawOpenRight();
            sleep(300);
            drive.followTrajectory(traj1_2);
            sleep(8000);
            drive.followTrajectory(traj1_3);
            sleep(300);
            drive.followTrajectory(traj1_4);
            sleep(300);
            placesqc(Spec.TAGA_PE_SPATE);
            sleep(300);
            hardware.clawServoHold.setPosition(0.45f);
            drive.followTrajectory(traj1_5);
            sleep(200);
            drive.followTrajectory(traj1_6);
        }

    }
}

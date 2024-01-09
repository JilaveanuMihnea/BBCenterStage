package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Rosu Frontsage", group="Autonomy")
public class RosuFrontstage extends ScheletRosu{
    ElapsedTime elapsedTime;
    boolean edgepark = false;
    float xpark;
    @Override
    public void runOpMode(){
        if(edgepark){
            xpark = 4f;
        }else{
            xpark = 48.5f;
        }
        init_auto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory  traj_ajutor, traj0, traj1_0, traj1_1,traj1_2, traj1_3, traj1_4, traj1_5,
                    traj2_0, traj2_1, traj2_2, traj2_3, traj2_4, traj2_5, traj2_6,
                    traj_ajutor_dreapta, traj3_0, traj3_1, traj3_2, traj3_3, traj3_4, traj3_5, traj3_6;

        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPos);
        // universal

        traj0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .build();

        // stanga

        traj1_0 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(32, -0.5, Math.toRadians(90)))
                .build();

        traj1_1 = drive.trajectoryBuilder(traj1_0.end())
                .lineToLinearHeading(new Pose2d(56, -0.5, Math.toRadians(90)))
                .build();

        traj1_2 = drive.trajectoryBuilder(traj1_1.end())
                .lineToLinearHeading(new Pose2d(56, -80, Math.toRadians(90)))
                .build();

        traj1_3 = drive.trajectoryBuilder(traj1_2.end())
                .lineToLinearHeading(new Pose2d(29,-80,Math.toRadians(90)))
                .build();

        traj1_4 = drive.trajectoryBuilder(traj1_3.end())
                .lineToLinearHeading(new Pose2d(xpark, -80, Math.toRadians(90)))
                .build();

        traj1_5 = drive.trajectoryBuilder(traj1_4.end())
                .lineToLinearHeading(new Pose2d(xpark-1.5, -94, Math.toRadians(90)))
                .build();


        // mijloc

        traj2_1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(20, 2, Math.toRadians(0)))
                .build();

        traj2_2 = drive.trajectoryBuilder(new Pose2d(20, 2, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(28, 2, Math.toRadians(90)))
                .build();
        traj2_3 = drive.trajectoryBuilder(traj2_2.end())
                .lineToLinearHeading(new Pose2d(29, -80, Math.toRadians(90)))
                .build();

        traj_ajutor = drive.trajectoryBuilder(traj2_3.end())
                .lineToLinearHeading(new Pose2d(24, -80, Math.toRadians(90)))
                .build();
        traj2_5 = drive.trajectoryBuilder(traj_ajutor.end())
                .lineToLinearHeading(new Pose2d(xpark, -82, Math.toRadians(90)))
                .build();

        traj2_6 = drive.trajectoryBuilder(traj2_5.end())
                .lineToLinearHeading(new Pose2d(xpark-1.5,-92, Math.toRadians(90)))
                .build();

        // dreaapta
        traj3_0 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(30 , 0 ,Math.toRadians(-90)))
                .build();

        traj3_1 = drive.trajectoryBuilder(traj3_0.end())
                .lineToLinearHeading(new Pose2d(30, 4, Math.toRadians(-90)))
                .build();


        traj3_2 = drive.trajectoryBuilder(traj3_1.end())
                .lineToLinearHeading(new Pose2d(53, 4, Math.toRadians(-90)))
                .build();

        traj3_3 = drive.trajectoryBuilder(new Pose2d(53, 4, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(54, -80.5, Math.toRadians(90)))
                .build();

        traj3_4 = drive.trajectoryBuilder(traj3_3.end())
                .lineToLinearHeading(new Pose2d(20.5, -80.5, Math.toRadians(90)))
                .build();

        traj3_5 = drive.trajectoryBuilder(traj3_4.end())
                .lineToLinearHeading(new Pose2d(xpark + 6, -80.5, Math.toRadians(90)))
                .build();

        traj3_6 = drive.trajectoryBuilder(traj3_5.end())
                .lineToLinearHeading(new Pose2d(xpark + 6, -97, Math.toRadians(90)))
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
            hardware.clawServoHold.setPosition(0.45f);
            clawOpenLeft();
            sleep(300);
            drive.followTrajectory(traj1_1);
            sleep(3100);
            drive.followTrajectory(traj1_2);
            sleep(200);
            drive.followTrajectory(traj1_3);
            sleep(300);
            placesqc(Spec.TAGA_PE_SPATE);
            sleep(300);
            hardware.clawServoHold.setPosition(0.45f);
            drive.followTrajectory(traj1_4);
            sleep(200);
            drive.followTrajectory(traj1_5);


        }else if((currentRecognitions.get(0).getLeft()+currentRecognitions.get(0).getRight())/2<650){//caz mijloc
            drive.followTrajectory(traj0);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            clawOpenLeft();
            sleep(300);
            hardware.clawServoHold.setPosition(0.45f);
            clawOpenLeft();
            drive.followTrajectory(traj2_1);
            sleep(50);
            drive.turn(Math.toRadians(90));
            sleep(50);
            drive.followTrajectory(traj2_2);
            sleep(3100);
            drive.followTrajectory(traj2_3);
            sleep(50);
            drive.followTrajectory(traj_ajutor);
            sleep(300);
            placesqc(Spec.TAGA_PE_SPATE);
            sleep(200);
            hardware.clawServoHold.setPosition(0.45f);
            sleep(200);
            drive.followTrajectory(traj2_5);
            sleep(50);
            drive.followTrajectory(traj2_6);


        }else{                                        // dreapta
            drive.followTrajectory(traj0);
            sleep(300);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            drive.followTrajectory(traj3_0);
            sleep(250);
            clawOpenLeft();
            sleep(100);
            hardware.clawServoHold.setPosition(0.45f);
            clawOpenLeft();
            sleep(100);
            drive.followTrajectory(traj3_1);
            sleep(200);
            drive.followTrajectory(traj3_2);
            sleep(200);
            drive.turn(Math.toRadians(-180));
            sleep(3100);
            drive.followTrajectory(traj3_3);
            sleep(200);
            drive.followTrajectory(traj3_4);
            sleep(300);
            placesqc(Spec.TAGA_PE_SPATE);
            sleep(300);
            hardware.clawServoHold.setPosition(0.45f);
            drive.followTrajectory(traj3_5);
            sleep(200);
            drive.followTrajectory(traj3_6);
        }

    }
}

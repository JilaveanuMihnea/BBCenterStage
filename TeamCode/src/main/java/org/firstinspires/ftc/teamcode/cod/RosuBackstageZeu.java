package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Rosu Backstage Zeu", group="Autonomy")
public class RosuBackstageZeu extends ScheletRosu {
    ElapsedTime elapsedTime;

    boolean edgepark = true;
    float xpark = 4f; //52 -> center, 4 -> edge

    @Override
    public void runOpMode() {
        if(edgepark){
            xpark = 4f;
        }else{
            xpark = 52f;
        }
        init_auto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1_0, traj1_1,traj1_2, traj1_3, traj1_4, traj1_5, traj1_6,
                traj2_0, traj2_1, traj2_2, traj2_3, traj2_4, traj2_5, traj2_6,
                traj3_0, traj3_1, traj3_2, traj3_3, traj3_4 ,
                traj_ZEU_STANGA1, traj_ZEU_STANGA2, traj_ZEU_STANGA3, traj_ZEU_STANGA4, traj_ZEU_STANGA5, traj_ZEU_STANGA6,
                traj_ZEU_DREAPTA2, traj_ZEU_DREAPTA3, traj_ZEU_DREAPTA4, traj_ZEU_DREAPTA5, traj_ZEU_DREAPTA6, traj_ZEU_DREAPTA1,
                traj_ZEU_MIJL1, traj_ZEU_MIJL2, traj_ZEU_MIJL3, traj_ZEU_MIJL4, traj_ZEU_MIJL5, traj_ZEU_MIJL6;

        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPos);

        //dreapta
        traj1_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(0)))
                .build();

        traj1_1 = drive.trajectoryBuilder(traj1_0.end())
                .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(90)))
                .build();

        traj1_2 = drive.trajectoryBuilder(traj1_1.end())
                .lineToLinearHeading(new Pose2d(34,-33 ,Math.toRadians(90)))
                .build();

        traj_ZEU_STANGA1 = drive.trajectoryBuilder(traj1_2.end())
                .lineToLinearHeading(new Pose2d(8, -34, Math.toRadians(90)))
                .build(); // aliniere la gate

        traj_ZEU_STANGA2 = drive.trajectoryBuilder(traj_ZEU_STANGA1.end())
                .lineToLinearHeading(new Pose2d(8, 50, Math.toRadians(90)))
                .build(); // trece prin gate

        traj_ZEU_STANGA3 = drive.trajectoryBuilder(traj_ZEU_STANGA2.end())
                .lineToLinearHeading(new Pose2d(28 , 69, Math.toRadians(90)))
                .build(); //stack

        traj_ZEU_STANGA4 = drive.trajectoryBuilder(traj_ZEU_STANGA3.end())
                .lineToLinearHeading(new Pose2d(60, 40, Math.toRadians(90)))
                .build(); // in fata la gate

        traj_ZEU_STANGA5 = drive.trajectoryBuilder(traj_ZEU_STANGA4.end())
                .lineToLinearHeading(new Pose2d(60, -30, Math.toRadians(90)))
                .build(); //prin gate

        traj_ZEU_STANGA6 = drive.trajectoryBuilder(traj_ZEU_STANGA5.end())
                .lineToLinearHeading(new Pose2d(26, -34, Math.toRadians(90)))
                .build(); // tabla

        traj1_3 = drive.trajectoryBuilder(traj1_2.end())
                .lineToLinearHeading(new Pose2d(xpark, -34, Math.toRadians(90)))
                .build();

        traj1_4 = drive.trajectoryBuilder(traj1_3.end())
                .lineToLinearHeading(new Pose2d(xpark, -46, Math.toRadians(90)))
                .build();

        //mijloc
        traj2_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .build();

        traj2_1 = drive.trajectoryBuilder(traj2_0.end())
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                .build();

        traj2_2 = drive.trajectoryBuilder(new Pose2d(20, 0, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(90)))
                .build();

        traj2_3 = drive.trajectoryBuilder(traj2_2.end())
                .lineToLinearHeading(new Pose2d(26, -33, Math.toRadians(90)))
                .build();

        traj2_4 = drive.trajectoryBuilder(traj2_3.end())
                .lineToLinearHeading(new Pose2d(xpark, -34, Math.toRadians(90)))
                .build();

        traj2_5 = drive.trajectoryBuilder(traj2_4.end())
                .lineToLinearHeading(new Pose2d(xpark, -46, Math.toRadians(90)))
                .build();

        traj_ZEU_MIJL1 = drive.trajectoryBuilder(traj2_3.end())
                .lineToLinearHeading(new Pose2d(7, -33, Math.toRadians(90)))
                .build(); // aliniere la gate

        traj_ZEU_MIJL2 = drive.trajectoryBuilder(traj_ZEU_MIJL1.end())
                .lineToLinearHeading(new Pose2d(7, 50, Math.toRadians(90)))
                .build(); // trece prin gate

        traj_ZEU_MIJL3 = drive.trajectoryBuilder(traj_ZEU_MIJL2.end())
                .lineToLinearHeading(new Pose2d(40, 69, Math.toRadians(90)))
                .build(); //stack

        traj_ZEU_MIJL4 = drive.trajectoryBuilder(traj_ZEU_MIJL3.end())
                .lineToLinearHeading(new Pose2d(60, 40, Math.toRadians(90)))
                .build(); // in fata la gate

        traj_ZEU_MIJL5 = drive.trajectoryBuilder(traj_ZEU_MIJL4.end())
                .lineToLinearHeading(new Pose2d(60, -30, Math.toRadians(90)))
                .build(); //prin gate

        traj_ZEU_MIJL6 = drive.trajectoryBuilder(traj_ZEU_MIJL5.end())
                .lineToLinearHeading(new Pose2d(26, -30, Math.toRadians(90)))
                .build(); // tabla
        //stanga
        traj3_0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(20, -13.5, Math.toRadians(0)))
                .build();

        traj3_1 = drive.trajectoryBuilder(traj3_0.end())
                .lineToLinearHeading(new Pose2d(17, -32, Math.toRadians(0)))
                .build();

        traj3_2 = drive.trajectoryBuilder(new Pose2d(17, -34, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(18, -33, Math.toRadians(90)))
                .build();

        traj3_3 = drive.trajectoryBuilder(traj3_2.end())
                .lineToLinearHeading(new Pose2d(xpark, -34, Math.toRadians(90)))
                .build();

        traj3_4 = drive.trajectoryBuilder(traj3_3.end())
                .lineToLinearHeading(new Pose2d(xpark, -46, Math.toRadians(90)))
                .build();

        traj_ZEU_DREAPTA1 = drive.trajectoryBuilder(traj3_2.end())
                .lineToLinearHeading(new Pose2d(5, -33, Math.toRadians(90)))
                .build(); // aliniere la gate

        traj_ZEU_DREAPTA2 = drive.trajectoryBuilder(traj_ZEU_DREAPTA1.end())
                .lineToLinearHeading(new Pose2d(5, 50, Math.toRadians(90)))
                .build(); // trece prin gate

        traj_ZEU_DREAPTA3 = drive.trajectoryBuilder(traj_ZEU_DREAPTA2.end())
                .lineToLinearHeading(new Pose2d(40, 69, Math.toRadians(90)))
                .build(); //stack

        traj_ZEU_DREAPTA4 = drive.trajectoryBuilder(traj_ZEU_DREAPTA3.end())
                .lineToLinearHeading(new Pose2d(60, 40, Math.toRadians(90)))
                .build(); // in fata la gate

        traj_ZEU_DREAPTA5 = drive.trajectoryBuilder(traj_ZEU_DREAPTA4.end())
                .lineToLinearHeading(new Pose2d(60, -30, Math.toRadians(90)))
                .build(); //prin gate

        traj_ZEU_DREAPTA6 = drive.trajectoryBuilder(traj_ZEU_DREAPTA5.end())
                .lineToLinearHeading(new Pose2d(26, -30, Math.toRadians(90)))
                .build(); // tabla

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while(!isStarted() && !isStopRequested()){
            telemetryTfod();
        }

        java.util.List<Recognition> currentRecognitions = tfod.getRecognitions();
        sleep(300);
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);

        if(currentRecognitions.size()==0){//caz gen sttanga
            drive.followTrajectory(traj1_0);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            drive.followTrajectory(traj1_1);
            sleep(200);
            clawOpenLeft();
            sleep(100);
            hardware.clawServoHold.setPosition(0.25f);
            clawOpenLeft();
            sleep(50);
            drive.followTrajectory(traj1_2);
            sleep(200);
            placesqc(Spec.TAGA_PE_SPATE);
            clawOpenBoth();
            sleep(200);
            drive.followTrajectory(traj_ZEU_STANGA1);
            sleep(10);
            drive.followTrajectory(traj_ZEU_STANGA2);
            sleep(10);
            clawOpenBoth();
            drive.followTrajectory(traj_ZEU_STANGA3);
            sleep(10);
            stackGrabRed(70);
            tagaFixativ();
            sleep(80);
            drive.followTrajectory(traj_ZEU_STANGA4);
            sleep(10);
            drive.followTrajectory(traj_ZEU_STANGA5);
            sleep(10);
            drive.followTrajectory(traj_ZEU_STANGA6);
            placesqc(Spec.TAGA_TICK_60DEG);
//            drive.followTrajectory(traj1_3);
//            sleep(50);
//            drive.followTrajectory(traj1_4);
        }else if((currentRecognitions.get(0).getLeft()+currentRecognitions.get(0).getRight())/2<650){//caz mijloc
            drive.followTrajectory(traj2_0);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            clawOpenLeft();
            sleep(200);
            hardware.clawServoHold.setPosition(0.25f);
            clawOpenLeft();
            drive.followTrajectory(traj2_1);
            sleep(50);
            drive.turn(Math.toRadians(90));
            sleep(50);
            drive.followTrajectory(traj2_2);
            sleep(50);
            drive.followTrajectory(traj2_3);
            sleep(100);
            placesqc(Spec.TAGA_PE_SPATE);
            hardware.clawServoHold.setPosition(0.25f);
            sleep(100);
            drive.followTrajectory(traj_ZEU_MIJL1);
            sleep(100);
            drive.followTrajectory(traj_ZEU_MIJL2);
            sleep(100);
            clawOpenLeft();
            drive.followTrajectory(traj_ZEU_MIJL3);
            sleep(100);
            stackGrabRed(100);
            tagaFixativ();
            sleep(300);
            drive.followTrajectory(traj_ZEU_MIJL4);
            sleep(100);
            drive.followTrajectory(traj_ZEU_MIJL5);
            sleep(100);
            drive.followTrajectory(traj_ZEU_MIJL6);
            placesqc(Spec.TAGA_TICK_60DEG);
            drive.followTrajectory(traj2_4);
            sleep(50);
            drive.followTrajectory(traj2_5);

        }else{                                        // dreapta
            drive.followTrajectory(traj3_0);
            hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            clawOpenLeft();
            sleep(200);
            hardware.clawServoHold.setPosition(0.25f);
            clawOpenLeft();
            drive.followTrajectory(traj3_1);
            sleep(50);
            drive.turn(Math.toRadians(90));
            sleep(50);
            drive.followTrajectory(traj3_2);
            sleep(200);
            placesqc(Spec.TAGA_PE_SPATE);
            sleep(100);
            drive.followTrajectory(traj_ZEU_DREAPTA1);
            sleep(100);
            drive.followTrajectory(traj_ZEU_DREAPTA2);
            sleep(100);
            clawOpenLeft();
            drive.followTrajectory(traj_ZEU_DREAPTA3);
            sleep(100);
            stackGrabRed(105);
            tagaFixativ();
            sleep(500);
            drive.followTrajectory(traj_ZEU_DREAPTA4);
            sleep(100);
            drive.followTrajectory(traj_ZEU_DREAPTA5);
            sleep(100);
            drive.followTrajectory(traj_ZEU_DREAPTA6);
            placesqc(Spec.TAGA_TICK_60DEG);
//            sleep(50);
//            drive.followTrajectory(traj3_3);
//            sleep(50);
//            drive.followTrajectory(traj3_4);
        }

    }
}

package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: autotest", group="Autonomy")
public class autotest extends ScheletRosu {

    private Trajectory traj_test, traj_1, traj_end, traj_gate_front_1, traj_stack_1, traj_stack_1_1, traj_gate_back_1, traj_gate_front_2, traj_stack_2,  traj_gate_back_2, traj_gate_front_1_1, traj_gate_back_1_1, traj_park_1, traj_park;
    private Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
    private SampleMecanumDrive drive = null;
    ElapsedTime elapsedTime;

    private double[][] mvgl =   {{30, -8, 90, 30, -25, 90}, {19, 0, 0, 27, -28, 90}, {16, -12, 0, 20, -27, 90}};
    private double[] dus = {5, 25, 60};
    private double[] intors = {5, 25, 60};
    private double[] stack = {28 , 40, 52};
    private double dusX, intorsX, stackX;

    private double xpark = 5;


    private void options(int d, int i, int s){
        dusX = dus[d];
        intorsX = intors[i];
        stackX = stack[s];
    }


    private void trajectorySetter(int c){
        //mov + galben
        traj_test = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(mvgl[c][0], mvgl[c][1], Math.toRadians(mvgl[c][2])))
                .addTemporalMarker(0.1, () -> {
                            sliderAuto(1000);
//                            tagaAuto(0, 0.5f);
                        }
                )
                .build();
        traj_1 = drive.trajectoryBuilder(traj_test.end())
                .lineToLinearHeading(new Pose2d(mvgl[c][3], mvgl[c][4], Math.toRadians(mvgl[c][5])))
                .addTemporalMarker(0.2, () -> {
                    tagaAuto(Spec.TAGA_PE_SPATE, 1f);
                    sliderAuto(1100);
                })
                .addTemporalMarker(0.5, () ->{
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE+((Spec.TAGA_PE_SPATE-840)/8.33-30)/355);
                })
                .build();

        //dus
        traj_gate_front_1 = drive.trajectoryBuilder(traj_1.end())
                .lineToLinearHeading(new Pose2d(dusX, -30, Math.toRadians(90)))
                .addTemporalMarker(0.2, () -> {
                    tagaAuto(70, 1);
                    sliderAuto(600);
                    hardware.clawServoHold.setPosition(Spec.HOLD_SAFE);
                })
                .build();
        traj_gate_front_1_1 = drive.trajectoryBuilder(traj_gate_front_1.end())
                .lineToLinearHeading(new Pose2d(dusX, 64, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    sliderAuto(0);
                })
                .addTemporalMarker(1, () -> {
                    tagaAuto(60, 0.5f);
                    sliderAuto(760);
                    hardware.clawServoRight.setPosition(0.8f);
                })
                .build();

        //stack
//        traj_stack_1= drive.trajectoryBuilder(traj_gate_front_1_1.end())
//                .lineToLinearHeading(new Pose2d(dusX, 65, Math.toRadians(90)))
//                .addTemporalMarker(0.1, () -> {
//                    tagaAuto(70, 0.5f);
//                    sliderAuto(760);
//                    hardware.clawServoRight.setPosition(0.6f);
////                    hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN-0.01f);
//                })
//                .build();

        traj_stack_1_1= drive.trajectoryBuilder(traj_gate_front_1_1.end())
                .lineToLinearHeading(new Pose2d(stackX-2,64,Math.toRadians(90)))
//                .addTemporalMarker(0.1, () -> {
//                    tagaAuto(70, 0.5f);
//                })
//                .addTemporalMarker(pathTime -> pathTime * 0.45, () -> {
//                    sliderAuto(760);
//                })
                .addTemporalMarker(pathTime -> pathTime * 0.55, () -> {
                    clawOpenBoth();
                })
                .build();

        //intors
        traj_gate_back_1 = drive.trajectoryBuilder(traj_stack_1_1.end())
                .lineToLinearHeading(new Pose2d(intorsX, 50, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE + 0.05f);
                    tagaAuto(110, 0.5f);
                    sliderAuto(0);
                })
                .build();
        traj_gate_back_1_1 = drive.trajectoryBuilder(traj_gate_back_1.end())
                .lineToLinearHeading(new Pose2d(intorsX, -30, Math.toRadians(90)))
                .addTemporalMarker(0.5, () -> {
                    hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                    sliderAuto(0);
                })
                .build();

        //place
        traj_end = drive.trajectoryBuilder(traj_gate_back_1_1.end())
                .lineToLinearHeading(new Pose2d(27, -36.5, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(Spec.TAGA_TICK_60DEG+230, 0.7f);
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE+((1250-840)/8.33-30)/355);
                })
                .build();

        traj_park = drive.trajectoryBuilder(traj_end.end())
                .lineToLinearHeading(new Pose2d(xpark, -30, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    sliderAuto(600);
                })
                .addTemporalMarker(0.2, ()->{
                    tagaAuto(0, 0.7f);
                })
                .build();

        traj_park_1 = drive.trajectoryBuilder(traj_park.end())
                .lineToLinearHeading(new Pose2d(xpark, -45, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    sliderAuto(0);
                })
                .build();
    }


    @Override
    public void runOpMode() {
        init_auto();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPos);

        options(0, 0, 0);

        while(!isStarted() && !isStopRequested()){
            telemetryTfod();
        }


        java.util.List<Recognition> currentRecognitions = tfod.getRecognitions();
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);


        if(currentRecognitions.size()==0) {
            /*stanga*/
            trajectorySetter(0);
        } else if((currentRecognitions.get(0).getLeft()+currentRecognitions.get(0).getRight())/2<650) {
            /*mijloc*/
            trajectorySetter(1);
        } else {
            /*dreapta*/
            trajectorySetter(2);
        }

        //control
        drive.followTrajectory(traj_test);
        sleep(200);
        clawOpenLeft();
        sleep(150);
        hardware.clawServoHold.setPosition(Spec.HOLD_SAFE);
        sleep(100);
        drive.followTrajectory(traj_1);
        sleep(600);
        clawOpenRight();
        sleep(500);
        drive.followTrajectory(traj_gate_front_1);
        sleep(200);
        drive.followTrajectory(traj_gate_front_1_1);
       // drive.followTrajectory(traj_stack_1);
        sleep(100);
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN + 0.02f);
        sleep(100);
        drive.followTrajectory(traj_stack_1_1);
//        sleep(50);
//        sliderAuto(760);
//        sleep(150);
//        clawOpenBoth();
        sleep(100);
        drive.followTrajectory(traj_gate_back_1);
        sleep(200);
        drive.followTrajectory(traj_gate_back_1_1);
        sleep(200);
        drive.followTrajectory(traj_end);
        sleep(350);
        clawOpenBoth();
        sleep(100);
        drive.followTrajectory(traj_park);
        sleep(100);
        hardware.clawServoHold.setPosition(Spec.HOLD_SAFE);
        sleep(100);
        drive.followTrajectory(traj_park_1);
        sliderAuto(0);
        tagaAuto(0, 1f);


    }

}

package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: 2+2 ALBASTRU", group="Autonomy")
public class Albastru2plus2 extends ScheletAlbastru {

    private Trajectory traj_test, traj_1, traj_end, traj_gate_front_1, traj_stack_1, traj_stack_1_1, traj_gate_back_1, traj_gate_front_2, traj_stack_2,  traj_gate_back_2, traj_gate_front_1_1, traj_gate_back_1_1, traj_park, traj_park_1;
    private Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
    private SampleMecanumDrive drive = null;
    ElapsedTime elapsedTime;

    private double[][] mvgl =   { {16, 14, 0, 20, 25, -90}, {18, 2, 0, 27, 25, -90}, {33, 8, -90, 32, 25, -90}};
    private double[] dus = {3, 25, 60};
    private double[] intors = {3, 25, 60};
    private double[] stack = {27.3 , 40, 52};
    private double dusX, intorsX, stackX;

    private double xpark = 2;


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
                            tagaAuto(30, 0.5f);
                        }
                )
                .build();
        traj_1 = drive.trajectoryBuilder(traj_test.end())
                .lineToLinearHeading(new Pose2d(mvgl[c][3], mvgl[c][4], Math.toRadians(mvgl[c][5])))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(Spec.TAGA_PE_SPATE, 1f);
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE+0.1f);
                })
                .addTemporalMarker(0.5, () ->{
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);
                })
                .build();

        //dus
        traj_gate_front_1 = drive.trajectoryBuilder(traj_1.end())
                .lineToLinearHeading(new Pose2d(dusX, 32, Math.toRadians(-90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(70, 1);
                    sliderAuto(600);
                    hardware.clawServoHold.setPosition(Spec.HOLD_SAFE);
                })
                .build();
        traj_gate_front_1_1 = drive.trajectoryBuilder(traj_gate_front_1.end())
                .lineToLinearHeading(new Pose2d(dusX, -62.5, Math.toRadians(-90)))
                .addTemporalMarker(0.1, () -> {
                    sliderAuto(0);
                })
                .addTemporalMarker(1, () -> {
                    tagaAuto(60, 0.5f);
                    sliderAuto(760);
                })
                .addTemporalMarker(pathTime -> pathTime * 80, () -> {
                    hardware.clawServoLeft.setPosition(0.2f);
                })
                .build();

        //stack
//        traj_stack_1= drive.trajectoryBuilder(traj_gate_front_1_1.end())
//                .lineToLinearHeading(new Pose2d(dusX, 65, Math.toRadians(90)))
//                .addTemporalMarker(0.1, () -> {
//                    tagaAuto(67, 0.5f);
//                    sliderAuto(760);
//                    hardware.clawServoRight.setPosition(0.6f);
////                    hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN-0.01f);
//                })
//                .build();

        traj_stack_1_1= drive.trajectoryBuilder(traj_gate_front_1_1.end())
                .lineToLinearHeading(new Pose2d(stackX+0.5,-62.7,Math.toRadians(-90)))
//                .addTemporalMarker(0.1, () -> {
//                    tagaAuto(75, 0.5f);
//                })
//                .addTemporalMarker(pathTime -> pathTime * 0.45, () -> {
//                    sliderAuto(760);
//                })
                .addTemporalMarker(pathTime -> pathTime * 0.57, () -> {
                    clawOpenBoth();
                })
                .build();

        //intors
        traj_gate_back_1 = drive.trajectoryBuilder(traj_stack_1_1.end())
                .lineToLinearHeading(new Pose2d(intorsX, -48, Math.toRadians(-90)))
                .addTemporalMarker(0.1, () -> {
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE + 0.02f);
                    tagaAuto(110, 0.5f);
                    sliderAuto(0);
                })
                .build();
        traj_gate_back_1_1 = drive.trajectoryBuilder(traj_gate_back_1.end())
                .lineToLinearHeading(new Pose2d(intorsX, 32, Math.toRadians(-90)))
                .addTemporalMarker(0.5, () -> {
                    hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                    sliderAuto(0);
                })
                .build();

        //place
        traj_end = drive.trajectoryBuilder(traj_gate_back_1_1.end())
                .lineToLinearHeading(new Pose2d(27, 37, Math.toRadians(-90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(Spec.TAGA_TICK_60DEG+150, 0.7f);
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);
                })
                .build();

        traj_park = drive.trajectoryBuilder(traj_end.end())
                .lineToLinearHeading(new Pose2d(xpark, 32, Math.toRadians(-90)))
                .addTemporalMarker(0.1, () -> {
                    sliderAuto(600);
                })
                .addTemporalMarker(0.2, ()->{
                    tagaAuto(0, 0.7f);
                })
                .build();

        traj_park_1 = drive.trajectoryBuilder(traj_park.end())
                .lineToLinearHeading(new Pose2d(xpark, 47, Math.toRadians(-90)))
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
        clawOpenRight();
        sleep(150);
        hardware.clawServoHold.setPosition(Spec.HOLD_SAFE);
        sleep(100);
        drive.followTrajectory(traj_1);
        sleep(500);
        clawOpenLeft();
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
        sleep(200);
        clawOpenBoth();
        sleep(100);
        drive.followTrajectory(traj_park);
        sleep(100);
        hardware.clawServoHold.setPosition(Spec.HOLD_SAFE);
        sleep(100);
        drive.followTrajectory(traj_park_1);

    }

}

package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: nu stie sa mearga in diagonala " , group="Autonomy")
public class DoiplusPatrupoate extends ScheletRosu {

    private Trajectory traj_test, traj_1, traj_end, traj_gate_front_1, traj_stack_1, traj_stack_1_1, traj_gate_back_1, traj_gate_front_2, traj_stack_2,  traj_gate_back_2, traj_gate_front_1_1, traj_gate_back_1_1, traj_park_1, traj_park;
    private Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
    private SampleMecanumDrive drive = null;
    ElapsedTime elapsedTime;

    private double[][] mvgl =   {{29, -8, 90, 32, -26, 90}, {18, 0, 0, 27, -25, 90}, {16, -12, 0, 20, -26, 90}};
    private double[] dus = {5, 25, 60};
    private double[] intors = {5, 25, 60};
    private double[] stack = {27 , 40, 52};
    private double dusX, intorsX, stackX;

    private double xpark = 4;


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
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);
                })
                .build();

        //dus
        traj_gate_front_1 = drive.trajectoryBuilder(traj_1.end())
                .lineToLinearHeading(new Pose2d(dusX, -30, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(60, 1);
                    sliderAuto(500);
                    hardware.clawServoHold.setPosition(0f);
                })
                .build();
        traj_gate_front_1_1 = drive.trajectoryBuilder(traj_gate_front_1.end())
                .lineToLinearHeading(new Pose2d(dusX, 64.5, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    sliderAuto(0);
                })
                .addTemporalMarker(1, () -> {
                    tagaAuto(80, 0.5f);
                    sliderAuto(760);
                    hardware.clawServoRight.setPosition(0.6f);
                })
                .build();

        //stack


        traj_stack_1_1= drive.trajectoryBuilder(traj_gate_front_1_1.end())
                .lineToLinearHeading(new Pose2d(stackX-2,64,Math.toRadians(90)))
//                .addTemporalMarker(0.1, () -> {
//                    tagaAuto(75, 0.5f);
//                })
//                .addTemporalMarker(pathTime -> pathTime * 0.45, () -> {
//                    sliderAuto(760);
//                })
                .addTemporalMarker(pathTime -> pathTime * 0.55, () -> {
                    clawOpenBoth();
                })
                .build();

        //intors
//        Trajectory traj_ajutor_stack;
//        traj_ajutor_stack= drive.trajectoryBuilder(traj_stack_1_1.end())
//                .lineToLinearHeading(new Pose2d(dusX, 65, Math.toRadians(90)))
//                .build();
        traj_gate_back_1 = drive.trajectoryBuilder(traj_stack_1_1.end())
                .lineToLinearHeading(new Pose2d(intorsX, 64, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                    tagaAuto(90, 0.5f);
                    sliderAuto(0);
                })
                .build();
        traj_gate_back_1_1 = drive.trajectoryBuilder(traj_gate_back_1.end())
                .lineToLinearHeading(new Pose2d(intorsX, -28, Math.toRadians(90)))
                .build();

        //place
        traj_end = drive.trajectoryBuilder(traj_gate_back_1_1.end())
                .lineToLinearHeading(new Pose2d(27, -34, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(Spec.TAGA_TICK_60DEG+150, 0.7f);
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);
                })
                .build();

        traj_park = drive.trajectoryBuilder(traj_end.end())
                .lineToLinearHeading(new Pose2d(xpark, -28, Math.toRadians(90)))
                .build();

        traj_park_1 = drive.trajectoryBuilder(traj_park.end())
                .lineToLinearHeading(new Pose2d(xpark, -41, Math.toRadians(90)))
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

        int caz;

        if(currentRecognitions.size()==0) {
            /*stanga*/
            trajectorySetter(0);
            caz=0;
        } else if((currentRecognitions.get(0).getLeft()+currentRecognitions.get(0).getRight())/2<650) {
            /*mijloc*/
            trajectorySetter(1);
            caz=1;
        } else {
            /*dreapta*/
            trajectorySetter(2);
            caz=2;
        }

        //control
        if (caz==1 || caz==2) drive.followTrajectory(traj_test);
        else
        {
            Trajectory traj_ajutor;
            traj_ajutor = drive.trajectoryBuilder(startPos)
                    .lineToLinearHeading(new Pose2d( 2, mvgl[0][1], Math.toRadians(0)))
                    .build();
            traj_test = drive.trajectoryBuilder(traj_ajutor.end())
                    .lineToLinearHeading(new Pose2d(mvgl[0][0], mvgl[0][1], Math.toRadians(0)))
                    .build();
            drive.followTrajectory(traj_ajutor);
            sleep(300);
            drive.followTrajectory(traj_test);
            sleep(300); //
            drive.turn(Math.toRadians(90));
            sleep(500);
            sliderAuto(1000);
            sleep(300);
        }
        sleep(200);

        clawOpenLeft();
        sleep(200);
        drive.followTrajectory(traj_1);
        sleep(500);
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
        sleep(300);
        drive.followTrajectory(traj_gate_back_1);
        sleep(200);
        drive.followTrajectory(traj_gate_back_1_1);
        sleep(200);
        drive.followTrajectory(traj_end);
        sleep(200);
        clawOpenBoth();
        sleep(500);
        tagaAuto(0,0.5f);
        hardware.clawServoHold.setPosition(0f);
        sleep(200);
        drive.followTrajectory(traj_park);
        sleep(100);
        drive.followTrajectory(traj_park_1);


    }

}

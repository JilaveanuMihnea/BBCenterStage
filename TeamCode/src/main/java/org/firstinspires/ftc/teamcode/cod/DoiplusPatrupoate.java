package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: 2+4???????", group="Autonomy")
public class DoiplusPatrupoate extends ScheletRosu {

    private Trajectory traj_test, traj_1, traj_end, traj_gate_front_1, traj_stack_1, traj_gate_back_1, traj_gate_front_2, traj_stack_2,  traj_gate_back_2, traj_gate_front_1_1, traj_gate_back_1_1;
    private Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
    private SampleMecanumDrive drive = null;
    ElapsedTime elapsedTime;

    private double[][] mvgl =   {{28, -8, 90}, {18, 0, 0}, {20, -10, 0}};
    private double[] dus = {5, 25, 60};
    private double[] intors = {5, 25, 60};
    private double[] stack = {27 , 40, 52};
    private double dusX, intorsX, stackX;


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
                        }
                )
                .build();
        traj_1 = drive.trajectoryBuilder(traj_test.end())
                .lineToLinearHeading(new Pose2d(27, -25, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(Spec.TAGA_PE_SPATE, 0.5f);
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);
                })
                .build();

        //dus
        traj_gate_front_1 = drive.trajectoryBuilder(traj_1.end())
                .lineToLinearHeading(new Pose2d(dusX, -30, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(78, 0.5f);
                    sliderAuto(0);
                })
                .build();
        traj_gate_front_1_1 = drive.trajectoryBuilder(traj_gate_front_1.end())
                .lineToLinearHeading(new Pose2d(dusX, 50, Math.toRadians(90)))
                .build();

        //stack
        traj_stack_1= drive.trajectoryBuilder(traj_gate_front_1_1.end())
                .lineToLinearHeading(new Pose2d(stackX, 62, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN + 0.02f);
                    tagaAuto(78, 0.5f);
                    sliderAuto(1000);
                })
                .build();

        //intors
        traj_gate_back_1 = drive.trajectoryBuilder(traj_stack_1.end())
                .lineToLinearHeading(new Pose2d(intorsX, 50, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                    tagaAuto(100, 0.5f);
                    sliderAuto(0);
                })
                .build();
        traj_gate_back_1_1 = drive.trajectoryBuilder(traj_gate_back_1.end())
                .lineToLinearHeading(new Pose2d(intorsX, -30, Math.toRadians(90)))
                .build();

        //place
        traj_end = drive.trajectoryBuilder(traj_gate_back_1_1.end())
                .lineToLinearHeading(new Pose2d(30, -30, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(Spec.TAGA_PE_SPATE, 0.7f);
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);
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
        sleep(300);
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
        sleep(500);
        clawOpenLeft();
        sleep(500);
        drive.followTrajectory(traj_1);
        sleep(1000);
        clawOpenRight();
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
        sleep(500);
        drive.followTrajectory(traj_gate_front_1);
        sleep(500);
        drive.followTrajectory(traj_gate_front_1_1);
        sleep(500);
        drive.followTrajectory(traj_stack_1);
        sleep(500);
        clawOpenBoth();
        sleep(300);
        drive.followTrajectory(traj_gate_back_1);
        sleep(500);
        drive.followTrajectory(traj_gate_back_1_1);
        sleep(500);
        drive.followTrajectory(traj_end);
        sleep(2000);
        drive.followTrajectory(traj_gate_front_2);
        sleep(200);
        drive.followTrajectory(traj_stack_2);
        sleep(200);
        drive.followTrajectory(traj_gate_back_2);
        sleep(200);
        drive.followTrajectory(traj_1);
        sleep(10000);

    }

}

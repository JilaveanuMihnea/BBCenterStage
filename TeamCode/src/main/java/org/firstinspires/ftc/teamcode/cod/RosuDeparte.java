package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Rosu departe?", group="Autonomy")

public class RosuDeparte extends ScheletRosu {

    Trajectory traj_test, traj_1, traj_ZEU_1, traj_ZEU_2, traj_gate_fata$spate, traj_infatalagayt, traj_gate_1, traj_stack_1, traj_gate_back_1, traj_end;
    private Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
    private SampleMecanumDrive drive = null;
    private double[][] mvgl = {{32, 3, 90, 29, -80, 90}, {28, 0, 0, 27, -70, 90}, {32, 5, 90, 25, -80, 90}};

    private double[][] zeu = {{56, 3, 90, 56, 22, 90},{25, 22, 90, 56, 18, 90},{ 56, 3, 90, 56, 22, 90}};
    private double[] dus = {5, 25, 60};
    private double[] intors = {5, 25, 60};
    private double[] stack = {27, 40, 52};
    private double dusX, intorsX, stackX;


    private void options(int d, int i, int s) {
        dusX = dus[d];
        intorsX = intors[i];
        stackX = stack[s];
    }


    private void trajectorySetter(int c) {
        //mov + galben
        if (c==2) {
            traj_test = drive.trajectoryBuilder(startPos)
                    .lineToLinearHeading(new Pose2d(mvgl[c][0], mvgl[c][1], Math.toRadians(mvgl[c][2])))
                    .addTemporalMarker(0.1, () -> {
                    tagaAuto(Spec.TAGA_MAX, 0.5f);
                    hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
            })
                    .build();
        }
            else traj_test = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(mvgl[c][0], mvgl[c][1], Math.toRadians(mvgl[c][2])))
                .build();

            traj_ZEU_1= drive.trajectoryBuilder(traj_test.end())
                    .lineToLinearHeading(new Pose2d(zeu[c][0], zeu[c][1], zeu[c][2]))
                    .build();
            traj_ZEU_2 = drive.trajectoryBuilder(traj_ZEU_1.end())
                    .lineToLinearHeading(new Pose2d(zeu[c][3],zeu[c][4],zeu[c][5]))
                    .build();

            traj_gate_fata$spate = drive.trajectoryBuilder(traj_ZEU_2.end())
                    .lineToLinearHeading(new Pose2d(56, -60, Math.toRadians(-90)))
                    .build();
            traj_1 = drive.trajectoryBuilder(traj_gate_fata$spate.end())
                    .lineToLinearHeading(new Pose2d(mvgl[c][3], mvgl[c][4], Math.toRadians(mvgl[c][5])))
                    .addTemporalMarker(0.1, () -> {
                        tagaAuto(Spec.TAGA_PE_SPATE, 1f);
                        hardware.clawServoHold.setPosition(Spec.HOLD_PLACE + 0.1f);
                    })
                    .build();

            traj_gate_1 = drive.trajectoryBuilder(traj_1.end())
                    .lineToLinearHeading(new Pose2d(56,-60,Math.toRadians(90)))
                    .addTemporalMarker(0.1, () ->{
                        tagaAuto(0, 0.5f);
                    })
                    .build();

            traj_stack_1 = drive.trajectoryBuilder(traj_gate_1.end())
                    .lineToLinearHeading(new Pose2d(56, 10, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () ->{
                        sliderAuto(1000);
                    })
                    .build();

            traj_gate_back_1 = drive.trajectoryBuilder(traj_stack_1.end())
                    .lineToLinearHeading(new Pose2d(56,-60, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () ->{
                      sliderAuto(0);
                    })
                    .build();

            traj_end = drive.trajectoryBuilder(traj_gate_back_1.end())
                    .lineToLinearHeading(new Pose2d(56, -70, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        tagaAuto(Spec.TAGA_PE_SPATE, 0.5f);
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

        drive.followTrajectory(traj_test);
        clawOpenLeft();
        sleep(300);
        tagaAuto(90,0.7f);
        drive.followTrajectory(traj_ZEU_1);
        if (caz==1) clawOpenLeft();
        sleep(300);
        drive.followTrajectory(traj_ZEU_2);
        if (caz!=1) clawOpenLeft();
        sleep(300);
        drive.followTrajectory(traj_gate_fata$spate);
        sleep(300);
        drive.followTrajectory(traj_1);
        clawOpenBoth();
        drive.followTrajectory(traj_gate_1);
        sleep(300);
        drive.followTrajectory(traj_stack_1);
        sleep(300);
        clawOpenBoth();
        sleep(100);
        drive.followTrajectory(traj_gate_back_1);
        sleep(300);
        drive.followTrajectory(traj_end);
        clawOpenBoth();
        tagaAuto(0,0.5f);
        sleep(3000);




    }
}
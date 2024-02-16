package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: ROSU DEPARTE", group="Autonomy")
public class RosuDeparte extends ScheletRosu {

    private Trajectory traj_test, traj_1, traj_2, traj_toboard, traj_place, traj_park, traj_park_1, traj_c1help;
    private Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
    private SampleMecanumDrive drive = null;
    ElapsedTime elapsedTime;

    private double[][] mvgl =   { {46, 14.51, 180, 33, -23, 90}, {48, 2, 0, 25, -23, 90}, {31.5, 13.5, 90, 18, -23, 90}};
    private double[] dus = {3, 25, 60};
    private double[] intors = {3, 25, 60};
    private double[] stack = {27.3 ,40, 52};
    private double dusX, intorsX, stackX;

    private double xpark = 52;


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
                            //sliderAuto(1000);
                            tagaAuto(30, 0.5f);
                            if(c==2){
                                tagaAuto(Spec.TAGA_MAX, 0.5f);
                            }
                        }
                )
                .build();

        traj_1 = drive.trajectoryBuilder(new Pose2d(mvgl[c][0], mvgl[c][1], Math.toRadians(mvgl[c][2] + (c==1?180:0))))
                .lineToLinearHeading(new Pose2d(54.3, mvgl[c][1], Math.toRadians(mvgl[c][2]+ (c==1?180:0))))
                .addTemporalMarker(0.1, () -> {
                    sliderAuto(800);
                })
                .addTemporalMarker(0.2, () -> {
                    tagaAuto(100, 0.5f);
//                    hardware.clawServoRight.setPosition(0.6f);
                })
                .build();

        traj_toboard = drive.trajectoryBuilder(new Pose2d(54.3, mvgl[c][1], Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(53.5, -79, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    sliderAuto(0);
                })
                .build();

        traj_place = drive.trajectoryBuilder(traj_toboard.end())
                .lineToLinearHeading(new Pose2d(mvgl[c][3], -79, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(Spec.TAGA_PE_SPATE, 1f);
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE+((1400-840)/8.33-30)/355);
                })
                .addTemporalMarker(0.5, () ->{
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE+((1400-840)/8.33-30)/355);
                })
                .build();

        traj_park = drive.trajectoryBuilder(traj_place.end())
                .lineToLinearHeading(new Pose2d(xpark, -74, Math.toRadians(90)))
                .addTemporalMarker(0.1, () ->{
                    tagaAuto(Spec.TAGA_TICK_60DEG, 1f);

                })
                .build();

        traj_park_1 = drive.trajectoryBuilder(traj_park.end())
                .lineToLinearHeading(new Pose2d(xpark, -88, Math.toRadians(90)))
                .addTemporalMarker(0.1, () ->{
                    sliderAuto(600);

                })
                .addTemporalMarker(0.2, ()->{
                    tagaAuto(0, 0.5f);
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

        int c;
        if(currentRecognitions.size()==0) {
            /*stanga*/c = 0;
            trajectorySetter(0);
        } else if((currentRecognitions.get(0).getLeft()+currentRecognitions.get(0).getRight())/2<650) {
            /*mijloc*/c = 1;
            trajectorySetter(1);
        } else {
            /*dreapta*/c = 2;
            trajectorySetter(2);
        }

        //control
        drive.followTrajectory(traj_test);
        if(c==1){
            sleep(100);
            drive.turn(Math.toRadians(180));
        }
        sleep(800);
        clawOpenLeft();
        sleep(150);
        hardware.clawServoHold.setPosition(Spec.HOLD_SAFE);
        if(c == 2){
            tagaAuto(Spec.TAGA_TICK_60DEG, 1f);
        }
        sleep(400);
        drive.followTrajectory(traj_1);


        sleep(100);
        if(c!=2)
            drive.turn(Math.toRadians(-90));
        sleep(100);
        clawOpenLeft();
        drive.followTrajectory(traj_toboard);
        sleep(200);
        drive.followTrajectory(traj_place);
        sleep(100);
        clawOpenBoth();
        sleep(200);
        drive.followTrajectory(traj_park);
        sleep(100);
        hardware.clawServoHold.setPosition(Spec.HOLD_SAFE);
        sleep(100);
        drive.followTrajectory(traj_park_1);
        sleep(600);
        sliderAuto(0);
        tagaAuto(0, 1f);
        sleep(30000);


    }

}

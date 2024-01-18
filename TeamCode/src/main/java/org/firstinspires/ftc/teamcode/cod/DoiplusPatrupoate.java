package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: 2+4???????", group="Autonomy")
public class DoiplusPatrupoate extends ScheletRosu {
    ElapsedTime elapsedTime;

    private void trajectorySetter(){

    }

    @Override
    public void runOpMode() {
        init_auto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj_test, traj_1, traj_end, traj_gate_front_1, traj_stack_1, traj_gate_back_1, traj_gate_front_2, traj_stack_2,  traj_gate_back_2, traj_gate_front_1_1, traj_gate_back_1_1;
        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
        char dus='L',intors='L',stack='L',dus2='L',stack2='L',intors2='L';

        drive.setPoseEstimate(startPos);

        while(!isStarted() && !isStopRequested()){
            telemetryTfod();
        }

        java.util.List<Recognition> currentRecognitions = tfod.getRecognitions();
        sleep(300);
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);

        // mov + galben
        /*stanga*/ if(currentRecognitions.size()==0) {
            traj_test = drive.trajectoryBuilder(startPos)
                    .lineToLinearHeading(new Pose2d(27, -11, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                                sliderAuto(1000);
                            }
                    )
                    .build();
            traj_1 = drive.trajectoryBuilder(traj_test.end())
                    .lineToLinearHeading(new Pose2d(27, -25, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        tagaAuto(Spec.TAGA_PE_SPATE, 0.7f);
                        hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);
                    })
                    .build();
        } /*mijloc*/ else if((currentRecognitions.get(0).getLeft()+currentRecognitions.get(0).getRight())/2<650) {
            traj_test = drive.trajectoryBuilder(startPos)
                    .lineToLinearHeading(new Pose2d(18, 0, Math.toRadians(0)))
                    .addTemporalMarker(0.1, () -> {
                                sliderAuto(1000);
                            }
                    )
                    .build();
            traj_1 = drive.trajectoryBuilder(traj_test.end())
                    .lineToLinearHeading(new Pose2d(27, -25, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        tagaAuto(Spec.TAGA_PE_SPATE, 0.7f);
                        hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);
                    })
                    .build();
        } /*dreapta*/ else {
            traj_test = drive.trajectoryBuilder(startPos)
                    .lineToLinearHeading(new Pose2d(20, -10, Math.toRadians(0)))
                    .addTemporalMarker(0.1, () -> {
                                sliderAuto(1000);
                            }
                    )
                    .build();
            traj_1 = drive.trajectoryBuilder(traj_test.end())
                    .lineToLinearHeading(new Pose2d(30, -25, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        tagaAuto(Spec.TAGA_PE_SPATE, 0.7f);
                        hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);
                    })
                    .build();
        }

        //cycle 1
        if (dus=='L') {
            traj_gate_front_1 = drive.trajectoryBuilder(traj_1.end())
                    .lineToLinearHeading(new Pose2d(5, -30, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        tagaAuto(100, 0.5f);
                        sliderAuto(0);
                    })
                    .build();
            traj_gate_front_1_1 = drive.trajectoryBuilder(traj_gate_front_1.end())
                    .lineToLinearHeading(new Pose2d(5, 50, Math.toRadians(90)))
                    .build();
        } else if (dus=='M') {
            traj_gate_front_1 = drive.trajectoryBuilder(traj_1.end())
                    .lineToLinearHeading(new Pose2d(25, -30, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        tagaAuto(100, 0.5f);
                        sliderAuto(0);
                        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                    })
                    .build();
            traj_gate_front_1_1 = drive.trajectoryBuilder(traj_gate_front_1.end())
                    .lineToLinearHeading(new Pose2d(25, 50, Math.toRadians(90)))
                    .build();
        } else {
            traj_gate_front_1 = drive.trajectoryBuilder(traj_1.end())
                    .lineToLinearHeading(new Pose2d(60, -30, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        tagaAuto(100, 0.5f);
                        sliderAuto(0);
                        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                    })
                    .build();
            traj_gate_front_1_1 = drive.trajectoryBuilder((traj_gate_front_1.end()))
                    .lineToLinearHeading(new Pose2d(60, 50, Math.toRadians(90)))
                    .build();
        }


        if (stack=='L') traj_stack_1= drive.trajectoryBuilder(traj_gate_front_1_1.end())
                .lineToLinearHeading(new Pose2d(27 , 62, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                    tagaAuto(100, 0.5f);
                    sliderAuto(1000);
                })
                .build();
        else if (stack=='M') traj_stack_1= drive.trajectoryBuilder(traj_gate_front_1_1.end())
                .lineToLinearHeading(new Pose2d(40 , 60, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                    tagaAuto(100, 0.5f);
                    sliderAuto(1000);
                })
                .build();
        else  traj_stack_1=drive.trajectoryBuilder(traj_gate_front_1_1.end())
                    .lineToLinearHeading(new Pose2d(52 , 60, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                        tagaAuto(200, 0.5f);
                        sliderAuto(1000);
                    })
                    .build();


        if (intors=='L') {
            traj_gate_back_1 = drive.trajectoryBuilder(traj_stack_1.end())
                    .lineToLinearHeading(new Pose2d(5, 50, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                        tagaAuto(100, 0.5f);
                        sliderAuto(0);
                    })
                    .build();
            traj_gate_back_1_1 = drive.trajectoryBuilder(traj_gate_back_1.end())
                    .lineToLinearHeading(new Pose2d(5, -30, Math.toRadians(90)))
                    .build();
        }
        else if (intors=='M') {
            traj_gate_back_1 = drive.trajectoryBuilder(traj_stack_1.end())
                    .lineToLinearHeading(new Pose2d(25, 50, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                        tagaAuto(100, 0.5f);
                        sliderAuto(0);

                    })
                    .build();
            traj_gate_back_1_1 = drive.trajectoryBuilder(traj_gate_back_1.end())
                    .lineToLinearHeading(new Pose2d(25, -30, Math.toRadians(90)))
                    .build();
        }
        else {
            traj_gate_back_1 = drive.trajectoryBuilder(traj_stack_1.end())
                    .lineToLinearHeading(new Pose2d(60, 50, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
                        tagaAuto(100, 0.5f);
                        sliderAuto(0);
                    })
                    .build();
            traj_gate_back_1_1 = drive.trajectoryBuilder(traj_gate_back_1.end())
                    .lineToLinearHeading(new Pose2d(60, -30, Math.toRadians(90)))
                    .build();
        }

        //cycle 2
        if (dus2=='L') traj_gate_front_2=drive.trajectoryBuilder(traj_1.end())
                .lineToLinearHeading(new Pose2d(8, -30, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(100,0.5f);
                    sliderAuto(0);
                })
//                .lineToLinearHeading(new Pose2d(8, 50, Math.toRadians(90)))
                .build();
        else if (dus2=='M') traj_gate_front_2=drive.trajectoryBuilder(traj_1.end())
                .lineToLinearHeading(new Pose2d(25, -30, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(100, 0.5f);
                    sliderAuto(0);
                })
//                .lineToLinearHeading(new Pose2d(25, 50, Math.toRadians(90)))
                .build();
        else traj_gate_front_2= drive.trajectoryBuilder(traj_1.end())
                    .lineToLinearHeading(new Pose2d(60, -30, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        tagaAuto(100, 0.5f);
                        sliderAuto(0);
                    })
                    //.lineToLinearHeading(new Pose2d(60, 50, Math.toRadians(90)))
                    .build();

        if (stack2=='L') traj_stack_2= drive.trajectoryBuilder(traj_gate_front_2.end())
                .lineToLinearHeading(new Pose2d(28 , 60, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(200, 0.5f);
                    sliderAuto(100);
                })
                .build();
        else if (stack2=='M') traj_stack_2= drive.trajectoryBuilder(traj_gate_front_2.end())
                .lineToLinearHeading(new Pose2d(40 , 60, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(200, 0.5f);
                    sliderAuto(100);
                })
                .build();
        else  traj_stack_2=drive.trajectoryBuilder(traj_gate_front_2.end())
                    .lineToLinearHeading(new Pose2d(52 , 60, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        tagaAuto(200, 0.5f);
                        sliderAuto(100);
                    })
                    .build();

        if (intors2=='L') traj_gate_back_2=drive.trajectoryBuilder(traj_stack_2.end())
                .lineToLinearHeading(new Pose2d(8, 50, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(100,0.5f);
                    sliderAuto(0);
                })
                //  .lineToLinearHeading(new Pose2d(8, -30, Math.toRadians(90)))
                .build();
        else if (intors2=='M') traj_gate_back_2=drive.trajectoryBuilder(traj_stack_2.end())
                .lineToLinearHeading(new Pose2d(25, 50, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(100, 0.5f);
                    sliderAuto(0);
                })
                // .lineToLinearHeading(new Pose2d(25, -30, Math.toRadians(90)))
                .build();
        else traj_gate_back_2 = drive.trajectoryBuilder(traj_stack_2.end())
                    .lineToLinearHeading(new Pose2d(60, 50, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () -> {
                        tagaAuto(100, 0.5f);
                        sliderAuto(0);
                    })
                    //  .lineToLinearHeading(new Pose2d(60, -30, Math.toRadians(90)))
                    .build();

        traj_end = drive.trajectoryBuilder(traj_gate_back_1_1.end())
                .lineToLinearHeading(new Pose2d(30, -25, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    tagaAuto(Spec.TAGA_PE_SPATE, 0.7f);
                    hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);
                })
                .build();

        // 2+4 poate
        drive.followTrajectory(traj_test);
        sleep(500);
        clawOpenLeft();
        sleep(500);
        drive.followTrajectory(traj_1);
        sleep(500);
        clawOpenRight();
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
        sleep(500);
        drive.followTrajectory(traj_gate_front_1);
        sleep(500);
        drive.followTrajectory(traj_gate_front_1_1);
        sleep(500);
        drive.followTrajectory(traj_stack_1);
        sleep(500);
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

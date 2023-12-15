package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Albastru Backstage", group="Autonomy")
public class AlbastruBackstage extends LinearOpMode {
    Hardware hardware;

    ElapsedTime elapsedTime;
    protected ElapsedTime runTime;
    @Override
    public void runOpMode() {


        runTime = new ElapsedTime();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj0, traj1, traj2;

        hardware = new Hardware(hardwareMap, true);

        Pose2d startPos = new Pose2d(36, 52, Math.toRadians(0));

        drive.setPoseEstimate(startPos);
        traj0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(36, 32, Math.toRadians(0)))
                .build();

        traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(46, 32, Math.toRadians(0)))
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(61, 40, Math.toRadians(0)))
                .build();

        elapsedTime = new ElapsedTime();

        waitForStart();
        while(isStopRequested()) return;

        drive.followTrajectory(traj0);

        sleep(1000);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        //drive.followTrajectory(traj1);

        while (!isStopRequested() && opModeIsActive()) ;

    }
}

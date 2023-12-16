package org.firstinspires.ftc.teamcode.cod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Albastru Backstage", group="Autonomy")
public class AlbastruBackstage extends Schelet {

    ElapsedTime elapsedTime;
    @Override
    public void runOpMode() {
        init_auto();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj0, traj1, traj2;

        Pose2d startPos = new Pose2d(36, 52, Math.toRadians(0));

        drive.setPoseEstimate(startPos);
        traj0 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(56, 66), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(56, 52, Math.toRadians(0)))
                .build();

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while(!isStarted() && !isStopRequested()){
            telemetryTfod();
        }

        drive.followTrajectory(traj0);

        sleep(1000);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

    }
}

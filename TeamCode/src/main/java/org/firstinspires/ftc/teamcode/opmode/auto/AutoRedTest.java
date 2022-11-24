package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.SleeveDetector;

@Autonomous
public class AutoRedTest extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(36, -64, Math.toRadians(90));
    Robot robot;
    SleeveDetector detector = new SleeveDetector();
    SleeveDetection.Color parkingPos = SleeveDetection.Color.BLUE;

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        waitForStart();
        robot.lift.turretmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        detector.init(hardwareMap, telemetry);

        TrajectorySequence main = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setReversed(true)
                .splineTo(new Vector2d(40,-52), Math.toRadians(30))
                // Preplaced
                .lineToLinearHeading(new Pose2d(36, -52, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(32, -8, Math.toRadians(180)))
                // Cycle #1
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(0))
                .lineTo(new Vector2d(61, -12))
                .setReversed(true)
                .lineTo(new Vector2d(40, -12))
                .splineToConstantHeading(new Vector2d(24, -8), Math.toRadians(90))
                // Cycle #2
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(0))
                .lineTo(new Vector2d(61, -12))
                .setReversed(true)
                .lineTo(new Vector2d(40, -12))
                .splineToConstantHeading(new Vector2d(24, -8), Math.toRadians(90))
                // Cycle #3
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(0))
                .lineTo(new Vector2d(61, -12))
                .setReversed(true)
                .lineTo(new Vector2d(40, -12))
                .splineToConstantHeading(new Vector2d(24, -8), Math.toRadians(90))
                // Cycle #4
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(0))
                .lineTo(new Vector2d(61, -12))
                .setReversed(true)
                .lineTo(new Vector2d(40, -12))
                .splineToConstantHeading(new Vector2d(24, -8), Math.toRadians(90))
                // Park
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(36, -24, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(62, -36), Math.toRadians(0))
                .build();

        robot.drive.setPoseEstimate(START_POSE);

        // Waiting for start
        while (!isStarted() && !isStopRequested()) {
            parkingPos = detector.getColor();
            telemetry.addData("Parking position", parkingPos);
            telemetry.update();
        }

        detector.stop();
        robot.init();

        robot.drive.followTrajectorySequence(main);

        while (!isStopRequested() && robot.drive.isBusy() && opModeIsActive()) {
            telemetry.update();
            robot.update();
        }
    }
}


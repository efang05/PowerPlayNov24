package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.dense.row.linsol.LinearSolver_DDRB_to_DDRM;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class LiftTest extends LinearOpMode {

    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        waitForStart();
        robot.init();
        while (!isStopRequested() && opModeIsActive()) {
            robot.lift.motor1.setPower(gamepad1.left_stick_y);
            robot.lift.motor2.setPower(-gamepad1.left_stick_y);
        }
    }
}

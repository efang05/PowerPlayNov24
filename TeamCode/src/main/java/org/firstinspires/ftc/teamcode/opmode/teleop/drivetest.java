package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@TeleOp
public class drivetest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(telemetry, hardwareMap);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            robot.drive.leftFront.setPower(gamepad1.left_stick_y);
            robot.drive.rightFront.setPower(gamepad1.left_stick_x);
            robot.drive.leftRear.setPower(gamepad1.right_stick_x);
            robot.drive.rightRear.setPower(gamepad1.right_stick_y);
        }
    }
}

package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
@TeleOp
public class MecanumTest extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(telemetry, hardwareMap);
        while(!isStopRequested() && opModeIsActive()){
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robot.drive.leftFront.setPower(y + x + rx);
            robot.drive.leftRear.setPower(y - x + rx);
            robot.drive.rightFront.setPower(y - x - rx);
            robot.drive.rightRear.setPower(y + x - rx);

        }
    }
}
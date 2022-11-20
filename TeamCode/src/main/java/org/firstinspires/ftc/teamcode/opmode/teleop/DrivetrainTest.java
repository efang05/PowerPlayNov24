package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.drive.Drivetrain;
@TeleOp
public class DrivetrainTest extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(telemetry, hardwareMap);
        while(!isStopRequested() && opModeIsActive()){
            if(gamepad1.y){
                robot.drive.leftFront.setPower(0.5);
            }
            else{
                robot.drive.leftFront.setPower(0);
            }

            if(gamepad1.x){
                robot.drive.leftRear.setPower(0.5);
            }
            else{
                robot.drive.leftRear.setPower(0);
            }

            if(gamepad1.a){
                robot.drive.rightFront.setPower(0.5);
            }
            else{
                robot.drive.rightFront.setPower(0);
            }

            if(gamepad1.b){
                robot.drive.rightRear.setPower(0.5);
            }
            else{
                robot.drive.rightRear.setPower(0);
            }

            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y, //controls forward
                            -gamepad1.left_stick_x, //controls strafing
                            -gamepad1.right_stick_x //controls turning
                    )
            );
        }
    }
}
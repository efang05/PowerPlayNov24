package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.apache.commons.math3.genetics.ElitisticListPopulation;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.lift.Lift;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ArmTuning extends LinearOpMode{
    private Robot robot;
    private ElapsedTime timer;
    public enum ArmPos {
        LEFT,
        RIGHT
    }

    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();

        waitForStart();
        robot.lift.turretmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.turretmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.setArmPos(LiftConstants.IdleArm);
        telemetry.addData("loop:", "started");
        ArmPos Pos = ArmPos.LEFT;
        timer.reset();
        while(!isStopRequested()) {
            robot.lift.setTargetHeight(37);
            telemetry.addData("turret goal", robot.lift.getTargetRotation());
            telemetry.addData("turret pos", robot.lift.getCurrentRotation());
            telemetry.addData("turret power", robot.lift.getTpower());
            telemetry.addData("turret mode", robot.lift.getTurretMode());

            robot.update();
            telemetry.update();
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y, //controls forward
                            -gamepad1.left_stick_x, //controls strafing
                            -gamepad1.right_stick_x //controls turning
                    )
            );
            switch (Pos) {
                case LEFT:
                    robot.lift.setTargetRotation(125);
                    if (timer.milliseconds() > 2500) {
                        Pos = ArmPos.RIGHT;
                        timer.reset();
                    }
                    break;
                case RIGHT:
                    robot.lift.setTargetRotation(-125);
                    if (timer.milliseconds() > 2500) {
                        Pos = ArmPos.LEFT;
                        timer.reset();
                    }
                    break;
            }
        }
    }

}


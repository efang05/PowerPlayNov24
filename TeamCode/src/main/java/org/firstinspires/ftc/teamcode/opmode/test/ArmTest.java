package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@TeleOp
@Config
public class ArmTest extends LinearOpMode {
    private Arm v4b;

    int test_case = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        v4b = new Arm(hardwareMap, telemetry);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {

            // Case 1: test set power
            switch (test_case) {
                case 0:
                    v4b.setPower(-gamepad1.left_stick_y);
                    telemetry.addData("Power",v4b.getCurrentPower());
                    telemetry.update();
                    break;

                case 1:
                    // Case 2: test set position
                    v4b.setPosition(-Math.round(gamepad1.right_stick_x*10));
                    telemetry.addData("Position",v4b.getCurrentPosition());
                    telemetry.update();
                    break;

                case 2:
                    // Case 2: test set velocity
                    v4b.setVelocity(90);
                    telemetry.addData("Velocity",v4b.getVelocity());
                    telemetry.update();
                    break;
                default:
                    break;
            }
        }
    }
}
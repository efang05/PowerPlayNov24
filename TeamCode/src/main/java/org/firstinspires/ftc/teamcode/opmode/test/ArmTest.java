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
                case 1:
                    // Case 2: test set position
                    v4b.setArm1Position(-Math.round(gamepad1.right_stick_x*10));
                    telemetry.addData("Position",v4b.getArm1Position());
                    telemetry.update();
                    break;

                default:
                    break;
            }
        }
    }
}
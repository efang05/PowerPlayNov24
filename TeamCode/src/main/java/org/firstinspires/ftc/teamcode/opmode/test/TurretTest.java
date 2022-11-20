//package org.firstinspires.ftc.teamcode.opmode.test;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.apache.commons.math3.geometry.euclidean.twod.Line;
//import org.firstinspires.ftc.teamcode.subsystem.Arm;
//import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
//import org.firstinspires.ftc.teamcode.subsystem.Turret;
//
//@TeleOp
//@Config
//public class TurretTest extends LinearOpMode {
//    private Turret turret;
//    int test_case = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        turret = new Turret(hardwareMap, telemetry);
//
//        waitForStart();
//
//        while(!isStopRequested() && opModeIsActive()) {
//
//            // Case 1: test set power
//            switch (test_case) {
//                case 0:
//                    turret.setPower(-gamepad1.left_stick_y);
//                    telemetry.addData("Power",turret.getCurrentPower());
//                    telemetry.update();
//                    break;
//
//                case 1:
//                    // Case 2: test set position
//                    turret.setPosition(-Math.round(gamepad1.right_stick_x*10));
//                    telemetry.addData("Position",turret.getCurrentPosition());
//                    telemetry.update();
//                    break;
//
//                case 2:
//                    // Case 2: test set velocity
//                    turret.setVelocity(90);
//                    telemetry.addData("Velocity",turret.getVelocity());
//                    telemetry.update();
//                    break;
//                default:
//                    break;
//            }
//        }
//    }
//}
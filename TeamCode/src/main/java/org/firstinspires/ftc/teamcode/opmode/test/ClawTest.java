//package org.firstinspires.ftc.teamcode.opmode.test;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//
//import org.apache.commons.math3.geometry.euclidean.twod.Line;
//import org.firstinspires.ftc.teamcode.subsystem.Arm;
//import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
//import org.firstinspires.ftc.teamcode.subsystem.Claw;
//
//@TeleOp
//@Config
//public class ClawTest extends LinearOpMode {
//    private Claw claw;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        GamepadEx gamepadEx1;
//        gamepadEx1 = new GamepadEx(gamepad1);
//        claw = new Claw(hardwareMap, telemetry);
//
//        waitForStart();
//
//        while(!isStopRequested() && opModeIsActive()) {
//            if(gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
//
//            }
//        }
//    }}
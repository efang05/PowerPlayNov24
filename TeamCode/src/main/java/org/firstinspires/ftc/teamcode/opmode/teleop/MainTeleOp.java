package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.RobotConstants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class MainTeleOp extends LinearOpMode {


    private Robot robot;
    private ScoreMode mode;
    private State state;
    private ElapsedTime timer;

    private GamepadEx gamepadEx2, gamepadEx1;

    private boolean lastAPress=false;
    private boolean lastBPress=false;

    private boolean mineralDetected=false;
    private double multi = 1;

    private double antiTipMulti = 0.5;
    public double OpenDelay = 500;
    public double CloseDelay = 500;
    public double UpLiftMulti = 0.3;
    public double DownLiftMulti =0.3;
    public double TurretMulti = 0.3;
    public double armpowermulti = 0.3;

    public enum State {
        IDLE,
        Idle2,
        IDLEBACK,
        IDLEBACK2,
        INTAKINGFront,
        IntakingBack,
        OuttakingBack,
        OuttakingLeft,
        OuttakingRight,
        OuttakingFront,
    }

    public enum ScoreMode {
        GROUND,
        LOW,
        MID,
        HIGH
    }


    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        state = State.IDLE;
        timer = new ElapsedTime();
        mode = ScoreMode.HIGH;
        gamepadEx2 = new GamepadEx(gamepad2);
        gamepadEx1 = new GamepadEx(gamepad1);

        robot.init();
        waitForStart();
        idleTransition();
        while(!isStopRequested() && opModeIsActive()) {

            //anti-tip + regular teleop code - ONLY ON PITCH RIGHT NOW
            double pitch = robot.drive.getOrientation().secondAngle;


            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepadEx1.getLeftY() * multi + antiTipMulti * pitch,
                            -gamepadEx1.getLeftX() * multi,
                            -gamepadEx1.getRightX() * multi
                    )
            );
            double Trigger1 = gamepadEx2.gamepad.left_trigger;
            double Trigger2 = -gamepadEx2.gamepad.right_trigger;

            robot.lift.setPower(Trigger1*UpLiftMulti);
            robot.lift.setPower(Trigger2*DownLiftMulti);

            double TurretControl = gamepadEx2.getLeftX();
            robot.turret.setPower(TurretControl*TurretMulti);

            double ArmControl = gamepadEx2.getLeftY();
            robot.arm.setArm1Position(robot.arm.getArm1Position() - ArmControl * armpowermulti);
            robot.arm.setArm2Position(robot.arm.getArm2Position() - ArmControl * armpowermulti);

            robot.claw.ClawOpen();

            if(gamepad2.dpad_up) {
                mode = ScoreMode.HIGH;
            }
            if(gamepad2.dpad_right) {
                mode = ScoreMode.LOW;
            }
            if(gamepad2.dpad_left) {
                mode = ScoreMode.MID;
            }
            if(gamepad2.dpad_down){
                mode = ScoreMode.GROUND;
            }

            switch(state) {
                case IDLE:
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.INTAKINGFront;
                        IntakingFrontTransition();
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                        state = State.IDLEBACK;
                        IdleBackTransition();
                    }
                    break;

                case INTAKINGFront:
                    if (robot.lift.getDistance() < 5) {
                        robot.claw.ClawClose();
                        if (timer.milliseconds() > CloseDelay) {
                            state = State.Idle2;
                            Idle2Transition();
                            timer.reset();
                        }
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                        robot.claw.ClawClose();
                        timer.reset();
                        if (timer.milliseconds() > CloseDelay) {
                            state = State.Idle2;
                            Idle2Transition();
                            timer.reset();
                        }
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.START)) {
                        state = State.IDLE;
                        idleTransition();
                    }
                    break;
                case Idle2:
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.OuttakingFront;
                        OuttakingFrontTransition();
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.OuttakingRight;
                        OuttakingRightTransition();
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
                        state = State.OuttakingLeft;
                        OuttakingLeftTransition();
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                        state = State.OuttakingBack;
                        OuttakingBackTransition();
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.START)) {
                        state = State.INTAKINGFront;
                        IntakingFrontTransition();
                    }
                    break;
                case IDLEBACK:
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                        state = State.IntakingBack;
                        IntakingBackTransition();
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.IDLE;
                        idleTransition();
                    }
                    break;
                case IntakingBack:
                    if (robot.lift.getDistance() < 5) {
                        robot.claw.ClawClose();
                        if (timer.milliseconds() > CloseDelay) {
                            state = State.IDLEBACK2;
                            IdleBack2Transition();
                            timer.reset();
                        }
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                        robot.claw.ClawClose();
                        if (timer.milliseconds() > CloseDelay) {
                            state = State.IDLEBACK2;
                            IdleBack2Transition();
                            timer.reset();
                        }
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.START)) {
                        state = State.IDLEBACK;
                        IdleBackTransition();
                    }
                    break;
                case IDLEBACK2:
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.OuttakingFront;
                        OuttakingFrontTransition();
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.OuttakingRight;
                        OuttakingRightTransition();
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
                        state = State.OuttakingLeft;
                        OuttakingLeftTransition();
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                        state = State.OuttakingBack;
                        OuttakingBackTransition();
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.START)) {
                        state = State.IntakingBack;
                        IntakingBackTransition();
                    }
                    break;
                case OuttakingFront:
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                        robot.claw.ClawOpen();
                        if (timer.milliseconds() > OpenDelay) {
                            state = State.IDLE;
                            idleTransition();
                            timer.reset();
                        }
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.BACK)) {
                        robot.claw.ClawOpen();
                        if (timer.milliseconds() > OpenDelay) {
                            state = State.IDLEBACK;
                            IdleBackTransition();
                            timer.reset();
                        }
                    }
                    setTurretPosition();
                    setLiftPosition();
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.START)) {
                        state = State.Idle2;
                        idleTransition();
                    }
                    break;
                case OuttakingBack:
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                        robot.claw.ClawOpen();
                        if (timer.milliseconds() > OpenDelay) {
                            state = State.IDLE;
                            idleTransition();
                            timer.reset();
                        }
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.BACK)) {
                        robot.claw.ClawOpen();
                        if (timer.milliseconds() > OpenDelay) {
                            state = State.IDLEBACK;
                            IdleBackTransition();
                            timer.reset();
                        }
                    }
                    setTurretPosition();
                    setLiftPosition();
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.START)) {
                        state = State.Idle2;
                        idleTransition();
                    }
                    break;
                case OuttakingLeft:
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
                        robot.claw.ClawOpen();
                        if (timer.milliseconds() > OpenDelay) {
                            state = State.IDLE;
                            idleTransition();
                            timer.reset();
                        }
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.BACK)) {
                        robot.claw.ClawOpen();
                        if (timer.milliseconds() > OpenDelay) {
                            state = State.IDLEBACK;
                            IdleBackTransition();
                            timer.reset();
                        }
                    }
                    setTurretPosition();
                    setLiftPosition();
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.START)) {
                        state = State.Idle2;
                        idleTransition();
                    }
                    break;
                case OuttakingRight:
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
                        robot.claw.ClawOpen();
                        if (timer.milliseconds() > OpenDelay) {
                            state = State.IDLE;
                            idleTransition();
                            timer.reset();
                        }
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.BACK)) {
                        robot.claw.ClawOpen();
                        if (timer.milliseconds() > OpenDelay) {
                            state = State.IDLEBACK;
                            IdleBackTransition();
                            timer.reset();
                        }   
                    }
                    setTurretPosition();


                    setLiftPosition();
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.START)) {
                        state = State.Idle2;
                        idleTransition();
                    }
                    break;
                }
            gamepadEx2.readButtons();
            gamepadEx1.readButtons();

            addDebugInfo();
            robot.update();
        }
    }
    public void setTurretPosition() {
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
            robot.turret.setPosition(RobotConstants.FrontTurret);
        }
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
            robot.turret.setPosition(RobotConstants.LeftTurret);
        }
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
            robot.turret.setPosition(RobotConstants.RightTurret);
        }
    }
    public void setLiftPosition() {
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            robot.lift.setTargetHeight(RobotConstants.HIGHJUNCTION);
        }
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            robot.lift.setTargetHeight(RobotConstants.MIDJUNCTION);
        }
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            robot.lift.setTargetHeight(RobotConstants.LOWJUNCTION);
        }
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            robot.lift.setTargetHeight(RobotConstants.GROUNDJUNCTION);
        }
    }
    public void addDebugInfo() {
        telemetry.addData("State", state);
        telemetry.addData("ScoreMode", mode);
        telemetry.addData("Distance", robot.lift.getDistance());
        telemetry.addData("Orientation", robot.drive.getOrientation());
//        telemetry.addData("Gamepad2 left stick y", gamepadEx2.getLeftY());
//        telemetry.addData("Gamepad2 left stick x", gamepadEx2.getLeftX());
//        telemetry.addData("Gamepad2 right stick x", gamepadEx2.getRightX());
    }

    public void idleTransition() {
//        robot.lift.setTargetHeight(LiftConstants.IdleHeight);
//        robot.lift.turretmotor.setTargetPosition(LiftConstants.FrontTurret);
//        robot.lift.setArmPos(LiftConstants.IdleArm);
    }

    public void IdleBackTransition() {
//        robot.lift.setTargetHeight(LiftConstants.IdleHeight);
//        robot.lift.turretmotor.setTargetPosition(LiftConstants.BackTurret);
//        robot.lift.setArmPos(LiftConstants.IdleArm);
    }

    public void IntakingFrontTransition() {
        robot.lift.setTargetHeight(RobotConstants.IntakingHeight);
        robot.turret.setPosition(RobotConstants.FrontTurret);
        robot.arm.setArm1Position(RobotConstants.IntakingArm);
        robot.arm.setArm2Position(RobotConstants.IntakingArm);
    }

    public void IntakingBackTransition() {
        robot.lift.setTargetHeight(RobotConstants.IntakingHeight);
        robot.turret.setPosition(RobotConstants.BackTurret);
        robot.arm.setArm1Position(RobotConstants.IntakingArm);
        robot.arm.setArm2Position(RobotConstants.IntakingArm);
    }
    public void OuttakingFrontTransition() {
    }

    public void OuttakingBackTransition() {
    }

    public void OuttakingLeftTransition() {
    }

    public void OuttakingRightTransition() {
    }

    public void Idle2Transition() {
        robot.lift.setTargetHeight(RobotConstants.IdleHeight);
        robot.turret.setPosition(RobotConstants.FrontTurret);
        robot.arm.setArm1Position(RobotConstants.ARM1INIT);
        robot.arm.setArm2Position(RobotConstants.ARM2INIT);
    }
    
    public void IdleBack2Transition() {
//        robot.lift.setTargetHeight(LiftConstants.IdleHeight);
//        robot.turret.setTargetPosition(LiftConstants.BackTurret);
//        robot.lift.setArmPos(LiftConstants.IdleArm);
    }

}

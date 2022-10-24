package org.firstinspires.ftc.teamcode.subsystem.lift;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class Lift implements Subsystem {

    public static double kG = 0.3;

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private AnalogSensor weightSensor;
    private RevColorSensorV3 colorRangeSensor;

    //constants in inches and RPM, everything else unitless
    private static double UP_WINCH_RADIUS = 40./25.4;
    private static double MOTOR_GEAR_RATIO =  1 + 46./11.;
    private static double MOTOR_BASE_RPM = 5960;
    private static double TICKS_PER_REVOLUTION = 28;

    private double MAX_POWER = 1;

    public static double kP = 0.6, kI = 0, kD = 0.01;

    private PIDController pid;

    private int bottomOffset = 0;

    private double targetHeight = 0;
    private double currentHeight = 0;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        motor1 = hardwareMap.get(DcMotorEx.class, "lift1");
        motor2 = hardwareMap.get(DcMotorEx.class, "lift2");

        //reverse correctly
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        colorRangeSensor = hardwareMap.get(RevColorSensorV3.class, "freightColorSensor");

        bottomOffset = motor2.getCurrentPosition();

        pid = new PIDController(kP, kI, kD);
    }

    @Override
    public void init() {
        setTargetHeight(0);
    }

    @Override
    public void update() {
        currentHeight = extensionLengthToHeight(encoderTicksToInches(getCurrentPosition()));
        double power;

        power = 0.2;
        if(targetHeight == 0 && currentHeight < 0.8) {
            setPower(0);
        } else if(motor1.getPower() > 0.5 && motor1.getCurrent(CurrentUnit.AMPS) > 10) {
            setPower(-1);
        } else {
            power += pid.calculate(currentHeight);
            setPower(power);
        }
    }

    @Override
    public void update(TelemetryPacket packet) {

    }


    public double getDistance() {
        return colorRangeSensor.getDistance(DistanceUnit.INCH);
    }

    public void setPower(double power) {
        power = Range.clip(power, -0.3,MAX_POWER);
        motor2.setPower(power);
        motor1.setPower(power);
    }


    public double getHeight() {
        return currentHeight = extensionLengthToHeight(encoderTicksToInches(getCurrentPosition()));
    }

    public double getTargetHeight() {
        return targetHeight;
    }

    public int getCurrentPosition() {
        return motor2.getCurrentPosition() - bottomOffset;
    }

    public void setTargetHeight(double height) {
        if(Math.abs(targetHeight - height) < 0.5) {
            return;
        }
        targetHeight = height;
        pid.setSetPoint(targetHeight);
    }

    public static double inchesToEncoderTicks(double inches) {
        return inches / (UP_WINCH_RADIUS * Math.PI) * (TICKS_PER_REVOLUTION * MOTOR_GEAR_RATIO);
    }

    public static double encoderTicksToInches(double ticks) {
        return ticks / (TICKS_PER_REVOLUTION * MOTOR_GEAR_RATIO) * UP_WINCH_RADIUS * Math.PI;
    }

    public static double extensionLengthToHeight(double length) {
        return length * Math.sin(Math.toRadians(70));
    }

    public static double heightToExtensionLength(double height) {
        return height / Math.sin(Math.toRadians(70));
    }

}

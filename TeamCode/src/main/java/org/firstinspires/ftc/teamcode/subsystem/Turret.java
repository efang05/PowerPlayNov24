package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret implements Subsystem {

    private DcMotorEx motorTurret;
    private double MAX_POWER = 1;
    private double MIN_POWER = -0.3;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        motorTurret = hardwareMap.get(DcMotorEx.class, "Turret");
        motorTurret.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Should we use RUN_TO_POSITION
        motorTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init() {
        setPower(0);
    }

    @Override
    public void update() {
        //motorTurret.setPower(power);
    }

    @Override
    public void update(TelemetryPacket packet) {
    }

    public void setPower(double power) {
        power = Range.clip(power, -0.3, MAX_POWER);
        motorTurret.setPower(power);
    }

    public double getCurrentPower() {
        return motorTurret.getPower();
    }

    // unit in degrees or radians
    public void setPosition(int position) {
        motorTurret.setTargetPosition(position);
    }

    public int getCurrentPosition() {
        return motorTurret.getCurrentPosition();
    }

    public void setVelocity(double angularRate) {
        motorTurret.setVelocity(angularRate);
    }

    public double getVelocity() {
        return motorTurret.getVelocity();
    }

}
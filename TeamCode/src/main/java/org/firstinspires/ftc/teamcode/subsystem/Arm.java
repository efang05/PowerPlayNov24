package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm implements Subsystem {

    private Servo servoArm1;
    private Servo servoArm2;
    private double MAX_POWER = 1;
    private double MIN_POWER = -1;
    private double INIT_POWER = -1;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        servoArm1 = hardwareMap.get(Servo.class, "arm1");
        servoArm2 = hardwareMap.get(Servo.class, "arm2");
    }
    @Override
    public void init() {
        setArm1Position(RobotConstants.ARM1INIT);
        setArm2Position(RobotConstants.ARM2INIT);
    }

    @Override
    public void update() {
    }

    @Override
    public void update(TelemetryPacket packet) {
    }

    public void setArm1Position(double position) {
        servoArm1.setPosition(position);
    }

    public void setArm2Position(double position) {
        servoArm2.setPosition(position);
    }

    public double getArm1Position() {
        return servoArm1.getPosition();
    }

    public double getArm2Position() {
        return servoArm2.getPosition();
    }
}
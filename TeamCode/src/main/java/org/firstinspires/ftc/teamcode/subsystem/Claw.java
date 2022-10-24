package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Claw implements Subsystem {

    private Servo clawServo1;
    private Servo clawServo2;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        clawServo1 = hardwareMap.get(Servo.class, "claw1");
        clawServo2 = hardwareMap.get(Servo.class, "claw2");
    }
    @Override
    public void init() {
        SetClaw1Position(RobotConstants.CLAWCLOSEPOS1);
        SetClaw2Position(RobotConstants.CLAWCLOSEPOS2);
    }

    @Override
    public void update() {
    }

    @Override
    public void update(TelemetryPacket packet) {

    }

    public void SetClaw1Position(double position){
        clawServo1.setPosition(position);
    }
    public void SetClaw2Position(double position){
        clawServo2.setPosition(position);
    }
    public void ClawOpen(){
        clawServo1.setPosition(RobotConstants.CLAWOPENPOS1);
        clawServo2.setPosition(RobotConstants.CLAWOPENPOS2);
    }
    public void ClawClose(){
        clawServo1.setPosition(RobotConstants.CLAWCLOSEPOS1);
        clawServo2.setPosition(RobotConstants.CLAWCLOSEPOS2);
    }

}

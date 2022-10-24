package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.lift.Lift;

import java.util.ArrayList;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Lift lift;
    public Claw claw;
    public Turret turret;
    public Arm arm;
    public Drivetrain drive;

    private ArrayList<Subsystem> subsystems;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        drive = new Drivetrain(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);


        subsystems = new ArrayList<>();
        subsystems.add(drive);
        subsystems.add(lift);
        subsystems.add(claw);
        subsystems.add(arm);
        subsystems.add(turret);
    }

    //Initializes hardware - NOTE: moves servos
    public void init() {
        for(Subsystem system : subsystems) {
            system.init();
        }
    }

    public void update() {
        for(Subsystem system : subsystems) {
            system.update();
        }
        telemetry.update();
    }
}
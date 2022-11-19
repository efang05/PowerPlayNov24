package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    // Lift
    public static double GROUNDJUNCTION = 0.5;
    public static double LOWJUNCTION = 15;
    public static double MIDJUNCTION = 25;
    public static double HIGHJUNCTION = 35;
    public static double IdleHeight = 4;
    public static double IntakingHeight = -1;
    // Claw
    public static double CLAWCLOSEPOS1 = 0.31;
    public static double CLAWOPENPOS1 = 0.52;
    // Turret
    public static int FrontTurret = 0;
    public static int LeftTurret = 56;
    public static int RightTurret = -56;
    public static int BackTurret = 102;
    // Arm
    public static double IdleArm = 0.8;
    public static double ARM1INIT = 0;
    public static double ARM2INIT = 0;
    public static double IntakingArm = 0.2;
    public static double GroundArm = 0.0;
    public static double HighArm = 0.2;
    public static double LowArm = 0.2;
    public static double MidArm = 0.2;
    public static double DROPPINGARM = 0.1;
}


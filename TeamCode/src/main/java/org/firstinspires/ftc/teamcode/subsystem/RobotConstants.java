package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    // Lift
    public static double GROUNDJUNCTION = 0.5;
    public static double LOWJUNCTION = 15;
    public static double MIDJUNCTION = 25;
    public static double HIGHJUNCTION = 35;
    public static double IdleHeight = 3;
    public static double IntakingHeight = 0.5;
    // Claw
    public static double CLAWCLOSEPOS1 = 0.3;
    public static double CLAWCLOSEPOS2 = 0.3;
    public static double CLAWOPENPOS1 = 0.7;
    public static double CLAWOPENPOS2 = 0.7;
    // Turret
    public static int FrontTurret = 0;
    public static int LeftTurret = -180;
    public static int RightTurret = 180;
    public static int BackTurret = 360;
    // Arm
    public static double ARM1INIT = 0;
    public static double ARM2INIT = 0;
    public static double IntakingArm = 0;
    public static double DROPPINGARM = 0.5;
}
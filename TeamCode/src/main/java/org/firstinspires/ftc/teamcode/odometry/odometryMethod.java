package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class odometryMethod extends LinearOpMode {

    //odometry constants (tune these)
    private double L = 9.5;   //distance between left and right odometers (in inches)
    private double B = 1.5;   //distance from center of left/right encoders to the perpendicular encoder (in inches)
    private double R = 1.0;   //wheel radius (in inches)
    private double N = 8192;  //encoder ticks per revoluton
    private double inPerTick = 2.0 * Math.PI * R / N;

    //changes starting location (in inches)
    public double X = 0;
    public double Y = 0;
    public double Heading = 0;

    //track encoder values between loops
    private int currentRightPos = 0;
    private int currentLeftPos = 0;
    private int currentPerpendicularPos = 0;
    private int oldRightPos = 0;
    private int oldLeftPos = 0;
    private int oldPerpendicularPos = 0;

    public void refresh()
    {
        //init hardware map
        odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);
        MathFunctions mathFunctions = new MathFunctions();

        //record last loop's encoder reading
        oldRightPos = currentRightPos;
        oldLeftPos = currentLeftPos;
        oldPerpendicularPos = currentPerpendicularPos;

        //record a new encoder reading this loop
        currentRightPos = robot.rightEncoder.getCurrentPosition();
        currentLeftPos = robot.leftEncoder.getCurrentPosition();
        currentPerpendicularPos = robot.perpendicularEncoder.getCurrentPosition();

        //find the delta encoder values of each encoder
        int dn1 = currentLeftPos - oldLeftPos;
        int dn2 = currentRightPos - oldRightPos;
        int dn3 = currentPerpendicularPos - oldPerpendicularPos;

        //find the delta of x,y,heading reletive to the robot
        double dtheta = inPerTick * (dn2 - dn1) / L;
        double dx = inPerTick * (dn1 + dn2) / 2.0;
        double dy = inPerTick * (dn3 - (dn2 - dn1) * B / L);

        //add the robots movement this loop to the global location
        double theta = (dtheta / 2.0);
        X += dx * Math.cos(theta) - dy * Math.sin(theta);
        Y += dx * Math.sin(theta) + dy * Math.cos(theta);
        Heading += dtheta;

        //makes heading 180 to -180
        mathFunctions.angleWrapRad(Heading);
    }

    @Override
    public void runOpMode() {}
}

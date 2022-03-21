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
    public double GlobalX = 0;
    public double GlobalY = 0;
    public double GlobalHeading = 0;

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
        GlobalX += dx * Math.cos(theta) - dy * Math.sin(theta);
        GlobalY += dx * Math.sin(theta) + dy * Math.cos(theta);
        GlobalHeading += dtheta;

        //makes heading 180 to -180
        angleWrapRad(GlobalHeading);
    }
    
    
    
    
    public void angleWrapRad(double angle)
    {
        while (angle > Math.PI)
        {
            angle -= Math.PI * 2;
        }
        while (angle < -Math.PI)
        {
            angle += Math.PI * 2;
        }
    }
    
    
    
    
    
    
    public void wait(double waitTime)
    {
        ElapsedTime time = new ElapsedTime();
        
        while (time <= waitTime)
        {
            refresh();
        }
    }
    
    
    
    
    
    public static void goToPos(double x, double y, double finalAngle, double moveSpeed, double turnSpeed, double followAngle)
    {
        odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);
        
        reset();
        
        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        
        double absoluteAngleToTarget = Math.atan2(y - GlobalY, x - GlobalX);
        
        double reletiveAngleToTarget = angleWrapRad(absoluteAngleToTarget - GlobalHeading);
        double reletiveXToTarget = Math.cos(reletiveAngleToTarget) * distanceToTarget;
        double reletiveYToTarget = Math.sin(reletiveAngleToTarget) * distanceToTarget;
        
        double movementXpower = (reletiveXToTarget / (Math.abs(reletiveXToTarget) + Math.abs(reletiveYToTarget))) * moveSpeed;
        double movementYpower = (reletiveYToTarget / (Math.abs(reletiveXToTarget) + Math.abs(reletiveYToTarget))) * moveSpeed;
        
        if (distanceToTarget > 10) 
        {
            double reletiveTurnAngle = reletiveAngleToTarget + followAngle;
            double movementTurnPower = Range.clip(reletiveTurnAngle / Math.toRadians(10), -1, 1) * turnSpeed;
        }
        
        else 
        {
            double movementTurnPower = ((finalAngle - GlobalHeading) /finalAngle)+ .2;
        }
        
        robot.motorRF.setPower((-movementXpower - movementYpower) - (movementTurnPower));
        robot.motorRB.setPower(-(-movementYpower + movementXpower) - (movementTurnPower));
        robot.motorLB.setPower((movementXpower + movementYpower) - (movementTurnPower));
        robot.motorLF.setPower((-movementYpower + movementXpower) - (movementTurnPower));
    }
    
    

    @Override
    public void runOpMode() {}
}

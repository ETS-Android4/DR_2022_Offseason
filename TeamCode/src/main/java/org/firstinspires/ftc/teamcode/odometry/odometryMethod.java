package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * how to use this code,
 *
 * the cordinant system is a rotated cordinate system with X being forward.
 *
 *       ^ x+
 *       |
 * y+    |
 * <-----O
 *
 *
 *
 * the drive code is written for a mecanum drive, but the localizer will work with any drive train.
 * as long as the odometry pods are set up correctly.
 *
 * there should be 2 forward facing odometry wheels, and one sideways odometry wheel.
 * illistrated bellow:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 *
 *
 *
 * to begin using the code
 * ensure the following lines are in your hardware map:
 * ---------------------------------------------------------------
 * motorRF = ahwMap.dcMotor.get("motorRF");
 * motorLF = ahwMap.dcMotor.get("motorLF");
 * motorRB = ahwMap.dcMotor.get("motorRB");
 * motorLB = ahwMap.dcMotor.get("motorLB");
 *
 * motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 * motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 * motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 * motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 *
 * leftEncoder = motorLF;
 * rightEncoder = motorRF;
 * perpendicularEncoder = motorRB;
 * ---------------------------------------------------------------
 * replace "____Encoder = motor__" with the correct motor (the same motor from the port that the odometry encoder shares)
 *
 **//* the blanks here refer to the words 'left', 'right', and 'perpendicular'*//**
 *
 * replace "odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);" with the chosen hardware map
 *
 *
 **//* the blanks here refer to a program that will be made in the future*//**
 * using ____, tune the values accordingly
 * 1. enter the radius of the odometer wheel as R
 *      to tune:
 *      - push the robot straight forward 100".
 *      - if the distance is not close to 100, adjust R
 *      - repeat till the distance read is close to 100"
 *
 *
 * 2. enter the distance between the 2 forward facing encoder wheels in inches as L.
 *      to tune:
 *      - spin 10 times
 *      - if the angle does not read around 0, change L up or down a small amount.
 *      - repeat till the angle reads around 0
 *
 *
 * 3. enter the distance from the middle of a forward facing encoder wheel to the middle of the sideways encoder wheel as B
 *      to tune:
 *      - spin 10 times
 *  *   - if the y distance does not read around 0, change B up or down a small amount.
 *  *   - repeat till the y distance reads around 0
 *
 *
 *
 *  notes:
 * when using the code in auto the refresh() method must be constantly updated
 * use a structure similar to bellow:
 * ---------------------------------------------------------------
 * odometry.goToPos(...);
 * odometry.wait(...);
 * move a servo
 * odometry.wait(...);
 * odometry.goToPos(...);
 * ect.
 * ---------------------------------------------------------------
 * this use of odometry.wait(...); allows the odometry to continue to update. compared to a sleep(...); which pauses all the code for the duration of the sleep.
 */

public class odometryMethod extends LinearOpMode {

    //odometry constants (tune these)
    double L = 9.33430667;   //distance between left and right odometers (in inches)
    double B = -1.77319;   //distance from center of left/right encoders to the perpendicular encoder (in inches)
    double R = 0.985;   //wheel radius (in inches)
    double N = 8192;  //encoder ticks per revoluton
    double inPerTick = 2.0 * Math.PI * R / N;

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


    /**
     * refresh() is the core of the odometry code.
     * calling this method will recalculate the location of the bot, but must be updating regularily inside a loop
     * it will save public values which can be accessed to identify the global position of the robot.
     *
     * enter motors into the array 0.left, 1.right, 2.perpendicular
     *
     * for a good explination of the math behind odometry watch this video:
     * https://www.youtube.com/watch?v=Av9ZMjS--gY
     */

    public void refresh(DcMotor[] odometers)
    {

        //record last loop's encoder reading
        oldRightPos = currentRightPos;
        oldLeftPos = currentLeftPos;
        oldPerpendicularPos = currentPerpendicularPos;

        //record a new encoder reading this loop
        currentRightPos = odometers[0].getCurrentPosition();
        currentLeftPos = odometers[1].getCurrentPosition();
        currentPerpendicularPos = odometers[2].getCurrentPosition();

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
    
    
    
    // used to mantain angle values between Pi and -Pi
    public double angleWrapRad(double angle)
    {
        while (angle > Math.PI)
        {
            angle -= Math.PI * 2;
        }
        while (angle < -Math.PI)
        {
            angle += Math.PI * 2;
        }

        return angle;
    }
    
    
    
    
    //use instead of sleep() in autonomus to keep the location updating
    public void wait(double waitTime, DcMotor[] odometers)
    {
        ElapsedTime time = new ElapsedTime();
        
        while (time.milliseconds() <= waitTime)
        {
            refresh(odometers);
        }
    }


    /**
     * this method is the key to using odometry
     * by imputing a location to drive to the robot will calculate an efficient path to the target.
     * if the robot is interfered with, it will recalculate and adjust accordingly
     *
     * at the beginning of the drive the robot will face the target (because mecanums are faster forward than sideways)
     * when it gets within a set distance of the target it will begin turning toward its desired final orientation
     * when it is within its accuracy requirements for the move it will exit the loop and set the motor powers to 0
     *
     * this code is pulled from the basis of pure pursuit.
     * for a better understanding of pure pursuit and if someone wants to improve this code, look into learning how the rest of gluten free's code works here:
     * https://www.youtube.com/watch?v=3l7ZNJ21wMo (5 parts)
     * the code below uses the code explains in parts 1 & 2
     */
    public void goToPos(DcMotor[] odometers, double x, double y, double finalAngle, double moveSpeed, double turnSpeed, double moveAccuracy, double angleAccuracy, double followAngle)
    {
        //bring in the encoder and motor objects
        odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);

        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)
        while(Math.abs(x-GlobalX) > moveAccuracy || Math.abs(y-GlobalY) > moveAccuracy || Math.abs(finalAngle - GlobalHeading) > angleAccuracy) {

            //update odometry location
            refresh(odometers);

            //math to calculate distances to the target
            double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
            double absoluteAngleToTarget = Math.atan2(y - GlobalY, x - GlobalX);
            double reletiveAngleToTarget = angleWrapRad(absoluteAngleToTarget - GlobalHeading);
            double reletiveXToTarget = Math.cos(reletiveAngleToTarget) * distanceToTarget;
            double reletiveYToTarget = Math.sin(reletiveAngleToTarget) * distanceToTarget;

            //slow down ensures the robot does not over shoot the target
            double slowDown = Range.clip(distanceToTarget / 5, -1, 1);

            //calculate the vector powers for the mecanum math
            double movementXpower = (reletiveXToTarget / (Math.abs(reletiveXToTarget) + Math.abs(reletiveYToTarget))) * moveSpeed * slowDown;
            double movementYpower = (reletiveYToTarget / (Math.abs(reletiveYToTarget) + Math.abs(reletiveXToTarget))) * moveSpeed * slowDown;

            //when far away from the target the robot will point at the target to get there faster.
            //at the end of the movement the robot will begin moving toward the desired final angle
            double movementTurnPower;
            if (distanceToTarget > 5) {
                double reletiveTurnAngle = reletiveAngleToTarget + followAngle;
                movementTurnPower = Range.clip(reletiveTurnAngle / Math.toRadians(10), -1, 1) * turnSpeed;
            } else {
                movementTurnPower = ((finalAngle - GlobalHeading) / finalAngle) + .2;
            }

            //set the motors to the correct powers to move toward the target
            robot.motorRF.setPower((-movementXpower - movementYpower) - (-movementTurnPower));
            robot.motorRB.setPower(-(-movementYpower + movementXpower) - (-movementTurnPower));
            robot.motorLB.setPower(-(movementXpower + movementYpower) - (-movementTurnPower));
            robot.motorLF.setPower(-(-movementYpower + movementXpower) - (movementTurnPower));
        }

        //at the end of the movement stop the motors
        robot.motorRF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);

    }


    /**
     * a simple version of goToPos used for testing
     */

    public void goToPosSimple(DcMotor[] odometers, double x, double y, double finalAngle, double moveAccuracy, double angleAccuracy, double followAngle)
    {
        //bring in the encoder and motor objects
        odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);

        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)
        while(Math.abs(x-GlobalX) > moveAccuracy || Math.abs(y-GlobalY) > moveAccuracy || Math.abs(finalAngle - GlobalHeading) > angleAccuracy) {

            //update odometry location
            refresh(odometers);

            //math to calculate distances to the target
            double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
            double absoluteAngleToTarget = Math.atan2(y - GlobalY, x - GlobalX);
            double reletiveAngleToTarget = angleWrapRad(absoluteAngleToTarget - GlobalHeading);
            double reletiveXToTarget = Math.cos(reletiveAngleToTarget) * distanceToTarget;
            double reletiveYToTarget = Math.sin(reletiveAngleToTarget) * distanceToTarget;

            //calculate the vector powers for the mecanum math
            double movementXpower = (reletiveXToTarget / (Math.abs(reletiveXToTarget) + Math.abs(reletiveYToTarget)));
            double movementYpower = (reletiveYToTarget / (Math.abs(reletiveYToTarget) + Math.abs(reletiveXToTarget)));

            //when far away from the target the robot will point at the target to get there faster.
            //at the end of the movement the robot will begin moving toward the desired final angle
            double reletiveTurnAngle = reletiveAngleToTarget + followAngle;
            double movementTurnPower = Range.clip(reletiveTurnAngle / Math.toRadians(10), -1, 1);


            //set the motors to the correct powers to move toward the target
            robot.motorRF.setPower((-movementXpower - movementYpower) - (-movementTurnPower));
            robot.motorRB.setPower(-(-movementYpower + movementXpower) - (-movementTurnPower));
            robot.motorLB.setPower(-(movementXpower + movementYpower) - (-movementTurnPower));
            robot.motorLF.setPower(-(-movementYpower + movementXpower) - (movementTurnPower));
        }

        //at the end of the movement stop the motors
        robot.motorRF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);

    }


    //only here to make linear opMode a valid extention
    @Override
    public void runOpMode() {}
}

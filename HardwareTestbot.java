package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Testbot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Motor channel:  Manipulator drive motor:  "arm motor"
 * Servo channel:  Servo to open claw:  "claw"
 */
public class HardwareTestbot
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    //    public DcMotor  armMotor    = null;
    public Servo claw = null;

    public ModernRoboticsI2cGyro gyro;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    public static final double CLAW_MIN = 0;
    public static double CLAW_MAX = .8;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    LinearOpMode opMode;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTestbot(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opMode) {
        // Save reference to Hardware map
        hwMap = opMode.hardwareMap;
        this.opMode = opMode;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left motor");
        rightMotor  = hwMap.dcMotor.get("right motor");
//        armMotor    = hwMap.dcMotor.get("arm motor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
//        armMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        claw = hwMap.servo.get("claw");
        claw.setPosition(MID_SERVO);

        // Define sensors
        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file

    }
    public void calibrateGyro() {
        // start calibrating the gyro.
        opMode.telemetry.addData("*", "Gyro Calibrating. Do Not move!");
        opMode.telemetry.update();

        opMode.sleep(1000); // wait 1 second for gyro to stabilize (may be movement from initializing servo)
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (gyro.isCalibrating())  {
            opMode.sleep(50);
        }

        opMode.telemetry.addData("*", "Gyro Calibrated.  Press Start.");
        opMode.telemetry.update();

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            opMode.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
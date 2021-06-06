package org.firstinspires.ftc.teamcode.drive.Comp.HardwareBot;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware {

    /* Public OpMode members. */
    public HardwareMap hwMap = null;
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx flywheel = null;
    public DcMotorEx intake = null;
    public DcMotorEx xEncoder = null;
    public DcMotorEx yEncoder = null;
    public Servo pushRing = null;
    public CRServo leftArm = null;
    public CRServo rightArm = null;
    public CRServo hand = null;
    public Servo intakepush = null;
    public CRServo  angle = null;


    /* local OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 20.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Constructor */
    public Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotorEx.class, "front_left_drive");
        rightFront = hwMap.get(DcMotorEx.class, "front_right_drive");
        leftRear = hwMap.get(DcMotorEx.class, "back_left_drive");
        rightRear = hwMap.get(DcMotorEx.class, "back_right_drive");
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        intake = hwMap.get(DcMotorEx.class, "intake");
        xEncoder = hwMap.get(DcMotorEx.class, "xEncoder");
        yEncoder = hwMap.get(DcMotorEx.class, "yEncoder");
        leftFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        xEncoder.setDirection(DcMotor.Direction.REVERSE);
        yEncoder.setDirection(DcMotor.Direction.REVERSE);

        // Define and initialize ALL installed servos.
        pushRing = hwMap.get(Servo.class, "push_ring");
        leftArm = hwMap.get(CRServo.class, "left_arm");
        rightArm = hwMap.get(CRServo.class, "right_arm");
        hand = hwMap.get(CRServo.class, "hand");
        intakepush = hwMap.get(Servo.class, "intake_push");
        angle = hwMap.get(CRServo.class, "angle");


    }

//    public void runToPosition(int position, double power) {
//        angle.setPower(0);
//        angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        angle.setTargetPosition(position);
//        angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (!(angle.getCurrentPosition() == angle.getTargetPosition())) {
//            angle.setPower(power);
//        }
//        angle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        angle.setPower(0);
//    }
}


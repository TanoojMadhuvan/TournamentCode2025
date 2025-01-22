package org.firstinspires.ftc.teamcode._CONFIG;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HwareV2 {

    //Port3: wrist
    //Port4: elbow1
    //Port5: intake claw
    //Port2: elbow2

    HardwareMap hardwareMap;
    public Motor frontRight, frontLeft, backRight, backLeft;
    public DcMotor horizontal1,horizontal2;
    public DcMotor vertical1, vertical2;
    public ServoEx rightElbow, leftElbow, intakeWrist, intakeClaw, rightNeck, leftNeck, jaw, outtakeClaw;

    //intake claw: port 5
    //elbow right: port 4
    //elbow left: port 0


    public void initialize(HardwareMap hwMap){
        hardwareMap = hwMap;

        // Drive Base
        frontRight = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        backRight = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);
        frontLeft = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        backLeft = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);

        // Horizontal Lift
        horizontal1 = hardwareMap.get(DcMotor.class, "leftHor");
        horizontal2 = hardwareMap.get(DcMotor.class, "rightHor");

        // Vertical Lift
        vertical1 = hardwareMap.get(DcMotor.class, "leftVer");
        vertical2 = hardwareMap.get(DcMotor.class, "rightVer");

        // Vertical Servos
        rightElbow = new SimpleServo(hardwareMap, "rightElbow", 0, 240);
        leftElbow = new SimpleServo(hardwareMap, "leftElbow", 0, 240);
        rightNeck = new SimpleServo(hardwareMap, "rightNeck", 0, 240);
        leftNeck = new SimpleServo(hardwareMap, "leftNeck", 0, 240);
        jaw = new SimpleServo(hardwareMap, "jaw", 0, 240);
        outtakeClaw = new SimpleServo(hardwareMap, "outtakeClaw", 0, 240);

        // Horizontal Servos
        intakeClaw = new SimpleServo(hardwareMap, "intake", 0, 240);
        intakeWrist = new SimpleServo(hardwareMap, "wrist", 0, 240);

        // Mutators
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);



        horizontal1.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setInverted(true);
        frontRight.setInverted(true);
        vertical1.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontal1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontal2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertical1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertical2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizontal1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);





    }


}
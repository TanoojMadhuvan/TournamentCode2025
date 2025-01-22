package org.firstinspires.ftc.teamcode._CONFIG;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hware {
    public Motor frontRight, frontLeft, backRight, backLeft;
    public Motor intake;
    public DcMotor horizontal1,horizontal2;
    public DcMotor vertical1, vertical2;
    public ServoEx rightElbow, leftElbow, intakeWrist, intakeClaw, rightNeck, leftNeck, jaw, outtakeClaw;
    HardwareMap hardwareMap;



    public Hware() {
        hardwareMap = null;
    }
     public void initialize(HardwareMap hwMap){
        hardwareMap = hwMap;
        frontRight = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        backRight = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);
        frontLeft = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        backLeft = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);

         //intake = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_312);


        horizontal1 = hardwareMap.get(DcMotor.class, "leftHor");
        horizontal2 = hardwareMap.get(DcMotor.class, "rightHor");
        vertical1 = hardwareMap.get(DcMotor.class, "leftVer");
        vertical2 = hardwareMap.get(DcMotor.class, "rightVer");
        rightElbow = new SimpleServo(hardwareMap, "rightElbow", 0, 240);
        leftElbow = new SimpleServo(hardwareMap, "leftElbow", 0, 240);
        rightNeck = new SimpleServo(hardwareMap, "rightNeck", 0, 240);
        leftNeck = new SimpleServo(hardwareMap, "leftNeck", 0, 240);
        jaw = new SimpleServo(hardwareMap, "jaw", 0, 240);
        outtakeClaw = new SimpleServo(hardwareMap, "outtakeClaw", 0, 240);


         intakeClaw = new SimpleServo(hardwareMap, "intake", 0, 240);
        intakeWrist = new SimpleServo(hardwareMap, "wrist", 0, 240);

         //        vRot1= new SimpleServo(hardwareMap, "vRot1", 0, 240);
//        vRot2= new SimpleServo(hardwareMap, "vRot2", 0, 240);
//        clawRot= new SimpleServo(hardwareMap, "clawRot", 0, 240);

         // intake.setInverted(true);




        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);



        horizontal1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horizontal1.setDirection(DcMotorSimple.Direction.REVERSE);

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

//hello
package org.firstinspires.ftc.teamcode._AUTON;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode._CONFIG.Hware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class BucketAuton1_3 extends LinearOpMode {

    Boolean RUNNOTON = true;
    Boolean RUNNOTONV = true;

    double SPEEDCONTROL = 1;
    double TURNCONTROL = 1;
    String vMSG = "PICK";
    String hMSG = "PICK";
    Hware robot;
    int intakeWait = 0;
    double wristPos = 0;
    public void setWrist(){
        robot.intakeWrist.setPosition(wristPos);
    }

    public void setWrist(double pos){
        robot.intakeWrist.setPosition(pos);
    }

    public void changeHorizontalPosition(int pos1, int pos2, double pow) {
        for(int i = 0; i < intakeWait; i++){

        }
        intakeWait = 0;
        RUNNOTON = false;
        robot.horizontal1.setTargetPosition(pos1);
        robot.horizontal2.setTargetPosition(pos2);
        robot.horizontal1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horizontal2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horizontal1.setPower(pow);
        robot.horizontal2.setPower(pow);
    }

    public void changeVerticalPosition(int pos1, int pos2, double pow) {
        robot.vertical1.setTargetPosition(pos1);
        robot.vertical2.setTargetPosition(pos2);
        robot.vertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertical2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertical1.setPower(pow);
        robot.vertical2.setPower(pow);
    }

    public void openIntakeClaw(){
        robot.intakeClaw.setPosition(0.2);
    }

    public void closeIntakeClaw(){
        robot.intakeClaw.setPosition(0.5);
    }

    public void openOuttakeClaw(){
        robot.outtakeClaw.setPosition(0.08);
    }

    public void closeOuttakeClaw(){
        robot.outtakeClaw.setPosition(0.25);
    }

    public void changeVerticalPosition(int Position1, int Position2) {
        RUNNOTONV = false;
        robot.vertical1.setTargetPosition(Position1);
        robot.vertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertical2.setTargetPosition(Position2);
        robot.vertical2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // INTAKE PORTION
    public void rotateIntakeToGround(boolean adjusting) {
        if(adjusting) {
            robot.rightElbow.setPosition(0.159);
            robot.leftElbow.setPosition(0.841);
        }else{
            robot.rightElbow.setPosition(0);
            robot.leftElbow.setPosition(1);
        }
    }

    public void rotateIntakeToTransfer() {
        robot.rightElbow.setPosition(0.55);
        robot.leftElbow.setPosition(0.45);

    }

    public void rotateIntakeToMove() {
        robot.rightElbow.setPosition(0.573);
        robot.leftElbow.setPosition(0.426);
        intakeWait = 0;
    }

    //Pick from wall: r: 0.873, l:0.127
    //Transfer: r: 0, l: 1


    public void rotateOuttakeToPickWall() {
        robot.rightNeck.setPosition(0.75);
        robot.leftNeck.setPosition(0.25);
        robot.jaw.setPosition(0.1);

    }

    public void rotateOuttakeToScoreBucket() {

        robot.rightNeck.setPosition(0.42);
        robot.leftNeck.setPosition(0.58);
        robot.jaw.setPosition(0.9);

    }

    public void rotateOuttakeToScoreBack() {
        robot.rightNeck.setPosition(0.67);
        robot.leftNeck.setPosition(0.33);
        robot.jaw.setPosition(0.1);

    }

    public void rotateOuttakeToScoreFront() {
        robot.rightNeck.setPosition(0);
        robot.leftNeck.setPosition(1);
        robot.jaw.setPosition(0.5);

    }

    public void rotateOuttakeToTransfer() {
        robot.rightNeck.setPosition(0);
        robot.leftNeck.setPosition(1);
        robot.jaw.setPosition(0.708);
    }

    public void rotateOuttakeToHangSpecimen() {
        robot.rightNeck.setPosition(0.82);
        robot.leftNeck.setPosition(0.18);
        robot.jaw.setPosition(0.555);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hware();
        robot.initialize(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence holdPreload = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(0, ()->{
                    closeOuttakeClaw();
                })
                .lineToConstantHeading(new Vector2d(0, 0.1),null, new ProfileAccelerationConstraint(30))
                .waitSeconds(0.2)
                .build();

        TrajectorySequence preloadMove = drive.trajectorySequenceBuilder(holdPreload.end())
                .addTemporalMarker(0, ()->{
                    rotateOuttakeToScoreBack();
                    changeVerticalPosition(-360, 360, 0.5);
                    setWrist(0.272);

                })
                .lineToConstantHeading(new Vector2d(-28, 10),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    rotateIntakeToTransfer();
                    changeVerticalPosition(0, 0, 0.8);
                })
                //.lineToConstantHeading(new Vector2d(-28, 10),null, new ProfileAccelerationConstraint(20))
                .build();

        TrajectorySequence waitTime = drive.trajectorySequenceBuilder(preloadMove.end())
                .waitSeconds(0.2)
                .build();

        TrajectorySequence preloadScore = drive.trajectorySequenceBuilder(waitTime.end())
                .addTemporalMarker(0, ()->{
                    rotateIntakeToTransfer();
                    changeVerticalPosition(-360,348,0.8);
                })
                .lineToConstantHeading(new Vector2d(-28, -6.1),null, new ProfileAccelerationConstraint(30))
                .build();

        TrajectorySequence pick1 = drive.trajectorySequenceBuilder(waitTime.end())
                .addTemporalMarker(0, ()->{
                    rotateIntakeToGround(true);
                    openIntakeClaw();
                    rotateOuttakeToTransfer();
                    changeVerticalPosition(0,0,0.2);
                }).waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(-27, -23, Math.toRadians(-120)),null, new ProfileAccelerationConstraint(30))
                .lineToConstantHeading(new Vector2d(-31.9, -22),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    rotateIntakeToGround(false);
                })
                //.lineToConstantHeading(new Vector2d(-28.01, -6.1),null, new ProfileAccelerationConstraint(30))
                .build();

        TrajectorySequence waitTime2 = drive.trajectorySequenceBuilder(pick1.end())
                .addTemporalMarker(0, ()->{
                    rotateOuttakeToPickWall();
                    openOuttakeClaw();
                })
                .waitSeconds(0.3)
                .build();

        TrajectorySequence bufferPick1 = drive.trajectorySequenceBuilder(waitTime2.end())
                .waitSeconds(0.5)
                .build();

        TrajectorySequence alignToScore1 = drive.trajectorySequenceBuilder(bufferPick1.end())
                .addTemporalMarker(0, ()->{
                    setWrist(0.158);
                    rotateIntakeToTransfer();
                    changeVerticalPosition(0,0,0.1);
                })
                .lineToLinearHeading(new Pose2d(-9, -39.3, Math.toRadians(-225)),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    rotateOuttakeToTransfer();
                })
                .build();

        TrajectorySequence waitTime3 = drive.trajectorySequenceBuilder(alignToScore1.end())
                .waitSeconds(1)
                .build();


        TrajectorySequence waitTime4 = drive.trajectorySequenceBuilder(waitTime3.end())
                .waitSeconds(0.3)
                .build();

        TrajectorySequence waitTime5 = drive.trajectorySequenceBuilder(waitTime4.end())
                .waitSeconds(1)
                .build();

        TrajectorySequence pick2 = drive.trajectorySequenceBuilder(waitTime5.end())
                .addTemporalMarker(0, ()->{
                    rotateIntakeToGround(true);
                    openIntakeClaw();
                    rotateOuttakeToTransfer();
                    changeVerticalPosition(-200,194,0.2);
                })
                .lineToLinearHeading(new Pose2d(-25, -43, Math.toRadians(-180)),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    rotateIntakeToGround(false);
                })
                .build();

        TrajectorySequence waitTime6 = drive.trajectorySequenceBuilder(pick2.end())
                .waitSeconds(0.3)
                .build();

        TrajectorySequence waitTime7 = drive.trajectorySequenceBuilder(waitTime6.end())
                .waitSeconds(0.3)
                .build();


        TrajectorySequence waitTime8 = drive.trajectorySequenceBuilder(waitTime7.end())
                .waitSeconds(0.3)
                .build();

        TrajectorySequence waitTime9 = drive.trajectorySequenceBuilder(waitTime8.end())
                .waitSeconds(1)
                .build();

        TrajectorySequence bufferPick2 = drive.trajectorySequenceBuilder(waitTime7.end())
                .waitSeconds(0.2)
                .build();

        TrajectorySequence alignToScore2 = drive.trajectorySequenceBuilder(bufferPick2.end())
                .addTemporalMarker(0, ()->{
                    setWrist(0.158);
                    rotateIntakeToTransfer();
                })
                .waitSeconds(.2)
                .lineToLinearHeading(new Pose2d(-9, -39.3, Math.toRadians(-225)),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    changeVerticalPosition(0,0,1);
                })
                //.waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-9, -39.31, Math.toRadians(-225)),null, new ProfileAccelerationConstraint(30))
                .waitSeconds(0.2)
                .addDisplacementMarker(()->{
                    closeOuttakeClaw();
                })

                .build();

        TrajectorySequence score2 = drive.trajectorySequenceBuilder(alignToScore2.end())
                .waitSeconds(3)
                .build();

        TrajectorySequence pick3 = drive.trajectorySequenceBuilder(waitTime9.end())
                .addTemporalMarker(0, ()->{
                    rotateIntakeToGround(true);
                    openIntakeClaw();
                    rotateOuttakeToTransfer();
                    changeVerticalPosition(-200,194,0.7);
                })

                .lineToLinearHeading(new Pose2d(-35.7, -41.6, Math.toRadians(-90)),null, new ProfileAccelerationConstraint(30))

                .addDisplacementMarker(()->{
                    rotateIntakeToGround(false);
                })
                .build();


        TrajectorySequence waitTime10 = drive.trajectorySequenceBuilder(pick3.end())
                .waitSeconds(0.3)
                .build();

        TrajectorySequence waitTime11 = drive.trajectorySequenceBuilder(waitTime10.end())
                .waitSeconds(0.3)
                .build();

        TrajectorySequence waitTime12 = drive.trajectorySequenceBuilder(waitTime11.end())
                .waitSeconds(0.3)
                .build();

        TrajectorySequence waitTime13 = drive.trajectorySequenceBuilder(waitTime12.end())
                .waitSeconds(1)
                .build();

        TrajectorySequence bufferPick3 = drive.trajectorySequenceBuilder(waitTime13.end())
                .waitSeconds(0.5)
                .build();

        TrajectorySequence alignToScore3 = drive.trajectorySequenceBuilder(bufferPick3.end())
                .addTemporalMarker(0, ()->{
                    setWrist(0.158);
                    rotateIntakeToTransfer();
                })
                .waitSeconds(.2)
                .lineToLinearHeading(new Pose2d(-9, -39.3, Math.toRadians(-225)),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    changeVerticalPosition(0,0,1);
                })
                //.waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-9, -39.31, Math.toRadians(-225)),null, new ProfileAccelerationConstraint(30))
                .waitSeconds(0.2)
                .addDisplacementMarker(()->{
                    closeOuttakeClaw();
                })
                .build();


        TrajectorySequence score3 = drive.trajectorySequenceBuilder(alignToScore3.end())
                .waitSeconds(3)
                .build();

//        TrajectorySequence park = drive.trajectorySequenceBuilder(waitTime13.end())
//                .lineToLinearHeading(new Pose2d(-39.2, -40.2, Math.toRadians(-90)),null, new ProfileAccelerationConstraint(30))
//                .lineToConstantHeading(new Vector2d(-60, -10),null, new ProfileAccelerationConstraint(30))
//                .build();


        rotateOuttakeToTransfer();
        closeOuttakeClaw();
        rotateIntakeToTransfer();
        closeIntakeClaw();
        waitForStart();

        if (isStopRequested()) return;

        //Preload Score
        drive.followTrajectorySequence(holdPreload);
        drive.followTrajectorySequence(preloadMove);
        drive.followTrajectorySequence(waitTime);
        openOuttakeClaw();
//        drive.followTrajectorySequence(preloadScore);

        //Pick 1
        drive.followTrajectorySequence(pick1);
        drive.followTrajectorySequence(waitTime2);
        closeIntakeClaw();
        drive.followTrajectorySequence(bufferPick1);

        //Score 1
        drive.followTrajectorySequence(alignToScore1);
        drive.followTrajectorySequence(waitTime3);
        closeOuttakeClaw();
        drive.followTrajectorySequence(bufferPick3);
        openIntakeClaw();
        changeVerticalPosition(-1079, 1081, 1);
        drive.followTrajectorySequence(waitTime4);
        rotateOuttakeToScoreBack();
        drive.followTrajectorySequence(waitTime5);
        openOuttakeClaw();

        //Pick 2

        drive.followTrajectorySequence(pick2);
        drive.followTrajectorySequence(waitTime6);
        closeIntakeClaw();
        drive.followTrajectorySequence(bufferPick2);

        //Score 2
        drive.followTrajectorySequence(alignToScore2);
        drive.followTrajectorySequence(waitTime7);
        openIntakeClaw();
        changeVerticalPosition(-1079, 1081, 1);
        drive.followTrajectorySequence(waitTime8);
        rotateOuttakeToScoreBack();
        drive.followTrajectorySequence(waitTime9);
        openOuttakeClaw();
        setWrist(0.324);

        //Pick 3
        drive.followTrajectorySequence(pick3);
        drive.followTrajectorySequence(waitTime10);
        closeIntakeClaw();
        drive.followTrajectorySequence(bufferPick3);

        //Score 3
        drive.followTrajectorySequence(alignToScore3);
        drive.followTrajectorySequence(waitTime11);
        openIntakeClaw();
        changeVerticalPosition(-1079, 1081, 1);
        drive.followTrajectorySequence(waitTime12);
        rotateOuttakeToScoreBack();
        drive.followTrajectorySequence(waitTime13);
        openOuttakeClaw();
        changeVerticalPosition(0, 0, 1);
        rotateOuttakeToTransfer();



//        drive.followTrajectorySequence(score1);
//        drive.followTrajectorySequence(pick2);
//        drive.followTrajectorySequence(bufferPick2);
//        drive.followTrajectorySequence(alignToScore2);
//        drive.followTrajectorySequence(score2);
//        drive.followTrajectorySequence(pick3);
//        drive.followTrajectorySequence(bufferPick3);
//        drive.followTrajectorySequence(alignToScore3);
//        drive.followTrajectorySequence(score3);
        //drive.followTrajectorySequence(park);

//        drive.followTrajectorySequence(alignToPush3);
//        drive.followTrajectorySequence(push3);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousMovement", group = "Autonomous")
public class AutonomousMovement extends LinearOpMode {

    private DcMotorEx slider;

    private Servo RightGrabber, LeftGrabber, Adjust, LeftArm, RightArm;

    private double x=0;

    private double y=0;

    private double heading=0;

    private double forwardDistance=0;

    private double backwardDistance=0;

    private double rightAngle=0;

    private double leftAngle=0;

    private double strafeRightDistance=0;

    private double strafeLeftDistance=0;

    private double countsPerInch=1917.0;

    private Pose2d pose=null;

    public void lineToHeading(double Xdis, double Ydis, double angle){

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose= new Pose2d();

        x=Xdis;
        y=Ydis;
        heading= Math.toRadians(angle);

        Trajectory lineTo= drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x,y,heading))
                .build();

        drive.followTrajectory(lineTo);

    }


    public void forward(double distance){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        forwardDistance=distance;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 25 inches
        Trajectory forwardTrajectory = drive.trajectoryBuilder(startPose)
                .forward(forwardDistance)
                .build();

        drive.followTrajectory(forwardTrajectory);

    }

    public void backward(double distance){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        backwardDistance=distance;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 25 inches
        Trajectory backwardTrajectory = drive.trajectoryBuilder(startPose)
                .back(backwardDistance)
                .build();

        drive.followTrajectory(backwardTrajectory);

    }

    public void turnRight(double turnAngle){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        rightAngle=turnAngle;

        drive.turn(Math.toRadians(-rightAngle));
    }

    public void turnLeft(double turnAngle){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        leftAngle=turnAngle;

        drive.turn(Math.toRadians(leftAngle));

    }

    public void strafeRight(double strafeRightDis){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        strafeRightDistance=strafeRightDis;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 25 inches
        Trajectory strafeRightTrajectory = drive.trajectoryBuilder(startPose)
                .strafeRight(strafeRightDistance)
                .build();

        drive.followTrajectory(strafeRightTrajectory);

    }

    public void strafeLeft(double strafeLeftDis){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        strafeLeftDistance=strafeLeftDis;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 25 inches
        Trajectory strafeLeftTrajectory = drive.trajectoryBuilder(startPose)
                .strafeLeft(strafeLeftDistance)
                .build();

        drive.followTrajectory(strafeLeftTrajectory);

    }

    public void InitServo() {
        LeftArm.setPosition(0.02);
        RightArm.setPosition(0.02);
        Adjust.setPosition(0.04);
        RightGrabber.setPosition(0.8);
        LeftGrabber.setPosition(0.8);

    }

    public void ArmDropping(double LA_POS, double RA_POS) {

        double  LA = LA_POS;
        double RA = RA_POS;

        LeftArm.setPosition(LA);
        RightArm.setPosition(RA);

    }

    public void PurplePixelDrop(double RG_POS) {

        double  RG = RG_POS;

        RightGrabber.setPosition(RG);
    }

    public void PurplePixelclose(){

        RightGrabber.setPosition(0.8);
    }
    public void sliderUp(){
        slider.setPower(0.5);
    }
    public void sliderDown(){
        slider.setPower(-0.3);
    }
    public void sliderZero(){
        slider.setPower(0);
    }

    public void adjustDrop(){
        Adjust.setPosition(0.115);
    }
    public void YellowPixelDrop(double LG_POS) {

        double  LG = LG_POS;


        LeftGrabber.setPosition(LG);


    }

    public void YellowArmClose(){

        LeftGrabber.setPosition(0.8);
    }

    public void BlueCenterDrop(){

        InitServo();
        sleep(500);
        backward(42);
        PurplePixelDrop(0.5);
        sleep(500);
        backward(9.5);
        PurplePixelclose();
        turnLeft(103.2);
        backward(80);
        turnLeft(104);
        backward(23);
        turnRight(106);
        sliderUp();
        sleep(600);
        sliderZero();
        ArmDropping(0.8,0.8);
        sleep(1500);
        adjustDrop();
        sleep(1000);
        backward(9.0);
        YellowPixelDrop(0);
        sleep(500);
        YellowArmClose();
        forward(3.0);
        sleep(500);
        InitServo();
        sleep(500);
        sliderDown();
        sleep(600);

    }

    public void BlueleftDrop(){

        InitServo();
        sleep(500);
        backward(26);
        turnRight(103.2);
        forward(8);
        PurplePixelDrop(0.5);
        sleep(500);
        backward(8);
        turnLeft(103.2);
        backward(23.5);
        PurplePixelclose();
        turnLeft(103.2);
        backward(80);
        turnLeft(104);
        backward(30.5);
        turnRight(108);
        sliderUp();
        sleep(600);
        sliderZero();
        ArmDropping(0.8,0.8);
        sleep(1000);
        adjustDrop();
        sleep(500);
        backward(11.0);
        YellowPixelDrop(0);
        sleep(500);
        YellowArmClose();
        forward(3.0);
        sleep(500);
        InitServo();
        sleep(500);
        sliderDown();
        sleep(600);

    }

    public void BluerightDrop(){

        InitServo();
        sleep(500);
        backward(29);
        turnLeft(103.2);
        forward(8);
        PurplePixelDrop(0.5);
        sleep(500);
        backward(8);
        turnRight(103.2);
        backward(20.5);
        PurplePixelclose();
        turnLeft(103.2);
        backward(80);
        turnLeft(104);
        backward(15.5);
        turnRight(108);
        sliderUp();
        sleep(600);
        sliderZero();
        ArmDropping(0.8,0.8);
        sleep(1000);
        adjustDrop();
        sleep(500);
        backward(11.0);
        YellowPixelDrop(0);
        sleep(500);
        YellowArmClose();
        forward(3.0);
        sleep(500);
        InitServo();
        sleep(500);
        sliderDown();
        sleep(600);

    }
    public void BlueShortCenter() {

        InitServo();
        sleep(500);
        forward(33);
        PurplePixelDrop(0.5);
        sleep(500);
        backward(8);
        PurplePixelclose();
        turnRight(103.2);
        backward(28.5);
        sliderUp();
        sleep(650);
        sliderZero();
        ArmDropping(0.9,0.9);
        sleep(1500);
        adjustDrop();
        sleep(1000);
        backward(10);
        sleep(2000);
        YellowPixelDrop(0);
        sleep(1000);
        YellowArmClose();
        forward(4.0);
        sleep(1000);
    }

    public void BlueShortleft() {

        InitServo();
        forward(30);
        turnRight(103.2);
        backward(16.0);
        PurplePixelDrop(0.5);
        sleep(1000);
        PurplePixelclose();
        backward(14);
        sliderUp();
        sleep(650);
        sliderZero();
        ArmDropping(0.9,0.9);
        sleep(1500);
        adjustDrop();
        sleep(1000);
        strafeRight(11.0);
        backward(10);
        sleep(2000);
        YellowPixelDrop(0);
        sleep(1000);
        YellowArmClose();
        forward(4.0);
        sleep(1000);

    }

    public void BlueShortRight(){

        InitServo();
        forward(29);
        turnRight(103.2);
        forward(7.0);
        PurplePixelDrop(0.5);
        sleep(1000);
        PurplePixelclose();
        backward(33.5);
        sliderUp();
        sleep(650);
        sliderZero();
        ArmDropping(0.9,0.9);
        sleep(1500);
        adjustDrop();
        sleep(1000);
        strafeLeft(5);
        backward(11.0);
        sleep(1500);
        YellowPixelDrop(0);
        sleep(1000);
        YellowArmClose();
        forward(4.0);
        sleep(1000);

    }

    public void RedShortCenter(){
        InitServo();
        sleep(500);
        backward(30);
        turnRight(202.5);
        forward(3.5);
        PurplePixelDrop(0.5);
        sleep(500);
        backward(7.5);
        PurplePixelclose();
        turnLeft(103.2);
        backward(32.5);
      strafeRight(4.0);
        sliderUp();
        sleep(650);
        sliderZero();
        ArmDropping(0.85,0.85);
        sleep(1500);
        adjustDrop();
        sleep(1000);
        backward(10);
        sleep(2000);
        YellowPixelDrop(0);
        sleep(1000);
        forward(3.0);
        sleep(1000);
        YellowArmClose();

    }

    public void RedShortleft(){
        InitServo();
        sleep(500);
        backward(26.5);
        turnRight(103.2);
        forward(6.5);
        PurplePixelDrop(0.5);
        sleep(500);
        PurplePixelclose();
        backward(38.0);
        strafeRight(7);

        sliderUp();
        sleep(650);
        sliderZero();
        ArmDropping(0.85,0.85);
        sleep(1500);
        adjustDrop();
        sleep(1000);
        backward(10.5);
        sleep(2000);
        YellowPixelDrop(0);
        sleep(1000);

        forward(4.0);

        sleep(1000);
        YellowArmClose();

    }

    public void RedShortright(){
        InitServo();
        sleep(500);
        backward(26.5);
        turnRight(103.2);
        backward(18.5);
        PurplePixelDrop(0.5);
        sleep(500);
        PurplePixelclose();
        backward(14.5);
        strafeLeft(8);
        sliderUp();
        sleep(650);
        sliderZero();
        ArmDropping(0.85,0.85);
        sleep(1500);
        adjustDrop();
        sleep(1000);
        backward(10.5);
        sleep(2000);
        YellowPixelDrop(0);
        sleep(1000);
        forward(4.0);
        sleep(1000);
        YellowArmClose();

    }

    public void RedCenterDrop(){

        InitServo();
        sleep(500);
        backward(42);
        PurplePixelDrop(0.5);
        sleep(500);
        backward(8);
        PurplePixelclose();
        turnRight(103.2);
        backward(80);
        turnRight(104);
        backward(24);
        turnLeft(106);
        sliderUp();
        sleep(600);
        sliderZero();
        ArmDropping(0.85,0.85);
        sleep(1500);
        adjustDrop();
        sleep(1000);
        backward(9.0);
        YellowPixelDrop(0);
        sleep(650);
      //  forward(3.0);
        sleep(500);
        YellowArmClose();
    }

    public void RedrightDrop(){

        InitServo();
        sleep(500);
        backward(26.5);
        turnLeft(103.2);
        forward(7);
        PurplePixelDrop(0.5);
        sleep(500);
        backward(7.5);
        turnRight(103.2);
        backward(24);
        PurplePixelclose();
        turnRight(103.2);
        backward(80);
        turnRight(103.2);
        backward(31);
        turnLeft(103);
        sliderUp();
        sleep(600);
        sliderZero();
        ArmDropping(0.85,0.85);
        sleep(1000);
        adjustDrop();
        sleep(500);
        backward(10);
        YellowPixelDrop(0);
        sleep(500);
        forward(3.0);
        sleep(500);
        YellowArmClose();
        sleep(500);

    }

    public void RedleftDrop(){

        InitServo();
        sleep(500);
        backward(30);
        turnRight(103.2);
        forward(6.5);
        PurplePixelDrop(0.5);
        sleep(500);
        backward(7.5);
        turnLeft(103.2);
        backward(21);
        PurplePixelclose();
        turnRight(103.2);
        backward(80);
        turnRight(103.2);
        backward(19.5);
        turnLeft(103.2);
        sliderUp();
        sleep(600);
        sliderZero();
        ArmDropping(0.85,0.85);
        sleep(1000);
        adjustDrop();
        sleep(500);
        backward(7.5);
        YellowPixelDrop(0);
        sleep(1000);
        YellowArmClose();
        forward(3.0);
        sleep(500);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        slider= hardwareMap.get(DcMotorEx.class,"lifter");
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightGrabber = hardwareMap.get(Servo.class, "FG");
        LeftGrabber = hardwareMap.get(Servo.class, "BG");
        LeftArm = hardwareMap.get(Servo.class, "LA");
        RightArm = hardwareMap.get(Servo.class, "RA");
        Adjust = hardwareMap.get(Servo.class, "adjust");

        LeftArm.setDirection(Servo.Direction.REVERSE);
        RightGrabber.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        if(isStopRequested()){
            return;
        }

        RedShortright();
    }
}

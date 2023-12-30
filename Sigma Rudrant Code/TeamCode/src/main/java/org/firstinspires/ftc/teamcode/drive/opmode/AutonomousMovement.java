package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousMovement", group = "Autonomous")
public class AutonomousMovement extends LinearOpMode {

    private Encoder leftEncoder;

    private double forwardDistance=0;

    private double backwardDistance=0;

    private double rightAngle=0;

    private double leftAngle=0;

    private double strafeRightDistance=0;

    private double strafeLeftDistance=0;

    private double countsPerInch=1917.0;


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

    public void variableForward(double distance){

         double accelerationRate=0.01;

         double decelerationRate=0.04;

         double maxSpeed=0.65;

         double minSpeed=0.2;

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        leftEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class, "left_back"));

        forwardDistance=distance;

        double targetEncoderValue= forwardDistance*countsPerInch;
        double currentEncoderValue=leftEncoder.getCurrentPosition();
        double currentSpeed=0.0;

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(currentEncoderValue<targetEncoderValue){

            if (currentEncoderValue < targetEncoderValue*0.2) {
                // Accelerate
                currentSpeed += accelerationRate;
            } else if ((currentEncoderValue > (targetEncoderValue*0.2)) && (currentEncoderValue < (targetEncoderValue-(targetEncoderValue*0.2)))) {
                // Maintain constant velocity
                    currentSpeed = maxSpeed;
            } else {
                // Decelerate
                if(currentEncoderValue>targetEncoderValue){
                    currentSpeed=0;
                }else if(currentSpeed<0.0 && currentEncoderValue<targetEncoderValue){
                    currentSpeed=minSpeed;
                }else {
                    currentSpeed -= decelerationRate;
                }
            }
            // Limit the speed to the maximum
            if (currentSpeed > maxSpeed) {
                currentSpeed = maxSpeed;
            }

            drive.setMotorPowers(currentSpeed,currentSpeed,currentSpeed,currentSpeed);

            currentEncoderValue=leftEncoder.getCurrentPosition();
        }

        drive.setMotorPowers(0,0,0,0);
        return;

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
                .strafeRight(strafeLeftDistance)
                .build();

        drive.followTrajectory(strafeLeftTrajectory);

    }

    public void centerDrop(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.InitServo();
        sleep(500);


        backward(42);
        auto.PurplePixelDrop(0.5);
        sleep(1000);
        ;

        backward(9.5);
        auto.PurplePixelclose();
        turnLeft(103.2);
        backward(80);
        turnLeft(104);
        backward(23);
        turnRight(106);
        auto.sliderUp();
        sleep(600);
        auto.sliderZero();
        auto.ArmDropping(0.8,0.8);
        sleep(2000);
        auto.adjustDrop();
        sleep(2000);
        backward(9.0);
        auto.YellowPixelDrop(0);
        sleep(1000);
        forward(3.0);
        auto.InitServo();
        sleep(2000);

    }

    public void leftDrop(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.InitServo();
        sleep(500);


        backward(26);
        turnRight(103.2);
        forward(8);
        auto.PurplePixelDrop(0.5);
        sleep(1000);
        backward(8);
        turnLeft(103.2);
        backward(23.5);
        auto.PurplePixelclose();
        turnLeft(103.2);
        backward(80);
        turnLeft(104);
        backward(30.5);
        turnRight(108);
        auto.sliderUp();
        sleep(600);
        auto.sliderZero();
        auto.ArmDropping(0.8,0.8);
        sleep(2000);
        auto.adjustDrop();
        sleep(2000);
        backward(11.0);
        auto.YellowPixelDrop(0);
        sleep(1000);
        forward(3.0);
        auto.InitServo();

    }

    public void rightDrop(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.InitServo();
        sleep(500);


        backward(29);
        turnLeft(103.2);
        forward(8);
        auto.PurplePixelDrop(0.5);
        sleep(1000);
        backward(8);
        turnRight(103.2);
        backward(20.5);
        auto.PurplePixelclose();
        turnLeft(103.2);
        backward(80);
        turnLeft(104);
        backward(15.5);
        turnRight(108);
        auto.sliderUp();
        sleep(600);
        auto.sliderZero();
        auto.ArmDropping(0.8,0.8);
        sleep(2000);
        auto.adjustDrop();
        sleep(2000);
        backward(11.0);
        auto.YellowPixelDrop(0);
        sleep(1000);
        forward(3.0);
        auto.InitServo();

    }

    public void ShortCenter() {

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);
        auto.InitServo();
        forward(33);
        auto.PurplePixelDrop(0.5);
        sleep(1000);
        backward(10);
        auto.PurplePixelclose();
        turnRight(103.2);
        backward(28.5);
        auto.sliderUp();
        sleep(600);
        auto.sliderZero();
        auto.ArmDropping(0.8,0.8);
        sleep(2000);
        auto.adjustDrop();
        sleep(2000);
        backward(9.3876);
        auto.YellowPixelDrop(0);
        sleep(1000);
        forward(3.0);
        auto.InitServo();
        backward(3);


    }

    public void Shortleftdrop() {

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);
        auto.InitServo();
        forward(30);
        turnRight(103.2);
        backward(15.5);
        auto.PurplePixelDrop(0.5);
        sleep(1000);
        auto.PurplePixelclose();
        backward(14);
        auto.sliderUp();
        sleep(600);
        auto.sliderZero();
        auto.ArmDropping(0.8,0.8);
        sleep(2000);
        auto.adjustDrop();
        sleep(2000);
        backward(9.3876);
        auto.YellowPixelDrop(0);
        sleep(1000);


    }

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        if(isStopRequested()){
            return;
        }
        Shortleftdrop();
    }
}

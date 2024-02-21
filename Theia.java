package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.CSVisionProcessor;
import org.firstinspires.ftc.teamcode.CSVisionProcessor.*;

import static org.firstinspires.ftc.teamcode.CSVisionProcessor.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class Theia extends LinearOpMode {

    public boolean RED = true, BACK = true; //RED is for red side or blue side, BACK is for front stage or backstage.

    private CSVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    private DcMotor frm, flm, brm, blm, arm1, arm2;
    private Servo rightclaw, leftclaw;
    private double pwr = 0.6;

    @Override
    public void runOpMode() {

        visionProcessor = new CSVisionProcessor(120, 5, 322, 260, 282, 500, 342);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
		
        frm = hardwareMap.get(DcMotor.class, "FRM");
        flm = hardwareMap.get(DcMotor.class, "FLM");
        brm = hardwareMap.get(DcMotor.class, "BRM");
        blm = hardwareMap.get(DcMotor.class, "BLM");
        arm1 = hardwareMap.get(DcMotor.class, "LM");
        arm2 = hardwareMap.get(DcMotor.class, "RM");
        rightclaw = hardwareMap.get(Servo.class, "FRS");
        leftclaw = hardwareMap.get(Servo.class, "FLS");

        CSVisionProcessor.StartingPosition startingPos = CSVisionProcessor.StartingPosition.NONE;

		//start by grabbing
		rightclaw.setPosition(1.0);
		leftclaw.setPosition(1.0);
		//grabLeft();
		//grabRight();
		
        //waitForStart();

		boolean finishedSettingLocation = false;
        while (!finishedSettingLocation) {
            if (gamepad1.a) {
                telemetry.addLine("confirmed starting location");
                finishedSettingLocation = true;
            } else if (gamepad1.dpad_down) {
                telemetry.addLine("pos set to red back");
                RED = true;
				BACK = false;
            } else if (gamepad1.dpad_right) {
                telemetry.addLine("pos set to red front");
                RED = true;
				BACK = true;
            } else if (gamepad1.dpad_up) {
                telemetry.addLine("pos set to blue back");
                RED = false;
				BACK = false;
            } else if (gamepad1.dpad_left) {
                telemetry.addLine("pos set to blue front");
                RED = false;
				BACK = true;
            }
        }
		
		// find starting position of the prop/pixel [based on saturation]
        while(!this.isStarted() && !this.isStopRequested()) {
            startingPos = visionProcessor.getStartingPosition();
            telemetry.addData("Identified", visionProcessor.getStartingPosition());
            telemetry.update();
        }

        visionPortal.stopStreaming();

        // run all tasks as long as op mode is active
		
		moveRight(200); //move up to the spike tape area
        if(!opModeIsActive()) return;
		
		switch(startingPos) {
		case LEFT:
			moveUp(200); //move to left tape
			if(!opModeIsActive()) return;
			
			releaseLeft(1000); //release purple pixel (left claw)
			if(!opModeIsActive()) return;
			
			moveDown(200); //move back
			break;
		case RIGHT:
			moveDown(200); //move to right tape
			if(!opModeIsActive()) return;
			
			releaseLeft(1000); //release purple pixel (left claw)
			if(!opModeIsActive()) return;
			
			moveUp(200); //move back
			break;
		case CENTER:
			moveRight(200); //move to center tape
			if(!opModeIsActive()) return;
			
			releaseLeft(1000); //release purple pixel (left claw)
			if(!opModeIsActive()) return;
			
			moveLeft(200); //move back
			break;
		default:
			break;
		}
        if(!opModeIsActive()) return;
		
		
		if(!BACK) {
			//move???(???); // possible adjustment location
			if(!opModeIsActive()) return;
			moveUp(1000); //move to other set of tapes to be consistent
			if(!opModeIsActive()) return;
			//move???(???); // possible adjustment location
			if(!opModeIsActive()) return;
		} else {
			//move???(???); // possible adjustment location
			if(!opModeIsActive()) return;
		}
		
		moveUp(2000); //move to the backthing
		if(!opModeIsActive()) return;
		
		switch(startingPos) {
		case LEFT:
			moveLeft(50); //move to left tape position
			if(!opModeIsActive()) return;
			
			releaseRight(1000); //release yellow pixel (right claw)
			if(!opModeIsActive()) return;
			
			moveRight(50); //move back
			break;
		case RIGHT:
			moveRight(350); //move to right tape position
			if(!opModeIsActive()) return;
			
			releaseRight(1000); //release yellow pixel (right claw)
			if(!opModeIsActive()) return;
			
			moveLeft(350); //move back
			break;
		case CENTER:
			moveRight(150); //move to center tape position
			if(!opModeIsActive()) return;
			
			releaseRight(1000); //release yellow pixel (right claw)
			if(!opModeIsActive()) return;
			
			moveLeft(150); //move back
			break;
		default:
			break;
		}
        if(!opModeIsActive()) return;
		
		moveUp(3500); //move to the front stack of pixels
		if(!opModeIsActive()) return;
		
		moveRight(50); //align
		if(!opModeIsActive()) return;
		
		grabBoth(1000); //grab as much of stack as possible
		if(!opModeIsActive()) return;
		
		moveLeft(50); //align
		if(!opModeIsActive()) return;
		
		moveDown(3500); //move to the backthingy
		if(!opModeIsActive()) return;
		
		releaseBoth(1000); //release as much of stack as possible
		if(!opModeIsActive()) return;
		
		moveUp(100); //already in parking but just like move off a bit yk
		if(!opModeIsActive()) return;
		
    }


	//these moves are based on the robot facing the front stage, ie [front] ._/-. [back] [robot with claw down facing front stage], the right and left are automatically swapped, so you pick the one
	//as if you were on the red/right side, and it will just be flipped if you are on the blue side
	
	public void moveRight(long time) {
		move(RED?1:-1, 0, 0, time);
        telemetry.update();
	}

	public void moveLeft(long time) {
		move(RED?-1:1, 0, 0, time);
        telemetry.update();
	}

	public void moveUp(long time) {
		move(0, 1, 0, time);
        telemetry.update();
	}

	public void moveDown(long time) {
		move(0, -1, 0, time);
        telemetry.update();
	}

	private void move(double dx, double dy, double da, long time) {
        dx=dx;
        dy=dy;
        da=da;
        frm.setPower((+dx-dy-da)*pwr);
        flm.setPower((+dx+dy-da)*pwr);
        brm.setPower((-dx-dy-da)*pwr);
        blm.setPower((-dx+dy-da)*pwr);
		sleep(time);
		halt();
    }
	
	private void halt() {
        frm.setPower(0);
        flm.setPower(0);
        brm.setPower(0);
        blm.setPower(0);
	}

	public void grabBoth(long time) {
		arm1.setPower(-0.4);
		arm2.setPower(0.4);
		sleep((long)(time*0.375));
		arm1.setPower(0);
		arm2.setPower(0);
		rightclaw.setPosition(1.0);
		leftclaw.setPosition(1.0);
		sleep((long)(time*0.25));
		arm1.setPower(0.4);
		arm2.setPower(-0.4);
		sleep((long)(time*0.375));
		arm1.setPower(0);
		arm2.setPower(0);
        telemetry.update();
	}

	public void grabLeft(long time) {
		arm1.setPower(-0.4);
		arm2.setPower(0.4);
		sleep((long)(time*0.375));
		arm1.setPower(0);
		arm2.setPower(0);
		leftclaw.setPosition(1.0);
		sleep((long)(time*0.25));
		arm1.setPower(0.4);
		arm2.setPower(-0.4);
		sleep((long)(time*0.375));
		arm1.setPower(0);
		arm2.setPower(0);
        telemetry.update();
	}
	
	public void releaseBoth(long time) {
		arm1.setPower(-0.4);
		arm2.setPower(0.4);
		sleep((long)(time*0.375));
		arm1.setPower(0);
		arm2.setPower(0);
		rightclaw.setPosition(0.0);
		leftclaw.setPosition(0.0);
		sleep((long)(time*0.25));
		arm1.setPower(0.5);
		arm2.setPower(-0.4);
		sleep((long)(time*0.375));
		arm1.setPower(0);
		arm2.setPower(0);
        telemetry.update();
	}
	
	public void releaseRight(long time) {
		arm1.setPower(-0.4);
		arm2.setPower(0.4);
		sleep((long)(time*0.375));
		arm1.setPower(0);
		arm2.setPower(0);
		rightclaw.setPosition(0.0);
		sleep((long)(time*0.25));
		arm1.setPower(0.4);
		arm2.setPower(-0.4);
		sleep((long)(time*0.375));
		arm1.setPower(0);
		arm2.setPower(0);
        telemetry.update();
	}
	
	public void releaseLeft(long time) {
		arm1.setPower(-0.4);
		arm2.setPower(0.4);
		sleep((long)(time*0.375));
		arm1.setPower(0);
		arm2.setPower(0);
		leftclaw.setPosition(0.0);
		sleep((long)(time*0.25));
		arm1.setPower(0.4);
		arm2.setPower(-0.4);
		sleep((long)(time*0.375));
		arm1.setPower(0);
		arm2.setPower(0);
        telemetry.update();
	}


}

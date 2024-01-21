    package com.ftc11392.sequoia;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.ColorSensor;

    @Autonomous

    //THIS IS FOR THE RAMP ONE NOT THE CLAW ONE
    public class AutoVanBismarck extends OpMode {

        final boolean red = true; //true: red side, false: blue side
        final boolean storage = false; //false: storage side, true: warehouse side

        /*
        TO-DO
        Update the position somehow?? lmao
        Calibrate the movement so that what happens is right lmao
        Copy and adjust for the other one lmao
        Make all the position values accurate to the field lmfao
        Any other random constants that i forgot exist make sure are accurate lmao

        AUTONOMOUS-TOTAL (60 pts)

        AUTONOMOUS-RAMP (18 pts)
        Note: use duck here
        1) Put in preloaded freight box. (6 pts)
            1.05) Lift the lift to appropriate level
            1.1) Drive up to shipping hub
            1.2) Move ramp servo, and wait sufficiently
        2) Travel to storage
            2.1) Back up
            2.2) Rotate facing it
            2.3) Move straight to it
        3) Pick up freight and drop off (2 pts)
            3.1) Start intake, while moving forward slowly - to ensure its picked up
            3.2) Lift, while moving backwards somewhat slowly - to not drop
            3.3) Move backwards to storage unit
            3.4) Move ramp servo, and wait sufficiently
        4) Park completely in nearest storage unit (10 pts)
            4.1) Move forwards
         */

        public double dx_,dy_,da_;

        public DcMotor frm, flm, brm, blm, intake, lift, duck;
        // to interact with specific motors
        public DcMotor[] motors;
        // this array is used to cycle through all motors to do checks on their tasks
        public boolean[] motorstates = new boolean[] {false,false,false,false,false,false,false};
        // this array is used to keep track of what motors had a task
        public CRServo topservo, bottomservo;
        // public ElapsedTime runTime;
        public ColorSensor left;

        double lastTime;
        // used for things that run based on time, is -1 when not in use
        int state, completed, specialCompleted;
        // state is used for the state of the robot, and completed is how many motors
        //  have completed their tasks, specialCompleted is the same but only for
        //  special motors, just so they can be progressed seperately.
        //boolean started = false;
        // this is used to run some init code if the robot has not started
        boolean targeting = false, specialTargeting = false, stating = false;
        // these is used to keep track of whether there are movements running
        // stating is used to start the state movement
        double x, y, a, xt, yt, at;
        // x and y coordinates are from 0 to 1000 each,
        //  with (0,0) being blue (left) carosel/alliance storage (back)
        //  and (1000,1000) being red (right) freight area (front)
        // a is degrees including 0 and up to 360,
        //  with 0 being facing the higher y value (front)
        //  and 90 being facing the higher x value (right)
        // "position" in code is used to refer to x y and a, whereas "coordinate" is used
        // just for x and y, this is just done out of ease

        // here xf yf af are all in the coordinate system units of 0 to 1000
        // although it can be inputted, there should never both be coordinate movement,
        // and turning, as this currently would mess up the calculations
        public void goTo(double xf, double yf, double af, double pwr) {
            // setting globals to reflect
            xt=xf;
            yt=yf;
            at=af;
            targeting=true;
            stating=true;

            // decode coordinates into movement
            double dx = xt-x;
            double dy = yt-y;
            double da = at-a;
            telemetry.addData("at",at);
            telemetry.addData("a",a);
            telemetry.addData("dahere",da);
            if(da>180)da-=360;
            if(da<-180)da+=360;
            double mag = Math.sqrt(Math.pow(dx,2)+Math.pow(dy,2));// normalise the
            // coordinate vectors that are passed into moveAbsolute as arguments
            double aMag = Math.abs(da);// angle calculation here is a bit crude,
            // it will end up with just a value of +1 or -1
            if(mag==0)mag=1;//prevent div by 0
            if(aMag==0)aMag=1;//prevent div by 0
            int ticks = (int)(Math.round(10*mag+0.2*aMag));
            // the 10 and 0.2 are obviously gonna need to be finalised, but this just
            // converts the coordinates to ticks, this part is probably the worst,
            // the angle and coordinate changes shouldn't add, but those shouldn't be
            // done parralel anyway, the real downfall is that all motors get the same
            // ticks, which will probably cause downfall later, i think it should be
            // equal to abs(their power * ticks) potentially, so if some motors stop
            // before others and some continue longer than they should, then try this

            // motor encoder stuff;
            brm.setTargetPosition(ticks);
            frm.setTargetPosition(ticks);
            blm.setTargetPosition(ticks);
            flm.setTargetPosition(ticks);
            brm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            blm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // moveAbsolute will get the powers set accordingly
            moveAbsolute(dx/mag,dy/mag, da/aMag, pwr);
            for(int i=0;i<4;i++)motorstates[i]=true;
        }

        // just takes in ticks, and moves that motor that amout of ticks, meant
        // for the auxiliary motors
        public void specialGoTo(int motor, int specialF, double pwr) {
            specialTargeting=true;
            stating=true;
            motors[motor].setTargetPosition(specialF);
            motors[motor].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[motor].setPower(pwr);
            motorstates[motor]=true;
        }

        // dx dy da do not refer to any specific units, just the ratio of motor powers
        // its relative to the actual field position, so it will convert absolute movement
        // to relative movement based on the current angle of the robot
        // the coordinates of the robot do not matter as it deals only in change of pos
        // the dy variable is split into two components, a dy*-sin(a) for the relative dx,
        // and a dy*cos(a) for the relative dy
        // the dx variable is split into two components, a dx*cos(a) for the relative dx,
        // and a dx*sin(a) for the relative dy
        // the angle doesn't need to be altered as its just the change of angle, not
        // setting to a specific angle
        // the pwr is obviously also kept the same
        public void moveAbsolute(double dx, double dy, double da, double pwr) {
            moveRelative(dx*Math.cos(Math.toRadians(a))+dy*-Math.sin(Math.toRadians(a)),dx*Math.sin(Math.toRadians(a))+dy*Math.cos(Math.toRadians(a)),da,pwr);
        }

        // dx dy da do not refer to any specific units, just the ratio of motor powers
        // its relative to robot's current position and orientation
        //  so forward is 0 degrees and positive y direction
        //  and right is 90 degrees and positive x direction
        public void moveRelative(double dx, double dy, double da, double pwr) {
            dx_=dx;
            dy_=dy;
            da_=da;
            frm.setPower((+dx-dy-da)*pwr);
            flm.setPower((+dx+dy-da)*pwr);
            brm.setPower((-dx-dy-da)*pwr);
            blm.setPower((-dx+dy-da)*pwr);
        }

        public void updatePosition() {
            x=xt;
            y=yt;
            a=at;
            DcMotor motor;
            for(int i=0;i<4;i++) {
                motor = motors[i];
                motorstates[i] = false;// stop
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// stop
                motor.setPower(0);// stop pt3
            }
            // obviously the above is just temporary ^^

            // do the actual stuff somehow, but tbh might just be too much

            // if a way to get motors position then it'd probably be helpful to do that
            // to monitor the specialGoTo() motor
        }

        // made for drive train running motors only
        public void progressUpdate(int targetCompleted, int nextStage) {
            // check completed and if so progress to next state

            telemetry.addData("PUtargeting",targeting);
            telemetry.addData("PUcompleted",completed);
            if(targeting&&completed==targetCompleted) {
                targeting=false;
                stating=false;
                completed=0;
                updatePosition();
                // thresholds in need of adjustment
                // if not fully at that target then redo it
                // otherwise continue to next stage
                state=nextStage;
    //            if(Math.abs(x-xt)>5||Math.abs(y-yt)>5||Math.abs(a-at)>5) goTo(xt,yt,at,0.4);
    //            else state=nextStage;
            }
        }

        // made for specially running motors only
        public void specificProgressUpdate(int targetCompleted, int nextStage) {
            // check completed and if so progress to next state
            if(specialTargeting&&specialCompleted==targetCompleted) {
                specialTargeting=false;
                stating=false;
                specialCompleted=0;
                updatePosition();
                // goes to next stage, maybe include a threshold redo like the other
                // if i can get the position thing, but probs not
                state=nextStage;
            }
        }

        @Override
        public void init() {
            frm = hardwareMap.get(DcMotor.class, "FRM");
            flm = hardwareMap.get(DcMotor.class, "FLM");
            brm = hardwareMap.get(DcMotor.class, "BRM");
            blm = hardwareMap.get(DcMotor.class, "BLM");
            intake = hardwareMap.get(DcMotor.class, "intake");
            lift = hardwareMap.get(DcMotor.class, "lift");
            duck = hardwareMap.get(DcMotor.class, "table turner");
            topservo = hardwareMap.crservo.get("top servo");
            bottomservo = hardwareMap.crservo.get("bottom servo");
            left = hardwareMap.get(ColorSensor.class, "left");
            brm.setDirection(DcMotorSimple.Direction.FORWARD);
            frm.setDirection(DcMotorSimple.Direction.FORWARD);
            blm.setDirection(DcMotorSimple.Direction.REVERSE);//incase it doesnt work, you changed it here
            flm.setDirection(DcMotorSimple.Direction.REVERSE); // here too
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            duck.setDirection(DcMotorSimple.Direction.FORWARD);
            topservo.setDirection(CRServo.Direction.FORWARD);
            bottomservo.setDirection(CRServo.Direction.FORWARD);
            brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            duck.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors = new DcMotor[] {frm,flm,brm,flm,intake,lift,duck};
            // defining array after values are initialised to be safe, although probably
            // not needed
            // the dummy coordinate values below need to be finalised when on the field obv
            // the angle values are inverted as it needs to deliver before intake
            // this is not the case in the other robot
            if(red) {
                if(storage){x=700;y=200;a=90;}
                else{x=700;y=500;a=90;}
            } else {
                if(storage){x=300;y=200;a=270;}
                else{x=300;y=500;a=270;}
            }
            xt=x;
            yt=y;
            at=a;
            state=110;
            completed=0;
            specialCompleted=0;
            lastTime=-1;
        }

        private boolean isObjectDetected() {
            return left.green()+left.alpha()>1000;
        }
        // check if this is actually run in a loop, or if i have to make the loop inside of it
        // i would assume it is run in a loop, but yeah
        @Override
        public void loop() {
            telemetry.addData("dx", dx_);
            telemetry.addData("dy", dy_);
            telemetry.addData("da", da_);
            telemetry.addData("red",left.red());
            telemetry.addData("green",left.green());
            telemetry.addData("blue",left.blue());
            telemetry.addData("He-4 nucleus",left.alpha());
            telemetry.addData("state",state);
            if(isObjectDetected())telemetry.addData("object","yes");
            else telemetry.addData("object","no");
            DcMotor motor;
            for(int i=0;i<motors.length;i++) {
                motor = motors[i];
                if(motorstates[i]&&!motor.isBusy()) { // if it thinks its busy, but is no longer
                    motorstates[i]=false;// stop
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// stop
                    motor.setPower(0);// stop pt3
                    // feedback into the state functions
                    if((i>3||!targeting)&&specialTargeting)specialCompleted+=1;
                    else completed+=1;
                }
            }
            // run start procedure
            //if(!started){started=true;}
            // divert to correct function depending on the stage
            switch(state) {
                case 105:
                    f105();
                    break;
                case 110:
                    f110();
                    break;
                case 120:
                    f120();
                    break;
                case 210:
                    f210();
                    break;
                case 220:
                    f220();
                    break;
                case 230:
                    f230();
                    break;
                case 310:
                    f310();
                    break;
                case 320:
                    f320();
                    break;
                case 330:
                    f330();
                    break;
                case 340:
                    f340();
                    break;
                case 410:
                    f410();
                    break;
                case -1:
                    requestOpModeStop();
                    return;
            }
            // constrain angle to a value of 0<=a<360
            if(a>=360)a-=360;
            else if(a<0)a+=360;
            // code below is kept incase it is needed later, but for now is not needed
            /*
            if(!targeting)return;
            //calculate difference from actual position to target position
            double dx = xt-x;
            double dy = yt-y;
            double da = at-a;
            if(da>180)da-=360;// doesn't turn the long way around
            else if(da<-180)da+=360;// the other long way around
            // if it is all close
            if(Math.abs(dx)<5&&Math.abs(dy)<5&&Math.abs(da)<3) {
                //then stop targetting
                targeting = false;
                stop();
                //x=getActualX();
                //y=getActualY();
                //a=getActualA();
                return;
            }
            */
        }
    /*
            // starting coordinates for reference:
            if(red) {
                if(storage){x=700;y=200;a=90;}
                else{x=700;y=500;a=90;}
            } else {
                if(storage){x=300;y=200;a=270;}
                else{x=300;y=500;a=270;}
            }
     */
        // 1.05) Lift the lift to appropriate level
        public void f105() {
            //start moving
            if(!stating) specialGoTo(5,500,0.8);
            //next state
            specificProgressUpdate(1,110);
        }

        // 1.1) Drive up to shipping hub
        public void f110() {
            //if(left.green()>100) state = 120;
            //start moving
            if(!stating) {
                int xf = 400;
                int yf = 500;
                int af = 270;
                if(red){xf=600;af=90;}
                if(storage)yf=200;
                goTo(xf,yf,af,0.6);
            }
            //next state
            progressUpdate(3,120);
        }

        // 1.2) Move ramp servo, and wait sufficiently
        public void f120() {
            //start moving
            //next state
            specificProgressUpdate(1,210);
            //start moving top servo
            if(lastTime==-1){
                lastTime=getRuntime();
                topservo.setPower(-1);

            }
            // after 0.5s accurate to the nanosecond reverse top servo and next state
            else if(getRuntime()-lastTime>1.5) {
                lastTime=getRuntime();
                topservo.setPower(1);
                state=210;
            }
        }

        // 2.1) Back up
        public void f210() {
            //check the previous returning
            if(lastTime>-1&&getRuntime()-lastTime>0.5) {
                lastTime = -1;
                topservo.setPower(0);
            }
            //start moving
            if(!stating) {
                int xf = 300;
                int yf = 500;
                int af = 270;
                if(red) {xf = 700;af=90;}
                if(storage) yf = 200;
                goTo(xf,yf,af,0.6);
            }
            //next state
            progressUpdate(3,230); //skip rotate
        }

        // 2.2) Rotate facing it
        public void f220() {
            //check the previous returning
            if(lastTime>-1&&getRuntime()-lastTime>0.5) {
                lastTime = -1;
                topservo.setPower(0);
            }
            //start moving
            if(!stating) {
                int xf = 300;
                int yf = 500;
                int af = 180;
                if(red) xf = 700;
                if(storage) yf = 200;
                goTo(xf,yf,af,0.6);
            }
            //next state
            progressUpdate(3,230);
        }

        // 2.3) Move straight to it
        public void f230() {
            //check the previous returning
            if(lastTime>-1&&getRuntime()-lastTime>0.5) {
                lastTime = -1;
                topservo.setPower(0);
            }
            //start moving
            if(!stating) {
                int xf = 300;
                int yf = 800;
                int af = 270;
                if(red) {xf = 700;af=90;}
                if(red) xf = 700;
                goTo(xf,yf,af,0.9);
            }
            //next state
            progressUpdate(3,410);//skip 3
        }

        // 3.1) Start intake, while moving forward slowly - to ensure its picked up
        public void f310() {
            //start moving
            if(!stating) {
                int xf = 300;
                int yf = 820;
                int af = 270;
                if(red) xf = 700;
                goTo(xf,yf,af,0.2);
                specialGoTo(4,200,0.5);
            }
            //next state
            progressUpdate(3,310);
            specificProgressUpdate(1,310);
            if(!(targeting||specialTargeting))state=320;
        }

        // 3.2) Lift, while moving backwards somewhat slowly - to not drop
        public void f320() {
            //start moving
            if(!stating) {
                int xf = 300;
                int yf = 800;
                int af = 0;
                if(red) xf = 700;
                goTo(xf,yf,af,0.2);
                specialGoTo(5,200,0.5);
            }
            //next state
            progressUpdate(3,320);
            specificProgressUpdate(1,320);
            if(!(targeting||specialTargeting))state=330;
        }

        // 3.3) Move backwards to storage unit
        public void f330() {
            //start moving
            if(!stating) {
                int xf = 300;
                int yf = 150;
                int af = 0;
                if(red) xf = 700;
                goTo(xf,yf,af,0.9);
            }
            //next state
            progressUpdate(3,340);
        }

        // 3.4) Move ramp servo, and wait sufficiently
        public void f340() {
            //start moving bottom servo
            if(lastTime==-1){
                lastTime=getRuntime();
                bottomservo.setPower(1);
            }
            // after 0.5s accurate to the nanosecond reverse bottom servo and next state
            else if(getRuntime()-lastTime>0.5) {
                lastTime=getRuntime();
                bottomservo.setPower(-1);
                state=410;
            }
        }

        // 4.1) Move forwards
        public void f410() {
            //check the previous returning
            if(lastTime>-1&&getRuntime()-lastTime>0.5) {
                lastTime = -1;
                bottomservo.setPower(0);
            }
            //start moving
            if(!stating) {
                int xf = 300;
                int yf = 950;
                int af = 270;
                if(red){ xf = 700;af=90;}
                goTo(xf,yf,af,0.5);
            }
            //next state
            progressUpdate(3,-1);
        }
    }

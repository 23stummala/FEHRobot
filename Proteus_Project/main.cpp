#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHMotor.h>
#include "FEHUtility.h"
#include <FEHServo.h>
#include <FEHBattery.h>
#include <FEHRCS.h>

//Define constants for computational use
#define ROTATION 318
#define WHEELRADIUS 1.5
#define PI 3.141592654
#define BOTWIDTH 7.3
#define DISTANCEPERCOUNT 2 * PI * WHEELRADIUS / ROTATION
#define MOTORPOWER 38

//Define all robotic components that will be plugged into a port
AnalogInputPin cds(FEHIO::P1_7);

DigitalInputPin frontLeft(FEHIO::P3_7);
DigitalInputPin frontRight(FEHIO::P0_0);

FEHMotor rightDrive(FEHMotor::Motor2, 9.0);
FEHMotor leftDrive(FEHMotor::Motor0, 9.0);

DigitalEncoder leftEncoder(FEHIO::P1_3);
DigitalEncoder rightEncoder(FEHIO::P1_0);

FEHServo fuelLever(FEHServo::Servo0); 
FEHServo passportLever(FEHServo::Servo1); 

float actualPower(){
    //return the actual power that should be used relative to the battery power
    return (11.5 / Battery.Voltage()) * MOTORPOWER;
}

void moveStraight(float distance){
    //reset the encoder counts
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    //If a positive distance, set the motors to move forward, otherwise backward
    if(distance > 0){
        leftDrive.SetPercent(-1 * actualPower());
        rightDrive.SetPercent(actualPower());
    }
    else{
        distance *= -1;
        leftDrive.SetPercent(actualPower());
        rightDrive.SetPercent(-1 * actualPower());
    }

    //Compute the number of rotations required to achive the distance
    float numRotations = distance / (2 * PI * WHEELRADIUS);

    //Loop while the encoder does not reach the required number of rotations
    while(leftEncoder.Counts() < ROTATION * numRotations && rightEncoder.Counts() < ROTATION * numRotations){}

    //Stop the motors when the distane is reached
    leftDrive.Stop();
    rightDrive.Stop();
}

void turnAboutCenter(float degrees){
    float slow = 0.5;
    //If positive degrees, have the right motor moving forward and the left backward, and vice versa
    if(degrees > 0){
        rightDrive.SetPercent(actualPower() * slow);
        leftDrive.SetPercent(actualPower() * slow);
        
    }
    else{
        leftDrive.SetPercent(-1 * actualPower() * slow);
        rightDrive.SetPercent(-1 * actualPower() * slow);
        degrees *= -1;
    }

    //Calculate the distance each motor would have to move to turn those degrees
    float distance = (2 * PI * BOTWIDTH / 2) / (360 / degrees);
    //Calculate the number of wheel rotations required for this turn
    float numRotations = distance / (2 * PI * WHEELRADIUS);

    //Reset the encoder counts
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    //Loop while the encoder determines the number of rotations are not reached
    while(leftEncoder.Counts() < ROTATION * numRotations && rightEncoder.Counts() < ROTATION * numRotations){
    }

    //Stop the motors upon completion of the turn
    leftDrive.Stop();
    rightDrive.Stop();

}

void turn(FEHMotor motor, DigitalEncoder encoder, float degrees, bool left){
    //finds the appropriate motorpower for the motor that is turning
    int percent = actualPower();

    //Update the percent and degrees if the passed in degrees are less than 0
    if(degrees < 0){
        percent *= -1;
        degrees *=-1;
    }
    //Update the motorpower if the turning motor is left, as it is the opposite of the intended
    if(left){
        percent *= -1;
    }

    //Set the motor power to the appropriate percent
    motor.SetPercent(percent);

    //Calculates the distance the motor will have to turn to achieve that number of degrees
    float distance = (2 * PI * BOTWIDTH) / (360 / degrees);
    //Calculate the number of wheel rotations required for this turn
    float numRotations = distance / (2 * PI * WHEELRADIUS);

    //Reset the encoder counts
    encoder.ResetCounts();
    //Loop while the encoder determines the number of rotations are not reached
    while(encoder.Counts() < ROTATION * numRotations){
    }

    //Stop the motor upon turn completion
    motor.Stop();

}

void moveWhileFrontUnbumped(){
    //Set the motors to their appropriate powers moving forward
    leftDrive.SetPercent(-1 * actualPower());
    rightDrive.SetPercent(actualPower());

    //Continue moving forward while both bump switches are unactivated
    while(frontLeft.Value() || frontRight.Value()){
        Sleep(0.1);
    }

    //Stop the motors upon proper bump and reset the encoder counts for future use
    leftDrive.Stop();
    rightDrive.Stop();
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
}

//This is an overloaded method that can take the motor power as an arguement for heightened adaptability
void moveWhileFrontUnbumpedPower(float power){
    //Set the motors to their appropriate powers moving forward
    leftDrive.SetPercent(-1 * power);
    rightDrive.SetPercent(power);

    //Continue moving forward while at least one of the bump switches is unactivated
    while(frontLeft.Value() && frontRight.Value()){
        Sleep(0.1);
    }

    //Stop the motors upon proper bump and reset the encoder counts for future use
    leftDrive.Stop();
    rightDrive.Stop();
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
}

bool isBlueLight(){
    //Sleep while the course cds Value is too dark to be a light
    //Initialize a min cds value
    float min = cds.Value();
    //Go forward slightly, updating the min as necessary
    for(int i = 0; i < 5; i++ ){
        leftDrive.SetPercent(-10);
        rightDrive.SetPercent(10);
        Sleep(0.25);
        leftDrive.Stop();
        rightDrive.Stop();
        if(cds.Value() < min){
            min = cds.Value();
        }
    }
    //Go backwards slightly to the initial position, updating the min as necessary
    for(int i = 0; i < 5; i++ ){
        leftDrive.SetPercent(10);
        rightDrive.SetPercent(-10);
        Sleep(0.25);
        leftDrive.Stop();
        rightDrive.Stop();
        if(cds.Value() < min){
            min = cds.Value();
        }
    }
    Sleep(0.5);
    //If the min cds value was greater than 1.3, infer the light color was blue
    if(min > 1.3){
        LCD.Write("Blue: ");
        LCD.Write(min);
        Sleep(0.5);
        //Return true, indicating the scanned light was blue
        return true;
    }
    //Else, infer the light color was red
    else{
        LCD.Write("Red: ");
        LCD.Write(min);
        Sleep(0.5);
        //Return false, indicating the scanned light was red
        return false;
    }
    
   return true;
}

//This method should take the robot from any fuel lever to the ticketing light area
void getUpLeftRamp(){
    moveWhileFrontUnbumped();
    Sleep(0.5);
    moveStraight(-3.0);
    turn(leftDrive, leftEncoder, 90, true);
    moveStraight(16.5);
    turnAboutCenter(-45);
    moveStraight(19.65);
    turnAboutCenter(90);
    moveWhileFrontUnbumped();
    moveStraight(-3);
}

//This function should take the robot from the ticketing light to the appropriate button
void hitTicketing(bool blue){
    moveStraight(-4);
    //If the scanned light was blue, hit the blue button on the ticketing kiosk
    if(blue){
        turnAboutCenter(-45);
        moveWhileFrontUnbumpedPower(35);
    }
    //Else, hit the red button on the ticketing kiosk
    else{
        //This is the additional backwards factor to travel to the red button
        moveStraight(-5);

        turnAboutCenter(-45);
        moveWhileFrontUnbumpedPower(35);
    }

    moveStraight(-12);
    turnAboutCenter(-90);
    moveWhileFrontUnbumped();
    moveStraight(-6);
    turnAboutCenter(90);
    moveStraight(6);

}

//This function takes the robot from the passport lever down to the final button
void getDownRamp(){
    moveStraight(-7);
    turnAboutCenter(-90);
    moveWhileFrontUnbumped();
    moveStraight(-3);
    turn(leftDrive, leftEncoder, 45, true);
    moveStraight(18);
    turnAboutCenter(-45);
    moveStraight(8);
    turnAboutCenter(90);
    moveWhileFrontUnbumped();
}

void flipLever(bool down){
    //If the fuel lever needs to be flipped down, flip the fuel lever down using servo, and then move back slightly
    if(down){
        fuelLever.SetDegree(160);
        fuelLever.SetDegree(110);
        Sleep(1.0);
        moveStraight(-2);
    }
    //Else, fold the mechanism down, move back into position, and flip the fuel lever up
    else{
        fuelLever.SetDegree(30);
        Sleep(1.0);
        moveStraight(2);
        Sleep(1.0);
        fuelLever.SetDegree(140);
        Sleep(1.0);
    }
}

void doFuelLever(){
    //Get the appropriat lever to flip via RCS signal
    int leverToFlip = RCS.GetCorrectLever();
    //Set the fuel lever up initially
    fuelLever.SetDegree(160);
    
    //The robot should already be in position for lever A
    float inches = 0.3;
    
    
    //Update the travel distance if the fuel lever is the middle one
    if(leverToFlip == 1){
        inches += 3.87;
    }
    //Update the travel distance if the fuel lever is C
    else if(leverToFlip == 2){
        inches += 6.75;
    }
    //Move the appropriate distance to get into position
    
    moveStraight(inches);

    //Set appropriate min and max for the fuel lever servo
    fuelLever.SetMin(500);
    fuelLever.SetMax(2450);

    Sleep(1.0);
    //Flip the lever down
    flipLever(true);
    //Pause for 5 seconds
    Sleep(5.0);
    //Flip the lever back up
    flipLever(false);
    Sleep(0.5);
}

void doPassportLever(){
    //Set appropriate min and max fo rthe passport arm servo
    passportLever.SetMin(500);
    passportLever.SetMax(2477);
    
    passportLever.SetDegree(180);
    Sleep(1.0);
    //Flip the passport arm up
    passportLever.SetDegree(60);
    //Sleep for a very short time to ensure high momentum
    Sleep(0.325);
    //Put the passport arm back down
    passportLever.SetDegree(180);
    Sleep(1.0);
    
}

void doLuggageDrop(){

    moveStraight(16.25);
    turn(leftDrive, leftEncoder, 45, true);
    //Move straight very quickly, ensuring the luggage will drop into the bin upon the sudden bump
    moveWhileFrontUnbumpedPower(85);
    //Back up 
    moveStraight(-6.0);
    //Turn 90 degrees to stay in position for the fuel lever
    turnAboutCenter(90);
    moveStraight(0.25);
}

void waitForLight(){
    //Wait still while the cds value is greater than 2
    while(cds.Value() > 2){
        Sleep(0.1);
    }
    //Hit the button to start the run
    moveStraight(-3);
    Sleep(0.1);
}

int main(){
    LCD.Clear();
    
    //Initialize the course to get RCS 
    RCS.InitializeTouchMenu("E3qvTBIic");
    //wait for the light
    waitForLight();
    //complete the luggage drop
    doLuggageDrop();
    //complete the fuel lever task
    doFuelLever();
    //get up the left ramp
    getUpLeftRamp();
    //determine the light color for ticketing
    bool blue = isBlueLight();
    //hit the appropriate ticketing button
    hitTicketing(blue);
    //complete the passport lever task
    doPassportLever();
    //get down the ramp to end the run
    getDownRamp();
    
    
}
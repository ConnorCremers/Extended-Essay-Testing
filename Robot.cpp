#include <iostream>   //include necessary libraries for program to function
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <CANTalon.h>
#include <Timer.h>
#include <fstream>
#include "WPIlib.h"
using namespace std;

class Robot: public frc::IterativeRobot {
public:
	void RobotInit() { //before the active movement
		prefs = Preferences::GetInstance();			//intialize quick tuning
		chooser.AddObject(LCTTest, LCTTest);		//add selection for two tests
		chooser.AddObject(LagTest, LagTest);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
	}


	void AutonomousInit() override {
		outputFile.open("/media/sda1/curve.csv"); //create .CSV to output to
		outputFile << "Time,Position,,P=,,Average Loop Timing,,Time To Settle" << endl; // write the file headers
		autoSelected = chooser.GetSelected(); //read which test to conduct
		Enc->Reset(); //zero encoder
		time = 0; //set time to zero
		kP = prefs->GetDouble("kP", kP); //read values from quick tuning
		kD = prefs->GetDouble("kD", kD);

		for(int i= 0; i < 50; i++){ positions[i] = 0; } //fill array with zeros so no false information is fed

		startTimer.Start(); //start the timer
	}

	void AutonomousPeriodic() {
		if (autoSelected == LCTTest) {
			lastTime = time; //used for finding change time for derivative component
			time = startTimer.Get(); //get time
			position = Enc->Get()/1024; //get position
			lastErr = curErr; //also used for derivative component
			curErr = finalPos - position; //calculate error
			float tErr = curErr * kP + (curErr - lastErr)/(time-lastTime)*kD; //calculate motor power output
			Wait(.8); //this delay changes the loop cycle time
			outputFile << time << "," << position << endl; //write information to .CSV
			motor.Set(tErr); //set motor power
		}
		else if(autoSelected == LagTest){
			for(int i = 49; i >= 0; i--){ //shift all values of position through the array
				positions[i]=positions[i-1];
			}
			positions[0]=Enc->Get()/1024; //update newest position
			lastTime = time; //used for calculating derivative component
			time = startTimer.Get(); //get time
			position = positions[15]; //change this value to change the induced lag
										//the lag should be equal to loop timing * value
			lastErr = curErr; 	//for calcualting derviative component
			curErr = finalPos - position; //calculate error
			float tErr = curErr * kP + (curErr - lastErr)/(time-lastTime)*kD; //calcoulate motor output
			outputFile << time << "," << positions[0] << "," << position << endl; //write values to .CSV
			motor.Set(tErr); //set motor power
			Wait(.08); //small delay
		}
	}


private:
	Preferences *prefs; //robot preferences for adjusting on the fly.

	frc::SendableChooser<std::string> chooser; // chooser for selecting test to conduct
	const std::string LCTTest = "LCTTest";
	const std::string LagTest = "LagTest";
	std::string autoSelected;

	CANTalon motor { 2 }; //declare motor on port 2 of CAN bus
	Encoder *Enc = new Encoder(8,9,true,Encoder::EncodingType::k4X); //declare encoder on digital ports 8 and 9

	float position, time, lastTime, kP, kD, curErr, lastErr; //necessary variables
	float finalPos = 150; //end position
	Timer startTimer; //timer
	ofstream outputFile; //output file
	float positions[20]; //array for storing old positions
};

START_ROBOT_CLASS(Robot)

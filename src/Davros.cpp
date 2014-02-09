#include "CommandBase.h"

#include "Davros.h"
#include "Robotmap.h"

Davros::Davros(): 
	encoderA (new Encoder(ENCODER_A_1, ENCODER_A_2, ENCODER_A_REV)), 
	encoderB (new Encoder(ENCODER_B_1, ENCODER_B_2, ENCODER_B_REV)), 
	encoderC (new Encoder(ENCODER_C_1, ENCODER_C_2, ENCODER_C_REV)), 
	encoderD (new Encoder(ENCODER_D_1, ENCODER_D_2, ENCODER_D_REV)) {
}

void Davros::RobotInit() {
    CommandBase::init();
    autonomousCommand = NULL;
    lw = LiveWindow::GetInstance();
    
    SmartDashboard::PutData(Scheduler::GetInstance());
    SmartDashboard::PutData(CommandBase::chassis);
}

void Davros::AutonomousInit() {
    //autonomousCommand->Start();
    
    encoderA->Reset();
    encoderB->Reset();
    encoderC->Reset();
    encoderD->Reset();
    
    encoderA->Start();
    encoderB->Start();
    encoderC->Start();
    encoderD->Start();
}

void Davros::AutonomousPeriodic() {
    Scheduler::GetInstance()->Run();
}

void Davros::TeleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to 
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //autonomousCommand->Cancel();
    
    encoderA->Reset();
    encoderB->Reset();
    encoderC->Reset();
    encoderD->Reset();
    
    encoderA->Start();
    encoderB->Start();
    encoderC->Start();
    encoderD->Start();
}

void Davros::TeleopPeriodic() {
    Scheduler::GetInstance()->Run();
    SmartDashboard::PutNumber("EncoderA(counts)", encoderA->Get());
	SmartDashboard::PutNumber("EncoderB(counts)", encoderB->Get());
	SmartDashboard::PutNumber("EncoderC(counts)", encoderC->Get());
	SmartDashboard::PutNumber("EncoderD(counts)", encoderD->Get());
	SmartDashboard::PutData("Chopsticks", CommandBase::chopsticks);
}

void Davros::TestPeriodic() {
    lw->Run();
}

START_ROBOT_CLASS(Davros);


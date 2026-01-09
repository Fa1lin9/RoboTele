#include "Ti5RobotBasicControlGUI.h"
#include "./ui_Ti5RobotBasicControlGUI.h"

Ti5RobotBasicControlGUI::Ti5RobotBasicControlGUI(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Ti5RobotBasicControlGUI)
{
    ui->setupUi(this);
    // Initialize the robot
    RobotHardware::BasicConfig config = {
        .robotType = RobotBase::RobotType::Ti5Robot,
    };
    this->physicalRobotPtr = RobotHardware::GetPtr(config);
}

Ti5RobotBasicControlGUI::~Ti5RobotBasicControlGUI()
{
    delete ui;
}

void Ti5RobotBasicControlGUI::printMessage(const std::string &message){
    ui->messageTextEdit->append(QString::fromStdString(message));
    std::cout<<message<<std::endl;
}

void Ti5RobotBasicControlGUI::CheckConnection(){
    if(!this->physicalRobotPtr->isConnect()){
        printMessage(" [Connect] Please connect to the robot first ! ");
        return;
    }
}

void Ti5RobotBasicControlGUI::UpdateConfig(){
    this->ti5RobotConfig.isLeftArmEnabled = ui->leftArmCheckBox->isChecked();
    this->ti5RobotConfig.isRightArmEnabled = ui->rightArmCheckBox->isChecked();
    this->ti5RobotConfig.isHeadEnabled = ui->headCheckBox->isChecked();
    this->ti5RobotConfig.isWaistEnabled = ui->waistCheckBox->isChecked();

    std::vector<std::string> enabledParts;

    if(this->ti5RobotConfig.isLeftArmEnabled){
        enabledParts.push_back("Left Arm");
    }
    if(this->ti5RobotConfig.isRightArmEnabled){
        enabledParts.push_back("Right Arm");
    }
    if(this->ti5RobotConfig.isHeadEnabled){
        enabledParts.push_back("Head");
    }
    if(this->ti5RobotConfig.isWaistEnabled){
        enabledParts.push_back("Waist");
    }

    if(!enabledParts.empty()){
        std::string msg = " [Config] Use ";
        for(size_t i = 0; i < enabledParts.size(); ++i){
            msg += enabledParts[i];
            if(i < enabledParts.size() - 1){
                msg += ", ";
            }
        }
        this->printMessage(msg);
    }

}

void Ti5RobotBasicControlGUI::on_socketConnectPushButton_clicked()
{
    //TODO
    printMessage(" [TODO]Socket Connect ");

}


void Ti5RobotBasicControlGUI::on_directConnectPushButton_clicked()
{
    // check connection
    if(this->physicalRobotPtr->isConnect()){
        printMessage(" [Connect] You have alreadly connected to the robot ! ");
        return;
    }

    this->physicalRobotPtr->Connect();
    if(!this->physicalRobotPtr->isConnect()){
        printMessage(" [Connect] Can't connect to the robot ! ");
    }else{
        printMessage(" [Connect] Sucessfully connect to the robot ! ");
        this->physicalRobotPtr->Init();
    }
}


void Ti5RobotBasicControlGUI::on_disconnectPushButton_clicked()
{
    // check connection
    if(!this->physicalRobotPtr->isConnect()){
        printMessage(" [Connect] You have alreadly disconnected to the robot ! ");
        return;
    }

    // disconnect
    this->physicalRobotPtr->Disconnect();
    if(!this->physicalRobotPtr->isConnect()){
        printMessage(" [Connect] Sucessfully disconnect to the robot ! ");
    }else{
        printMessage(" [Connect] Can't disconnect to the robot ! ");
    }
}



void Ti5RobotBasicControlGUI::on_emergencyStopPushButton_clicked()
{
    CheckConnection();

    this->physicalRobotPtr->EmergencyStop();
    this->printMessage(" [Stop] Emergency Stop ! ");
}


void Ti5RobotBasicControlGUI::on_getJointsValuePushButton_clicked()
{
    CheckConnection();

    // TODO
}


void Ti5RobotBasicControlGUI::on_getJointsPosePushButton_clicked()
{
    CheckConnection();

    // TODO
}


void Ti5RobotBasicControlGUI::on_leftArmCheckBox_stateChanged(int arg1)
{
    this->UpdateConfig();
}


void Ti5RobotBasicControlGUI::on_rightArmCheckBox_stateChanged(int arg1)
{
    this->UpdateConfig();
}


void Ti5RobotBasicControlGUI::on_headCheckBox_stateChanged(int arg1)
{
    this->UpdateConfig();
}


void Ti5RobotBasicControlGUI::on_waistCheckBox_stateChanged(int arg1)
{
    this->UpdateConfig();
}


void Ti5RobotBasicControlGUI::on_moveJPushButton_clicked()
{
    CheckConnection();

    std::vector<double> leftArmTarget(7);
    std::vector<double> rightArmTarget(7);

    std::array<QLineEdit*, 7> leftArmLineEdits = {
        ui->leftArmJ1LineEdit,
        ui->leftArmJ2LineEdit,
        ui->leftArmJ3LineEdit,
        ui->leftArmJ4LineEdit,
        ui->leftArmJ5LineEdit,
        ui->leftArmJ6LineEdit,
        ui->leftArmJ7LineEdit,
    };

    std::array<QLineEdit*, 7> rightArmLineEdits = {
        ui->rightArmJ1LineEdit,
        ui->rightArmJ2LineEdit,
        ui->rightArmJ3LineEdit,
        ui->rightArmJ4LineEdit,
        ui->rightArmJ5LineEdit,
        ui->rightArmJ6LineEdit,
        ui->rightArmJ7LineEdit,
    };

    bool ok = true;
    for (size_t i = 0; i < leftArmLineEdits.size(); ++i) {
        // parse value
        double val = leftArmLineEdits[i]->text().toDouble(&ok);
        if (!ok) {
            std::string message =
                    " [MoveJ] Invalid input at left arm joint " + std::to_string(i+1);
            this->printMessage(message);
            val = 0.0;
        }
        leftArmTarget[i] = val;
    }

    for (size_t i = 0; i < rightArmLineEdits.size(); ++i) {
        double val = rightArmLineEdits[i]->text().toDouble(&ok);
        if (!ok) {
            std::string message =
                    " [MoveJ] Invalid input at right arm joint " + std::to_string(i+1);
            this->printMessage(message);
            val = 0.0;
        }
        rightArmTarget[i] = val;

    }

    this->ti5RobotConfig.qTargetLeftArm = leftArmTarget;
    this->ti5RobotConfig.qTargetRightArm = rightArmTarget;
    this->NormalizeAngle();
    this->physicalRobotPtr->MoveJ(this->ti5RobotConfig);

}


void Ti5RobotBasicControlGUI::on_moveLPushButton_clicked()
{
    CheckConnection();

    // TODO
}


void Ti5RobotBasicControlGUI::on_clearMessagePushButton_clicked()
{
    ui->messageTextEdit->setText("");
}


void Ti5RobotBasicControlGUI::on_backToZeroPushButton_clicked()
{
    CheckConnection();

//    std::vector<double> zeroVec = {0 , 0 , 0 , 0 , 0 , 0 , 0};

//    if(this->ti5RobotConfig.useLeftArm){
//        this->ti5RobotConfig.leftArmJointsValue=zeroVec;
//    }else if(this->ti5RobotConfig.useRightArm){
//        this->ti5RobotConfig.rightArmJointsValue=zeroVec;
//    }

    this->physicalRobotPtr->BackToZero(this->ti5RobotConfig);
}

std::vector<double> Ti5RobotBasicControlGUI::ConvertDegrees2Radians(const std::vector<double>& degrees){
    std::vector<double> radians;

    for(const auto& degree:degrees){
        radians.push_back(degree * (this->pi / 180.0));
    }

    return radians;
}

std::vector<double> Ti5RobotBasicControlGUI::ConvertRadians2Degrees(const std::vector<double>& radians){
    std::vector<double> degrees;

    for(const auto& radian:radians){
        degrees.push_back(radian * 180.0 / this->pi);
    }

    return degrees;
}

void Ti5RobotBasicControlGUI::NormalizeAngle(){
    // turn to degree
    if(ui->radianRadioButton->isChecked()){
//        this->ti5RobotConfig.leftArmJointsValue
//                = ConvertRadians2Degrees(this->ti5RobotConfig.leftArmJointsValue);
//        this->ti5RobotConfig.rightArmJointsValue
//                = ConvertRadians2Degrees(this->ti5RobotConfig.rightArmJointsValue);
    // turn to radian
    }else{
        this->ti5RobotConfig.qTargetLeftArm
                = ConvertDegrees2Radians(this->ti5RobotConfig.qTargetLeftArm);
        this->ti5RobotConfig.qTargetRightArm
                = ConvertDegrees2Radians(this->ti5RobotConfig.qTargetRightArm);
    }
}



void Ti5RobotBasicControlGUI::on_resetLeftArmPushButton_clicked()
{
    std::array<QLineEdit*, 7> leftArmLineEdits = {
        ui->leftArmJ1LineEdit,
        ui->leftArmJ2LineEdit,
        ui->leftArmJ3LineEdit,
        ui->leftArmJ4LineEdit,
        ui->leftArmJ5LineEdit,
        ui->leftArmJ6LineEdit,
        ui->leftArmJ7LineEdit,
    };

    double value = 0.0;

    for(size_t i = 0;i<leftArmLineEdits.size();i++){
        leftArmLineEdits[i]->setText(QString::number(value,'f',1));
    }
}


void Ti5RobotBasicControlGUI::on_resetRightArmPushButton_clicked()
{
    std::array<QLineEdit*, 7> rightArmLineEdits = {
        ui->rightArmJ1LineEdit,
        ui->rightArmJ2LineEdit,
        ui->rightArmJ3LineEdit,
        ui->rightArmJ4LineEdit,
        ui->rightArmJ5LineEdit,
        ui->rightArmJ6LineEdit,
        ui->rightArmJ7LineEdit,
    };

    double value = 0.0;

    for(size_t i = 0;i<rightArmLineEdits.size();i++){
        rightArmLineEdits[i]->setText(QString::number(value,'f',1));
    }
}


void Ti5RobotBasicControlGUI::on_backToInitPosePushButton_clicked()
{
    CheckConnection();

    this->physicalRobotPtr->BackToInitPose(this->ti5RobotConfig);

}


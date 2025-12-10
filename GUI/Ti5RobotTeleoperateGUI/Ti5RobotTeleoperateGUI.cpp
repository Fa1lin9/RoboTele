#include "Ti5RobotTeleoperateGUI.h"
#include "./ui_Ti5RobotTeleoperateGUI.h"

Ti5RobotTeleoperateGUI::Ti5RobotTeleoperateGUI(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Ti5RobotTeleoperateGUI)
{
    ui->setupUi(this);

    // infoTreeWidget
    ui->infoTreeWidget->setHeaderLabels({"Key", "Value"});
    ui->infoTreeWidget->setAlternatingRowColors(true);

    // let key can't be edit
//    ui->infoTreeWidget->setItemDelegateForColumn(0, new QStyledItemDelegate(ui->infoTreeWidget));
}

Ti5RobotTeleoperateGUI::~Ti5RobotTeleoperateGUI()
{
    delete ui;
}


void Ti5RobotTeleoperateGUI::on_chooseFilePushBtn_clicked()
{
    std::string temp = static_cast<std::string>(SOURCE_FILE_PATH) + "/config";
    QString filePath = QFileDialog::getOpenFileName(this,
                                                    "Choose the Configuration",
                                                    QString::fromStdString(temp));

    if(!filePath.isEmpty()){
        ui->chooseFileLineEdit->setText(filePath);
    }

    // update configPath
    this->configPath = ui->chooseFileLineEdit->text().toStdString();

    std::cout<<"[Ti5RobotTeleoperateGUI] The configPath: "<<this->configPath<<std::endl;
}

void Ti5RobotTeleoperateGUI::AddJsonValue2Tree(QTreeWidgetItem *parent, json::value &v){
    if(v.is_object()){
        for(auto &p: v.get_object()){
            auto *item = new QTreeWidgetItem(parent);
            item->setText(0, QString::fromStdString(static_cast<std::string>(p.key())));
            AddJsonValue2Tree(item, p.value());
        }
    }
    else if(v.is_array()){
        int i = 0;
        for(auto &elem: v.get_array()){
            auto *item = new QTreeWidgetItem(parent);
            item->setText(0, QString("[%1]").arg(i++));
            AddJsonValue2Tree(item, elem);
        }
    }
    else{
        // let value can be edited
//        parent->setFlags( parent->flags() | Qt::ItemIsEditable);
        parent->setText(1,QString::fromStdString(boost::json::serialize(v)));
    }

}

void Ti5RobotTeleoperateGUI::on_loadFilePushBtn_clicked()
{
    this->configPath = ui->chooseFileLineEdit->text().toStdString();

    // get value
    json::value jsonValue = JsonParser(this->configPath).GetJsonValue();

//    std::cout<<"configPath: "<<this->configPath<<std::endl;
//    std::cout<<"jsonValue: "<<jsonValue<<std::endl;

    ui->infoTreeWidget->clear();

    QTreeWidgetItem *root = new QTreeWidgetItem(ui->infoTreeWidget);

    QFileInfo info(QString::fromStdString(this->configPath));
    root->setText(0, info.baseName());

    this->AddJsonValue2Tree(root, jsonValue);

    ui->infoTreeWidget->expandAll();

}


void Ti5RobotTeleoperateGUI::on_startTeleBtn_clicked()
{
    // 如果已经存在线程，避免重复启动
    if (teleopThread != nullptr) {
        std::cout<< "Teleoperation already running!"<<std::endl;
        return;
    }

    // 1. 创建 teleop
    teleop = RobotTeleoperate::GetPtr(this->configPath);

    // 2. 创建线程
    teleopThread = QThread::create([this](){
        teleop->StartTeleoperate(false);
    });

    // 可选：线程结束后删除自身并清空指针
    connect(teleopThread, &QThread::finished, this, [this]() {
        teleopThread->deleteLater();
        teleopThread = nullptr;
        teleop.reset();
    });

    teleopThread->start();
}

void Ti5RobotTeleoperateGUI::on_stopTeleBtn_clicked()
{
    if (teleop) {
        teleop->StopTeleoperate();
    }
}


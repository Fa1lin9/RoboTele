#include "SimpleGUI.h"
#include "ui_SimpleGUI.h"

SimpleGUI::SimpleGUI(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

SimpleGUI::~SimpleGUI()
{
    delete ui;
}


#pragma once
#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class SimpleGUI : public QMainWindow
{
    Q_OBJECT

public:
    SimpleGUI(QWidget *parent = nullptr);
    ~SimpleGUI();

private:
    Ui::MainWindow *ui;
};


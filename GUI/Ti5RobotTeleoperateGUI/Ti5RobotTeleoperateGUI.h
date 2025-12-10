#ifndef TI5ROBOTTELEOPERATEGUI_H
#define TI5ROBOTTELEOPERATEGUI_H

#include <QMainWindow>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QTreeWidget>
#include <JsonParser/JsonParser.hpp>
#include <QFileInfo>
#include <QStyledItemDelegate>
#include <QThread>

#include <RobotTeleoperate/RobotTeleoperate.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class Ti5RobotTeleoperateGUI; }
QT_END_NAMESPACE

class Ti5RobotTeleoperateGUI : public QMainWindow
{
    Q_OBJECT

public:
    Ti5RobotTeleoperateGUI(QWidget *parent = nullptr);
    ~Ti5RobotTeleoperateGUI();

private slots:
    void on_chooseFilePushBtn_clicked();

    void on_loadFilePushBtn_clicked();

    void on_startTeleBtn_clicked();

    void on_stopTeleBtn_clicked();

private:
    void AddJsonValue2Tree(QTreeWidgetItem *parent, json::value &v);

    Ui::Ti5RobotTeleoperateGUI *ui;

    std::string configPath;

    boost::shared_ptr<RobotTeleoperate> teleop;
    QThread* teleopThread = nullptr;

};
#endif // TI5ROBOTTELEOPERATEGUI_H

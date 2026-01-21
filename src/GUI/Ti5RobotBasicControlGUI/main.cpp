#include "Ti5RobotBasicControlGUI.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Ti5RobotBasicControlGUI w;
    w.show();
    return a.exec();
}

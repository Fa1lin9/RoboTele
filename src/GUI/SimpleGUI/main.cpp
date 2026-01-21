#include "SimpleGUI.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SimpleGUI w;
    w.show();
    return a.exec();
}

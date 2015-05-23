#include "mainwindow.h"
#include <QApplication>
#include "myimgtest.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
//    MyImgTest* widget = new MyImgTest();
//    widget->show();

    MainWindow w;
    w.show();

    return a.exec();
}

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_displayButton_clicked();

    void on_selectSource_clicked();

    void on_selectResult_clicked();

    void on_selectAlgorithmButton_clicked();

    void on_QueryButton_clicked();

    void on_buildGraph_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H

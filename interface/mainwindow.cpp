#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qgraphicsview.h>
#include <iostream>
#include <string>
#include <QMessageBox>
#include <ctime>
#include <Graph.h>
#include <fstream>

using namespace std;

Graph g;

void addPoint(QGraphicsScene tempScene);

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    g.buildFromFile(ifstream("E:/Lecture/gps location/experiment2/Sensor_Locating/interface/data/net1_pos.txt"),
                    ifstream("E:/Lecture/gps location/experiment2/Sensor_Locating/interface/data/net1_topo-error free.txt"));

    g.shortestPath_Floyd();

    dvhop(g);
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

//添加传感器点
void addPoint(float x,float y,QGraphicsScene *tempScene,QPen pen,QBrush brush){

    tempScene->addEllipse(x,y,10,10,pen,brush);

}

void MainWindow::on_pushButton_clicked()
{
    //inti pen and brush
    QPen pen(Qt:: green);
    QBrush brush(Qt::red);//result point is red
    QGraphicsScene *scene = new QGraphicsScene();
    scene->setSceneRect(0,0,400,400);//inti QGraphicsScene
//    float x[300];
//    float y[300];
//    float m[300];
//    float n[300];
//    srand((unsigned)time(0));
//    for(int i = 0;i < 300;i++){
//        x[i]=rand()%400;
//    }
//    for(int i = 0;i < 300;i++){
//        y[i]=rand()%400;
//    }
//    for(int i = 0;i < 300;i++){
//        m[i]=rand()%400;
//    }
//    for(int i = 0;i < 300;i++){
//        n[i]=rand()%400;
//    }

    if(!ui->addReal->isChecked() && !ui->addResult->isChecked()){//no type
        QMessageBox::about(0,tr("Warning:"),tr("Please Choose Display Type!"));
        return;
    }
    if(!ui->addReal->isChecked() && ui->addResult->isChecked()){//show the result point
        for(auto &p : g.points){
            addPoint(p.cx,p.cy,scene,pen,brush);
        }
    }
    if(ui->addReal->isChecked() && !ui->addResult->isChecked()){//show the real point
        pen.setColor(Qt::yellow);
        brush.setColor(Qt::blue);//real point is blue
        for(auto &p : g.points){
            addPoint(p.getX(),p.getY(),scene,pen,brush);
        }
    }
    if(ui->addReal->isChecked() && ui->addResult->isChecked()){//show both of them
        pen.setColor(Qt::green);
        brush.setColor(QColor(0xff,0x00,0x00,0xd0));//red
        for(auto &p : g.points){
            addPoint(p.cx,p.cy,scene,pen,brush);
        }

        pen.setColor(Qt::yellow);
        brush.setColor(QColor(0x00,0x00,0xff,0xd0));//blue
        for(auto &p : g.points){
            addPoint(p.getX(),p.getY(),scene,pen,brush);
        }
    }

//    scene->addLine(0,0,400,400);
//    scene->addEllipse(0,0,10,10,pen,brush);
//    scene->addEllipse(390,390,10,10,pen,brush);

    ui->graphicsView->setScene(scene);
    ui->graphicsView->show();
    cout<<"Run"<<endl;

}

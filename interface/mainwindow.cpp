#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qgraphicsview.h>
#include <iostream>
#include <string>
#include <QMessageBox>
#include <ctime>
#include <QFileDialog>
#include <Graph.h>
#include <fstream>
#include <vector>
using namespace std;

void addPoint(QGraphicsScene tempScene);
void addPoint(const vector<double> &x, const vector<double> &y,QGraphicsScene *tempScene,QPen &pen,QBrush &brush);
QString sourcePath_1;
QString sourcePath_2;
Graph g, origin;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

//添加传感器点
void addPoint(const vector<double> &x, const vector<double> &y,QGraphicsScene *tempScene,QPen &pen,QBrush &brush){


    for(int i = 0;i < x.size() ;i++){
        tempScene->addEllipse(x[i]-5,y[i]-5,10,10,pen,brush);
    }

}
//添加偏离线
void addmyLine(const vector<double> &cx, const vector<double> &cy,const vector<double> &x, const vector<double> &y,QGraphicsScene *tempScene){

    QPen pen(Qt::black);
    pen.setWidth(1);
    for(int i = 0;i < x.size() ;i++){
        tempScene->addLine(cx[i],cy[i],x[i],y[i],pen);
    }
}

//选择源文件按钮事件响应
void MainWindow::on_selectSource_clicked()
{
    //获取源文件地址1为sourcePath_1
    sourcePath_1=QFileDialog::getOpenFileName(this,tr("Choose　Source File"),"../data",tr("TXT　Files(*.TXT　*.txt)"));
    ui->sourceFile_1->setText(sourcePath_1);
}

//选择输出文件按钮事件响应
void MainWindow::on_selectResult_clicked()
{
    //获取源文件地址2为sourcePath_2
    sourcePath_2=QFileDialog::getOpenFileName(this,tr("Choose　Result File"),"../data",tr("TXT　Files(*.TXT　*.txt)"));
    ui->sourceFile_2->setText(sourcePath_2);
}

//建图
void MainWindow::on_buildGraph_clicked()
{
    ifstream a(sourcePath_1.toStdString());
    ifstream b(sourcePath_2.toStdString());

    origin.buildFromFile(a,b);
    origin.shortestPath_Floyd();

    QMessageBox::about(0,tr("Build:"),tr("Success!"));
}
//选择算法事件响应
void MainWindow::on_selectAlgorithmButton_clicked()
{
    //警告未选择文件
    if(ui->sourceFile_1->text().isEmpty() || ui->sourceFile_2->text().isEmpty()){
        QMessageBox::about(0,tr("Warning:"),tr("Please Choose File!"));
        return;
    }
    //警告未选择算法
    if(!ui->recursiveButton->isChecked() && !ui->PDMButton->isChecked() && !ui->DV_HOPButton->isChecked() && !ui->MDSButton->isChecked()){
        QMessageBox::about(0,tr("Warning:"),tr("Please Choose Algorithm!"));
        return;
    }

    g = origin;

    //选择多变迭代
    if(ui->recursiveButton->isChecked()){
        recursiveTri(g);
        QMessageBox::about(0,tr("Recursive:"),tr("Finish!"));
    }
    //选择PDM
    if(ui->PDMButton->isChecked()){
        pdm(g);
        QMessageBox::about(0,tr("PDM:"),tr("Finish!"));
    }
    //选择MDS
    if(ui->MDSButton->isChecked()){
        mds(g);
        QMessageBox::about(0,tr("MDS:"),tr("Finish!"));
    }
    //选择DV_HOP
    if(ui->DV_HOPButton->isChecked()){
        dvhop(g);
        QMessageBox::about(0,tr("DV_HOP:"),tr("Finish!"));
    }
}

//显示所有点事件响应
void MainWindow::on_displayButton_clicked()
{
    //inti pen and brush
    QPen pen(Qt::black);
    QBrush brush(Qt::red);//result point is red
    QGraphicsScene *scene = new QGraphicsScene();
    scene->setSceneRect(0,0,600,600);//inti QGraphicsScene

    if(g.size()==0){
        QMessageBox::about(0,tr("Warning:"),tr("No Point Information!"));
        return;
    }
    //未选择显示方式
    if(!ui->addReal->isChecked() && !ui->addResult->isChecked()&& !ui->addLine->isChecked()){//no type
        QMessageBox::about(0,tr("Warning:"),tr("Please Choose Display Type!"));
        return;
    }

    vector<double> x, y, cx, cy;
    for(int i = 0; i < g.size(); i ++) {
        Node &p = g.getPoint(i);
        x.push_back(p.getX() * 3);
        y.push_back(p.getY() * 3);
        cx.push_back(p.cx * 3);
        cy.push_back(p.cy * 3);
    }

    //只显示偏离线
    if(!ui->addReal->isChecked() && !ui->addResult->isChecked()&& ui->addLine->isChecked()){
        addmyLine(cx,cy,x,y,scene);

    }
    //显示计算点
    if(!ui->addReal->isChecked() && ui->addResult->isChecked()){//show the result point
        //显示连通线
        for(int i = 0;i < g.size();i++){
            Node &q = g.getPoint(i);
            for(auto &it : q.neighbourDistance){
                Node &p = g.getPoint(it.first);
                QPen pen_1(QColor(0xe1,0xaa,0xaa,0x22));
                scene->addLine(q.cx*3,q.cy*3,p.cx*3,p.cy*3,pen_1);
            }
        }
        //显示计算点
        addPoint(cx,cy,scene,pen,brush);
        //显示偏离线
        if(ui->addLine->isChecked()){
            addmyLine(cx,cy,x,y,scene);
            cout<<"Run"<<endl;
        }

    }

    //显示真实点
    if(ui->addReal->isChecked() && !ui->addResult->isChecked()){//show the real point

        //显示连通线
        for(int i = 0;i < g.size();i++){
            Node &q = g.getPoint(i);
            for(auto &it : q.neighbourDistance){
                Node &p = g.getPoint(it.first);
                QPen pen_1(QColor(0xaa,0xaa,0xe1,0x22));
                scene->addLine(q.getX()*3,q.getY()*3,p.getX()*3,p.getY()*3,pen_1);
            }
        }
        //显示真实点
        brush.setColor(Qt::blue);//real point is blue
        addPoint(x,y,scene,pen,brush);
        //显示偏离线
        if(ui->addLine->isChecked()){
          addmyLine(cx,cy,x,y,scene);
        }
    }

    //混合显示点
    if(ui->addReal->isChecked() && ui->addResult->isChecked()){//show both of them
        //计算点显示连通线
        for(int i = 0;i < g.size();i++){
            Node &q = g.getPoint(i);
            for(auto &it : q.neighbourDistance){
                Node &p = g.getPoint(it.first);
                QPen pen_1(QColor(0xe1,0xaa,0xaa,0x22));
                scene->addLine(q.cx*3,q.cy*3,p.cx*3,p.cy*3,pen_1);
            }
        }
        //显示真实点连通线
        for(int i = 0;i < g.size();i++){
            Node &q = g.getPoint(i);
            for(auto &it : q.neighbourDistance){
                Node &p = g.getPoint(it.first);
                QPen pen_1(QColor(0xaa,0xaa,0xe1,0x22));
                scene->addLine(q.getX()*3,q.getY()*3,p.getX()*3,p.getY()*3,pen_1);
            }
        }
        //显示点
        brush.setColor(QColor(0xff,0x00,0x00,0xd0));//red
        addPoint(cx,cy,scene,pen,brush);

        brush.setColor(QColor(0x00,0x00,0xff,0xd0));//blue
        addPoint(x,y,scene,pen,brush);
        //显示偏离线
        if(ui->addLine->isChecked()){
           addmyLine(cx,cy,x,y,scene);
        }
    }
    ui->graphicsView->setScene(scene);
    ui->graphicsView->show();
}
vector<double> x, y, cx, cy;

//查询传感器点位置事件响应
void MainWindow::on_QueryButton_clicked()
{

    QPen pen(Qt::black);
    QBrush brush(QColor(0xff,0xff,0x00,0xff));//result point is red
    QGraphicsScene *scene =  ui->graphicsView->scene();
    int pointNum;
    vector<double> x, y, cx, cy;
    double diffx,diffy;
    double diffsum = 0;

    if(g.size()==0){
        QMessageBox::about(0,tr("Warning:"),tr("No Point Information!"));
        ui->pointNum->clear();
        return;
    }

    pointNum= ui->pointNum->text().toInt();
    if(pointNum <= 0 || pointNum >g.size()){
        QMessageBox::about(0,tr("Warning:"),tr("Input Positive Integer!"));
        ui->pointNum->clear();
        return;
    }

    for(int i = 0; i < g.size(); i ++) {
        Node &p = g.getPoint(i);
        if(p.getId() == pointNum -1){
            x.push_back(p.getX());
            y.push_back(p.getY());
            cx.push_back(p.cx);
            cy.push_back(p.cy);
            diffx = abs(p.getX()-p.cx)/p.getX()*100;
            diffy = abs(p.getY()-p.cy)/p.getY()*100;
        }
        diffsum += abs(p.getX()-p.cx)/p.getX()*100 + abs(p.getY()-p.cy)/p.getY()*100;
    }
    ui->resultX->setText(QString::number(cx[0]));
    ui->resultY->setText(QString::number(cy[0]));
    ui->realX->setText(QString::number(x[0]));
    ui->realY->setText(QString::number(y[0]));
    ui->offsetX->setText(QString::number(diffx) + "%");
    ui->offsetY->setText(QString::number(diffy) + "%");
    ui->offsetSum->setText(QString::number(diffsum / g.size() / 2) + "%");

    addPoint(cx,cy,scene,pen,brush);//计算点为黄

    brush.setColor(QColor(0x00,0xff,0x00,0xff));
    addPoint(x,y,scene,pen,brush);//真实点为绿
    ui->graphicsView->setScene(scene);
    ui->graphicsView->show();
    cout<<pointNum<<endl;
}


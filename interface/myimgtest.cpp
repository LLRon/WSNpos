#include "MyImgTest.h"
#include <QPainter>
//#include <QRect>
//#include <QPointF>
//#include <QLineF>
MyImgTest::MyImgTest(QWidget* parent) : QWidget(parent)
{
setFixedSize(360,360); //固定大小
setWindowTitle("Hello DevDiv!");
}
MyImgTest::~MyImgTest()
{
}
void MyImgTest::paintEvent(QPaintEvent *)
{
 // 画一条直线
 QPainter painter(this); // 创建QPainter一个对象
 QPen pen;
 //pen.setStyle(Qt::DashLine);
pen.setColor(Qt::red); // 设置画笔为黄色
painter.setPen(pen); // 设置画笔
painter.drawLine(rect().topLeft(), rect().bottomRight());
 // 画一个空心矩形
pen.setColor(Qt::darkRed);
painter.setPen(pen);
painter.drawRect(1, 1, 100, 100); //painter.drawRect(QRect(1, 1, 100, 100));
 // 画一个实心矩形
QBrush bruch(Qt::FDiagPattern); // 画刷
painter.setBrush(bruch); // 设置画刷
painter.drawRect(QRect(105, 1, 100, 100));
 // 画一个多点线
pen.setColor(Qt::black);
painter.setPen(pen);
bruch.setStyle(Qt::NoBrush); // 将画刷设置成null//fow what
painter.setBrush(bruch);
 static const QPointF points[4] = {QPointF(210.0, 1), QPointF(220.0, 50.3), QPointF(300, 100.4), QPointF(260.4, 120.0)};
painter.drawPolyline(points, 4);
 // 画多个点
 QPointF pointf[10];
 for (int i=0; i<10; ++i)
 {
pointf[i].setX(2.0+i*10.0);
pointf[i].setY(130.0);
 }
painter.drawPoints(pointf, 10);
 // 画多条线
 QLineF linef[5];
 for (int j=0; j<5; ++j)
 {
linef[j].setP1(QPointF(110.9+j*10, 120.0));
linef[j].setP2(QPointF(120.8+j*12, 200.0));
 }
painter.drawLines(linef, 5);
 // 画一个多边形
 QPolygonF polygon;
polygon << QPointF(200.0, 120.0) << QPointF(230.0, 130.0) << QPointF(260.0, 180.0) << QPointF(200.0, 200.0);
bruch.setStyle(Qt::CrossPattern);
painter.setBrush(bruch);
painter.drawPolygon(polygon, Qt::WindingFill);
 // 画一个圆角矩形
 QRectF rectangle(290.0, 110.0, 50, 50);
bruch.setStyle(Qt::SolidPattern);
painter.setBrush(bruch);
painter.drawRoundedRect(rectangle, 20.0, 15.0);
 // 画一个QString
painter.drawText(50, 300, "Hello DevDiv!");
}

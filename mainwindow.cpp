#include "mainwindow.h"
#include <QLayout>
#include <astar.h>
#include <odometrymap.h>
#include <QKeyEvent>
#include <QFile>
#include <QTextStream>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    mainWidget = new QWidget(this);
    mainLayout = new QGridLayout(mainWidget);
    originalMap_ = new OdometryMap(mainWidget);
    reducedMap_ = new OdometryMap(mainWidget);
    mainLayout->addWidget(originalMap_, 0, 0, 1, 1, Qt::AlignCenter);
    mainLayout->addWidget(reducedMap_, 0, 1, 1, 1, Qt::AlignCenter);
    mainWidget->setLayout(mainLayout);

    setCentralWidget(mainWidget);
    astar_ = new AStar(this);

    QFile mapFile("/home/konstantin/ros/map.txt");
    if(!mapFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Can't open map file";
        exit(-1);
    }
    QTextStream stream(&mapFile);
    QString params;
    stream >> params;
    QStringList paramsList = params.split(';');
    float mapResolution = paramsList[0].toDouble();
    int mapWidth = paramsList[1].toInt();
    int mapHeight = paramsList[2].toInt();
    float x = paramsList[3].toDouble();
    float y = paramsList[4].toDouble();
    QString mapString;
    stream >> mapString;
    QStringList mapList = mapString.split(';');
    vector<int8_t> mapValues;
    for(auto &value : mapList)
    {
        mapValues.push_back(value.toInt());
    }
    Map newMap(mapResolution, x, y, mapWidth, mapHeight, mapValues);
    originalMap_->setMap(x, y, 0, mapWidth, mapHeight, mapResolution, mapValues);
    mapFile.close();

    QFile posFile("/home/konstantin/ros/pos0.txt");
    if(!posFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Can't open pos file";
        exit(-1);
    }
    stream.setDevice(&posFile);
    QString posString;
    stream >> posString;
    QStringList posList = posString.split(';');
    Point2D start;
    start.y = posList[0].toDouble();
    start.x = posList[1].toDouble();
    posFile.close();

    posFile.setFileName("/home/konstantin/ros/pos1.txt");
    if(!posFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Can't open pos file";
        exit(-1);
    }
    stream.setDevice(&posFile);
    stream >> posString;
    posList = posString.split(';');
    Point2D stop;
    stop.y = posList[0].toDouble();
    stop.x = posList[1].toDouble();
    posFile.close();

    astar_ = new AStar(this);
    astar_->setMap(newMap);
    qDebug() << start.x << start.y;
    astar_->setStart(start.x, start.y);
    astar_->setStop(stop.x, stop.y);
    astar_->findPath();
    reducedMap_->setMapStruct(astar_->getMap());
    qDebug() << "Start" << start.x << start.y << x << y << mapResolution;
    //reducedMap_->addPoint(start);
    //reducedMap_->addPoint(stop);
    //originalMap_->addPoint(start);
    //originalMap_->addPoint(stop);
    const auto path = astar_->getPath();
    for(const auto &p : path)
    {
        originalMap_->addPoint(p);
        reducedMap_->addPoint(p);
    }


    qDebug() << mapResolution << mapWidth << mapHeight;
}



MainWindow::~MainWindow()
{

}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Escape)
    {
        close();
    }
}

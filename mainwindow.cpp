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

    QFile mapFile("../AStar/map.txt");
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

    astar_ = new AStar(this);
    astar_->setMap(newMap);
    reducedMap_->setMapStruct(astar_->getMap());
    const auto path = astar_->getPath();
    for(const auto &p : path)
    {
        originalMap_->addPoint(p);
        reducedMap_->addPoint(p);
    }

    connect(originalMap_, &OdometryMap::pressedPoint, this,
            &MainWindow::setPoint);
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

void MainWindow::setPoint(Point2D p)
{
    static int i = 0;
    if(i == 0)
    {
        astar_->setStart(p.x, p.y);
        i++;
        return;
    }
    else
    {
        i = 0;
        astar_->setStop(p.x, p.y);
        astar_->findPath();
        originalMap_->clearPoints();
        const auto path = astar_->getPath();
        for(const auto &p : path)
        {
            originalMap_->addPoint(p);
        }
        reducedMap_->setMapStruct(astar_->getMap());
    }
}

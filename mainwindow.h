#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "common.h"

class OdometryMap;
class AStar;
class QGridLayout;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent *event);

private:
    // GUI
    QWidget *mainWidget;
    QGridLayout *mainLayout;
    OdometryMap *originalMap_;
    OdometryMap *reducedMap_;
    AStar *astar_;

public slots:
    void setPoint(Point2D p);
};

#endif // MAINWINDOW_H

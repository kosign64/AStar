#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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
};

#endif // MAINWINDOW_H

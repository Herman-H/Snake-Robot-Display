#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include "matlabinterface.h"
#include "graphicsitems.h"
#include <chrono>
#include <bitset>
#include "dimensions.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    enum { ForcePerSegmentColor = Qt::yellow };
    enum { SpeedPerSegmentColor = Qt::blue };
    enum { TorquePerSegmentColor = Qt::red };
    enum { TotalForceColor = Qt::yellow };
    enum { TotalSpeedColor = Qt::blue };

    enum {
        READ_STATE_NONE,
        READ_STATE_FILE,
        READ_STATE_MMAP
    };

    enum {
        SIM_PAUSED,
        SIM_PLAYING
    };

    enum { SLIDER_MAX_VALUE = 1000000 };

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    static const quint32 REFRESH_INTERVAL_MILLISEC = 20;
    Ui::MainWindow *ui;
    matlabSharedMemoryInterface * mli;
    matlabFileInterface * mlf;
    QGraphicsScene * m_graphics;
    bool exit;
    bool isRefreshing;
    bool isOnFirstIteration;
    int numSegments;
    int iteration;
    int readState;

    float timeScale;
    float simulationTime;
    std::chrono::time_point<std::chrono::system_clock> beginRt;
    int simState;

    QVector<GraphicsSegmentItem*> segments;
    QVector<GraphicsArrowItem*> forces;
    QVector<GraphicsArrowItem*> speeds;
    GraphicsTorqueDisplay* torques;
    GraphicsArrowItem* totalForce;
    GraphicsArrowItem* totalSpeed;

    float getHeadingAngleOfSnake(const float headAngle, const std::vector<snakeSectionData> & sections);

    void removeAll(GraphicsArrowItem *items);
    void removeAll(GraphicsTorqueDisplay* items);
    void removeAll(QVector<GraphicsArrowItem*>& items);
    void removeAll(QVector<GraphicsSegmentItem*>& items);

    void toggleGroup(QVector<GraphicsArrowItem*>& g, bool state, bool &toggled);
    void toggleGroup(GraphicsArrowItem * g, bool state, bool &toggled);
    void toggleGroup(GraphicsTorqueDisplay * g, bool state, bool &toggled);


    bool showForcesStateChanged;
    bool showSpeedsStateChanged;
    bool showTorquesStateChanged;
    bool showTotForceStateChanged;
    bool showTotSpeedStateChanged;


    void changeSegments(const int numberOfSegments, float headX, float headY, float headAngle, std::vector<snakeSectionData> & sections);
    void updateSegments(const int, float headX, float headY, float headAngle, std::vector<snakeSectionData> &sections);

    void printState();

    std::pair<float,float> getTotalForce(std::vector<snakeSectionData>& sections)
    {
        float rx = 0.0f;
        float ry = 0.0f;
        for(unsigned int i = 0; i < sections.size(); ++i)
        {
            rx += float(sections[i].f_res_x);
            ry += float(sections[i].f_res_y);
        }

        return std::pair<float,float>(rx,ry);
    }

    std::pair<float,float> getMCSpeed(std::vector<snakeSectionData>& sections)
    {
        float rx = 0.0f;
        float ry = 0.0f;
        float f = 1.0f/float(sections.size());
        for(unsigned int i = 0; i < sections.size(); ++i)
        {
            rx += f*float(sections[i].dx);
            ry += f*float(sections[i].dy);
        }

        return std::pair<float,float>(rx,ry);
    }

    std::pair<float,float> getMCPos(std::vector<snakeSectionData>& sections)
    {
        float rx = 0.0f;
        float ry = 0.0f;
        float f = SCALE_ALL_FACTOR*1.0f/float(sections.size());
        for(unsigned int i = 0; i < sections.size(); ++i)
        {
            rx += f*float(sections[i].x);
            ry += f*float(sections[i].y);
        }

        return std::pair<float,float>(rx,ry);
    }

    float getSnakeTangent(std::vector<snakeSectionData>& sections)
    {
        // derive this from the forward speed
        std::pair<float,float> spd = getMCSpeed(sections);
        float avgtheta = atan2(spd.second,spd.first);
        return avgtheta;
    }

    float getSnakeLength(std::vector<snakeSectionData>& sections)
    {
        typedef robot_dimensions<SCALE_ALL_FACTOR> RD;
        float len = 0.0f;
        for(unsigned int i = 0; i < sections.size(); ++i)
        {
            len += RD::segmentMCtoBackwardJointConnection(i) +
                   RD::segmentMCtoForwardJointConnection(i);
        }
        return len;
    }

    std::pair<float,float> getMCSpeedArrowPos(std::vector<snakeSectionData>& sections)
    {
        float len = getSnakeLength(sections);
        float tangentAngle = getSnakeTangent(sections);

        float normalLen = len*0.3f; // Arbitrary constant, whatever seems fit
        float normalX = normalLen*cos(tangentAngle+3.14f/2.0f);
        float normalY = normalLen*sin(tangentAngle+3.14f/2.0f);
        float tangentOffsetLen = len*0.15f;
        float tangentX = tangentOffsetLen*cos(tangentAngle);
        float tangentY = tangentOffsetLen*sin(tangentAngle);

        std::pair<float,float> mcPos = getMCPos(sections);

        return std::pair<float,float>(mcPos.first + normalX + tangentX, mcPos.second + normalY + tangentY);
    }

    std::pair<float,float> getMCForceArrowPos(std::vector<snakeSectionData>& sections)
    {
        float len = getSnakeLength(sections);
        float tangentAngle = getSnakeTangent(sections);

        float normalLen = len*0.3f; // Arbitrary constant, whatever seems fit
        float normalX = normalLen*cos(tangentAngle+3.14f/2.0f);
        float normalY = normalLen*sin(tangentAngle+3.14f/2.0f);
        float tangentOffsetLen = -1.0f*len*0.15f;
        float tangentX = tangentOffsetLen*cos(tangentAngle);
        float tangentY = tangentOffsetLen*sin(tangentAngle);

        std::pair<float,float> mcPos = getMCPos(sections);

        return std::pair<float,float>(mcPos.first + normalX + tangentX, mcPos.second + normalY + tangentY);
    }

    std::pair<float,float> getTorqueDisplayPos(std::vector<snakeSectionData>& sections)
    {
        float len = getSnakeLength(sections);
        float tangentAngle = getSnakeTangent(sections);

        float normalLen = len*0.4f; // Arbitrary constant, whatever seems fit
        float normalX = -normalLen*cos(tangentAngle+3.14f/2.0f);
        float normalY = -normalLen*sin(tangentAngle+3.14f/2.0f);

        std::pair<float,float> mcPos = getMCPos(sections);

        return std::pair<float,float>(mcPos.first + normalX, mcPos.second + normalY);
    }

    void displayMCSpeed(bool show, std::vector<snakeSectionData>& sections, GraphicsArrowItem* totSpd);
    void displayTotalForce(bool show, std::vector<snakeSectionData>& sections, GraphicsArrowItem* totFrc);
    void displayTorque(bool show, std::vector<snakeSectionData>& sections, GraphicsTorqueDisplay* torques);


private slots:
    void refresh(bool doOnce);
    void refreshChain();
    void on_forceVecsButton_toggled(bool);
    void on_speedVecsButton_toggled(bool);
    void on_torquesButton_toggled(bool);
    void on_totForceButton_toggled(bool);
    void on_totSpeedButton_toggled(bool);
    void openFile();
    void openMmap();
    void openDefaultMmap();
    void on_horizontalSlider_sliderMoved(int position);
    void on_playButton_clicked();
    void on_comboBox_currentIndexChanged(const QString &arg1);
};

#endif // MAINWINDOW_H

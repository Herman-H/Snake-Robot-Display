#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>
#include <chrono>
#include <QGraphicsView>
#include <QFileDialog>
#include <QtOpenGL/QGLWidget>
#include <QWindow>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    exit(false),
    isRefreshing(false),
    isOnFirstIteration(true),
    readState(READ_STATE_NONE),
    simState(SIM_PAUSED),
    segments(),
    forces(),
    speeds(),
    torques(nullptr),
    totalForce(nullptr),
    totalSpeed(nullptr),
    timeScale(1.0f)
{
    ui->setupUi(this);
    ui->statusBar->showMessage("No input file is specified");
    QString filePath = QCoreApplication::applicationDirPath() + "/display2Dconnection.dat";
    ui->filepathOut->setText(filePath);
    mli = nullptr;
    mlf = nullptr;
    ui->graphicsView->setScene(m_graphics = new QGraphicsScene());
    ui->graphicsView->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
    ui->graphicsView->setBackgroundBrush(Qt::gray);
    ui->graphicsView->centerOn(0.0f,0.0f);
    ui->graphicsView->update();
    ui->graphicsView->show();
    ui->horizontalSlider->setRange(0, SLIDER_MAX_VALUE);
    ui->horizontalSlider->setEnabled(false);
    ui->playButton->setEnabled(false);
    ui->timeLabel->setEnabled(false);
    showForcesStateChanged = false;
    showSpeedsStateChanged = false;
    showTorquesStateChanged = false;
    showTotForceStateChanged = false;
    showTotSpeedStateChanged = false;
    printState();
    refresh(false);

    QObject::connect(ui->actionSelect_simulation_file,SIGNAL(triggered()),this,SLOT(openFile()));
    QObject::connect(ui->actionSelect_shared_memory_file,SIGNAL(triggered()),this,SLOT(openMmap()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::printState()
{
    if(mli)
    {
        ui->turnOut->setText(QString::number(mli->getTurn()));
        ui->readHBOut->setText(QString::number(mli->getReadHeartbeat()));
        ui->writeHBOut->setText(QString::number(mli->getWriteHeartbeat()));
        ui->numsecOut->setText(QString::number(mli->getNumberOfSections()));
        ui->iterationOut->setText(QString::number(mli->getIteration()));
        ui->msgROut->setText(mli->messageRead() ? "true" : "false");
        ui->msgWOut->setText(mli->messageWritten() ? "true" : "false");
    }
}

void MainWindow::refresh(bool doOnce)
{
    std::chrono::time_point<std::chrono::system_clock> begin = std::chrono::system_clock::now();

    if(mlf && readState == READ_STATE_FILE)
    {
        if(simState == SIM_PLAYING)
        {
            std::chrono::time_point<std::chrono::system_clock> endRt = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_milliseconds = endRt-beginRt;
            beginRt = std::chrono::system_clock::now();

            simulationTime+=timeScale*float(elapsed_milliseconds.count());
            ui->horizontalSlider->blockSignals(true);
            ui->horizontalSlider->setValue(int((simulationTime/mlf->get_lastTime())*float(SLIDER_MAX_VALUE)));
            ui->horizontalSlider->blockSignals(false);
        }
        if(simulationTime >= mlf->get_lastTime())
        {
            simulationTime = mlf->get_lastTime();
        }
        mlf->iterateToClosestTimePoint(simulationTime);
        ui->timeLabel->setText(QString::number(simulationTime,'g',4));
        // Måla upp roboten här
        std::vector<snakeSectionData> sections;
        for(int i = 0; i < mlf->getNumberOfSections(); ++i)
        {
            sections.push_back(mlf->getSection(i));
        }
        updateSegments(mlf->getNumberOfSections(),mlf->get_headX(),mlf->get_headY(),mlf->get_headAngle(),sections);
    }

    if(mli && readState == READ_STATE_MMAP)
    {
        if(mli->readData())
        {
            ui->l1->setText(QString::number(mli->get_headX()));
            ui->l2->setText(QString::number(mli->get_headY()));
            ui->l3->setText(QString::number(mli->get_headAngle()));
            ui->l4->setText(QString::number(mli->getSection(0).d_phi));
            ui->l5->setText(QString::number(mli->getSection(9).f_res_x));
            ui->l6->setText(QString::number(mli->getSection(9).f_res_y));
            ui->l7->setText(QString::number(mli->getSection(0).x));
            ui->l8->setText(QString::number(mli->getSection(0).y));
            ui->l9->setText(QString::number(mli->getSection(0).torque));

            // Prepare data for showing
            std::vector<snakeSectionData> sections;
            sections.clear();
            for(int i = 0; i < mli->getNumberOfSections(); ++i)
            {
                sections.push_back(mli->getSection(i));
            }

            if(mli->getIteration() != iteration && isOnFirstIteration)
            {
                isOnFirstIteration = false;
                iteration = mli->getIteration();
                numSegments = mli->getNumberOfSections();
                changeSegments(mli->getNumberOfSections(),mli->get_headX(),mli->get_headY(),mli->get_headAngle(),sections);
                //changeSegments(mli->getNumberOfSections(),0.0f,0.0f,0.0f,sections);
            }
            else if(mli->getNumberOfSections() != numSegments)
            {
                numSegments = mli->getNumberOfSections();
                iteration = mli->getIteration();
                changeSegments(mli->getNumberOfSections(),mli->get_headX(),mli->get_headY(),mli->get_headAngle(),sections);
                //changeSegments(mli->getNumberOfSections(),0.0f,0.0f,0.0f,sections);
            }
            else if(mli->getIteration() != iteration || doOnce)
            {
                iteration = mli->getIteration();
                updateSegments(mli->getNumberOfSections(),mli->get_headX(),mli->get_headY(),mli->get_headAngle(),sections);
                //updateSegments(mli->getNumberOfSections(),0.0f,0.0f,0.0f,sections);
            }
            ui->graphicsView->update();
            ui->graphicsView->show();

        }
    }

    printState();
    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_milliseconds = end-begin;
    if(!exit)
    {
        if(static_cast<double>(REFRESH_INTERVAL_MILLISEC)-elapsed_milliseconds.count() < 0 && !doOnce)
        {
            refresh(false);
        }
        else if(!doOnce)
        {
            QTimer::singleShot(REFRESH_INTERVAL_MILLISEC - elapsed_milliseconds.count(), this, SLOT(refreshChain()));
        }
    }
}

void MainWindow::refreshChain()
{
    refresh(false);
}

void MainWindow::changeSegments(const int numberOfSegments,
                                float headX, float headY, float headAngle,
                                std::vector<snakeSectionData> & sections)
{
    typedef robot_dimensions<SCALE_ALL_FACTOR> RD;
    removeAll(segments);
    removeAll(forces);
    removeAll(speeds);
    removeAll(torques);
    removeAll(totalForce);
    removeAll(totalSpeed);
    m_graphics->clear();
    showForcesStateChanged = true;
    showSpeedsStateChanged = true;
    showTorquesStateChanged = true;
    showTotForceStateChanged = true;
    showTotSpeedStateChanged = true;
    totalForce = new GraphicsArrowItem(TotalForceColor);
    m_graphics->addItem(totalForce);
    totalSpeed = new GraphicsArrowItem(TotalSpeedColor);
    m_graphics->addItem(totalSpeed);
    torques = new GraphicsTorqueDisplay(0.25f,getSnakeLength(sections),numberOfSegments-1,TorquePerSegmentColor);
    m_graphics->addItem(torques);
    float dx=0.0f,dy=0.0f,rot=0.0f;
    for(int i = 0; i < numberOfSegments; ++i)
    {
        GraphicsSegmentItem * seg = new GraphicsSegmentItem(i,numberOfSegments);
        GraphicsArrowItem * force = new GraphicsArrowItem(/*ForcePerSegmentColor*/Qt::yellow);
        GraphicsArrowItem * speed = new GraphicsArrowItem(SpeedPerSegmentColor);
        seg->setZValue(1.0f);
        force->setZValue(2.0f);
        speed->setZValue(3.0f);
        //GraphicsArrowItem * torque = new GraphicsArrowItem(QColor(TorquePerSegmentColor));
        segments.push_back(seg);
        forces.push_back(force);
        speeds.push_back(speed);
        //torques.push_back(torque);
        m_graphics->addItem(seg);
        m_graphics->addItem(force);
        m_graphics->addItem(speed);
        //m_graphics->addItem(torque);
        if(i == 0)
        {
            rot = headAngle;
            dx = headX*SCALE_ALL_FACTOR;
            dy = headY*SCALE_ALL_FACTOR;
        }
        else
        {
            dx-=cos(rot)*RD::segmentMCtoBackwardJointConnection(i-1);
            dy-=sin(rot)*RD::segmentMCtoBackwardJointConnection(i-1);
            rot+=sections.at(i-1).phi;
            dx-=cos(rot)*RD::segmentMCtoForwardJointConnection(i);
            dy-=sin(rot)*RD::segmentMCtoForwardJointConnection(i);
        }
        seg->setRotation(rot*180/3.14);
        seg->setPos(dx,dy);
        force->setPos(dx,dy);
        force->modify(sections[i].f_res_x,sections[i].f_res_y,ui->forceVecsButton->isChecked());
        speed->setPos(dx,dy);
        speed->modify(sections[i].dx,sections[i].dy,ui->speedVecsButton->isChecked());
        displayMCSpeed(ui->totSpeedButton->isChecked(),sections,totalSpeed);
        displayTotalForce(ui->totForceButton->isChecked(),sections,totalForce);
        displayTorque(ui->torquesButton->isChecked(),sections,torques);
    }
    toggleGroup(forces,ui->forceVecsButton->isChecked(),showForcesStateChanged);
    toggleGroup(speeds,ui->speedVecsButton->isChecked(),showSpeedsStateChanged);
    toggleGroup(torques,ui->torquesButton->isChecked(),showTorquesStateChanged);
    toggleGroup(totalForce,ui->totForceButton->isChecked(),showTotForceStateChanged);
    toggleGroup(totalSpeed,ui->totSpeedButton->isChecked(),showTotSpeedStateChanged);
    showForcesStateChanged = false;
    showSpeedsStateChanged = false;
    showTorquesStateChanged = false;
    showTotForceStateChanged = false;
    showTotSpeedStateChanged = false;
}

void MainWindow::updateSegments(const int /*numberOfSegments*/,
                                float headX, float headY,float headAngle,
                                std::vector<snakeSectionData> & sections)
{
    typedef robot_dimensions<SCALE_ALL_FACTOR> RD;
    float dx=0.0f,dy=0.0f,rot=0.0f;
    for(int i = 0; i < segments.size(); ++i)
    {
        QGraphicsItem* const seg = segments[i];
        GraphicsArrowItem* const force = static_cast<GraphicsArrowItem* const>(forces[i]);
        GraphicsArrowItem* const speed = static_cast<GraphicsArrowItem* const>(speeds[i]);
        //QGraphicsItem* torque = segments->childItems().at(i);
        if(i == 0)
        {
            rot = headAngle;
            dx = headX*SCALE_ALL_FACTOR;
            dy = headY*SCALE_ALL_FACTOR;
        }
        else
        {
            dx-=cos(rot)*RD::segmentMCtoBackwardJointConnection(i-1);
            dy-=sin(rot)*RD::segmentMCtoBackwardJointConnection(i-1);
            rot+=sections.at(i-1).phi;
            dx-=cos(rot)*RD::segmentMCtoForwardJointConnection(i);
            dy-=sin(rot)*RD::segmentMCtoForwardJointConnection(i);

        }
        seg->setRotation(rot*180/3.14);
        seg->setPos(dx,dy);
        force->setPos(dx,dy);
        force->modify(sections[i].f_res_x,sections[i].f_res_y,ui->forceVecsButton->isChecked());
        speed->setPos(dx,dy);
        speed->modify(sections[i].dx,sections[i].dy,ui->speedVecsButton->isChecked());
        displayMCSpeed(ui->totSpeedButton->isChecked(),sections,totalSpeed);
        displayTotalForce(ui->totForceButton->isChecked(),sections,totalForce);
        displayTorque(ui->torquesButton->isChecked(),sections,torques);
    }
    toggleGroup(forces,ui->forceVecsButton->isChecked(),showForcesStateChanged);
    toggleGroup(speeds,ui->speedVecsButton->isChecked(),showSpeedsStateChanged);
    toggleGroup(torques,ui->torquesButton->isChecked(),showTorquesStateChanged);
    toggleGroup(totalForce,ui->totForceButton->isChecked(),showTotForceStateChanged);
    toggleGroup(totalSpeed,ui->totSpeedButton->isChecked(),showTotSpeedStateChanged);
    showForcesStateChanged = false;
    showSpeedsStateChanged = false;
    showTorquesStateChanged = false;
    showTotForceStateChanged = false;
    showTotSpeedStateChanged = false;
}

float MainWindow::getHeadingAngleOfSnake(const float headAngle, const std::vector<snakeSectionData> & sections)
{
    float r = headAngle;
    float factor = 1.0f/float(sections.size());
    for(int i = 0; i < sections.size(); ++i)
    {
        r+=sections[i].phi;
    }
    return factor*r; /* According to the book, (2.2) */
}

void MainWindow::removeAll(GraphicsArrowItem * g)
{
    if(g)
    {
        m_graphics->removeItem(g);
        delete g;
    }
}
void MainWindow::removeAll(GraphicsTorqueDisplay * g)
{
    if(g)
    {
        m_graphics->removeItem(g);
        delete g;
    }
}
void MainWindow::removeAll(QVector<GraphicsArrowItem*>& g)
{
    for(int i = 0; i < g.size(); ++i)
    {
        m_graphics->removeItem(g[i]);
        delete g[i];
    }
    g.clear();
}
void MainWindow::removeAll(QVector<GraphicsSegmentItem*>& g)
{
    for(int i = 0; i < g.size(); ++i)
    {
        m_graphics->removeItem(g[i]);
        delete g[i];
    }
    g.clear();
}

void MainWindow::toggleGroup(GraphicsArrowItem* g, bool state, bool & toggled)
{
    if(state && toggled)
    {
        g->setVisible(true);
    }
    else if(toggled)
    {
        g->setVisible(false);
    }
    toggled = false;
}

void MainWindow::toggleGroup(GraphicsTorqueDisplay* g, bool state, bool & toggled)
{
    if(state && toggled)
    {
        g->setVisible(true);
    }
    else if(toggled)
    {
        g->setVisible(false);
    }
    toggled = false;
}

void MainWindow::toggleGroup(QVector<GraphicsArrowItem*>& g, bool state, bool & toggled)
{
    if(state && toggled)
    {
        for(int i = 0; i < g.size(); ++i)
        {
            g[i]->setVisible(true);
        }
    }
    else if(toggled)
    {
        for(int i = 0; i < g.size(); ++i)
        {
            g[i]->setVisible(false);
        }
    }
    toggled = false;
}


void MainWindow::on_forceVecsButton_toggled(bool /*checked*/)
{
    showForcesStateChanged = true;
    refresh(true);
}

void MainWindow::on_speedVecsButton_toggled(bool /*checked*/)
{
    showSpeedsStateChanged = true;
    refresh(true);
}

void MainWindow::on_torquesButton_toggled(bool /*checked*/)
{
    showTorquesStateChanged = true;
    refresh(true);
}

void MainWindow::on_totForceButton_toggled(bool /*checked*/)
{
    showTotForceStateChanged = true;
    refresh(true);
}

void MainWindow::on_totSpeedButton_toggled(bool /*checked*/)
{
    showTotSpeedStateChanged = true;
    refresh(true);
}

void MainWindow::openFile()
{
    // use *.datf
    QString fname = QFileDialog::getOpenFileName(this,"Load File",QCoreApplication::applicationDirPath(), "Simulation Files (*.datf)");

    // Prevents crashing in case no filename was selected
    if(fname.length() == 0)
    {
        return;
    }
    if(mlf)
    {
        delete mlf;
        mlf = nullptr;
    }
    mlf = new matlabFileInterface(fname);
    simulationTime = 0;
    ui->horizontalSlider->setEnabled(true);
    ui->playButton->setEnabled(true);
    ui->timeLabel->setEnabled(true);

    std::vector<snakeSectionData> sections;
    for(int i = 0; i < mlf->getNumberOfSections(); ++i)
    {
        sections.push_back(mlf->getSection(i));
    }
    changeSegments(mlf->getNumberOfSections(),mlf->get_headX(),mlf->get_headY(),mlf->get_headAngle(),sections);

    ui->statusBar->showMessage(QString("Reading from file ") + fname);
    readState = READ_STATE_FILE;
}

void MainWindow::openMmap()
{
    // use *.datm
    QString fname = QFileDialog::getOpenFileName(this,"Listen on File", QCoreApplication::applicationDirPath(), "Simulation Memory Mapped Files (*.datm)");

    // Prevents crashing in case no filename was selected
    if(fname.length() == 0)
    {
        return;
    }

    if(mli)
    {
        delete mli;
        mli = nullptr;
    }
    mli = new matlabSharedMemoryInterface(fname);

    simState = SIM_PAUSED;
    ui->horizontalSlider->setEnabled(false);
    ui->playButton->setEnabled(false);
    ui->timeLabel->setEnabled(false);

    ui->statusBar->showMessage(QString("Listening on file ") + fname);
    readState = READ_STATE_MMAP;
}

void MainWindow::openDefaultMmap()
{
    // use *.datm
    ui->horizontalSlider->setEnabled(false);
    ui->playButton->setEnabled(false);
    ui->timeLabel->setEnabled(false);
    QString fname("display2Dconnection.datm");

    if(mli)
    {
        delete mli;
        mli = nullptr;
    }
    mli = new matlabSharedMemoryInterface(fname);

    ui->statusBar->showMessage(QString("Listening on file ") + fname);
    readState = READ_STATE_MMAP;
}

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    simulationTime = mlf->get_lastTime()*(float(position)/float(SLIDER_MAX_VALUE));
    mlf->iterateToClosestTimePoint(simulationTime);
    simulationTime = mlf->get_time();
}

void MainWindow::on_playButton_clicked()
{
    if(simState == SIM_PAUSED)
    {
        simState = SIM_PLAYING;
        ui->playButton->setText("Pause");
        beginRt = std::chrono::system_clock::now();
    }
    else if(simState == SIM_PLAYING)
    {
        simState = SIM_PAUSED;
        ui->playButton->setText("Play");
        std::chrono::time_point<std::chrono::system_clock> endRt = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_milliseconds = endRt-beginRt;
        simulationTime+=float(elapsed_milliseconds.count());
        ui->horizontalSlider->blockSignals(true);
        ui->horizontalSlider->setValue(int((simulationTime/mlf->get_lastTime())*float(SLIDER_MAX_VALUE)));
        ui->horizontalSlider->blockSignals(false);
    }
}

void MainWindow::displayMCSpeed(bool show, std::vector<snakeSectionData> &sections, GraphicsArrowItem *totSpd)
{
    std::pair<float,float> pos = getMCSpeedArrowPos(sections);
    std::pair<float,float> spd = getMCSpeed(sections);
    totSpd->setPos(pos.first,pos.second);
    totSpd->modify(spd.first,spd.second,show);
}

void MainWindow::displayTotalForce(bool show, std::vector<snakeSectionData> &sections, GraphicsArrowItem *totFrc)
{
    std::pair<float,float> pos = getMCForceArrowPos(sections);
    std::pair<float,float> frc = getTotalForce(sections);
    totFrc->setPos(pos.first,pos.second);
    totFrc->modify(frc.first*5.0f,frc.second*5.0f,show);
}

void MainWindow::displayTorque(bool show, std::vector<snakeSectionData> &sections, GraphicsTorqueDisplay *torques)
{
    std::pair<float,float> pos = getTorqueDisplayPos(sections);
    float tangentAngle = getSnakeTangent(sections);
    torques->setPos(pos.first,pos.second);
    torques->setRotation(tangentAngle*180.0f/3.14f);
    for(int i = 0; i < sections.size()-1; ++i)
    {
        torques->modify(i,sections[i].torque);
    }
}

void MainWindow::on_comboBox_currentIndexChanged(const QString &arg1)
{
    switch(ui->comboBox->currentIndex())
    {
    case 0:
        timeScale = 2.0f;
        break;
    case 1:
        timeScale = 1.5f;
        break;
    case 2:
        timeScale = 1.0f;
        break;
    case 3:
        timeScale = 0.75f;
        break;
    case 4:
        timeScale = 0.66f;
        break;
    case 5:
        timeScale = 0.5f;
        break;
    case 6:
        timeScale = 0.33f;
        break;
    case 7:
        timeScale = 0.25f;
    case 8:
        timeScale = 0.1f;
    }
}






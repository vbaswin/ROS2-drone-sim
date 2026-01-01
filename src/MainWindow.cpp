#include "MainWindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTimer>
#include <vtkCamera.h>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    // 1. UI Layout Construction
    QWidget* central = new QWidget;
    setCentralWidget(central);
    QVBoxLayout* mainLayout = new QVBoxLayout(central);

    // 2. VTK Integration
    vtkWidget_ = new QVTKOpenGLNativeWidget();
    auto renWin = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    vtkWidget_->setRenderWindow(renWin);
    
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderer_->SetBackground(0.15, 0.15, 0.15); // Dark grey background
    renWin->AddRenderer(renderer_);
    
    mainLayout->addWidget(vtkWidget_);

    // 3. Control Panel
    QHBoxLayout* controls = new QHBoxLayout;
    QPushButton* btnSim = new QPushButton("Start/Stop Sim");
    QPushButton* btnRec = new QPushButton("Record Video");
    controls->addWidget(btnSim);
    controls->addWidget(btnRec);
    mainLayout->addLayout(controls);

    // 4. Signal Connections
    connect(btnSim, &QPushButton::clicked, this, &MainWindow::onToggleSim);
    connect(btnRec, &QPushButton::clicked, this, &MainWindow::onToggleRecord);

    // 5. ROS Worker Setup
    worker_ = new RosWorker(this);
    connect(worker_, &RosWorker::droneStateReceived, this, &MainWindow::onDroneUpdate);
    worker_->start();

    // 6. Camera Defaults
    renderer_->GetActiveCamera()->SetPosition(5, -5, 5);
    renderer_->GetActiveCamera()->SetFocalPoint(0, 0, 0);
    renderer_->GetActiveCamera()->SetViewUp(0, 0, 1);

    // 7. Video Recording Setup
    windowToImage_ = vtkSmartPointer<vtkWindowToImageFilter>::New();
    videoWriter_ = vtkSmartPointer<vtkAVIWriter>::New();
    videoWriter_->SetRate(30); // 30 FPS
    
    recordTimer_ = new QTimer(this);
    connect(recordTimer_, &QTimer::timeout, this, &MainWindow::recordFrame);
}

MainWindow::~MainWindow() {
    if (isRecording_) {
        videoWriter_->End();
    }
}

void MainWindow::onDroneUpdate(QString id, double x, double y, double z, double vx, double vy, double vz) {
    // Lazy initialization of drone actors
    if (drones_.find(id) == drones_.end()) {
        // Assign color based on ID hash for variety
        double r = (qHash(id) % 255) / 255.0;
        double g = (qHash(id + "g") % 255) / 255.0;
        double b = (qHash(id + "b") % 255) / 255.0;
        
        auto drone = std::make_shared<DroneActor>(r, g, b);
        renderer_->AddActor(drone->getBody());
        renderer_->AddActor(drone->getTrail());
        renderer_->AddActor(drone->getPrediction());
        drones_[id] = drone;
        
        // Reset camera to focus on first drone
        if (drones_.size() == 1) {
            renderer_->ResetCamera();
        }
    }

    drones_[id]->updateState(x, y, z, vx, vy, vz);
    vtkWidget_->renderWindow()->Render(); // Request OpenGL repaint
}

void MainWindow::onToggleSim() {
    simPaused_ =!simPaused_;
    worker_->setSimulationPaused(simPaused_);
}

void MainWindow::onToggleRecord() {
    isRecording_ =!isRecording_;
    if (isRecording_) {
        // Start Recording
        videoWriter_->SetFileName("flight_demo.avi");
        windowToImage_->SetInput(vtkWidget_->renderWindow());
        windowToImage_->SetInputBufferTypeToRGB();
        windowToImage_->ReadFrontBufferOff(); // Read from back buffer for speed
        videoWriter_->Start();
        recordTimer_->start(33); // ~30ms interval
    } else {
        // Stop Recording
        recordTimer_->stop();
        videoWriter_->End();
    }
}

void MainWindow::recordFrame() {
    if (isRecording_) {
        windowToImage_->Modified();
        videoWriter_->SetInputConnection(windowToImage_->GetOutputPort());
        videoWriter_->Write();
    }
}

#pragma once
#include <QMainWindow>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>
#include <vtkAVIWriter.h> 
#include <map>
#include "RosWorker.h"
#include "DroneActor.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void onDroneUpdate(QString id, double x, double y, double z, double vx, double vy, double vz);
    void onToggleSim();
    void onToggleRecord();
    void recordFrame(); // Called by timer

private:
    QVTKOpenGLNativeWidget* vtkWidget_;
    vtkSmartPointer<vtkRenderer> renderer_;
    RosWorker* worker_;
    
    std::map<QString, std::shared_ptr<DroneActor>> drones_;
    
    // Simulation State
    bool simPaused_ = false;
    
    // Recording State
    bool isRecording_ = false;
    QTimer* recordTimer_;
    vtkSmartPointer<vtkAVIWriter> videoWriter_;
    vtkSmartPointer<vtkWindowToImageFilter> windowToImage_;
};

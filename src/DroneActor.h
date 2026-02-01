#pragma once

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkLineSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkType.h>

/**
 * @brief Manages the visualization actors for a single drone.
 * Responsibilities: Render body, update trail history, project prediction vector.
 */
class DroneActor {
public:
    DroneActor(double r, double g, double b) {
        // 1. Drone Body (Simple Sphere)
        auto sphere = vtkSmartPointer<vtkSphereSource>::New();
        sphere->SetRadius(0.25);
        sphere->SetThetaResolution(16);
        sphere->SetPhiResolution(16);

        auto droneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        droneMapper->SetInputConnection(sphere->GetOutputPort());

        droneActor_ = vtkSmartPointer<vtkActor>::New();
        droneActor_->SetMapper(droneMapper);
        droneActor_->GetProperty()->SetColor(r, g, b);    // Unique color
        droneActor_->GetProperty()->SetAmbient(0.3);      // ADD: Better lighting
        droneActor_->GetProperty()->SetDiffuse(0.8);      // ADD: Better shading
        droneActor_->GetProperty()->SetSpecular(0.5);     // ADD: Shiny highlight
        droneActor_->GetProperty()->SetSpecularPower(30); // ADD: Tight specular

        // 2. Trajectory Trail (PolyLine)
        trailPoints_ = vtkSmartPointer<vtkPoints>::New();
        trailCells_ = vtkSmartPointer<vtkCellArray>::New();
        trailPolyData_ = vtkSmartPointer<vtkPolyData>::New();
        trailPolyData_->SetPoints(trailPoints_);
        trailPolyData_->SetLines(trailCells_);

        auto trailMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        trailMapper->SetInputData(trailPolyData_);

        trailActor_ = vtkSmartPointer<vtkActor>::New();
        trailActor_->SetMapper(trailMapper);
        trailActor_->GetProperty()->SetColor(r, g, b);
        trailActor_->GetProperty()->SetOpacity(0.8);
        trailActor_->GetProperty()->SetLineWidth(3.0); // Thicker trail

        // 3. Prediction Vector (Line)
        // "Drew out future paths as lines to guess where they’ll go next."
        predictionSource_ = vtkSmartPointer<vtkLineSource>::New();
        auto predMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        predMapper->SetInputConnection(predictionSource_->GetOutputPort());

        predictionActor_ = vtkSmartPointer<vtkActor>::New();
        predictionActor_->SetMapper(predMapper);
        predictionActor_->GetProperty()->SetColor(1.0, 1.0, 0.0);       // Yellow for prediction
        predictionActor_->GetProperty()->SetLineStipplePattern(0xF0F0); // Dashed look
        predictionActor_->GetProperty()->SetLineWidth(2.0);
    }

    // Updates the visual state based on new telemetry
    void updateState(double x, double y, double z, double vx, double vy, double vz) {
        // Move the drone body
        droneActor_->SetPosition(x, y, z);

        // Update the Trail
        // We simply append points.
        // vtkIdType pid = trailPoints_->InsertNextPoint(x, y, z);
        // Ring buffer behavior for trail
        if (trailPoints_->GetNumberOfPoints() >= MAX_TRAIL_POINTS) {
            // Rebuild trail from last N-1 points + new point
            auto newPoints = vtkSmartPointer<vtkPoints>::New();
            auto newCells = vtkSmartPointer<vtkCellArray>::New();

            for (vtkIdType i = 1; i < trailPoints_->GetNumberOfPoints(); ++i) {
                double pt[3];
                trailPoints_->GetPoint(i, pt);
                newPoints->InsertNextPoint(pt);
            }
            newPoints->InsertNextPoint(x, y, z);

            // Rebuild line cells
            for (vtkIdType i = 1; i < trailPoints_->GetNumberOfPoints() - 1; ++i) {
                auto line = vtkSmartPointer<vtkLine>::New();
                line->GetPointIds()->SetId(0, i);
                line->GetPointIds()->SetId(1, i + 1);
                newCells->InsertNextCell(line);
            }
            trailPoints_ = newPoints;
            trailCells_ = newCells;
            trailPolyData_->SetPoints(trailPoints_);
            trailPolyData_->SetLines(trailCells_);
        } else {
            vtkIdType pid = trailPoints_->InsertNextPoint(x, y, z);
            if (pid > 0) {
                auto line = vtkSmartPointer<vtkLine>::New();
                line->GetPointIds()->SetId(0, pid - 1);
                line->GetPointIds()->SetId(1, pid);
                trailCells_->InsertNextCell(line);
            }
        }
        trailPolyData_->Modified();

        // Update Prediction
        // Simple kinematic projection: P_future = P_current + Velocity * Time_Horizon
        double lookahead = 2.0; // Seconds
        predictionSource_->SetPoint1(x, y, z);
        predictionSource_->SetPoint2(x + vx * lookahead, y + vy * lookahead, z + vz * lookahead);
        predictionSource_->Update();
    }

    // Accessors for adding to the Renderer
    vtkSmartPointer<vtkActor> getBody() const { return droneActor_; }
    vtkSmartPointer<vtkActor> getTrail() const { return trailActor_; }
    vtkSmartPointer<vtkActor> getPrediction() const { return predictionActor_; }

private:
    vtkSmartPointer<vtkActor> droneActor_;
    vtkSmartPointer<vtkActor> trailActor_;
    vtkSmartPointer<vtkActor> predictionActor_;

    vtkSmartPointer<vtkPoints> trailPoints_;
    vtkSmartPointer<vtkCellArray> trailCells_;
    vtkSmartPointer<vtkPolyData> trailPolyData_;

    vtkSmartPointer<vtkLineSource> predictionSource_;
    static constexpr size_t MAX_TRAIL_POINTS = 1000;
};

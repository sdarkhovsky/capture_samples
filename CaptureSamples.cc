/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/gui/GuiEvents.hh>

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>

#include "gz/sim/Entity.hh"

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Model.hh>

#include "gz/gui/Application.hh"
#include "gz/gui/GuiEvents.hh"
#include "gz/gui/MainWindow.hh"

#include "gz/math/Rand.hh"

#include <gz/common/Image.hh>

#include "CaptureSamples.hh"

#include <mutex>
#include <vector>
#include <map>

#include <sstream>


namespace capture_samples
{

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return std::move(out).str();
}  

enum CaptureState { moveModels, waitForSceneChange, captureSample };

struct ModelParams
{
  gz::math::Pose3d pose;
};

class CaptureSamplesPrivate
{
  public: 
    void Configure();

    void LoadFrameParams();

    void ResetCaptureWaypointsState();

    void ResetCaptureSamplesState();

    void GenerateSceneCaptureParameters();

    void WriteXMLFrameElement(
      tinyxml2::XMLDocument& xmlDoc,
      std::map<std::string, ModelParams>& trajPoses,
      tinyxml2::XMLElement *scene_capture_paramsXml);

    std::string baseDirectory;

    /// \brief Directory to save screenshots
    std::string picturesDirectory;

    /// \brief SceneCaptureParams.xml file path
    std::string frameParamsFile;

    std::vector< std::map<std::string, ModelParams>> frameParams;

    unsigned long int frameIndex{0};

    unsigned int captureState{CaptureState::moveModels};

    std::mutex captureMutex;

    std::mutex waypointSelectionMutex;    

    gz::sim::Entity lastSelectedEntity;

    bool captureSelectedModelPose{false};

    std::vector< std::map<std::string, ModelParams>> waypointMaps;
    std::map<std::string, ModelParams> waypointMap;

    /// \brief Pointer to the user camera.
    gz::rendering::CameraPtr userCamera{nullptr};    
};

void CaptureSamplesPrivate::ResetCaptureWaypointsState()
{
  waypointMaps.clear();
  waypointMap.clear();  
}

void CaptureSamplesPrivate::ResetCaptureSamplesState()
{
  captureState = CaptureState::moveModels;
  frameIndex = 0;
}

void CaptureSamplesPrivate::Configure()
{
  if (baseDirectory.empty() && !gz::common::env("SCENE_CAPTURE_PATH", baseDirectory)) 
  {
    std::string home;
    gz::common::env(GZ_HOMEDIR, home);
    baseDirectory = 
      gz::common::joinPaths(home, ".ir", "capture_samples");
  }

  // default directory
  picturesDirectory =
      gz::common::joinPaths(baseDirectory, "pictures");

  if (!gz::common::exists(picturesDirectory))
  {
    if (!gz::common::createDirectories(picturesDirectory))
    {
      gzerr << "Unable to create directory [" << picturesDirectory
            << " Images will not be captured!"
            << std::endl;
      picturesDirectory.clear();
    }
  }

  // default directory
  frameParamsFile =
      gz::common::joinPaths(baseDirectory, "SceneCaptureParams.xml");

  LoadFrameParams();

  ResetCaptureWaypointsState();
  ResetCaptureSamplesState();    
}

void CaptureSamplesPrivate::LoadFrameParams()
{
  // if a new base directory is set
  frameParams.clear();

  tinyxml2::XMLDocument xmlDoc;
  if (xmlDoc.LoadFile(frameParamsFile.c_str()) != tinyxml2::XML_SUCCESS) 
  {
    return;
  }

  if (auto frame = xmlDoc.RootElement()->FirstChildElement("frame"))
  {
    while(frame) 
    {
      ModelParams modelParams;
      std::map<std::string, ModelParams> frameMap;

      auto model = frame->FirstChildElement("model");
      while(model)
      {
        auto model_name = model->Attribute("name");
        auto pose = model->FirstChildElement("pose");

        std::stringstream ss(pose->GetText());
        ss >> modelParams.pose;

        frameMap[model_name] = modelParams;

        model = model->NextSiblingElement("model");          
      }

      // todo: check that move constructors are used wherever possible
      frameParams.push_back(frameMap);

      frame = frame->NextSiblingElement("frame");
    }
  }
}

void CaptureSamplesPrivate::WriteXMLFrameElement(tinyxml2::XMLDocument& xmlDoc,
    std::map<std::string, ModelParams>& trajPoses,
    tinyxml2::XMLElement *scene_capture_paramsXml)
{
    tinyxml2::XMLElement *frameXml = xmlDoc.NewElement("frame");

    for (auto model: trajPoses)
    {
        tinyxml2::XMLElement *modelXml = xmlDoc.NewElement("model");
        modelXml->SetAttribute("name", model.first.c_str());
        tinyxml2::XMLElement *poseXml = xmlDoc.NewElement("pose");
        auto pose3d = model.second.pose;
        auto pose = to_string_with_precision(pose3d.X(),2) + " " +
                to_string_with_precision(pose3d.Y(),2) + " " +
                to_string_with_precision(pose3d.Z(),2) + " " +
                to_string_with_precision(pose3d.Roll(),2) + " " +
                to_string_with_precision(pose3d.Pitch(),2) + " " +
                to_string_with_precision(pose3d.Yaw(),2);

        gzmsg << "model " << model.first << " pose3d " << pose3d << std::endl;

        poseXml->LinkEndChild(xmlDoc.NewText(pose.c_str()));
        modelXml->LinkEndChild(poseXml);
        frameXml->LinkEndChild(modelXml);
    }
    scene_capture_paramsXml->LinkEndChild(frameXml);

}

void CaptureSamplesPrivate::GenerateSceneCaptureParameters() 
{
  if (!waypointMap.empty()) 
  {
    waypointMaps.push_back(waypointMap);
    waypointMap.clear();
  }

  tinyxml2::XMLDocument xmlDoc;

  // XML declaration
  xmlDoc.NewDeclaration();

  // Asset element
  tinyxml2::XMLElement *scene_capture_paramsXml = xmlDoc.NewElement("scene_capture_params");
  xmlDoc.LinkEndChild(scene_capture_paramsXml);
  scene_capture_paramsXml->SetAttribute("version", "0.9.0");

  gzmsg << "waypointMaps.size() " << waypointMaps.size() << std::endl;

  std::map<std::string, ModelParams> prevWaypointMap;

  if (waypointMaps.empty())
  {
      gzerr << "Waypoint Maps are empty. Generating Scene Capture Parameters aborted." << std::endl;
      return;
  }

  // check that waypointMaps has the same models in every map
  // and collect the list of the models
  std::vector<std::string> models;
  for (auto model: waypointMaps[0]) 
  {
      models.push_back(model.first);
  }

  for (auto wpMap: waypointMaps)
  {
    for (auto model: models)
    {
      auto search = wpMap.find(model);
      if ( search == wpMap.end())
      {
        gzerr << "model " << model 
          << " is present only in some waypoints. Generating Scene Capture Parameters aborted."
          << std::endl;
        return;
      }
    }
  }

  for (auto wpMap: waypointMaps)
  {
    if (prevWaypointMap.empty())
    {
      prevWaypointMap = wpMap;
      continue;
    }

    // trajectory from previous waypoints prevWaypointMap to the current ones wpMap
    const int num_steps = 10; 
    for (int step = 0; step < num_steps; step++)
    {
        std::map<std::string, ModelParams> trajPoses;
        for (auto model: models)
        {
          double t = static_cast<double>(step) / static_cast<double>(num_steps);
          double x = prevWaypointMap[model].pose.X() + 
                t*(wpMap[model].pose.X()-prevWaypointMap[model].pose.X());
          double y = prevWaypointMap[model].pose.Y() + 
              t*(wpMap[model].pose.Y()-prevWaypointMap[model].pose.Y());
          double z = prevWaypointMap[model].pose.Z() + 
              t*(wpMap[model].pose.Z()-prevWaypointMap[model].pose.Z());
          double roll = prevWaypointMap[model].pose.Roll() + 
              t*(wpMap[model].pose.Roll()-prevWaypointMap[model].pose.Roll());
          double pitch = prevWaypointMap[model].pose.Pitch() + 
              t*(wpMap[model].pose.Pitch()-prevWaypointMap[model].pose.Pitch());
          double yaw = prevWaypointMap[model].pose.Yaw() + 
              t*(wpMap[model].pose.Yaw()-prevWaypointMap[model].pose.Yaw());

          double rand_min = 0.0;
          double rand_max = 1.0;
          trajPoses[model].pose.Set(
              x+gz::math::Rand::DblUniform(rand_min, rand_max),
              y+gz::math::Rand::DblUniform(rand_min, rand_max),
              z+gz::math::Rand::DblUniform(rand_min, rand_max),
              roll+gz::math::Rand::DblUniform(rand_min, rand_max),
              pitch+gz::math::Rand::DblUniform(rand_min, rand_max),
              yaw+gz::math::Rand::DblUniform(rand_min, rand_max));
        }
        WriteXMLFrameElement(xmlDoc, trajPoses, scene_capture_paramsXml);
    }

    prevWaypointMap = wpMap;
  }
  
  if (xmlDoc.SaveFile(frameParamsFile.c_str()) != tinyxml2::XML_SUCCESS) 
  {
      gzerr << "Could not save the generated scene capture parameters to the file " 
            << frameParamsFile
            << " Try to find the cause and press the Generate button again"
            << std::endl;
  }
  else
  {
    ResetCaptureWaypointsState();
  }

  return; 
}

/////////////////////////////////////////////////
CaptureSamples::CaptureSamples()
  : dataPtr(std::make_unique<CaptureSamplesPrivate>())
{
  this->dataPtr->Configure();
  this->BaseDirectoryChanged();  
}

/////////////////////////////////////////////////
CaptureSamples::~CaptureSamples() = default;


/////////////////////////////////////////////////
QString CaptureSamples::BaseDirectory() const
{
  return QString::fromStdString(this->dataPtr->baseDirectory);
}

/////////////////////////////////////////////////
void CaptureSamples::SetBaseDirectory(const QString &_baseDirectory)
{
  QString baseDirectory = QUrl(_baseDirectory).toLocalFile();
  this->dataPtr->baseDirectory = baseDirectory.toStdString();
  this->dataPtr->Configure();
  this->BaseDirectoryChanged();
}

/////////////////////////////////////////////////
void CaptureSamples::OnCaptureSelectedModelPose()
{
  // remember which entity was clicked, save its name and pose
  std::lock_guard<std::mutex> lock(this->dataPtr->waypointSelectionMutex);  

  this->dataPtr->captureSelectedModelPose = true;
}

/////////////////////////////////////////////////
void CaptureSamples::OnGenerateSceneCaptureParameters()
{
  this->dataPtr->GenerateSceneCaptureParameters();
}

/////////////////////////////////////////////////
void CaptureSamples::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "CaptureSamples";

  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);  
}

/////////////////////////////////////////////////
bool CaptureSamples::eventFilter(QObject *_obj, QEvent *_event)
{
  // GUI events:  https://doc.qt.io/qt-6/qevent.html
  // gz-sim/include/gz/sim/gui/GuiEvents.hh
  // gz-gui/include/gz/gui/GuiEvents.hh
  if (_event->type() == gz::sim::gui::events::EntitiesSelected::kType) 
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->waypointSelectionMutex);
    gz::sim::gui::events::EntitiesSelected *entitiesSelected =
      static_cast<gz::sim::gui::events::EntitiesSelected*>(_event);

      /// \brief An Entity identifies a single object in simulation such as
      /// a model, link, or light. At its core, an Entity is just an identifier.
      /// See gz-sim/include/gz/sim/Entity.hh
      std::vector<gz::sim::Entity> entities = entitiesSelected->Data();
      this->dataPtr->lastSelectedEntity = entities[0];

      gzmsg << "entities.size() " << entities.size() << std::endl;
      for (auto e: entities) {
        gzmsg << "e " << e << std::endl;
      }
  }

  /// \brief Event called in the render thread of a 3D scene after the user
  /// camera has rendered (gz-gui/include/gz/gui/GuiEvents.hh)
  if (_event->type() == gz::gui::events::Render::kType)
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->captureMutex);

    if (this->dataPtr->captureState >= CaptureState::waitForSceneChange &&
        this->dataPtr->captureState < CaptureState::captureSample) 
    {
      this->dataPtr->captureState++;
    } 
    else if (this->dataPtr->captureState >= CaptureState::captureSample)
    {
      this->dataPtr->captureState = CaptureState::moveModels;
      std::string fileName = std::to_string(this->dataPtr->frameIndex);
      this->dataPtr->frameIndex++;

      this->SaveScreenshot(fileName);
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void CaptureSamples::FindUserCamera()
{
  if (nullptr != this->dataPtr->userCamera)
    return;

  // Get first scene
  auto scene = gz::rendering::sceneFromFirstRenderEngine();

  if (!scene)
    return;

  for (unsigned int i = 0; i < scene->NodeCount(); ++i)
  {
    auto cam = std::dynamic_pointer_cast<gz::rendering::Camera>(
        scene->NodeByIndex(i));

    if (nullptr != cam)
    {
      this->dataPtr->userCamera = cam;
      gzdbg << "CaptureSamples plugin taking pictures of camera ["
             << this->dataPtr->userCamera->Name() << "]" << std::endl;
      break;
    }
  }
}

//////////////////////////////////////////////////
void CaptureSamples::UpdateWaypointMap(const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->waypointSelectionMutex);

  if (!_info.paused) 
  {
    return;
  }

  // save the name and pose of the selected entity
  if (this->dataPtr->captureSelectedModelPose)
  {
      this->dataPtr->captureSelectedModelPose = false;

      gzmsg << "UpdateWaypointMap captureSelectedModelPose" << std::endl;

      _ecm.Each<gz::sim::components::Name,
                gz::sim::components::Model>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Name *_name,
            const gz::sim::components::Model *)->bool
      {

        gzmsg << "UpdateWaypointMap lastSelectedEntity " 
              << this->dataPtr->lastSelectedEntity 
              << " _entity " << _entity << std::endl;


        if (_entity == this->dataPtr->lastSelectedEntity) 
        {
            auto search = this->dataPtr->waypointMap.find(_name->Data());
            if ( search != this->dataPtr->waypointMap.end())
            {
              gzmsg << "Storing to waypointMaps" << std::endl;

              // store previous waypointMaps and start a new one if 
              // the same object was clicked again
              this->dataPtr->waypointMaps.push_back(this->dataPtr->waypointMap);
              this->dataPtr->waypointMap.clear();
            } 
            auto poseComp = _ecm.Component<gz::sim::components::Pose>(
                _entity);
            ModelParams modelParams = {poseComp->Data()};
            this->dataPtr->waypointMap[_name->Data()] = modelParams;

            // break out of loop
            return false;
        }

        return true;
      });
  }
}

//////////////////////////////////////////////////
void CaptureSamples::UpdateModelPoses(const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->captureMutex);

  if (_info.paused) 
  {
    return;
  }

  // reset captureState if the reset was clicked in WorldControl GUI
  if (_info.iterations == 1)
  {
    this->dataPtr->ResetCaptureSamplesState();
  }  

  if (this->dataPtr->captureState == CaptureState::moveModels) 
  {
    if (this->dataPtr->frameIndex < this->dataPtr->frameParams.size()) 
    {
      this->dataPtr->captureState = CaptureState::waitForSceneChange;

      const std::map<std::string, ModelParams>& frameMap = 
        this->dataPtr->frameParams[this->dataPtr->frameIndex];


      _ecm.Each<gz::sim::components::Name,
                gz::sim::components::Model>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Name *_name,
            const gz::sim::components::Model *)->bool
      {
        // gz::sim::components are in gz-sim/include/gz/sim/components
        // gz::math::Pose3d is in gz-math/include/gz/math/Pose3.hh
        auto search = frameMap.find(_name->Data());
        if ( search != frameMap.end()) 
        {
          auto poseComp = _ecm.Component<gz::sim::components::Pose>(
              _entity);

          gz::math::Pose3d& pose = poseComp->Data();
          pose = search->second.pose;

          // Updates to pose component can usually be missed, so we need to mark this
          // as an important one-time change.
          _ecm.SetChanged(_entity, gz::sim::components::Pose::typeId,
              gz::sim::ComponentState::OneTimeChange);
        }

        // continue the loop
        return true;
      });
    }
  }
}

//////////////////////////////////////////////////
void CaptureSamples::Update(const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager &_ecm)
{
  UpdateWaypointMap(_info,_ecm);
  UpdateModelPoses(_info,_ecm);
}

///////////////////////////////////////////////// 
// fileName: file name without an extension, e.g. "1"
void CaptureSamples::SaveScreenshot(const std::string& fileName)
{
  FindUserCamera();

  if (nullptr == this->dataPtr->userCamera)
    return;

  if (this->dataPtr->picturesDirectory.empty()) {
      gzerr << "The directory " << this->dataPtr->picturesDirectory
            << " doesn't exists. Images will not be captured!"
            << std::endl;
      return;
  }

  unsigned int width = this->dataPtr->userCamera->ImageWidth();
  unsigned int height = this->dataPtr->userCamera->ImageHeight();

  auto cameraImage = this->dataPtr->userCamera->CreateImage();
  this->dataPtr->userCamera->Copy(cameraImage);
  auto formatStr =
      gz::rendering::PixelUtil::Name(this->dataPtr->userCamera->ImageFormat());
  auto format = gz::common::Image::ConvertPixelFormat(formatStr);

  std::string savePath = gz::common::joinPaths(this->dataPtr->picturesDirectory, fileName +".png");

  gz::common::Image image;
  image.SetFromData(cameraImage.Data<unsigned char>(), width, height, format);

  image.SavePNG(savePath);

  gzdbg << "Saved image to [" << savePath << "]" << std::endl;
}

}   // namespace capture_samples

// Register this plugin
GZ_ADD_PLUGIN(capture_samples::CaptureSamples,
              gz::gui::Plugin)

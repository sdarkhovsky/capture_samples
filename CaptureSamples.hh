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

#ifndef GZ_SIM_CAPTURESAMPLES_HH_
#define GZ_SIM_CAPTURESAMPLES_HH_

#include <memory>

#include <gz/sim/gui/GuiSystem.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Scene.hh>
#include <gz/sim/components/Pose.hh>

namespace capture_samples
{

class CaptureSamplesPrivate;

/// \brief Example of a GUI plugin that has access to entities and components.
class CaptureSamples : public gz::sim::GuiSystem
{
  Q_OBJECT

  /// \brief Custom property. Use this to create properties that can be read
  /// from the QML file. See the declarations below.
  Q_PROPERTY(
    QString baseDirectory
    READ BaseDirectory
    WRITE SetBaseDirectory
    NOTIFY BaseDirectoryChanged
  )  

  /// \brief Constructor
  public: CaptureSamples();

  /// \brief Destructor
  public: ~CaptureSamples() override;

  /// \brief `gz::gui::Plugin`s can overload this function to
  /// receive custom configuration from an XML file. Here, it comes from the
  /// SDF.
  ///
  /// <gui>
  ///   <plugin ...> <!-- this is the plugin element -->
  ///     ...
  ///   </plugin>
  /// </gui>
  ///
  /// \param[in] _pluginElem SDF <plugin> element. Will be null if the plugin
  /// is loaded without any XML configuration.
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief GUI systems can overload this function to receive updated simulation
  /// state. This is called whenever the server sends state updates to the GUI.
  /// \param[in] _info Simulation information such as time.
  /// \param[in] _ecm Entity component manager, which can be used to get the
  /// latest information about entities and components.
  public: void Update(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  private: void UpdateModelPoses(const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager &_ecm);

  private: void UpdateWaypointMap(const gz::sim::UpdateInfo & _info,
      gz::sim::EntityComponentManager &_ecm);

  public: void FindUserCamera();

  public: void SaveScreenshot(const std::string& fileName);

  private: bool eventFilter(QObject *_obj, QEvent *_event);

  /// \brief Callback when capture of a model's pose is requested from the GUI.
  public slots: void OnCaptureSelectedModelPose();

  /// \brief Callback when generation of the scene capture parameters is requested from the GUI.
  public slots: void OnGenerateSceneCaptureParameters();

  /// \internal
  /// \brief Pointer to private data.
  private: std::unique_ptr<CaptureSamplesPrivate> dataPtr;

  /// \brief Get the base directory as a string.
  /// \return Base directory
  public: Q_INVOKABLE QString BaseDirectory() const;

  /// \brief Set the base directory from a string.
  /// \param[in] _baseDirectory Base directory
  public: Q_INVOKABLE void SetBaseDirectory(const QString &_baseDirectory);

  /// \brief Notify that base directory has changed
  signals: void BaseDirectoryChanged();
};

} // namespace capture_samples


#endif

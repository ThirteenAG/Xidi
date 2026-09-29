/***************************************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 ***************************************************************************************************
 * Authored by Samuel Grossman
 * Copyright (c) 2016-2026
 ***********************************************************************************************//**
 * @file PhysicalController.cpp
 *   Implementation of all functionality for communicating with physical controllers.
 **************************************************************************************************/

#include "PhysicalController.h"

#include <cstdint>
#include <mutex>
#include <set>
#include <stop_token>
#include <string>
#include <thread>

#include <Infra/Core/Message.h>

#include "ApiWindows.h"
#include "ConcurrencyWrapper.h"
#include "ForceFeedbackDevice.h"
#include "Globals.h"
#include "ImportApiWinMM.h"
#include "Mapper.h"
#include "PhysicalControllerBackend.h"
#include "PhysicalControllerBackendXInput.h"
#include "PhysicalControllerTypes.h"
#include "PluginRegistry.h"
#include "Strings.h"
#include "VirtualController.h"
#include "VirtualControllerTypes.h"

#include "Extensions/PhysicalController.inl"

namespace Xidi
{
  namespace Controller
  {
    /// Raw physical state data for each of the possible physical controllers.
    static ConcurrencyWrapper<SPhysicalState> physicalControllerState[kVirtualControllerMaxCount];

    /// State data for each of the possible physical controllers after it is passed through a mapper
    /// but without any further processing.
    static ConcurrencyWrapper<SState> rawVirtualControllerState[kVirtualControllerMaxCount];

    /// Per-controller force feedback device buffer objects.
    /// These objects are not safe for dynamic initialization, so they are initialized later by
    /// pointer.
    static ForceFeedback::Device* physicalControllerForceFeedbackBuffer;

    /// Pointers to the virtual controller objects registered for force feedback with each physical
    /// controller.
    static std::set<const VirtualController*>
        physicalControllerForceFeedbackRegistration[kVirtualControllerMaxCount];

    /// Mutex objects for protecting against concurrent accesses to the physical controller force
    /// feedback registration data.
    static std::mutex physicalControllerForceFeedbackMutex[kVirtualControllerMaxCount];

    /// Interface pointer for the configured physical controller backend.
    static IPhysicalControllerBackend* physicalControllerBackend = nullptr;

    /// Computes an opaque source identifier from a given controller identifier.
    /// @param [in] controllerIdentifier Identifier of the physical controller for which an
    /// identifier is needed.
    /// @return Opaque identifier that can be passed to mappers.
    static inline uint32_t OpaqueControllerSourceIdentifier(
        TControllerIdentifier controllerIdentifier)
    {
      return (uint32_t)controllerIdentifier;
    }

    /// Reads physical controller state. Called very often and therefore, for the sake of
    /// performance, accesses the backend pointer directly.
    /// @param [in] controllerIdentifier Identifier of the controller on which to operate.
    /// @return Physical state of the identified controller.
    static SPhysicalState ReadPhysicalControllerState(TControllerIdentifier controllerIdentifier)
    {
      return physicalControllerBackend->ReadInputState(controllerIdentifier);
    }

    /// Scales a vibration strength value by the specified scaling factor. If the resulting strength
    /// exceeds the maximum possible strength it is saturated at the maximum possible strength.
    /// @param [in] vibrationStrength Physical motor vibration strength value.
    /// @param [in] scalingFactor Scaling factor by which to scale up or down the physical motor
    /// vibration strength value.
    /// @return Scaled physical motor vibration strength value that can then be sent directly to the
    /// physical motor.
    static ForceFeedback::TPhysicalActuatorValue ScaledVibrationStrength(
        ForceFeedback::TPhysicalActuatorValue vibrationStrength, double scalingFactor)
    {
      if (0.0 == scalingFactor)
        return 0;
      else if (1.0 == scalingFactor)
        return static_cast<ForceFeedback::TPhysicalActuatorValue>(vibrationStrength);

      constexpr double kMaxVibrationStrength =
          static_cast<double>(std::numeric_limits<ForceFeedback::TPhysicalActuatorValue>::max());
      const double scaledVibrationStrength = static_cast<double>(vibrationStrength) * scalingFactor;

      return static_cast<ForceFeedback::TPhysicalActuatorValue>(
          std::min(scaledVibrationStrength, kMaxVibrationStrength));
    }

    /// Writes a vibration command to a physical controller. Called very often and therefore, for
    /// the sake of performance, accesses the backend pointer directly.
    /// @param [in] controllerIdentifier Identifier of the controller on which to operate.
    /// @param [in] vibration Physical actuator vibration vector.
    /// @return `true` if successful, `false` otherwise.
    static bool WritePhysicalControllerVibration(
        TControllerIdentifier controllerIdentifier,
        ForceFeedback::SPhysicalActuatorComponents vibration)
    {
      static const double kForceFeedbackEffectStrengthScalingFactor =
          static_cast<double>(
              Globals::GetConfigurationData()
                  [Strings::kStrConfigurationSectionProperties]
                  [Strings::kStrConfigurationSettingPropertiesForceFeedbackEffectStrengthPercent]
                      .ValueOr(100)) /
          100.0;

      const ForceFeedback::SPhysicalActuatorComponents scaledVibration = {
          .leftMotor = ScaledVibrationStrength(
              vibration.leftMotor, kForceFeedbackEffectStrengthScalingFactor),
          .rightMotor = ScaledVibrationStrength(
              vibration.rightMotor, kForceFeedbackEffectStrengthScalingFactor),
          .leftImpulseTrigger = ScaledVibrationStrength(
              vibration.leftImpulseTrigger, kForceFeedbackEffectStrengthScalingFactor),
          .rightImpulseTrigger = ScaledVibrationStrength(
              vibration.rightImpulseTrigger, kForceFeedbackEffectStrengthScalingFactor)};

      return physicalControllerBackend->WriteForceFeedbackState(
          controllerIdentifier, scaledVibration);
    }

    /// Periodically plays force feedback effects on the physical controller actuators.
    /// @param [in] controllerIdentifier Identifier of the controller on which to operate.
    static void ForceFeedbackActuateEffects(TControllerIdentifier controllerIdentifier)
    {
      constexpr ForceFeedback::TOrderedMagnitudeComponents kVirtualMagnitudeVectorZero = {};

      ForceFeedback::SPhysicalActuatorComponents previousPhysicalActuatorValues;
      ForceFeedback::SPhysicalActuatorComponents currentPhysicalActuatorValues;

      const Mapper* mapper = Mapper::GetConfigured(controllerIdentifier);
      bool lastActuationResult = true;

      while (true)
      {
        if (true == lastActuationResult)
          Sleep(kPhysicalForceFeedbackPeriodMilliseconds);
        else
          Sleep(kPhysicalErrorBackoffPeriodMilliseconds);

        if (true == Globals::DoesCurrentProcessHaveInputFocus())
        {
          mapper = Extensions::ForceFeedbackMapper(controllerIdentifier, mapper);
          ForceFeedback::TEffectValue overallEffectGain = 10000;
          ForceFeedback::SPhysicalActuatorComponents physicalActuatorVector = {};
          ForceFeedback::TOrderedMagnitudeComponents virtualMagnitudeVector =
              physicalControllerForceFeedbackBuffer[controllerIdentifier].PlayEffects();

          if (kVirtualMagnitudeVectorZero != virtualMagnitudeVector)
          {
            std::unique_lock lock(physicalControllerForceFeedbackMutex[controllerIdentifier]);

            // Gain is modified downwards by each virtual controller object.
            // Typically there would only be one, in which case the properties of that object would
            // be effective. Otherwise this loop is essentially modeled as multiple volume knobs
            // connected in sequence, each lowering the volume of the effects by the value of its
            // own device-wide gain property.
            for (auto virtualController :
                 physicalControllerForceFeedbackRegistration[controllerIdentifier])
              overallEffectGain *=
                  ((ForceFeedback::TEffectValue)virtualController->GetForceFeedbackGain() /
                   ForceFeedback::kEffectModifierMaximum);

            physicalActuatorVector = mapper->MapForceFeedbackVirtualToPhysical(
                virtualMagnitudeVector, overallEffectGain);
          }

          currentPhysicalActuatorValues = physicalActuatorVector;
          Extensions::ApplyRequestedVibration(controllerIdentifier, currentPhysicalActuatorValues);
        }
        else
        {
          currentPhysicalActuatorValues = {};
        }

        if (previousPhysicalActuatorValues != currentPhysicalActuatorValues)
        {
          lastActuationResult =
              WritePhysicalControllerVibration(controllerIdentifier, currentPhysicalActuatorValues);
          previousPhysicalActuatorValues = currentPhysicalActuatorValues;
        }
        else
        {
          lastActuationResult = true;
        }
      }
    }

    /// Periodically polls for physical controller state.
    /// On detected state change, updates the internal data structure and notifies all waiting
    /// threads.
    /// @param [in] controllerIdentifier Identifier of the controller on which to operate.
    static void PollForPhysicalControllerStateChanges(TControllerIdentifier controllerIdentifier)
    {
      SPhysicalState newPhysicalState = physicalControllerState[controllerIdentifier].Get();
      Extensions::ProfileSwitcher profileSwitcher(
          controllerIdentifier, OpaqueControllerSourceIdentifier(controllerIdentifier));

      while (true)
      {
        if (EPhysicalDeviceStatus::Ok == newPhysicalState.deviceStatus)
          Sleep(kPhysicalPollingPeriodMilliseconds);
        else
          Sleep(kPhysicalErrorBackoffPeriodMilliseconds);

        newPhysicalState = ReadPhysicalControllerState(controllerIdentifier);
        const bool mapperChanged = profileSwitcher.Refresh(newPhysicalState);

        if ((true == physicalControllerState[controllerIdentifier].Update(newPhysicalState)) ||
            (true == mapperChanged))
        {
          const SState newRawVirtualState =
              ((EPhysicalDeviceStatus::Ok == newPhysicalState.deviceStatus)
                   ? profileSwitcher.GetMapper()
                         ->MapStatePhysicalToVirtual(
                             newPhysicalState,
                             OpaqueControllerSourceIdentifier(controllerIdentifier))
                   : profileSwitcher.GetMapper()
                         ->MapNeutralPhysicalToVirtual(
                             OpaqueControllerSourceIdentifier(controllerIdentifier)));

          rawVirtualControllerState[controllerIdentifier].Update(newRawVirtualState);
        }
      }
    }

    /// Monitors physical controller status for events like hardware connection or disconnection and
    /// error conditions. Used exclusively for logging. Intended to be a thread entry point, one
    /// thread per monitored physical controller.
    /// @param [in] controllerIdentifier Identifier of the controller to monitor.
    static void MonitorPhysicalControllerStatus(TControllerIdentifier controllerIdentifier)
    {
      if (controllerIdentifier >= VirtualController::GetActualCount())
      {
        Infra::Message::OutputFormatted(
            Infra::Message::ESeverity::Error,
            L"Attempted to monitor physical controller with invalid identifier %u.",
            controllerIdentifier);
        return;
      }

      SPhysicalState oldPhysicalState = GetCurrentPhysicalControllerState(controllerIdentifier);
      SPhysicalState newPhysicalState = oldPhysicalState;

      while (true)
      {
        WaitForPhysicalControllerStateChange(
            controllerIdentifier, newPhysicalState, std::stop_token());

        // Look for status changes and output to the log, as appropriate.
        switch (newPhysicalState.deviceStatus)
        {
          case EPhysicalDeviceStatus::Ok:
            switch (oldPhysicalState.deviceStatus)
            {
              case EPhysicalDeviceStatus::Ok:
                break;

              case EPhysicalDeviceStatus::NotConnected:
                Infra::Message::OutputFormatted(
                    Infra::Message::ESeverity::Info,
                    L"Physical controller %u: Hardware connected.",
                    (1 + controllerIdentifier));
                break;

              default:
                Infra::Message::OutputFormatted(
                    Infra::Message::ESeverity::Warning,
                    L"Physical controller %u: Cleared previous error condition.",
                    (1 + controllerIdentifier));
                break;
            }
            break;

          case EPhysicalDeviceStatus::NotConnected:
            if (newPhysicalState.deviceStatus != oldPhysicalState.deviceStatus)
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Info,
                  L"Physical controller %u: Hardware disconnected.",
                  (1 + controllerIdentifier));
            break;

          default:
            if (newPhysicalState.deviceStatus != oldPhysicalState.deviceStatus)
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Warning,
                  L"Physical controller %u: Encountered an error condition.",
                  (1 + controllerIdentifier));
            break;
        }

        oldPhysicalState = newPhysicalState;
      }
    }

    /// Initializes the configured physical controller backend. Idempotent and concurrency-safe.
    static void InitializePhysicalControllerBackend(void)
    {
      // There is overhead to using call_once, even after the operation is completed, and physical
      // controller functions are called frequently. Using this additional flag avoids that overhead
      // in the common case.
      static bool isInitialized = false;
      if (true == isInitialized) return;

      static std::once_flag initFlag;
      std::call_once(
          initFlag,
          []() -> void
          {
            // Plugins may contain physical controller backends, so they must be loaded before
            // proceeding. This call is idempotent.
            LoadConfiguredPlugins();

            std::wstring_view selectedBackend =
                Globals::GetConfigurationData()[Infra::Configuration::kSectionNameGlobal]
                                               [Strings::kStrConfigurationSettingControllerBackend]
                                                   .ValueOr(L"");
            physicalControllerBackend =
                ((true == selectedBackend.empty())
                     ? new PhysicalControllerBackendXInput()
                     : GetPhysicalControllerBackendInterface(selectedBackend));

            if (nullptr == physicalControllerBackend)
            {
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Error,
                  L"Physical controller backend \"%.*s\" could not be located. Using the built-in default backend instead.",
                  static_cast<int>(selectedBackend.length()),
                  selectedBackend.data());
              physicalControllerBackend = new PhysicalControllerBackendXInput();
            }

            const bool backendInitializationResult = physicalControllerBackend->Initialize();
            if (false == backendInitializationResult)
            {
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Error,
                  L"Physical controller backend \"%.*s\" failed to initialize. Using the built-in default backend instead.",
                  static_cast<int>(selectedBackend.length()),
                  selectedBackend.data());
              physicalControllerBackend = new PhysicalControllerBackendXInput();
            }

            isInitialized = true;
          });
    }

    /// Initializes internal data structures and creates worker threads. Idempotent and
    /// concurrency-safe.
    static void Initialize(void)
    {
      // There is overhead to using call_once, even after the operation is completed, and physical
      // controller functions are called frequently. Using this additional flag avoids that overhead
      // in the common case.
      static bool isInitialized = false;
      if (true == isInitialized) return;

      static std::once_flag initFlag;
      std::call_once(
          initFlag,
          []() -> void
          {
            Infra::Message::OutputFormatted(
                Infra::Message::ESeverity::Info,
                L"Selected physical controller backend \"%.*s\" and creating %u virtual controller%s.",
                static_cast<int>(GetPhysicalControllerBackend()->PluginName().length()),
                GetPhysicalControllerBackend()->PluginName().data(),
                static_cast<unsigned int>(VirtualController::GetActualCount()),
                ((1 == VirtualController::GetActualCount()) ? L"" : L"s"));

            // Initialize controller state data structures.
            for (auto controllerIdentifier = 0;
                 controllerIdentifier < VirtualController::GetActualCount();
                 ++controllerIdentifier)
            {
              const SPhysicalState initialPhysicalState =
                  ReadPhysicalControllerState(controllerIdentifier);
              const SState initialRawVirtualState =
                  Mapper::GetConfigured(controllerIdentifier)
                      ->MapStatePhysicalToVirtual(
                          initialPhysicalState,
                          OpaqueControllerSourceIdentifier(controllerIdentifier));

              physicalControllerState[controllerIdentifier].Set(initialPhysicalState);
              rawVirtualControllerState[controllerIdentifier].Set(initialRawVirtualState);
            }

            // Ensure the system timer resolution is suitable for the desired polling frequency.
            TIMECAPS timeCaps;
            MMRESULT timeResult = ImportApiWinMM::timeGetDevCaps(&timeCaps, sizeof(timeCaps));
            if (MMSYSERR_NOERROR == timeResult)
            {
              timeResult = ImportApiWinMM::timeBeginPeriod(timeCaps.wPeriodMin);

              if (MMSYSERR_NOERROR == timeResult)
                Infra::Message::OutputFormatted(
                    Infra::Message::ESeverity::Info,
                    L"Set the system timer resolution to %u ms.",
                    timeCaps.wPeriodMin);
              else
                Infra::Message::OutputFormatted(
                    Infra::Message::ESeverity::Warning,
                    L"Failed with code %u to set the system timer resolution.",
                    timeResult);
            }
            else
            {
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Warning,
                  L"Failed with code %u to obtain system timer resolution information.",
                  timeResult);
            }

            // Create and start the polling threads.
            for (auto controllerIdentifier = 0;
                 controllerIdentifier < VirtualController::GetActualCount();
                 ++controllerIdentifier)
            {
              std::thread(PollForPhysicalControllerStateChanges, controllerIdentifier).detach();
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Info,
                  L"Initialized the physical controller state polling thread for controller %u. Desired polling period is %u ms.",
                  (unsigned int)(1 + controllerIdentifier),
                  kPhysicalPollingPeriodMilliseconds);
            }

            // Allocate the force feedback device buffers, then create and start the force feedback
            // threads.
            physicalControllerForceFeedbackBuffer =
                new ForceFeedback::Device[kVirtualControllerMaxCount];
            for (auto controllerIdentifier = 0;
                 controllerIdentifier < VirtualController::GetActualCount();
                 ++controllerIdentifier)
            {
              std::thread(ForceFeedbackActuateEffects, controllerIdentifier).detach();
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Info,
                  L"Initialized the physical controller force feedback actuation thread for controller %u. Desired actuation period is %u ms.",
                  (unsigned int)(1 + controllerIdentifier),
                  kPhysicalForceFeedbackPeriodMilliseconds);
            }

            // Create and start the physical controller hardware status monitoring threads, but only
            // if the messages generated by those threads will actually be delivered as output.
            if (Infra::Message::WillOutputMessageOfSeverity(Infra::Message::ESeverity::Warning))
            {
              for (auto controllerIdentifier = 0;
                   controllerIdentifier < VirtualController::GetActualCount();
                   ++controllerIdentifier)
              {
                std::thread(MonitorPhysicalControllerStatus, controllerIdentifier).detach();
                Infra::Message::OutputFormatted(
                    Infra::Message::ESeverity::Info,
                    L"Initialized the physical controller hardware status monitoring thread for controller %u.",
                    (unsigned int)(1 + controllerIdentifier));
              }
            }

            isInitialized = true;
          });
    }

    IPhysicalControllerBackend* GetPhysicalControllerBackend(void)
    {
      InitializePhysicalControllerBackend();
      return physicalControllerBackend;
    }

    SCapabilities GetVirtualControllerCapabilities(TControllerIdentifier controllerIdentifier)
    {
      Initialize();
      return Mapper::GetConfigured(controllerIdentifier)->GetCapabilities();
    }

    SPhysicalState GetCurrentPhysicalControllerState(TControllerIdentifier controllerIdentifier)
    {
      Initialize();
      return physicalControllerState[controllerIdentifier].Get();
    }

    SState GetCurrentRawVirtualControllerState(TControllerIdentifier controllerIdentifier)
    {
      Initialize();
      return rawVirtualControllerState[controllerIdentifier].Get();
    }

    ForceFeedback::Device* PhysicalControllerForceFeedbackRegister(
        TControllerIdentifier controllerIdentifier, const VirtualController* virtualController)
    {
      Initialize();

      if (controllerIdentifier >= VirtualController::GetActualCount())
      {
        Infra::Message::OutputFormatted(
            Infra::Message::ESeverity::Error,
            L"Attempted to register with a physical controller for force feedback with invalid identifier %u.",
            controllerIdentifier);
        return nullptr;
      }

      std::unique_lock lock(physicalControllerForceFeedbackMutex[controllerIdentifier]);
      physicalControllerForceFeedbackRegistration[controllerIdentifier].insert(virtualController);

      return &physicalControllerForceFeedbackBuffer[controllerIdentifier];
    }

    void PhysicalControllerForceFeedbackUnregister(
        TControllerIdentifier controllerIdentifier, const VirtualController* virtualController)
    {
      Initialize();

      if (controllerIdentifier >= VirtualController::GetActualCount())
      {
        Infra::Message::OutputFormatted(
            Infra::Message::ESeverity::Error,
            L"Attempted to unregister with a physical controller for force feedback with invalid identifier %u.",
            controllerIdentifier);
        return;
      }

      std::unique_lock lock(physicalControllerForceFeedbackMutex[controllerIdentifier]);
      physicalControllerForceFeedbackRegistration[controllerIdentifier].erase(virtualController);
    }

    bool WaitForPhysicalControllerStateChange(
        TControllerIdentifier controllerIdentifier,
        SPhysicalState& state,
        std::stop_token stopToken)
    {
      Initialize();

      if (controllerIdentifier >= VirtualController::GetActualCount()) return false;

      return physicalControllerState[controllerIdentifier].WaitForUpdate(state, stopToken);
    }

    bool WaitForRawVirtualControllerStateChange(
        TControllerIdentifier controllerIdentifier, SState& state, std::stop_token stopToken)
    {
      Initialize();

      if (controllerIdentifier >= VirtualController::GetActualCount()) return false;

      return rawVirtualControllerState[controllerIdentifier].WaitForUpdate(state, stopToken);
    }
  } // namespace Controller
} // namespace Xidi

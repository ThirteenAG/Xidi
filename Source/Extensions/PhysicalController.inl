/***************************************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 ***************************************************************************************************
 * Fork extension. Kept out of the upstream source files so that upstream changes merge cleanly.
 ***********************************************************************************************//**
 * @file PhysicalController.inl
 *   Application-facing extensions to physical controller functionality: switching mappers
 *   ("profiles") at runtime under application control, and application-driven vibration.
 *   Included by PhysicalController.cpp at file scope, after its other includes.
 **************************************************************************************************/

#include <algorithm>
#include <array>
#include <atomic>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>

#include <Infra/Core/Message.h>

#include "ElementMapper.h"
#include "ForceFeedbackTypes.h"
#include "Mapper.h"
#include "PhysicalController.h"
#include "PhysicalControllerTypes.h"
#include "VirtualController.h"
#include "VirtualControllerTypes.h"

namespace Xidi
{
  namespace Controller
  {
    namespace Extensions
    {
      // Published by each controller's existing polling thread, before mapping.
      // Timestamps share GetTickCount64's clock with application keyboard/mouse input.
      static std::array<std::atomic<uint64_t>, kVirtualControllerMaxCount> lastControllerActivity{};

      class ControllerActivityTracker
      {
      public:
        bool Sample(SPhysicalState state)
        {
          if (EPhysicalDeviceStatus::Ok != state.deviceStatus)
          {
            previous = {};
            return false;
          }
          // Reject centered-stick drift and quantize analog noise, including triggers.
          for (size_t stick = 0; stick < state.stick.size(); stick += 2)
          {
            auto& x = state.stick[stick];
            auto& y = state.stick[stick + 1];
            const int64_t deadzone = (stick == 0) ? 7849 : 8689;
            if (int64_t(x) * x + int64_t(y) * y <= deadzone * deadzone)
              x = y = 0;
            else
            {
              x = int16_t(x / 1024 * 1024);
              y = int16_t(y / 1024 * 1024);
            }
          }
          for (auto& trigger : state.trigger)
            trigger = (trigger < 30) ? 0 : uint8_t(trigger / 8 * 8);

          bool activity = (state.button & ~previous.button).any();
          for (size_t i = 0; i < state.stick.size(); ++i)
            activity |= (state.stick[i] != 0 && state.stick[i] != previous.stick[i]);
          for (size_t i = 0; i < state.trigger.size(); ++i)
            activity |= (state.trigger[i] != 0 && state.trigger[i] != previous.trigger[i]);
          previous = state;
          return activity;
        }

      private:
        SPhysicalState previous{};
      };

      /// Application-supplied callback that selects the mapper to use by returning its name.
      /// Returning `nullptr` or an empty string means the callback expresses no preference.
      using TProfileCallback = const wchar_t* (*)(void);

      /// Maximum number of profile callbacks that can be registered.
      inline constexpr size_t kProfileCallbackMaxCount = 8;

      /// Number of physical controller elements in a mapper's element map.
      inline constexpr unsigned int kElementMapCount =
          static_cast<unsigned int>(std::extent_v<decltype(Mapper::UElementMap::all)>);

      /// Distance from center beyond which the left stick counts as held. Same as the deadzone
      /// recommended by XInput for the left stick.
      inline constexpr int32_t kHeldThresholdStickLeft = 7849;

      /// Distance from center beyond which the right stick counts as held. Same as the deadzone
      /// recommended by XInput for the right stick.
      inline constexpr int32_t kHeldThresholdStickRight = 8689;

      /// Value above which a trigger counts as held. Same as the threshold recommended by XInput.
      inline constexpr uint8_t kHeldThresholdTrigger = 30;

      /// Registered profile callbacks, in registration order. Slots are claimed by incrementing
      /// #profileCallbackCount before being filled, so readers must tolerate empty slots.
      static std::array<std::atomic<TProfileCallback>, kProfileCallbackMaxCount> profileCallbacks;

      /// Number of profile callback slots that have been claimed.
      static std::atomic<size_t> profileCallbackCount;

      /// Mapper most recently requested by the application for each physical controller, or
      /// `nullptr` if none has been requested. Written by the polling threads and read by the force
      /// feedback threads. Only ever points to registered mappers, which are never destroyed.
      static std::atomic<const Mapper*> requestedMappers[kVirtualControllerMaxCount];

      // Copy stable registered mapper pointers, never a temporary composite mapper.
      // This also preserves the per-element mappings held over during a profile switch.
      static std::mutex promptMapperGuard;
      static std::array<const Mapper*, kElementMapCount> promptMappers[kVirtualControllerMaxCount];

      static void PublishPromptMappers(TControllerIdentifier id,
          const std::array<const Mapper*, kElementMapCount>& mappers)
      {
        std::scoped_lock lock(promptMapperGuard);
        promptMappers[id] = mappers;
      }

      /// Vibration requested by the application for each specific physical controller, packed by
      /// #PackVibration.
      static std::atomic<uint32_t> requestedVibration[kVirtualControllerMaxCount];

      /// Vibration requested by the application for whichever physical controller is the first
      /// connected one, packed by #PackVibration.
      static std::atomic<uint32_t> requestedVibrationFirstConnected;

      /// Invokes the registered profile callbacks in registration order.
      /// @return Name of the mapper requested by the first callback that expresses a preference,
      /// or `nullptr` if none does.
      static const wchar_t* QueryProfileCallbacks(void)
      {
        const size_t callbackCount = profileCallbackCount.load(std::memory_order_acquire);

        for (size_t i = 0; i < callbackCount; ++i)
        {
          const TProfileCallback callback = profileCallbacks[i].load(std::memory_order_acquire);
          if (nullptr == callback) continue;

          const wchar_t* const profileName = callback();
          if ((nullptr != profileName) && (L'\0' != profileName[0])) return profileName;
        }

        return nullptr;
      }

      /// Retrieves the mapper whose force feedback actuator configuration should be used for the
      /// specified physical controller.
      /// @param [in] controllerIdentifier Identifier of the physical controller.
      /// @param [in] defaultMapper Mapper to use if the application has not requested one.
      /// @return Mapper to use for force feedback.
      static const Mapper* ForceFeedbackMapper(
          TControllerIdentifier controllerIdentifier, const Mapper* defaultMapper)
      {
        const Mapper* const mapper =
            requestedMappers[controllerIdentifier].load(std::memory_order_acquire);
        return ((nullptr != mapper) ? mapper : defaultMapper);
      }

      /// Switches the mapper used for a physical controller to the one requested by the
      /// application. The switch happens element by element: an element that is held (button
      /// pressed, trigger pulled, or stick deflected) when a switch is requested keeps being
      /// handled by the mapper it was pressed under until it is released. A switch therefore never
      /// releases or re-presses anything the user is holding. For example, a button held to keep an
      /// in-game menu open stays held across the switch to the menu's mapper, even if that mapper
      /// does not map the button at all. While any element is held over this way, a temporary
      /// unregistered mapper is assembled from copies of the individual element mappers in effect.
      /// Each instance is owned and used exclusively by the polling thread of one controller.
      class ProfileSwitcher
      {
      public:

        /// @param [in] controllerIdentifier Identifier of the physical controller.
        /// @param [in] sourceControllerIdentifier Opaque source identifier that is passed to the
        /// mapper when mapping state for this physical controller.
        ProfileSwitcher(
            TControllerIdentifier controllerIdentifier, uint32_t sourceControllerIdentifier)
            : controllerIdentifier(controllerIdentifier),
              sourceControllerIdentifier(sourceControllerIdentifier),
              configuredMapper(Mapper::GetConfigured(controllerIdentifier)),
              requestedMapper(configuredMapper),
              currentMapper(configuredMapper),
              elementMappers(),
              compositeMapper(),
              hasHeldOverElements(false),
              lastRequestedProfileName(),
              lastRequestedProfileMapper(configuredMapper)
        {
          elementMappers.fill(configuredMapper);
          PublishPromptMappers(controllerIdentifier, elementMappers);
        }

        ProfileSwitcher(const ProfileSwitcher&) = delete;

        ProfileSwitcher& operator=(const ProfileSwitcher&) = delete;

        /// Retrieves the mapper that should currently be used for mapping physical controller
        /// state to virtual controller state.
        /// @return Mapper to use.
        inline const Mapper* GetMapper(void) const
        {
          return currentMapper;
        }

        /// Asks the application which mapper it wants and hands over to it every element that is
        /// not being held. Intended to be invoked once per polling iteration, before mapping.
        /// @param [in] physicalState Most recent physical controller state.
        /// @return `true` if the mapper to use changed, in which case virtual controller state must
        /// be recomputed even if physical controller state is unchanged, `false` otherwise.
        bool Refresh(const SPhysicalState& physicalState)
        {
          const bool activity = activityTracker.Sample(physicalState);
          if (EPhysicalDeviceStatus::Ok != physicalState.deviceStatus)
            lastControllerActivity[controllerIdentifier].store(0, std::memory_order_relaxed);
          else if (activity)
            lastControllerActivity[controllerIdentifier].store(GetTickCount64(), std::memory_order_relaxed);

          // Callbacks run application code, so they are invoked only while there is a connected
          // controller whose input actually needs to be mapped.
          if (EPhysicalDeviceStatus::Ok == physicalState.deviceStatus)
            SetRequestedMapper(QueryRequestedMapper());

          if ((currentMapper == requestedMapper) && (false == hasHeldOverElements)) return false;

          const std::bitset<kElementMapCount> heldElements = HeldElements(physicalState);
          bool elementHandedOver = false;
          bool elementHeldOver = false;
          SState discardedState = {};

          for (unsigned int elementIndex = 0; elementIndex < kElementMapCount; ++elementIndex)
          {
            if (elementMappers[elementIndex] == requestedMapper) continue;

            if (true == heldElements[elementIndex])
            {
              elementHeldOver = true;
              continue;
            }

            // Let the outgoing element mapper undo its side effects, such as pressed keyboard keys
            // or ongoing mouse movement. Whatever it contributes to virtual controller state is
            // discarded because that state is recomputed from scratch using the new mapper.
            const IElementMapper* const outgoingElementMapper =
                elementMappers[elementIndex]->ElementMap().all[elementIndex].get();
            if (nullptr != outgoingElementMapper)
              outgoingElementMapper->ContributeNeutral(
                  discardedState, SourceIdentifierForElement(elementIndex));

            elementMappers[elementIndex] = requestedMapper;
            elementHandedOver = true;
          }

          hasHeldOverElements = elementHeldOver;
          PublishPromptMappers(controllerIdentifier, elementMappers);

          if (false == hasHeldOverElements)
          {
            // Every element is now handled by the requested mapper, so it can be used directly.
            compositeMapper.reset();
            currentMapper = requestedMapper;
            return true;
          }

          // Nothing changed if no element was handed over. The current mapper, whether registered
          // or composite, already reflects the per-element assignments.
          if (false == elementHandedOver) return false;

          compositeMapper = BuildCompositeMapper();
          currentMapper = compositeMapper.get();
          return true;
        }

      private:

        ControllerActivityTracker activityTracker;

        /// Determines whether an analog stick is deflected far enough to count as held.
        static constexpr bool IsStickHeld(int16_t valueX, int16_t valueY, int32_t threshold)
        {
          const int64_t distanceSquared =
              (static_cast<int64_t>(valueX) * valueX) + (static_cast<int64_t>(valueY) * valueY);
          return (distanceSquared > (static_cast<int64_t>(threshold) * threshold));
        }

        /// Determines which physical controller elements are held.
        /// @param [in] physicalState Physical controller state to examine.
        /// @return Held elements, indexed in element map order.
        static std::bitset<kElementMapCount> HeldElements(const SPhysicalState& physicalState)
        {
          std::bitset<kElementMapCount> heldElements;
          if (EPhysicalDeviceStatus::Ok != physicalState.deviceStatus) return heldElements;

          // Both axes of a stick are held over together so that a stick is never split between two
          // different mappers.
          const bool isStickLeftHeld = IsStickHeld(
              physicalState[EPhysicalStick::LeftX],
              physicalState[EPhysicalStick::LeftY],
              kHeldThresholdStickLeft);
          const bool isStickRightHeld = IsStickHeld(
              physicalState[EPhysicalStick::RightX],
              physicalState[EPhysicalStick::RightY],
              kHeldThresholdStickRight);

          heldElements[ELEMENT_MAP_INDEX_OF(stickLeftX)] = isStickLeftHeld;
          heldElements[ELEMENT_MAP_INDEX_OF(stickLeftY)] = isStickLeftHeld;
          heldElements[ELEMENT_MAP_INDEX_OF(stickRightX)] = isStickRightHeld;
          heldElements[ELEMENT_MAP_INDEX_OF(stickRightY)] = isStickRightHeld;

          heldElements[ELEMENT_MAP_INDEX_OF(triggerLT)] =
              (physicalState[EPhysicalTrigger::LT] > kHeldThresholdTrigger);
          heldElements[ELEMENT_MAP_INDEX_OF(triggerRT)] =
              (physicalState[EPhysicalTrigger::RT] > kHeldThresholdTrigger);

          heldElements[ELEMENT_MAP_INDEX_OF(dpadUp)] = physicalState[EPhysicalButton::DpadUp];
          heldElements[ELEMENT_MAP_INDEX_OF(dpadDown)] = physicalState[EPhysicalButton::DpadDown];
          heldElements[ELEMENT_MAP_INDEX_OF(dpadLeft)] = physicalState[EPhysicalButton::DpadLeft];
          heldElements[ELEMENT_MAP_INDEX_OF(dpadRight)] = physicalState[EPhysicalButton::DpadRight];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonA)] = physicalState[EPhysicalButton::A];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonB)] = physicalState[EPhysicalButton::B];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonX)] = physicalState[EPhysicalButton::X];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonY)] = physicalState[EPhysicalButton::Y];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonLB)] = physicalState[EPhysicalButton::LB];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonRB)] = physicalState[EPhysicalButton::RB];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonBack)] = physicalState[EPhysicalButton::Back];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonStart)] = physicalState[EPhysicalButton::Start];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonLS)] = physicalState[EPhysicalButton::LS];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonRS)] = physicalState[EPhysicalButton::RS];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonGuide)] = physicalState[EPhysicalButton::Guide];
          heldElements[ELEMENT_MAP_INDEX_OF(buttonShare)] = physicalState[EPhysicalButton::Share];

          return heldElements;
        }

        /// Assembles an unregistered mapper that uses, for each element, a copy of the element
        /// mapper from the mapper currently assigned to that element.
        /// @return Newly-created composite mapper.
        std::unique_ptr<const Mapper> BuildCompositeMapper(void) const
        {
          Mapper::UElementMap compositeElements;

          for (unsigned int elementIndex = 0; elementIndex < kElementMapCount; ++elementIndex)
          {
            const IElementMapper* const elementMapper =
                elementMappers[elementIndex]->ElementMap().all[elementIndex].get();
            if (nullptr != elementMapper)
              compositeElements.all[elementIndex] = elementMapper->Clone();
          }

          return std::make_unique<Mapper>(
              std::move(compositeElements.named),
              requestedMapper->GetForceFeedbackActuatorMap().named);
        }

        /// Asks the application which mapper it wants to use.
        /// @return Requested mapper, or the configured mapper if the application has no preference
        /// or requests a mapper that does not exist.
        const Mapper* QueryRequestedMapper(void)
        {
          const wchar_t* const profileName = QueryProfileCallbacks();

          if (nullptr == profileName)
          {
            lastRequestedProfileName.clear();
            lastRequestedProfileMapper = configuredMapper;
            return configuredMapper;
          }

          if (lastRequestedProfileName != profileName)
          {
            lastRequestedProfileName = profileName;
            lastRequestedProfileMapper = Mapper::GetByName(lastRequestedProfileName);

            if (nullptr == lastRequestedProfileMapper)
            {
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Warning,
                  L"Physical controller %u: Application requested unknown mapper \"%s\". Using the configured mapper instead.",
                  (1 + static_cast<unsigned int>(controllerIdentifier)),
                  lastRequestedProfileName.c_str());
              lastRequestedProfileMapper = configuredMapper;
            }
          }

          return lastRequestedProfileMapper;
        }

        /// Records the mapper requested by the application and publishes it to the force feedback
        /// thread.
        /// @param [in] mapper Requested mapper.
        void SetRequestedMapper(const Mapper* mapper)
        {
          if (mapper == requestedMapper) return;

          requestedMapper = mapper;
          requestedMappers[controllerIdentifier].store(mapper, std::memory_order_release);

          Infra::Message::OutputFormatted(
              Infra::Message::ESeverity::Info,
              L"Physical controller %u: Switching to mapper \"%.*s\".",
              (1 + static_cast<unsigned int>(controllerIdentifier)),
              static_cast<int>(mapper->GetName().length()),
              mapper->GetName().data());
        }

        /// Computes the opaque source identifier that the mapper passes to an element mapper.
        /// Must match the computation in Mapper.cpp so that side effects are attributed to the
        /// same source regardless of which code path invokes the element mapper.
        /// @param [in] elementIndex Index of the element within the element map.
        /// @return Opaque source identifier for the element mapper.
        inline uint32_t SourceIdentifierForElement(unsigned int elementIndex) const
        {
          return (sourceControllerIdentifier << 8) + elementIndex;
        }

        /// Identifier of the physical controller.
        const TControllerIdentifier controllerIdentifier;

        /// Opaque source identifier passed to the mapper for this physical controller.
        const uint32_t sourceControllerIdentifier;

        /// Mapper configured for this physical controller, used when the application has no
        /// preference.
        const Mapper* const configuredMapper;

        /// Mapper most recently requested by the application.
        const Mapper* requestedMapper;

        /// Mapper currently used for mapping, either a registered mapper or #compositeMapper.
        const Mapper* currentMapper;

        /// Registered mapper currently responsible for each element, in element map order.
        std::array<const Mapper*, kElementMapCount> elementMappers;

        /// Composite mapper in use while elements are held over, `nullptr` otherwise.
        std::unique_ptr<const Mapper> compositeMapper;

        /// Whether any element is held over from a mapper other than the requested one.
        bool hasHeldOverElements;

        /// Name most recently returned by the profile callbacks, used to avoid repeated lookups.
        std::wstring lastRequestedProfileName;

        /// Mapper resolved from #lastRequestedProfileName.
        const Mapper* lastRequestedProfileMapper;
      };

      /// Packs a pair of motor speeds into a single value that can be stored atomically.
      static constexpr uint32_t PackVibration(uint16_t leftMotorSpeed, uint16_t rightMotorSpeed)
      {
        return ((static_cast<uint32_t>(leftMotorSpeed) << 16) | rightMotorSpeed);
      }

      /// Extracts the left motor speed from a value packed by #PackVibration.
      static constexpr uint16_t UnpackVibrationLeft(uint32_t vibration)
      {
        return static_cast<uint16_t>(vibration >> 16);
      }

      /// Extracts the right motor speed from a value packed by #PackVibration.
      static constexpr uint16_t UnpackVibrationRight(uint32_t vibration)
      {
        return static_cast<uint16_t>(vibration & 0xffff);
      }

      /// Determines which physical controller is the first connected one.
      /// @return Identifier of the first connected physical controller, if any is connected.
      static std::optional<TControllerIdentifier> FirstConnectedController(void)
      {
        for (TControllerIdentifier controllerIdentifier = 0;
             controllerIdentifier < VirtualController::GetActualCount();
             ++controllerIdentifier)
        {
          if (EPhysicalDeviceStatus::Ok ==
              GetCurrentPhysicalControllerState(controllerIdentifier).deviceStatus)
            return controllerIdentifier;
        }

        return std::nullopt;
      }

      /// Combines vibration requested by the application with the physical actuator values that
      /// result from force feedback effects. Each motor runs at the stronger of the two. Intended
      /// to be invoked by the force feedback thread only while the process has input focus.
      /// @param [in] controllerIdentifier Identifier of the physical controller.
      /// @param [in,out] physicalActuatorValues Physical actuator values to be modified.
      static void ApplyRequestedVibration(
          TControllerIdentifier controllerIdentifier,
          ForceFeedback::SPhysicalActuatorComponents& physicalActuatorValues)
      {
        uint32_t vibration =
            requestedVibration[controllerIdentifier].load(std::memory_order_relaxed);

        const uint32_t vibrationFirstConnected =
            requestedVibrationFirstConnected.load(std::memory_order_relaxed);
        if ((0 != vibrationFirstConnected) && (FirstConnectedController() == controllerIdentifier))
        {
          vibration = PackVibration(
              std::max(
                  UnpackVibrationLeft(vibration), UnpackVibrationLeft(vibrationFirstConnected)),
              std::max(
                  UnpackVibrationRight(vibration), UnpackVibrationRight(vibrationFirstConnected)));
        }

        if (0 == vibration) return;

        physicalActuatorValues.leftMotor = std::max(
            physicalActuatorValues.leftMotor,
            static_cast<ForceFeedback::TPhysicalActuatorValue>(UnpackVibrationLeft(vibration)));
        physicalActuatorValues.rightMotor = std::max(
            physicalActuatorValues.rightMotor,
            static_cast<ForceFeedback::TPhysicalActuatorValue>(UnpackVibrationRight(vibration)));
      }
    } // namespace Extensions
  } // namespace Controller
} // namespace Xidi

/// Registers a callback that selects the mapper to use by name. Callbacks are invoked from the
/// physical controller polling threads, frequently, and must therefore be fast and thread-safe.
/// When more than one is registered, the first one that returns a non-empty name wins. Elements
/// held while the mapper changes keep their previous mapping until released.
/// @param [in] callback Callback to register.
/// @return `true` if the callback was registered, `false` if it is `nullptr` or all slots are
/// taken.
extern "C" __declspec(dllexport) bool XidiRegisterProfileCallback(const wchar_t* (*callback)())
{
  using namespace Xidi::Controller::Extensions;

  if (nullptr == callback) return false;

  size_t callbackIndex = profileCallbackCount.load(std::memory_order_acquire);
  while (callbackIndex < kProfileCallbackMaxCount)
  {
    if (true == profileCallbackCount.compare_exchange_weak(callbackIndex, callbackIndex + 1))
    {
      profileCallbacks[callbackIndex].store(callback, std::memory_order_release);
      return true;
    }
  }

  return false;
}

/// Sets the vibration of a physical controller. Takes effect alongside any force feedback effects
/// the application plays through DirectInput, with each motor running at the stronger of the two,
/// and only while the application has input focus. Remains in effect until changed, so the
/// application must set both speeds to 0 to stop.
/// @param [in] controllerIndex Zero-based index of the physical controller, or -1 for the first
/// connected one.
/// @param [in] leftMotorSpeed Left (low-frequency) motor speed.
/// @param [in] rightMotorSpeed Right (high-frequency) motor speed.
/// @return `true` if the target physical controller is connected, `false` otherwise.
extern "C" __declspec(dllexport) bool XidiSendVibration(
    short controllerIndex, unsigned short leftMotorSpeed, unsigned short rightMotorSpeed)
{
  using namespace Xidi::Controller;
  using namespace Xidi::Controller::Extensions;

  const uint32_t vibration = PackVibration(leftMotorSpeed, rightMotorSpeed);

  if (-1 == controllerIndex)
  {
    requestedVibrationFirstConnected.store(vibration, std::memory_order_relaxed);
    return FirstConnectedController().has_value();
  }

  if ((controllerIndex < 0) ||
      (static_cast<TControllerIdentifier>(controllerIndex) >= VirtualController::GetActualCount()))
    return false;

  const TControllerIdentifier controllerIdentifier =
      static_cast<TControllerIdentifier>(controllerIndex);
  requestedVibration[controllerIdentifier].store(vibration, std::memory_order_relaxed);
  return (
      EPhysicalDeviceStatus::Ok ==
      GetCurrentPhysicalControllerState(controllerIdentifier).deviceStatus);
}

/// Returns a bitmask of physical elements feeding a virtual button in the current effective
/// profile. Button index is zero-based; bit positions match XidiSetButtonLabel. Zero means
/// unmapped/unavailable. No game callbacks are invoked on the caller's thread.
extern "C" __declspec(dllexport) uint32_t XidiGetPhysicalButtonMask(unsigned int controllerIndex, unsigned int buttonIndex)
{
  using namespace Xidi::Controller;
  using namespace Xidi::Controller::Extensions;
  if (controllerIndex >= kVirtualControllerMaxCount || buttonIndex >= static_cast<unsigned int>(EButton::Count)) return 0;
  std::array<const Mapper*, kElementMapCount> snapshot;
  {
    std::scoped_lock lock(promptMapperGuard);
    snapshot = promptMappers[controllerIndex];
  }
  uint32_t mask = 0;
  for (unsigned int i = 0; i < kElementMapCount; ++i)
  {
    if (nullptr == snapshot[i]) continue;
    const auto* element = snapshot[i]->ElementMap().all[i].get();
    if (nullptr == element) continue;
    for (int j = 0; j < element->GetTargetElementCount(); ++j)
    {
      const auto target = element->GetTargetElementAt(j);
      if (target && target->type == EElementType::Button && static_cast<unsigned int>(target->button) == buttonIndex)
        mask |= uint32_t(1) << i;
    }
  }
  return mask;
}

/// GetTickCount64 timestamp of the last button press or deliberate analog movement, before
/// profile mapping. Works with every configured backend, including non-XInput controllers.
/// Returns 0 for no activity, a disconnected controller, or an invalid zero-based index.
/// Reads an atomic snapshot only: never polls hardware or invokes application callbacks.
extern "C" __declspec(dllexport) uint64_t XidiGetLastControllerActivity(unsigned int controllerIndex)
{
  using namespace Xidi::Controller;
  using namespace Xidi::Controller::Extensions;
  if (controllerIndex >= kVirtualControllerMaxCount) return 0;
  return lastControllerActivity[controllerIndex].load(std::memory_order_relaxed);
}

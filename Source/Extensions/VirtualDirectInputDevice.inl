/***************************************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 ***************************************************************************************************
 * Fork extension. Kept out of the upstream source files so that upstream changes merge cleanly.
 ***********************************************************************************************//**
 * @file VirtualDirectInputDevice.inl
 *   Application-facing extension that lets the application choose the names DirectInput reports
 *   for virtual controller elements, for example to show gamepad button names in a key binding
 *   menu. Included by VirtualDirectInputDevice.cpp at file scope, after its other includes.
 **************************************************************************************************/

#include <array>
#include <cstddef>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>

#include "ApiWindows.h"
#include "ElementMapper.h"
#include "Mapper.h"
#include "VirtualControllerTypes.h"

namespace Xidi
{
  namespace Extensions
  {
    /// Custom labels for virtual controller elements, identified by the physical controller
    /// element that is mapped to them, in element map order.
    class ElementLabels
    {
    public:

      /// Number of physical controller elements that can be labelled.
      static constexpr int kLabelCount =
          static_cast<int>(std::extent_v<decltype(Controller::Mapper::UElementMap::all)>);

      /// Retrieves the singleton instance of this class.
      static ElementLabels& GetInstance(void)
      {
        static ElementLabels instance;
        return instance;
      }

      /// Sets or clears the label for a physical controller element.
      /// @param [in] index Index of the physical controller element, in element map order.
      /// @param [in] label Label to set, or `nullptr` or an empty string to clear it.
      /// @return `true` if the index is valid, `false` otherwise.
      bool SetLabel(int index, const char* label)
      {
        if ((index < 0) || (index >= kLabelCount)) return false;

        std::scoped_lock lock(labelsGuard);
        labels[index] = ((nullptr != label) ? label : "");
        return true;
      }

      /// Replaces the name of a virtual controller element with the label of the first physical
      /// controller element that has a label and that the configured mapper maps to the virtual
      /// controller element. The name is left unchanged if no such label exists.
      /// @tparam CharType Either `char` or `wchar_t` depending on the DirectInput character mode.
      /// @param [in] controllerIdentifier Identifier of the virtual controller.
      /// @param [in] element Virtual controller element whose name is being generated.
      /// @param [in,out] name Name buffer, already filled with the default name.
      template <typename CharType, size_t kNameCount> void ApplyLabel(
          Controller::TControllerIdentifier controllerIdentifier,
          Controller::SElementIdentifier element,
          CharType (&name)[kNameCount])
      {
        if ((Controller::EElementType::Axis != element.type) &&
            (Controller::EElementType::Button != element.type))
          return;

        const Controller::Mapper::UElementMap& elementMap =
            Controller::Mapper::GetConfigured(controllerIdentifier)->ElementMap();

        std::scoped_lock lock(labelsGuard);

        for (int index = 0; index < kLabelCount; ++index)
        {
          if (true == labels[index].empty()) continue;

          const Controller::IElementMapper* const elementMapper = elementMap.all[index].get();
          if (nullptr == elementMapper) continue;

          for (int targetIndex = 0; targetIndex < elementMapper->GetTargetElementCount();
               ++targetIndex)
          {
            const std::optional<Controller::SElementIdentifier> maybeTargetElement =
                elementMapper->GetTargetElementAt(targetIndex);
            if ((false == maybeTargetElement.has_value()) ||
                (false == (maybeTargetElement.value() == element)))
              continue;

            if constexpr (std::is_same_v<CharType, wchar_t>)
            {
              const int convertedLength =
                  MultiByteToWideChar(CP_ACP, 0, labels[index].c_str(), -1, nullptr, 0);
              if (convertedLength <= 0) return;

              std::wstring convertedLabel(static_cast<size_t>(convertedLength), L'\0');
              MultiByteToWideChar(
                  CP_ACP, 0, labels[index].c_str(), -1, convertedLabel.data(), convertedLength);
              wcsncpy_s(name, kNameCount, convertedLabel.c_str(), _TRUNCATE);
            }
            else
            {
              strncpy_s(name, kNameCount, labels[index].c_str(), _TRUNCATE);
            }

            return;
          }
        }
      }

    private:

      ElementLabels(void) = default;

      /// Label for each physical controller element, empty if not set.
      std::array<std::string, kLabelCount> labels;

      /// For ensuring proper concurrency control of accesses to the labels.
      std::mutex labelsGuard;
    };

    /// Applies a custom label, if one is set, to the name of a virtual controller element.
    /// Intended to be invoked after the object instance information structure has been filled.
    /// @param [in] controllerIdentifier Identifier of the virtual controller.
    /// @param [in] element Virtual controller element whose name is being generated.
    /// @param [in,out] name Name buffer, already filled with the default name.
    template <typename CharType, size_t kNameCount> static inline void ApplyCustomElementLabel(
        Controller::TControllerIdentifier controllerIdentifier,
        Controller::SElementIdentifier element,
        CharType (&name)[kNameCount])
    {
      ElementLabels::GetInstance().ApplyLabel(controllerIdentifier, element, name);
    }
  } // namespace Extensions
} // namespace Xidi

/// Sets the label reported to the application as the name of whichever virtual controller element
/// the specified physical controller element is mapped to. Labels apply to buttons and axes, and
/// take effect for subsequent object enumerations and object information queries.
/// @param [in] index Index of the physical controller element, in element map order: left stick X,
/// left stick Y, right stick X, right stick Y, d-pad up, d-pad down, d-pad left, d-pad right, LT,
/// RT, A, B, X, Y, LB, RB, Back, Start, LS, RS, Guide, Share.
/// @param [in] label Label to set, in the system's ANSI code page, or `nullptr` or an empty string
/// to restore the default name.
extern "C" __declspec(dllexport) void XidiSetButtonLabel(int index, const char* label)
{
  Xidi::Extensions::ElementLabels::GetInstance().SetLabel(index, label);
}

namespace Xidi
{
  namespace Extensions
  {
    /// Type-specific parameters of a condition effect (spring, damper, inertia, friction) for one
    /// axis, as supplied by the application in a DICONDITION structure.
    struct SConditionParameters
    {
      Controller::ForceFeedback::TEffectValue offset;
      Controller::ForceFeedback::TEffectValue positiveCoefficient;
      Controller::ForceFeedback::TEffectValue negativeCoefficient;
      Controller::ForceFeedback::TEffectValue positiveSaturation;
      Controller::ForceFeedback::TEffectValue negativeSaturation;
      Controller::ForceFeedback::TEffectValue deadBand;

      constexpr bool operator==(const SConditionParameters& other) const = default;
    };

    /// Condition effect. Conditions describe a force that depends on the position or motion of
    /// the device's own axes, such as a steering wheel's centering spring. Gamepad rumble motors
    /// cannot render such a force, so the effect produces no output. Supporting it still matters:
    /// racing games written for wheels create these effects on every device and, if creation
    /// fails, retry every frame or refuse force feedback altogether.
    class ConditionEffect
        : public Controller::ForceFeedback::EffectWithTypeSpecificParameters<SConditionParameters>
    {
    public:

      // Effect
      std::unique_ptr<Effect> Clone(void) const override
      {
        return std::make_unique<ConditionEffect>(*this);
      }

    protected:

      // Effect
      Controller::ForceFeedback::TEffectValue ComputeRawMagnitude(
          Controller::ForceFeedback::TEffectTimeMs rawTime) const override
      {
        return 0;
      }
    };

    /// DirectInput effect object for condition effects. Only the first axis's DICONDITION
    /// structure is kept, which is enough to report the parameters back to the application.
    template <EDirectInputVersion diVersion> class ConditionDirectInputEffect
        : public VirtualDirectInputEffectWithTypeSpecificParameters<
              diVersion,
              DICONDITION,
              SConditionParameters>
    {
    public:

      inline ConditionDirectInputEffect(
          VirtualDirectInputDeviceBase<diVersion>& associatedDevice,
          const ConditionEffect& effect,
          const GUID& effectGuid)
          : VirtualDirectInputEffectWithTypeSpecificParameters<
                diVersion,
                DICONDITION,
                SConditionParameters>(associatedDevice, effect, effectGuid)
      {}

    protected:

      // VirtualDirectInputEffectWithTypeSpecificParameters
      SConditionParameters ConvertFromDirectInput(
          const DICONDITION& diTypeSpecificParams) const override
      {
        using Controller::ForceFeedback::TEffectValue;
        return {
            .offset = (TEffectValue)diTypeSpecificParams.lOffset,
            .positiveCoefficient = (TEffectValue)diTypeSpecificParams.lPositiveCoefficient,
            .negativeCoefficient = (TEffectValue)diTypeSpecificParams.lNegativeCoefficient,
            .positiveSaturation = (TEffectValue)diTypeSpecificParams.dwPositiveSaturation,
            .negativeSaturation = (TEffectValue)diTypeSpecificParams.dwNegativeSaturation,
            .deadBand = (TEffectValue)diTypeSpecificParams.lDeadBand};
      }

      DICONDITION ConvertToDirectInput(
          const SConditionParameters& typeSpecificParams) const override
      {
        return {
            .lOffset = (LONG)typeSpecificParams.offset,
            .lPositiveCoefficient = (LONG)typeSpecificParams.positiveCoefficient,
            .lNegativeCoefficient = (LONG)typeSpecificParams.negativeCoefficient,
            .dwPositiveSaturation = (DWORD)typeSpecificParams.positiveSaturation,
            .dwNegativeSaturation = (DWORD)typeSpecificParams.negativeSaturation,
            .lDeadBand = (LONG)typeSpecificParams.deadBand};
      }
    };

    /// Condition effect GUIDs supported by the fork.
    inline constexpr const GUID* kConditionEffectGuids[] = {
        &GUID_Spring, &GUID_Damper, &GUID_Inertia, &GUID_Friction};

    /// Determines the DirectInput effect type of a condition effect GUID.
    /// @param [in] rguidEffect Effect GUID.
    /// @return `DIEFT_CONDITION` for supported condition effects, nothing otherwise.
    inline std::optional<DWORD> ConditionEffectType(REFGUID rguidEffect)
    {
      for (const GUID* guid : kConditionEffectGuids)
        if (rguidEffect == *guid) return DIEFT_CONDITION;

      return std::nullopt;
    }

    /// Creates a condition effect object. Has the signature of the device's effect creator
    /// functions so that it can be returned from their registry lookup.
    template <EDirectInputVersion diVersion>
    std::unique_ptr<VirtualDirectInputEffect<diVersion>> CreateConditionEffect(
        REFGUID rguidEffect, VirtualDirectInputDeviceBase<diVersion>& associatedDevice)
    {
      return std::make_unique<ConditionDirectInputEffect<diVersion>>(
          associatedDevice, ConditionEffect(), rguidEffect);
    }

    /// Looks up the creator function for a condition effect GUID.
    /// @return Creator function, or `nullptr` if the GUID is not a supported condition effect.
    template <EDirectInputVersion diVersion>
    std::unique_ptr<VirtualDirectInputEffect<diVersion>> (*ConditionEffectCreator(
        REFGUID rguidEffect))(REFGUID, VirtualDirectInputDeviceBase<diVersion>&)
    {
      if (false == ConditionEffectType(rguidEffect).has_value()) return nullptr;
      return &CreateConditionEffect<diVersion>;
    }

    /// Enumerates the supported condition effects, after the effect types handled upstream.
    /// @param [in] fillEffectInfo Fills the remaining effect information members.
    /// @return Result to return from EnumEffects if the application stopped the enumeration or
    /// its callback failed, nothing if enumeration should continue normally.
    template <EDirectInputVersion diVersion> std::optional<HRESULT> EnumConditionEffects(
        typename DirectInputTypes<diVersion>::EnumEffectsCallbackType lpCallback,
        LPVOID pvRef,
        DWORD dwEffType,
        void (*fillEffectInfo)(typename DirectInputTypes<diVersion>::EffectInfoType*))
    {
      if ((DIEFT_ALL != dwEffType) && (DIEFT_CONDITION != DIEFT_GETTYPE(dwEffType)))
        return std::nullopt;

      typename DirectInputTypes<diVersion>::EffectInfoType effectDescriptor;
      for (const GUID* guid : kConditionEffectGuids)
      {
        effectDescriptor = {
            .dwSize = sizeof(effectDescriptor), .guid = *guid, .dwEffType = DIEFT_CONDITION};
        fillEffectInfo(&effectDescriptor);

        switch (lpCallback(&effectDescriptor, pvRef))
        {
          case DIENUM_CONTINUE:
            break;
          case DIENUM_STOP:
            return DI_OK;
          default:
            return DIERR_INVALIDPARAM;
        }
      }

      return std::nullopt;
    }
  } // namespace Extensions
} // namespace Xidi

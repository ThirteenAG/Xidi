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

/***************************************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 ***************************************************************************************************
 * Fork extension. Kept out of the upstream source files so that upstream changes merge cleanly.
 ***********************************************************************************************//**
 * @file WrapperIDirectInput.inl
 *   Support for IDirectInput8::EnumDevicesBySemantics, which is forwarded to the system with the
 *   devices handled by Xidi filtered out, instead of failing. Some applications refuse to use
 *   DirectInput at all if the call fails. Included by WrapperIDirectInput.cpp at file scope,
 *   after its other includes.
 **************************************************************************************************/

#include <Infra/Core/Message.h>
#include <Infra/Core/TemporaryBuffer.h>

#include "ApiDirectInput.h"
#include "ApiWindows.h"
#include "ControllerIdentification.h"
#include "PhysicalController.h"
#include "Strings.h"

namespace Xidi
{
  namespace Extensions
  {
    /// Contains all information required to intercept callbacks to EnumDevicesBySemantics.
    template <EDirectInputVersion diVersion> struct SEnumDevicesBySemanticsCallbackInfo
    {
      /// Underlying IDirectInput object that is performing the enumeration.
      typename DirectInputTypes<diVersion>::IDirectInputType* underlyingDIObject;

      /// Application-supplied callback function.
      typename DirectInputTypes<diVersion>::EnumDevicesBySemanticsCallbackType lpCallback;

      /// Application-supplied callback parameter.
      LPVOID pvRef;
    };

    /// Callback for EnumDevicesBySemantics that hides devices supported by the configured physical
    /// controller backend and passes everything else through to the application. Parameters are
    /// the same as for the application-supplied callback.
    template <EDirectInputVersion diVersion> static BOOL __stdcall
        CallbackEnumDevicesBySemanticsFiltered(
            const typename DirectInputTypes<diVersion>::DeviceInstanceType* lpddi,
            typename DirectInputTypes<diVersion>::IDirectInputDeviceCompatType* lpdid,
            DWORD dwFlags,
            DWORD dwRemaining,
            LPVOID pvRef)
    {
      const SEnumDevicesBySemanticsCallbackInfo<diVersion>* const callbackInfo =
          static_cast<const SEnumDevicesBySemanticsCallbackInfo<diVersion>*>(pvRef);

      if (true ==
          DoesDirectInputControllerSupportConfiguredBackend<diVersion>(
              callbackInfo->underlyingDIObject, lpddi->guidInstance))
      {
        if (Infra::Message::WillOutputMessageOfSeverity(Infra::Message::ESeverity::Debug))
        {
          Infra::Message::OutputFormatted(
              Infra::Message::ESeverity::Debug,
              L"EnumDevicesBySemantics: DirectInput device \"%s\" with instance GUID %s supports physical controller backend \"%.*s\" and will not be presented to the application.",
              Infra::TemporaryString(lpddi->tszProductName).AsCString(),
              Strings::GuidToString(lpddi->guidInstance).AsCString(),
              static_cast<int>(Controller::GetPhysicalControllerBackend()->PluginName().length()),
              Controller::GetPhysicalControllerBackend()->PluginName().data());
        }

        return DIENUM_CONTINUE;
      }

      return callbackInfo->lpCallback(lpddi, lpdid, dwFlags, dwRemaining, callbackInfo->pvRef);
    }

    /// Forwards EnumDevicesBySemantics to the underlying IDirectInput object, filtering out devices
    /// supported by the configured physical controller backend. Parameters other than the first
    /// are the same as for IDirectInput8::EnumDevicesBySemantics.
    /// @param [in] underlyingDIObject Underlying IDirectInput object to which to forward the call.
    /// @return Result of the underlying enumeration.
    template <EDirectInputVersion diVersion> static HRESULT EnumDevicesBySemanticsFiltered(
        typename DirectInputTypes<diVersion>::IDirectInputType* underlyingDIObject,
        typename DirectInputTypes<diVersion>::ConstStringType ptszUserName,
        typename DirectInputTypes<diVersion>::ActionFormatType* lpdiActionFormat,
        typename DirectInputTypes<diVersion>::EnumDevicesBySemanticsCallbackType lpCallback,
        LPVOID pvRef,
        DWORD dwFlags)
    {
      SEnumDevicesBySemanticsCallbackInfo<diVersion> callbackInfo = {
          .underlyingDIObject = underlyingDIObject, .lpCallback = lpCallback, .pvRef = pvRef};

      const HRESULT enumResult = underlyingDIObject->EnumDevicesBySemantics(
          ptszUserName,
          lpdiActionFormat,
          &CallbackEnumDevicesBySemanticsFiltered<diVersion>,
          &callbackInfo,
          dwFlags);

      Infra::Message::OutputFormatted(
          Infra::Message::ESeverity::Info,
          L"EnumDevicesBySemantics: Forwarded to the system with devices supported by physical controller backend \"%.*s\" filtered out, result = 0x%08x.",
          static_cast<int>(Controller::GetPhysicalControllerBackend()->PluginName().length()),
          Controller::GetPhysicalControllerBackend()->PluginName().data(),
          static_cast<unsigned int>(enumResult));

      return enumResult;
    }
  } // namespace Extensions
} // namespace Xidi

/*****************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 *****************************************************************************
 * Authored by Samuel Grossman
 * Copyright (c) 2016-2020
 *************************************************************************//**
 * @file WrapperIDirectInput.h
 *   Declaration of the wrapper class for IDirectInput.
 *****************************************************************************/

#pragma once

#include "ApiDirectInput.h"


namespace Xidi
{
    /// Helper types for differentiating between Unicode and ASCII interface versions.
    template <bool useUnicode> struct DirectInputHelper
    {
        typedef LPCTSTR ConstStringType;
        typedef DIDEVICEINSTANCE DeviceInstanceType;
        typedef EarliestIDirectInput EarliestIDirectInputType;
        typedef EarliestIDirectInputDevice EarliestIDirectInputDeviceType;
        typedef LPDIENUMDEVICESCALLBACK EnumDevicesCallbackType;
        typedef LatestIDirectInput LatestIDirectInputType;
#if DIRECTINPUT_VERSION >= 0x0800
        typedef LPDIACTIONFORMAT ActionFormatType;
        typedef LPDICONFIGUREDEVICESPARAMS ConfigureDevicesParamsType;
        typedef LPDIENUMDEVICESBYSEMANTICSCB EnumDevicesBySemanticsCallbackType;
#endif
    };

    template <> struct DirectInputHelper<false> : public LatestIDirectInputA
    {
        typedef LPCSTR ConstStringType;
        typedef DIDEVICEINSTANCEA DeviceInstanceType;
        typedef EarliestIDirectInputA EarliestIDirectInputType;
        typedef EarliestIDirectInputDeviceA EarliestIDirectInputDeviceType;
        typedef LPDIENUMDEVICESCALLBACKA EnumDevicesCallbackType;
        typedef LatestIDirectInputA LatestIDirectInputType;
#if DIRECTINPUT_VERSION >= 0x0800
        typedef LPDIACTIONFORMATA ActionFormatType;
        typedef LPDICONFIGUREDEVICESPARAMSA ConfigureDevicesParamsType;
        typedef LPDIENUMDEVICESBYSEMANTICSCBA EnumDevicesBySemanticsCallbackType;
#endif
    };

    template <> struct DirectInputHelper<true> : public LatestIDirectInputW
    {
        typedef LPCWSTR ConstStringType;
        typedef DIDEVICEINSTANCEW DeviceInstanceType;
        typedef EarliestIDirectInputW EarliestIDirectInputType;
        typedef EarliestIDirectInputDeviceW EarliestIDirectInputDeviceType;
        typedef LPDIENUMDEVICESCALLBACKW EnumDevicesCallbackType;
        typedef LatestIDirectInputW LatestIDirectInputType;
#if DIRECTINPUT_VERSION >= 0x0800
        typedef LPDIACTIONFORMATW ActionFormatType;
        typedef LPDICONFIGUREDEVICESPARAMSW ConfigureDevicesParamsType;
        typedef LPDIENUMDEVICESBYSEMANTICSCBW EnumDevicesBySemanticsCallbackType;
#endif
    };

    /// Wraps the IDirectInput8 interface to hook into all calls to it.
    /// Holds an underlying instance of an IDirectInput object but wraps all method invocations.
    /// @tparam useUnicode Specifies whether to use underlying Unicode interfaces (i.e. the "A" versions of interfaces and types).
    template <bool useUnicode> class WrapperIDirectInput : public DirectInputHelper<useUnicode>
    {
    private:
        // -------- INSTANCE VARIABLES --------------------------------------------- //

        /// The underlying IDirectInput8 object that this instance wraps.
        DirectInputHelper<useUnicode>::LatestIDirectInputType* underlyingDIObject;


    public:
        // -------- CONSTRUCTION AND DESTRUCTION ----------------------------------- //

        /// Default constructor. Should never be invoked.
        WrapperIDirectInput(void) = delete;

        /// Constructs an WrapperIDirectInput object, given an underlying IDirectInput8 object to wrap.
        WrapperIDirectInput(DirectInputHelper<useUnicode>::LatestIDirectInputType* underlyingDIObject);


        // -------- METHODS: IUnknown ---------------------------------------------- //
        HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, LPVOID* ppvObj) override;
        ULONG STDMETHODCALLTYPE AddRef(void) override;
        ULONG STDMETHODCALLTYPE Release(void) override;


        // -------- METHODS: IDirectInput COMMON ----------------------------------- //
        HRESULT STDMETHODCALLTYPE CreateDevice(REFGUID rguid, DirectInputHelper<useUnicode>::EarliestIDirectInputDeviceType** lplpDirectInputDevice, LPUNKNOWN pUnkOuter) override;
        HRESULT STDMETHODCALLTYPE EnumDevices(DWORD dwDevType, DirectInputHelper<useUnicode>::EnumDevicesCallbackType lpCallback, LPVOID pvRef, DWORD dwFlags) override;
        HRESULT STDMETHODCALLTYPE FindDevice(REFGUID rguidClass, DirectInputHelper<useUnicode>::ConstStringType ptszName, LPGUID pguidInstance) override;
        HRESULT STDMETHODCALLTYPE GetDeviceStatus(REFGUID rguidInstance) override;
        HRESULT STDMETHODCALLTYPE Initialize(HINSTANCE hinst, DWORD dwVersion) override;
        HRESULT STDMETHODCALLTYPE RunControlPanel(HWND hwndOwner, DWORD dwFlags) override;


        // -------- CALLBACKS: IDirectInput COMMON --------------------------------- //

        // Callback used to scan for any XInput-compatible game controllers.
        static BOOL STDMETHODCALLTYPE CallbackEnumGameControllersXInputScan(const DirectInputHelper<useUnicode>::DeviceInstanceType* lpddi, LPVOID pvRef);

        // Callback used to enumerate all devices to the application, filtering out those already seen.
        static BOOL STDMETHODCALLTYPE CallbackEnumDevicesFiltered(const DirectInputHelper<useUnicode>::DeviceInstanceType* lpddi, LPVOID pvRef);

#if DIRECTINPUT_VERSION >= 0x0800
        // -------- METHODS: IDirectInput8 ONLY ------------------------------------ //
        HRESULT STDMETHODCALLTYPE ConfigureDevices(LPDICONFIGUREDEVICESCALLBACK lpdiCallback, DirectInputHelper<useUnicode>::ConfigureDevicesParamsType lpdiCDParams, DWORD dwFlags, LPVOID pvRefData) override;
        HRESULT STDMETHODCALLTYPE EnumDevicesBySemantics(DirectInputHelper<useUnicode>::ConstStringType ptszUserName, DirectInputHelper<useUnicode>::ActionFormatType lpdiActionFormat, DirectInputHelper<useUnicode>::EnumDevicesBySemanticsCallbackType lpCallback, LPVOID pvRef, DWORD dwFlags) override;
#else
        // -------- METHODS: IDirectInput LEGACY ----------------------------------- //
        HRESULT STDMETHODCALLTYPE CreateDeviceEx(REFGUID rguid, REFIID riid, LPVOID* lplpDirectInputDevice, LPUNKNOWN pUnkOuter) override;
#endif
    };
}

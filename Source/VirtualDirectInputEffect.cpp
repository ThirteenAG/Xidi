/*****************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 *****************************************************************************
 * Authored by Samuel Grossman
 * Copyright (c) 2016-2021
 *************************************************************************//**
 * @file VirtualDirectInputEffect.cpp
 *   Implementation of an IDirectInputEffect interface wrapper around force
 *   feedback effects that are associated with virtual controllers.
 *****************************************************************************/

#include "ForceFeedbackDevice.h"
#include "ForceFeedbackEffect.h"
#include "ForceFeedbackParameters.h"
#include "ForceFeedbackTypes.h"
#include "Message.h"
#include "VirtualDirectInputDevice.h"
#include "VirtualDirectInputEffect.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>


// -------- MACROS --------------------------------------------------------- //

/// Logs a DirectInput interface method invocation and returns.
#define LOG_INVOCATION_AND_RETURN(result, severity) \
    do \
    { \
        const HRESULT kResult = (result); \
        Message::OutputFormatted(severity, L"Invoked %s on a force feedback effect associated with Xidi virtual controller %u, result = 0x%08x.", __FUNCTIONW__ L"()", (1 + associatedDevice.GetVirtualController().GetIdentifier()), kResult); \
        return kResult; \
    } while (false)


namespace Xidi
{
    // -------- INTERNAL FUNCTIONS ----------------------------------------- //

    /// Internal implementation of downloading a force feedback effect to a force feedback device.
    /// Called by interface methods that require this functionality.
    /// @param [in] effect Effect to be downloaded.
    /// @param [in] device Target device.
    /// @return DirectInput return code indicating the result of the operation.
    static HRESULT DownloadEffectToDevice(const Controller::ForceFeedback::Effect& effect, Controller::ForceFeedback::Device& device)
    {
        if (false == effect.IsCompletelyDefined())
            return DIERR_INCOMPLETEEFFECT;

        if (false == device.AddOrUpdateEffect(effect))
            return DIERR_DEVICEFULL;

        return DI_OK;
    }

    /// Selects the coordinate system that should be used to represent the coordinates set in the specified direction vector, subject to the specified flags.
    /// @param [in] directionVector Vector from which coordinates are to be extracted.
    /// @param [in] dwFlags Flags specifying the allowed coordinate systems using DirectInput constants.
    /// @return Selected coordinate system if it is valid. An example of an invalid situation is polar coordinates specified by the flags but the direction vector contains only one axis or more than two axes.
    static std::optional<Controller::ForceFeedback::ECoordinateSystem> PickCoordinateSystem(const Controller::ForceFeedback::DirectionVector& directionVector, DWORD dwFlags)
    {
        if (1 == directionVector.GetNumAxes())
        {
            // Only Cartesian coordinates are valid with a single axis.
            if (0 == (dwFlags & DIEFF_CARTESIAN))
                return std::nullopt;
        }
        else if (2 != directionVector.GetNumAxes())
        {
            // Polar coordinates are only valid when there are two axes present.
            if (DIEFF_POLAR == (dwFlags & (DIEFF_CARTESIAN | DIEFF_POLAR | DIEFF_SPHERICAL)))
                return std::nullopt;
        }

        // Match the original coordinate system if possible.
        // The output would be exactly what was originally supplied as input when setting parameters.
        switch (directionVector.GetOriginalCoordinateSystem())
        {
        case Controller::ForceFeedback::ECoordinateSystem::Cartesian:
            if (0 != (dwFlags & DIEFF_CARTESIAN))
                return Controller::ForceFeedback::ECoordinateSystem::Cartesian;
            break;

        case Controller::ForceFeedback::ECoordinateSystem::Polar:
            if (0 != (dwFlags & DIEFF_POLAR))
                return Controller::ForceFeedback::ECoordinateSystem::Polar;
            break;

        case Controller::ForceFeedback::ECoordinateSystem::Spherical:
            if (0 != (dwFlags & DIEFF_SPHERICAL))
                return Controller::ForceFeedback::ECoordinateSystem::Spherical;
            break;
        }

        // Try other coordinate systems in Xidi-preferred order.
        if (0 != (dwFlags & DIEFF_SPHERICAL))
            return Controller::ForceFeedback::ECoordinateSystem::Spherical;
        else if (0 != (dwFlags & DIEFF_POLAR))
            return Controller::ForceFeedback::ECoordinateSystem::Polar;
        else if (0 != (dwFlags & DIEFF_CARTESIAN))
            return Controller::ForceFeedback::ECoordinateSystem::Cartesian;

        return std::nullopt;
    }


    // -------- CONSTRUCTION AND DESTRUCTION ------------------------------- //
    // See "VirtualDirectInputEffect.h" for documentation.

    template <ECharMode charMode> VirtualDirectInputEffect<charMode>::VirtualDirectInputEffect(VirtualDirectInputDevice<charMode>& associatedDevice, const Controller::ForceFeedback::Effect& effect, const GUID& effectGuid) : associatedDevice(associatedDevice), effect(effect.Clone()), effectGuid(effectGuid), refCount(1)
    {
        associatedDevice.AddRef();
        associatedDevice.ForceFeedbackEffectRegister((void*)this);
    }

    // --------

    template <ECharMode charMode> VirtualDirectInputEffect<charMode>::~VirtualDirectInputEffect(void)
    {
        associatedDevice.ForceFeedbackEffectUnregister((void*)this);
        associatedDevice.Release();
    }


    // -------- METHODS: IUnknown ------------------------------------------ //
    // See IUnknown documentation for more information.

    template <ECharMode charMode> HRESULT VirtualDirectInputEffect<charMode>::QueryInterface(REFIID riid, LPVOID* ppvObj)
    {
        if (nullptr == ppvObj)
            return E_POINTER;

        bool validInterfaceRequested = false;

        if (IsEqualIID(riid, IID_IUnknown) || IsEqualIID(riid, IID_IDirectInputEffect))
            validInterfaceRequested = true;

        if (true == validInterfaceRequested)
        {
            AddRef();
            *ppvObj = this;
            return S_OK;
        }

        return E_NOINTERFACE;
    }

    // --------

    template <ECharMode charMode> ULONG VirtualDirectInputEffect<charMode>::AddRef(void)
    {
        return ++refCount;
    }

    // --------

    template <ECharMode charMode> ULONG VirtualDirectInputEffect<charMode>::Release(void)
    {
        const unsigned long numRemainingRefs = --refCount;

        if (0 == numRemainingRefs)
            delete this;

        return (ULONG)numRemainingRefs;
    }


    // -------- METHODS: IDirectInputEffect -------------------------------- //
    // See DirectInput documentation for more information.

    template <ECharMode charMode> HRESULT VirtualDirectInputEffect<charMode>::Initialize(HINSTANCE hinst, DWORD dwVersion, REFGUID rguid)
    {
        // Not required for Xidi virtual force feedback effects as they are implemented now.
        // However, this method is needed for creating IDirectInputDevice objects via COM.

        constexpr Message::ESeverity kMethodSeverity = Message::ESeverity::Info;
        LOG_INVOCATION_AND_RETURN(DI_OK, kMethodSeverity);
    }

    // --------

    template <ECharMode charMode> HRESULT VirtualDirectInputEffect<charMode>::GetEffectGuid(LPGUID pguid)
    {
        constexpr Message::ESeverity kMethodSeverity = Message::ESeverity::Info;

        if (nullptr == pguid)
            LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

        *pguid = effectGuid;
        LOG_INVOCATION_AND_RETURN(DI_OK, kMethodSeverity);
    }

    // --------

    template <ECharMode charMode> HRESULT VirtualDirectInputEffect<charMode>::GetParameters(LPDIEFFECT peff, DWORD dwFlags)
    {
        constexpr Message::ESeverity kMethodSeverity = Message::ESeverity::Info;

        if (nullptr == peff)
            LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

        switch (peff->dwSize)
        {
            case sizeof(DIEFFECT) :
                // These parameters are present in the new version of the structure but not in the old.
                if (0 != (dwFlags & DIEP_STARTDELAY))
                    peff->dwStartDelay = (DWORD)effect->GetStartDelay();
                break;

            case sizeof(DIEFFECT_DX5):
                // No parameters are present in the old version of the structure but removed from the new.
                break;

            default:
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
        }

        HRESULT axesResult = DI_OK;
        if (0 != (dwFlags & DIEP_AXES))
        {
            if (false == effect->HasAssociatedAxes())
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

            const Controller::ForceFeedback::SAssociatedAxes& kAssociatedAxes = effect->GetAssociatedAxes().value();
            if (peff->cAxes < (DWORD)kAssociatedAxes.count)
            {
                peff->cAxes = (DWORD)kAssociatedAxes.count;
                axesResult = DIERR_MOREDATA;
            }
            else
            {
                if (nullptr == peff->rgdwAxes)
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

                switch (peff->dwFlags & (DIEFF_OBJECTIDS | DIEFF_OBJECTOFFSETS))
                {
                case DIEFF_OBJECTIDS:
                    for (int i = 0; i < kAssociatedAxes.count; ++i)
                    {
                        const std::optional<DWORD> kMaybeObjectId = associatedDevice.IdentifyObjectById({.type = Controller::EElementType::Axis, .axis = kAssociatedAxes.type[i]});
                        if (false == kMaybeObjectId.has_value())
                        {
                            // This should never happen. It means an axis object was successfully set on a force feedback effect but it could not be mapped back to its object ID.
                            Message::OutputFormatted(Message::ESeverity::Error, L"Internal error while mapping force feedback axes to object IDs on Xidi virtual controller %u.", (1 + associatedDevice.GetVirtualController().GetIdentifier()));
                            LOG_INVOCATION_AND_RETURN(DIERR_GENERIC, kMethodSeverity);
                        }

                        peff->rgdwAxes[i] = kMaybeObjectId.value();
                    }
                    break;

                case DIEFF_OBJECTOFFSETS:
                    for (int i = 0; i < kAssociatedAxes.count; ++i)
                    {
                        const std::optional<DWORD> kMaybeOffset = associatedDevice.IdentifyObjectByOffset({.type = Controller::EElementType::Axis, .axis = kAssociatedAxes.type[i]});
                        if (false == kMaybeOffset.has_value())
                        {
                            // This can happen if the application's data format is not set or if it somehow changed and now does not contain one of the axes that are associated with this effect object.
                            LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
                        }

                        peff->rgdwAxes[i] = kMaybeOffset.value();
                    }
                    break;

                default:
                    // It is an error if the caller does not specify exactly one specific way of identifying axis objects.
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
                }
            }
        }

        HRESULT directionResult = DI_OK;
        if (0 != (dwFlags & DIEP_DIRECTION))
        {
            if (false == effect->HasDirection())
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

            const Controller::ForceFeedback::DirectionVector& kDirectionVector = effect->Direction();
            if (peff->cAxes < (DWORD)kDirectionVector.GetNumAxes())
            {
                peff->cAxes = (DWORD)kDirectionVector.GetNumAxes();
                directionResult = DIERR_MOREDATA;
            }
            else
            {
                if (nullptr == peff->rglDirection)
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

                if (0 == (peff->dwFlags & (DIEFF_CARTESIAN | DIEFF_POLAR | DIEFF_SPHERICAL)))
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

                const std::optional<Controller::ForceFeedback::ECoordinateSystem> kMaybeCoordinateSystem = PickCoordinateSystem(kDirectionVector, peff->dwFlags);
                if (false == kMaybeCoordinateSystem.has_value())
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

                Controller::ForceFeedback::TEffectValue coordinates[Controller::ForceFeedback::kEffectAxesMaximumNumber] = {};
                int numCoordinates = 0;

                // The selected coordinate system is identified to the caller by ensuring exactly one coordinate system is saved into the flags on output.
                // First the coordinate system flags are all cleared, then in the switch block below a single system is added back into the flags.
                peff->dwFlags = (peff->dwFlags & ~(DIEFF_CARTESIAN | DIEFF_POLAR | DIEFF_SPHERICAL));

                switch (kMaybeCoordinateSystem.value())
                {
                case Controller::ForceFeedback::ECoordinateSystem::Cartesian:
                    numCoordinates = kDirectionVector.GetCartesianCoordinates(coordinates, _countof(coordinates));
                    peff->dwFlags |= DIEFF_CARTESIAN;
                    break;

                case Controller::ForceFeedback::ECoordinateSystem::Polar:
                    numCoordinates = kDirectionVector.GetPolarCoordinates(coordinates, _countof(coordinates));
                    peff->dwFlags |= DIEFF_POLAR;
                    break;

                case Controller::ForceFeedback::ECoordinateSystem::Spherical:
                    numCoordinates = kDirectionVector.GetSphericalCoordinates(coordinates, _countof(coordinates));
                    peff->dwFlags |= DIEFF_SPHERICAL;
                    break;
                }

                if (0 == numCoordinates)
                {
                    // This should never happen. It means the direction is supposedly present and the coordinate system selected is supposedly valid but coordinate values were unable to be retrieved.
                    Message::OutputFormatted(Message::ESeverity::Error, L"Internal error while retrieving direction components using coordinate system %d on Xidi virtual controller %u.", (int)(kMaybeCoordinateSystem.value()), (1 + associatedDevice.GetVirtualController().GetIdentifier()));
                    LOG_INVOCATION_AND_RETURN(DIERR_GENERIC, kMethodSeverity);
                }

                for (DWORD i = 0; i < (DWORD)numCoordinates; ++i)
                    peff->rglDirection[i] = (LONG)coordinates[i];

                for (DWORD i = (DWORD)numCoordinates; i < peff->cAxes; ++i)
                    peff->rglDirection[i] = 0;
            }
        }

        if (0 != (dwFlags & DIEP_DURATION))
        {
            if (false == effect->HasDuration())
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

            peff->dwDuration = (DWORD)effect->GetDuration().value();
        }

        if (0 != (dwFlags & DIEP_ENVELOPE))
        {
            if (false == effect->HasEnvelope())
                peff->lpEnvelope = nullptr;
            else
            {
                if (nullptr == peff->lpEnvelope)
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

                if (sizeof(DIENVELOPE) != peff->lpEnvelope->dwSize)
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

                const Controller::ForceFeedback::SEnvelope kEnvelope = effect->GetEnvelope().value();
                peff->lpEnvelope->dwAttackLevel = (DWORD)kEnvelope.attackLevel;
                peff->lpEnvelope->dwAttackTime = (DWORD)kEnvelope.attackTime;
                peff->lpEnvelope->dwFadeLevel = (DWORD)kEnvelope.fadeLevel;
                peff->lpEnvelope->dwFadeTime = (DWORD)kEnvelope.fadeTime;
            }
        }

        if (0 != (dwFlags & DIEP_GAIN))
            peff->dwGain = (DWORD)effect->GetGain();

        if (0 != (dwFlags & DIEP_SAMPLEPERIOD))
            peff->dwSamplePeriod = (DWORD)effect->GetSamplePeriod();

        if (0 != (dwFlags & DIEP_STARTDELAY))
            peff->dwStartDelay = (DWORD)effect->GetStartDelay();

        HRESULT typeSpecificParameterResult = DI_OK;
        if (0 != (dwFlags & DIEP_TYPESPECIFICPARAMS))
            typeSpecificParameterResult = GetTypeSpecificParameters(peff);

        const HRESULT kOverallResult = std::max({axesResult, directionResult, typeSpecificParameterResult});
        LOG_INVOCATION_AND_RETURN(kOverallResult, kMethodSeverity);
    }

    // --------

    template <ECharMode charMode> HRESULT VirtualDirectInputEffect<charMode>::SetParameters(LPCDIEFFECT peff, DWORD dwFlags)
    {
        constexpr Message::ESeverity kMethodSeverity = Message::ESeverity::Info;

        if (nullptr == peff)
            LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

        // These flags control the behavior of this method if all parameters are updated successfully.
        // Per DirectInput documentation, at most one of them is allowed to be passed.
        switch (dwFlags & (DIEP_NODOWNLOAD | DIEP_NORESTART | DIEP_START))
        {
        case 0:
        case DIEP_NODOWNLOAD:
        case DIEP_NORESTART:
        case DIEP_START:
            break;

        default:
            LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
        }

        // This cloned effect will receive all the parameter updates and will be synced back to the original effect once all parameter values are accepted.
        // Doing this means that an invalid value for a parameter means the original effect remains untouched.
        std::unique_ptr<Controller::ForceFeedback::Effect> updatedEffect;

        if (0 != (dwFlags & DIEP_TYPESPECIFICPARAMS))
        {
            if (nullptr == peff->lpvTypeSpecificParams)
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

            updatedEffect = CloneAndSetTypeSpecificParameters(peff);
            if (nullptr == updatedEffect)
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
        }
        else
        {
            updatedEffect = effect->Clone();
        }

        switch (peff->dwSize)
        {
            case sizeof(DIEFFECT) :
                // These parameters are present in the new version of the structure but not in the old.
                if (0 != (dwFlags & DIEP_STARTDELAY))
                {
                    if (false == updatedEffect->SetStartDelay((Controller::ForceFeedback::TEffectTimeMs)peff->dwStartDelay))
                        LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
                }
                break;

                case sizeof(DIEFFECT_DX5) :
                    // No parameters are present in the old version of the structure but removed from the new.
                    break;

                default:
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
        }

        if (0 != (dwFlags & DIEP_AXES))
        {
            if (peff->cAxes > Controller::ForceFeedback::kEffectAxesMaximumNumber)
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

            if (nullptr == peff->rgdwAxes)
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

            Controller::ForceFeedback::SAssociatedAxes newAssociatedAxes = {.count = (int)peff->cAxes};
            if ((size_t)newAssociatedAxes.count > newAssociatedAxes.type.size())
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

            DWORD identifyElementMethod = 0;

            switch (peff->dwFlags & (DIEFF_OBJECTIDS | DIEFF_OBJECTOFFSETS))
            {
            case DIEFF_OBJECTIDS:
                identifyElementMethod = DIPH_BYID;
                break;

            case DIEFF_OBJECTOFFSETS:
                identifyElementMethod = DIPH_BYOFFSET;
                break;

            default:
                // It is an error if the caller does not specify exactly one specific way of identifying axis objects.
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
            }

            for (int i = 0; i < newAssociatedAxes.count; ++i)
            {
                const std::optional<Controller::SElementIdentifier> kMaybeElement = associatedDevice.IdentifyElement(peff->rgdwAxes[i], identifyElementMethod);
                if (false == kMaybeElement.has_value())
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
                
                const Controller::SElementIdentifier kElement = kMaybeElement.value();
                if (Controller::EElementType::Axis != kElement.type)
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

                newAssociatedAxes.type[i] = kElement.axis;
            }

            if (false == updatedEffect->SetAssociatedAxes(newAssociatedAxes))
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
        }

        if (0 != (dwFlags & DIEP_DIRECTION))
        {
            if (peff->cAxes > Controller::ForceFeedback::kEffectAxesMaximumNumber)
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

            if (nullptr == peff->rglDirection)
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

            Controller::ForceFeedback::TEffectValue coordinates[Controller::ForceFeedback::kEffectAxesMaximumNumber] = {};
            int numCoordinates = peff->cAxes;

            for (int i = 0; i < numCoordinates; ++i)
                coordinates[i] = (Controller::ForceFeedback::TEffectValue)peff->rglDirection[i];

            bool coordinateSetResult = false;
            switch (peff->dwFlags & (DIEFF_CARTESIAN | DIEFF_POLAR | DIEFF_SPHERICAL))
            {
            case DIEFF_CARTESIAN:
                coordinateSetResult = updatedEffect->Direction().SetDirectionUsingCartesian(coordinates, numCoordinates);
                break;

            case DIEFF_POLAR:
                coordinateSetResult = updatedEffect->Direction().SetDirectionUsingPolar(coordinates, numCoordinates);
                break;

            case DIEFF_SPHERICAL:
                coordinateSetResult = updatedEffect->Direction().SetDirectionUsingSpherical(coordinates, numCoordinates);
                break;

            default:
                // It is an error if the caller does not specify exactly one coordinate system.
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
            }

            if (true != coordinateSetResult)
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
        }

        if (0 != (dwFlags & DIEP_DURATION))
        {
            if (false == updatedEffect->SetDuration((Controller::ForceFeedback::TEffectTimeMs)peff->dwDuration))
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
        }

        if (0 != (dwFlags & DIEP_ENVELOPE))
        {
            if (nullptr == peff->lpEnvelope)
            {
                updatedEffect->ClearEnvelope();
            }
            else
            {
                if (sizeof(DIENVELOPE) != peff->lpEnvelope->dwSize)
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

                const Controller::ForceFeedback::SEnvelope kNewEnvelope = {
                    .attackTime = (Controller::ForceFeedback::TEffectTimeMs)peff->lpEnvelope->dwAttackTime,
                    .attackLevel = (Controller::ForceFeedback::TEffectValue)peff->lpEnvelope->dwAttackLevel,
                    .fadeTime = (Controller::ForceFeedback::TEffectTimeMs)peff->lpEnvelope->dwFadeTime,
                    .fadeLevel = (Controller::ForceFeedback::TEffectValue)peff->lpEnvelope->dwFadeLevel
                };

                if (false == updatedEffect->SetEnvelope(kNewEnvelope))
                    LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
            }
        }

        if (0 != (dwFlags & DIEP_GAIN))
        {
            if (false == updatedEffect->SetGain((Controller::ForceFeedback::TEffectValue)peff->dwGain))
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
        }

        if (0 != (dwFlags & DIEP_SAMPLEPERIOD))
        {
            if (false == updatedEffect->SetSamplePeriod((Controller::ForceFeedback::TEffectTimeMs)peff->dwSamplePeriod))
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
        }

        // Final sync operation is expected to succeed.
        if (false == effect->SyncParametersFrom(*updatedEffect))
        {
            Message::OutputFormatted(Message::ESeverity::Error, L"Internal error while syncing new parameters for a force feedback effect associated with Xidi virtual controller %u.", (1 + associatedDevice.GetVirtualController().GetIdentifier()));
            LOG_INVOCATION_AND_RETURN(DIERR_GENERIC, kMethodSeverity);
        }

        // Destroying this object now means that any future references to it will trigger crashes during testing.
        updatedEffect = nullptr;

        // At this point parameter updates were successful. What happens next depends on the flag values.
        // The effect could either be downloaded, downloaded and (re)started, or none of these.
        if (0 != (dwFlags & DIEP_NODOWNLOAD))
        {
            LOG_INVOCATION_AND_RETURN(DI_DOWNLOADSKIPPED, kMethodSeverity);
        }
        else
        {
            // It is not an error if the physical device has not been acquired in exclusive mode.
            // In that case, the download operation is skipped but parameters are still updated.
            Controller::ForceFeedback::Device* forceFeedbackDevice = associatedDevice.GetVirtualController().ForceFeedbackGetDevice();
            if (nullptr == forceFeedbackDevice)
                LOG_INVOCATION_AND_RETURN(DI_DOWNLOADSKIPPED, kMethodSeverity);

            // If the download operation fails the parameters have still been updated but the caller needs to be provided the reason for the failure.
            const HRESULT kDownloadResult = DownloadEffectToDevice(*effect, *forceFeedbackDevice);
            if (DI_OK != kDownloadResult)
                LOG_INVOCATION_AND_RETURN(kDownloadResult, kMethodSeverity);
        }

        // Default behavior is to update an effect without changing its play state. Playing effects are updated on-the-fly, and non-playing effects are not started.
        if (0 != (dwFlags & DIEP_START))
        {
            // Getting to this point means the effect exists on the device.
            // Starting or restarting the effect requires that the device be acquired in exclusive mode, although since the download succeeded this should be the case already.
            Controller::ForceFeedback::Device* forceFeedbackDevice = associatedDevice.GetVirtualController().ForceFeedbackGetDevice();
            if (nullptr == forceFeedbackDevice)
            {
                // This should never happen. It means an effect exists on the device and yet the device is somehow not acquired in exclusive mode.
                Message::OutputFormatted(Message::ESeverity::Error, L"Internal error while attempting to start or restart a force feedback effect after setting its parameters on Xidi virtual controller %u.", (1 + associatedDevice.GetVirtualController().GetIdentifier()));
                LOG_INVOCATION_AND_RETURN(DIERR_GENERIC, kMethodSeverity);
            }

            forceFeedbackDevice->StopEffect(effect->Identifier());

            if (false == forceFeedbackDevice->StartEffect(effect->Identifier(), 1))
            {
                // This should never happen. It means an effect that in theory should be downloaded and ready to play is somehow unable to be started.
                Message::OutputFormatted(Message::ESeverity::Error, L"Internal error while attempting to start or restart a force feedback effect after setting its parameters on Xidi virtual controller %u.", (1 + associatedDevice.GetVirtualController().GetIdentifier()));
                LOG_INVOCATION_AND_RETURN(DIERR_GENERIC, kMethodSeverity);
            }
        }

        LOG_INVOCATION_AND_RETURN(DI_OK, kMethodSeverity);
    }

    // --------

    template <ECharMode charMode> HRESULT VirtualDirectInputEffect<charMode>::Start(DWORD dwIterations, DWORD dwFlags)
    {
        constexpr Message::ESeverity kMethodSeverity = Message::ESeverity::Info;

        if (0 == dwIterations)
            LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

        Controller::ForceFeedback::Device* const forceFeedbackDevice = associatedDevice.GetVirtualController().ForceFeedbackGetDevice();
        if (nullptr == forceFeedbackDevice)
            LOG_INVOCATION_AND_RETURN(DIERR_NOTEXCLUSIVEACQUIRED, kMethodSeverity);

        if (0 != (dwFlags & DIES_NODOWNLOAD))
        {
            // The download operation was skipped by the caller.
            // If the effect does not already exist on the device then the effect cannot be played.
            if (false == forceFeedbackDevice->IsEffectOnDevice(effect->Identifier()))
                LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);
        }
        else
        {
            // The download operation was not skipped by the caller.
            // If the effect exists on the device its parameters will get updated, otherwise the effect will be downloaded.
            // If for some reason the download attempt fails then the effect cannot be played.
            const HRESULT kDownloadResult = DownloadEffectToDevice(*effect, *forceFeedbackDevice);
            if (DI_OK != kDownloadResult)
                LOG_INVOCATION_AND_RETURN(kDownloadResult, kMethodSeverity);
        }

        if (0 != (dwFlags & DIES_SOLO))
            forceFeedbackDevice->StopAllEffects();
        else
            forceFeedbackDevice->StopEffect(effect->Identifier());

        if (false == forceFeedbackDevice->StartEffect(effect->Identifier(), (unsigned int)dwIterations))
        {
            Message::OutputFormatted(Message::ESeverity::Error, L"Internal error while starting a force feedback effect associated with Xidi virtual controller %u.", (1 + associatedDevice.GetVirtualController().GetIdentifier()));
            LOG_INVOCATION_AND_RETURN(DIERR_GENERIC, kMethodSeverity);
        }

        LOG_INVOCATION_AND_RETURN(DI_OK, kMethodSeverity);
    }

    // --------

    template <ECharMode charMode> HRESULT VirtualDirectInputEffect<charMode>::Stop(void)
    {
        constexpr Message::ESeverity kMethodSeverity = Message::ESeverity::Info;

        Controller::ForceFeedback::Device* const forceFeedbackDevice = associatedDevice.GetVirtualController().ForceFeedbackGetDevice();
        if (nullptr == forceFeedbackDevice)
            LOG_INVOCATION_AND_RETURN(DIERR_NOTEXCLUSIVEACQUIRED, kMethodSeverity);

        forceFeedbackDevice->StopEffect(effect->Identifier());
        LOG_INVOCATION_AND_RETURN(DI_OK, kMethodSeverity);
    }

    // --------

    template <ECharMode charMode> HRESULT VirtualDirectInputEffect<charMode>::GetEffectStatus(LPDWORD pdwFlags)
    {
        constexpr Message::ESeverity kMethodSeverity = Message::ESeverity::Info;

        if (nullptr == pdwFlags)
            LOG_INVOCATION_AND_RETURN(DIERR_INVALIDPARAM, kMethodSeverity);

        *pdwFlags = 0;

        Controller::ForceFeedback::Device* const forceFeedbackDevice = associatedDevice.GetVirtualController().ForceFeedbackGetDevice();
        if (nullptr == forceFeedbackDevice)
            LOG_INVOCATION_AND_RETURN(DI_OK, kMethodSeverity);

        if (true == forceFeedbackDevice->IsEffectPlaying(effect->Identifier()))
            *pdwFlags |= DIEGES_PLAYING;

        LOG_INVOCATION_AND_RETURN(DI_OK, kMethodSeverity);
    }

    // --------

    template <ECharMode charMode> HRESULT VirtualDirectInputEffect<charMode>::Download(void)
    {
        constexpr Message::ESeverity kMethodSeverity = Message::ESeverity::Info;

        Controller::ForceFeedback::Device* const forceFeedbackDevice = associatedDevice.GetVirtualController().ForceFeedbackGetDevice();
        if (nullptr == forceFeedbackDevice)
            LOG_INVOCATION_AND_RETURN(DIERR_NOTEXCLUSIVEACQUIRED, kMethodSeverity);

        const HRESULT kDownloadResult = DownloadEffectToDevice(*effect, *forceFeedbackDevice);
        LOG_INVOCATION_AND_RETURN(kDownloadResult, kMethodSeverity);
    }

    // --------

    template <ECharMode charMode> HRESULT VirtualDirectInputEffect<charMode>::Unload(void)
    {
        constexpr Message::ESeverity kMethodSeverity = Message::ESeverity::Info;

        Controller::ForceFeedback::Device* const forceFeedbackDevice = associatedDevice.GetVirtualController().ForceFeedbackGetDevice();
        if (nullptr == forceFeedbackDevice)
            LOG_INVOCATION_AND_RETURN(DIERR_NOTEXCLUSIVEACQUIRED, kMethodSeverity);
        
        forceFeedbackDevice->RemoveEffect(effect->Identifier());
        LOG_INVOCATION_AND_RETURN(DI_OK, kMethodSeverity);
    }

    // --------

    template <ECharMode charMode> HRESULT VirtualDirectInputEffect<charMode>::Escape(LPDIEFFESCAPE pesc)
    {
        constexpr Message::ESeverity kMethodSeverity = Message::ESeverity::Info;
        LOG_INVOCATION_AND_RETURN(DIERR_UNSUPPORTED, kMethodSeverity);
    }


    // -------- EXPLICIT TEMPLATE INSTANTIATION ---------------------------- //
    // Instantiates both the ASCII and Unicode versions of this class.

    template class VirtualDirectInputEffect<ECharMode::A>;
    template class VirtualDirectInputEffect<ECharMode::W>;
}

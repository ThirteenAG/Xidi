/*****************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 *****************************************************************************
 * Authored by Samuel Grossman
 * Copyright (c) 2016-2022
 *************************************************************************//**
 * @file Mapper.cpp
 *   Implementation of functionality used to implement mappings of an entire
 *   XInput controller layout to a virtual controller layout.
 *****************************************************************************/

#include "ApiBitSet.h"
#include "ApiWindows.h"
#include "Configuration.h"
#include "ControllerTypes.h"
#include "ElementMapper.h"
#include "ForceFeedbackTypes.h"
#include "Globals.h"
#include "Mapper.h"
#include "Message.h"
#include "Strings.h"

#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <string_view>
#include <xinput.h>


namespace Xidi
{
    namespace Controller
    {
        // -------- INTERNAL TYPES ----------------------------------------- //

        /// Holds a mapping from strings to instances of mapper objects.
        /// Implemented as a singleton object and intended for internal use.
        class MapperRegistry
        {
        private:
            // -------- INSTANCE VARIABLES --------------------------------- //

            /// Implements the registry of known mappers.
            std::map<std::wstring_view, const Mapper*> knownMappers;

            /// Holds the map key that corresponds to the default mapper.
            /// The first type of mapper that is registered becomes the default.
            std::wstring_view defaultMapper;


            // -------- CONSTRUCTION AND DESTRUCTION ----------------------- //

            /// Default constructor. Objects cannot be constructed externally.
            MapperRegistry(void) = default;


        public:
            // -------- CLASS METHODS -------------------------------------- //

            /// Returns a reference to the singleton instance of this class.
            /// @return Reference to the singleton instance.
            static MapperRegistry& GetInstance(void)
            {
                static MapperRegistry mapperRegistry;
                return mapperRegistry;
            }


            // -------- INSTANCE METHODS ----------------------------------- //

            /// Dumps all mappers in this registry.
            void DumpRegisteredMappers(void)
            {
                constexpr Message::ESeverity kDumpSeverity = Message::ESeverity::Info;

                if (Message::WillOutputMessageOfSeverity(kDumpSeverity))
                {
                    Message::Output(kDumpSeverity, L"Begin dump of all known mappers.");
                    
                    for (const auto& knownMapper : knownMappers)
                    {
                        const std::wstring_view kKnownMapperName = knownMapper.first;
                        const SCapabilities kKnownMapperCapabilities = knownMapper.second->GetCapabilities();

                        Message::OutputFormatted(kDumpSeverity, L"  %s:", kKnownMapperName.data());
                        
                        Message::OutputFormatted(kDumpSeverity, L"    numAxes = %u", (unsigned int)kKnownMapperCapabilities.numAxes);
                        for (unsigned int i = 0; i < kKnownMapperCapabilities.numAxes; ++i)
                            Message::OutputFormatted(kDumpSeverity, L"      axisCapabilities[%u] = { type = %s, supportsForceFeedback = %s }", i, Strings::AxisTypeString(kKnownMapperCapabilities.axisCapabilities[i].type), ((true == kKnownMapperCapabilities.axisCapabilities[i].supportsForceFeedback) ? L"true" : L"false"));

                        Message::OutputFormatted(kDumpSeverity, L"    numButtons = %u", (unsigned int)kKnownMapperCapabilities.numButtons);
                        Message::OutputFormatted(kDumpSeverity, L"    hasPov = %s", ((true == kKnownMapperCapabilities.hasPov) ? L"true" : L"false"));
                    }

                    Message::Output(kDumpSeverity, L"End dump of all known mappers.");
                }
            }
            
            /// Registers a mapper object with this registry.
            /// @param [in] name Name to associate with the mapper.
            /// @param [in] object Corresponding mapper object.
            void RegisterMapper(std::wstring_view name, const Mapper* object)
            {
                if (true == name.empty())
                {
                    Message::Output(Message::ESeverity::Error, L"Internal error: Attempting to register a mapper without a name.");
                    return;
                }

                knownMappers[name] = object;

                if (true == defaultMapper.empty())
                    defaultMapper = name;
            }

            /// Unregisters a mapper object from this registry, if the registration details provided match the contents of the registry.
            /// @param [in] name Name associated with the mapper.
            /// @param [in] object Corresponding mapper object.
            void UnregisterMapper(std::wstring_view name, const Mapper* object)
            {
                if (true == name.empty())
                {
                    Message::Output(Message::ESeverity::Error, L"Internal error: Attempting to unregister a mapper without a name.");
                    return;
                }

                if (false == knownMappers.contains(name))
                {
                    Message::OutputFormatted(Message::ESeverity::Error, L"Internal error: Attempting to unregister unknown mapper %s.", name.data());
                    return;
                }

                if (object != knownMappers.at(name))
                {
                    Message::OutputFormatted(Message::ESeverity::Error, L"Internal error: Object mismatch while attempting to unregister mapper %s.", name.data());
                    return;
                }

                knownMappers.erase(name);

                if (defaultMapper == name)
                    defaultMapper = std::wstring_view();
            }

            /// Retrieves a pointer to the mapper object that corresponds to the specified name, if it exists.
            /// @param [in] mapperName Desired mapper name.
            /// @return Pointer to the corresponding mapper object, or `nullptr` if it does not exist in the registry.
            const Mapper* GetMapper(std::wstring_view mapperName)
            {
                if (true == mapperName.empty())
                    mapperName = defaultMapper;

                const auto mapperRecord = knownMappers.find(mapperName);
                if (knownMappers.cend() != mapperRecord)
                    return mapperRecord->second;

                return nullptr;
            }
        };


        // -------- INTERNAL FUNCTIONS ------------------------------------- //

        /// Derives the capabilities of the controller that is described by the specified element mappers in aggregate.
        /// Number of axes is determined as the total number of unique axes on the virtual controller to which element mappers contribute.
        /// Number of buttons is determined by looking at the highest button number to which element mappers contribute.
        /// Presence or absence of a POV is determined by whether or not any element mappers contribute to a POV direction, even if not all POV directions have a contribution.
        /// @param [in] elements Per-element controller map.
        /// @param [in] forceFeedbackActuators Per-element force feedback actuator map.
        /// @return Virtual controller capabilities as derived from the per-element map in aggregate.
        static SCapabilities DeriveCapabilitiesFromElementMap(const Mapper::UElementMap& elements, Mapper::UForceFeedbackActuatorMap forceFeedbackActuators)
        {
            SCapabilities capabilities;
            ZeroMemory(&capabilities, sizeof(capabilities));

            BitSetEnum<EAxis> axesPresent = Mapper::kRequiredAxes;
            BitSetEnum<EAxis> axesForceFeedback = Mapper::kRequiredForceFeedbackAxes;

            int highestButtonSeen = Mapper::kMinNumButtons - 1;
            bool povPresent = Mapper::kIsPovRequired;

            for (int i = 0; i < _countof(elements.all); ++i)
            {
                if (nullptr != elements.all[i])
                {
                    for (int j = 0; j < elements.all[i]->GetTargetElementCount(); ++j)
                    {
                        const std::optional<SElementIdentifier> maybeTargetElement = elements.all[i]->GetTargetElementAt(j);
                        if (false == maybeTargetElement.has_value())
                            continue;

                        const SElementIdentifier targetElement = maybeTargetElement.value();
                        switch (targetElement.type)
                        {
                        case EElementType::Axis:
                            if ((int)targetElement.axis < (int)EAxis::Count)
                                axesPresent.insert((int)targetElement.axis);
                            break;

                        case EElementType::Button:
                            if ((int)targetElement.button < (int)EButton::Count)
                            {
                                if ((int)targetElement.button > highestButtonSeen)
                                    highestButtonSeen = (int)targetElement.button;
                            }
                            break;

                        case EElementType::Pov:
                            povPresent = true;
                            break;
                        }
                    }
                }
            }

            for (int i = 0; i < _countof(forceFeedbackActuators.all); ++i)
            {
                if (true == forceFeedbackActuators.all[i].isPresent)
                {
                    switch (forceFeedbackActuators.all[i].mode)
                    {
                    case ForceFeedback::EActuatorMode::SingleAxis:
                        axesPresent.insert((int)forceFeedbackActuators.all[i].singleAxis.axis);
                        axesForceFeedback.insert((int)forceFeedbackActuators.all[i].singleAxis.axis);
                        break;

                    case ForceFeedback::EActuatorMode::MagnitudeProjection:
                        axesPresent.insert((int)forceFeedbackActuators.all[i].magnitudeProjection.axisFirst);
                        axesPresent.insert((int)forceFeedbackActuators.all[i].magnitudeProjection.axisSecond);
                        axesForceFeedback.insert((int)forceFeedbackActuators.all[i].magnitudeProjection.axisFirst);
                        axesForceFeedback.insert((int)forceFeedbackActuators.all[i].magnitudeProjection.axisSecond);
                        break;

                    default:
                        break;
                    }
                }
            }

            for (auto axisPresent : axesPresent)
                capabilities.AppendAxis(
                    {
                        .type = (EAxis)((int)axisPresent),
                        .supportsForceFeedback = axesForceFeedback.contains(axisPresent)
                    }
                );

            capabilities.numButtons = highestButtonSeen + 1;
            capabilities.hasPov = povPresent;

            return capabilities;
        }

        /// Filters (by saturation) analog stick values that might be slightly out of range due to differences between the implemented range and the XInput actual range.
        /// @param [in] analogValue Raw analog value.
        /// @return Filtered analog value, which will most likely be the same as the input.
        static inline int16_t FilterAnalogStickValue(int16_t analogValue)
        {
            if (analogValue > Controller::kAnalogValueMax)
                return Controller::kAnalogValueMax;
            else if (analogValue < Controller::kAnalogValueMin)
                return Controller::kAnalogValueMin;
            else
                return analogValue;
        }

        /// Filters and inverts analog stick values based on presentation differences between XInput and virtual controller needs.
        /// Useful for XInput axes that have opposite polarity as compared to virtual controller axes.
        /// @param [in] analogValue Raw analog value.
        /// @return Filtered and inverted analog value.
        static inline int16_t FilterAndInvertAnalogStickValue(int16_t analogValue)
        {
            return -FilterAnalogStickValue(analogValue);
        }

        /// Computes the physical force feedback actuator value for the specified actuator given a vector of magnitude components.
        /// @param [in] virtualEffectComponents Virtual force feedback vector expressed as a magnitude component vector.
        /// @param [in] actuatorElement Physical force feedback actuator element for which an actuator value is desired.
        /// @param [in] gain Gain modifier to apply as a scalar multiplier on the physical actuator value.
        /// @return Physical force feedback actuator value that can be sent directly to the actuator itself.
        static inline ForceFeedback::TPhysicalActuatorValue ForceFeedbackActuatorValue(ForceFeedback::TOrderedMagnitudeComponents virtualEffectComponents, ForceFeedback::SActuatorElement actuatorElement, ForceFeedback::TEffectValue gain)
        {
            if (false == actuatorElement.isPresent)
                return 0;

            ForceFeedback::TEffectValue virtualActuatorStrengthRaw = 0;

            switch (actuatorElement.mode)
            {
            case ForceFeedback::EActuatorMode::SingleAxis:
                do {
                    if (ForceFeedback::kEffectForceMagnitudeZero == virtualEffectComponents[(int)actuatorElement.singleAxis.axis])
                        return 0;

                    const bool kActuatorDirectionIsNegative = std::signbit(virtualEffectComponents[(int)actuatorElement.singleAxis.axis]);
                    if ((EAxisDirection::Positive == actuatorElement.singleAxis.direction) && (true == kActuatorDirectionIsNegative))
                        return 0;
                    if ((EAxisDirection::Negative == actuatorElement.singleAxis.direction) && (false == kActuatorDirectionIsNegative))
                        return 0;

                    virtualActuatorStrengthRaw = virtualEffectComponents[(int)actuatorElement.singleAxis.axis];
                } while (false);
                break;

            case ForceFeedback::EActuatorMode::MagnitudeProjection:
                virtualActuatorStrengthRaw = (ForceFeedback::TEffectValue)std::sqrt(std::pow(virtualEffectComponents[(int)actuatorElement.magnitudeProjection.axisFirst], 2) + std::pow(virtualEffectComponents[(int)actuatorElement.magnitudeProjection.axisSecond], 2));
                break;

            default:
                return 0;
            }

            constexpr ForceFeedback::TEffectValue kPhysicalActuatorRange = (ForceFeedback::TEffectValue)(std::numeric_limits<ForceFeedback::TPhysicalActuatorValue>::max() - std::numeric_limits<ForceFeedback::TPhysicalActuatorValue>::min());
            constexpr ForceFeedback::TEffectValue kVirtualMagnitudeRange = ForceFeedback::kEffectForceMagnitudeMaximum - ForceFeedback::kEffectForceMagnitudeZero;
            constexpr ForceFeedback::TEffectValue kScalingFactor = kPhysicalActuatorRange / kVirtualMagnitudeRange;

            const ForceFeedback::TEffectValue kGainMultiplier = gain / ForceFeedback::kEffectModifierMaximum;
            const ForceFeedback::TEffectValue kVirtualActuatorStrengthMax = (ForceFeedback::kEffectForceMagnitudeMaximum - ForceFeedback::kEffectForceMagnitudeZero) * kGainMultiplier;

            const ForceFeedback::TEffectValue kVirtualActuatorStrength = std::min(kVirtualActuatorStrengthMax, kGainMultiplier * std::abs(virtualActuatorStrengthRaw - ForceFeedback::kEffectForceMagnitudeZero));
            const long kPhysicalActuatorStrength = std::lround(kVirtualActuatorStrength * kScalingFactor);

            return (ForceFeedback::TPhysicalActuatorValue)kPhysicalActuatorStrength;
        }


        // -------- CONSTRUCTION AND DESTRUCTION --------------------------- //
        // See "Mapper.h" for documentation.

        Mapper::UElementMap::UElementMap(const UElementMap& other) : named()
        {
            for (int i = 0; i < _countof(all); ++i)
            {
                if (nullptr != other.all[i])
                    all[i] = other.all[i]->Clone();
            }
        }

        // --------

        Mapper::Mapper(const std::wstring_view name, SElementMap&& elements, SForceFeedbackActuatorMap forceFeedbackActuators) : elements(std::move(elements)), forceFeedbackActuators(forceFeedbackActuators), capabilities(DeriveCapabilitiesFromElementMap(this->elements, forceFeedbackActuators)), name(name)
        {
            if (false == name.empty())
                MapperRegistry::GetInstance().RegisterMapper(name, this);
        }

        // --------

        Mapper::Mapper(SElementMap&& elements, SForceFeedbackActuatorMap forceFeedbackActuators) : Mapper(L"", std::move(elements), forceFeedbackActuators)
        {
            // Nothing to do here.
        }

        // --------

        Mapper::~Mapper(void)
        {
            if (false == name.empty())
                MapperRegistry::GetInstance().UnregisterMapper(name, this);
        }


        // -------- OPERATORS ---------------------------------------------- //
        // See "Mapper.h" for documentation.

        Mapper::UElementMap& Mapper::UElementMap::operator=(const UElementMap& other)
        {
            for (int i = 0; i < _countof(all); ++i)
            {
                if (nullptr == other.all[i])
                    all[i] = nullptr;
                else
                    all[i] = other.all[i]->Clone();
            }

            return *this;
        }

        // --------

        Mapper::UElementMap& Mapper::UElementMap::operator=(UElementMap&& other)
        {
            for (int i = 0; i < _countof(all); ++i)
                all[i] = std::move(other.all[i]);

            return *this;
        }


        // -------- CLASS METHODS ------------------------------------------ //
        // See "Mapper.h" for documentation.

        void Mapper::DumpRegisteredMappers(void)
        {
            MapperRegistry::GetInstance().DumpRegisteredMappers();
        }

        // --------

        const Mapper* Mapper::GetByName(std::wstring_view mapperName)
        {
            return MapperRegistry::GetInstance().GetMapper(mapperName);
        }

        // --------

        const Mapper* Mapper::GetConfigured(TControllerIdentifier controllerIdentifier)
        {
            static const Mapper* configuredMapper[kPhysicalControllerCount];
            static std::once_flag configuredMapperFlag;

            std::call_once(configuredMapperFlag, []() -> void
                {
                    const Configuration::ConfigurationData& configData = Globals::GetConfigurationData();

                    if (true == configData.SectionExists(Strings::kStrConfigurationSectionMapper))
                    {
                        // Mapper section exists in the configuration file.
                        // If the controller-independent type setting exists, it will be used as the fallback default, otherwise the default mapper will be used for this purpose.
                        // If any per-controller type settings exist, they take precedence.
                        const auto& mapperConfigData = configData[Strings::kStrConfigurationSectionMapper];

                        const Mapper* fallbackMapper = nullptr;
                        if (true == mapperConfigData.NameExists(Strings::kStrConfigurationSettingMapperType))
                        {
                            std::wstring_view fallbackMapperName = mapperConfigData[Strings::kStrConfigurationSettingMapperType].FirstValue().GetStringValue();
                            fallbackMapper = GetByName(fallbackMapperName);

                            if (nullptr == fallbackMapper)
                                Message::OutputFormatted(Message::ESeverity::Warning, L"Could not locate mapper \"%s\" specified in the configuration file as the default.", fallbackMapperName.data());
                        }

                        if (nullptr == fallbackMapper)
                        {
                            fallbackMapper = GetDefault();

                            if (nullptr == fallbackMapper)
                            {
                                Message::Output(Message::ESeverity::Error, L"Internal error: Unable to locate the default mapper.");
                                fallbackMapper = GetNull();
                            }
                        }
                        
                        for (TControllerIdentifier i = 0; i < _countof(configuredMapper); ++i)
                        {
                            if (true == mapperConfigData.NameExists(Strings::MapperTypeConfigurationNameString(i)))
                            {
                                std::wstring_view configuredMapperName = mapperConfigData[Strings::MapperTypeConfigurationNameString(i)].FirstValue().GetStringValue();
                                configuredMapper[i] = GetByName(configuredMapperName.data());

                                if (nullptr == configuredMapper[i])
                                {
                                    Message::OutputFormatted(Message::ESeverity::Warning, L"Could not locate mapper \"%s\" specified in the configuration file for controller %u.", configuredMapperName.data(), (unsigned int)(1 + i));
                                    configuredMapper[i] = fallbackMapper;
                                }
                            }
                            else
                            {
                                configuredMapper[i] = fallbackMapper;
                            }
                        }
                    }
                    else
                    {
                        // Mapper section does not exist in the configuration file.
                        const Mapper* defaultMapper = GetDefault();
                        if (nullptr == defaultMapper)
                        {
                            Message::Output(Message::ESeverity::Error, L"Internal error: Unable to locate the default mapper. Virtual controllers will not function.");
                            defaultMapper = GetNull();
                        }

                        for (TControllerIdentifier i = 0; i < _countof(configuredMapper); ++i)
                            configuredMapper[i] = defaultMapper;
                    }

                    Message::Output(Message::ESeverity::Info, L"Mappers assigned to controllers...");
                    for (TControllerIdentifier i = 0; i < _countof(configuredMapper); ++i)
                        Message::OutputFormatted(Message::ESeverity::Info, L"    [%u]: %s", (unsigned int)(1 + i), configuredMapper[i]->GetName().data());
                }
            );

            if (controllerIdentifier >= _countof(configuredMapper))
            {
                Message::OutputFormatted(Message::ESeverity::Error, L"Internal error: Requesting a mapper for out-of-bounds controller %u.", (unsigned int)(1 + controllerIdentifier));
                return GetNull();
            }

            return configuredMapper[controllerIdentifier];
        }

        // --------

        const Mapper* Mapper::GetNull(void)
        {
            static const Mapper kNullMapper({});
            return &kNullMapper;
        }


        // -------- INSTANCE METHODS --------------------------------------- //
        // See "Mapper.h" for documentation.

        ForceFeedback::SPhysicalActuatorComponents Mapper::MapForceFeedbackVirtualToPhysical(ForceFeedback::TOrderedMagnitudeComponents virtualEffectComponents, ForceFeedback::TEffectValue gain) const
        {
            return {
                .leftMotor = ForceFeedbackActuatorValue(virtualEffectComponents, forceFeedbackActuators.named.leftMotor, gain),
                .rightMotor = ForceFeedbackActuatorValue(virtualEffectComponents, forceFeedbackActuators.named.rightMotor, gain),
                .leftImpulseTrigger = ForceFeedbackActuatorValue(virtualEffectComponents, forceFeedbackActuators.named.leftImpulseTrigger, gain),
                .rightImpulseTrigger = ForceFeedbackActuatorValue(virtualEffectComponents, forceFeedbackActuators.named.rightImpulseTrigger, gain)
            };
        }

        SState Mapper::MapStatePhysicalToVirtual(XINPUT_GAMEPAD physicalState) const
        {
            SState controllerState = {};

            // Left and right stick values need to be saturated at the virtual controller range due to a very slight difference between XInput range and virtual controller range.
            // This difference (-32768 extreme negative for XInput vs -32767 extreme negative for Xidi) does not affect functionality when filtered by saturation.
            // Vertical analog axes additionally need to be inverted because XInput presents up as positive and down as negative whereas Xidi needs to do the opposite.
            
            if (nullptr != elements.named.stickLeftX) elements.named.stickLeftX->ContributeFromAnalogValue(controllerState, FilterAnalogStickValue(physicalState.sThumbLX));
            if (nullptr != elements.named.stickLeftY) elements.named.stickLeftY->ContributeFromAnalogValue(controllerState, FilterAndInvertAnalogStickValue(physicalState.sThumbLY));

            if (nullptr != elements.named.stickRightX) elements.named.stickRightX->ContributeFromAnalogValue(controllerState, FilterAnalogStickValue(physicalState.sThumbRX));
            if (nullptr != elements.named.stickRightY) elements.named.stickRightY->ContributeFromAnalogValue(controllerState, FilterAndInvertAnalogStickValue(physicalState.sThumbRY));

            if (nullptr != elements.named.dpadUp) elements.named.dpadUp->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_DPAD_UP)));
            if (nullptr != elements.named.dpadDown) elements.named.dpadDown->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_DPAD_DOWN)));
            if (nullptr != elements.named.dpadLeft) elements.named.dpadLeft->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_DPAD_LEFT)));
            if (nullptr != elements.named.dpadRight) elements.named.dpadRight->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT)));

            if (nullptr != elements.named.triggerLT) elements.named.triggerLT->ContributeFromTriggerValue(controllerState, physicalState.bLeftTrigger);
            if (nullptr != elements.named.triggerRT) elements.named.triggerRT->ContributeFromTriggerValue(controllerState, physicalState.bRightTrigger);

            if (nullptr != elements.named.buttonA) elements.named.buttonA->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_A)));
            if (nullptr != elements.named.buttonB) elements.named.buttonB->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_B)));
            if (nullptr != elements.named.buttonX) elements.named.buttonX->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_X)));
            if (nullptr != elements.named.buttonY) elements.named.buttonY->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_Y)));

            if (nullptr != elements.named.buttonLB) elements.named.buttonLB->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER)));
            if (nullptr != elements.named.buttonRB) elements.named.buttonRB->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER)));

            if (nullptr != elements.named.buttonBack) elements.named.buttonBack->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_BACK)));
            if (nullptr != elements.named.buttonStart) elements.named.buttonStart->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_START)));

            if (nullptr != elements.named.buttonLS) elements.named.buttonLS->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_LEFT_THUMB)));
            if (nullptr != elements.named.buttonRS) elements.named.buttonRS->ContributeFromButtonValue(controllerState, (0 != (physicalState.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB)));

            // Once all contributions have been committed, saturate all axis values at the extreme ends of the allowed range.
            // Doing this at the end means that intermediate contributions are computed with much more range than the controller is allowed to report, which can increase accuracy when there are multiple interfering mappers contributing to axes.
            for (auto& axisValue : controllerState.axis)
            {
                if (axisValue > kAnalogValueMax)
                    axisValue = kAnalogValueMax;
                else if (axisValue < kAnalogValueMin)
                    axisValue = kAnalogValueMin;
            }

            return controllerState;
        }
    }
}

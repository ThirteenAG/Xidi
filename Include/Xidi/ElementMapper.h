/*****************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 *****************************************************************************
 * Authored by Samuel Grossman
 * Copyright (c) 2016-2022
 *************************************************************************//**
 * @file ElementMapper.h
 *   Declaration of functionality used to implement mappings from individual
 *   XInput controller elements to virtual DirectInput controller elements.
 *****************************************************************************/

#pragma once

#include "ControllerTypes.h"
#include "Keyboard.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>


namespace Xidi
{
    namespace Controller
    {
        /// Interface for mapping an XInput controller element's state reading to an internal controller state data structure value.
        /// An instance of this object exists for each XInput controller element in a mapper.
        class IElementMapper
        {
        public:
            // -------- CONSTRUCTION AND DESTRUCTION ----------------------- //

            /// Default destructor.
            virtual ~IElementMapper(void) = default;


            // -------- ABSTRACT INSTANCE METHODS -------------------------- //

            /// Allocates, constructs, and returns a pointer to a copy of this element mapper.
            /// @return Smart pointer to a copy of this element mapper.
            virtual std::unique_ptr<IElementMapper> Clone(void) const = 0;

            /// Calculates the contribution to controller state from a given analog reading in the standard XInput axis range -32768 to +32767.
            /// Contribution is aggregated with anything that already exists in the controller state.
            /// @param [in,out] controllerState Controller state data structure to be updated.
            /// @param [in] analogStickValue Raw analog stick value from the XInput controller.
            virtual void ContributeFromAnalogValue(SState& controllerState, int16_t analogValue) const = 0;

            /// Calculates the contribution to controller state from a given button pressed status reading.
            /// Contribution is aggregated with anything that already exists in the controller state.
            /// @param [in,out] controllerState Controller state data structure to be updated.
            /// @param [in] buttonPressed Button state from the XInput controller, `true` if pressed and `false` otherwise.
            virtual void ContributeFromButtonValue(SState& controllerState, bool buttonPressed) const = 0;

            /// Calculates the contribution to controller state from a given trigger reading in the standard XInput trigger range 0 to 255.
            /// Contribution is aggregated with anything that already exists in the controller state.
            /// @param [in,out] controllerState Controller state data structure to be updated.
            /// @param [in] buttonPressed Button state from the XInput controller, `true` if pressed and `false` otherwise.
            virtual void ContributeFromTriggerValue(SState& controllerState, uint8_t triggerValue) const = 0;

            /// Specifies the number of virtual controller elements that are the target of any contributions from this element mapper.
            /// @return Number of virtual controller elements to which contributions are targetted.
            virtual int GetTargetElementCount(void) const = 0;

            /// Specifies one of the virtual controller elements that is the target of any contributions from this element mapper.
            /// @param [in] index Index of the target element. Must be less than the count reported by #GetTargetElementCount.
            /// @return Identifier of the targert virtual controller element, if it exists.
            virtual std::optional<SElementIdentifier> GetTargetElementAt(int index) const = 0;


            // -------- CONCRETE INSTANCE METHODS -------------------------- //

            /// Specifies that the element mapper should make a neutral state contribution to the virtual controller.
            /// Primarily intended for element mappers that have side effects so that they can reset their side effects in response to not making any contribution.
            /// It is optional to override this method, as a default empty implementation is supplied.
            /// @param [in] controllerIdentifier Identifier of the controller for which an update contribution is being requested.
            /// @param [in,out] controllerState Controller state data structure to be updated.
            virtual void ContributeNeutral(SState& controllerState) const
            {
                // Nothing to do here.
            }
        };

        /// Maps a single XInput controller element such that it contributes to an axis value on a virtual controller.
        /// For analog sticks and triggers, the value read is mapped directly to the corresponding virtual controller axis. Half-axis mode generally makes sense only for triggers because they can share an axis, but it is implemented by range mapping for analog stick axes as well.
        /// For buttons, the value is either negative extreme if the button is not pressed or positive extreme if the value is pressed. Use a half-axis configuration to map to either neutral (not pressed) or extreme value (pressed).
        class AxisMapper : public IElementMapper
        {
        protected:
            // -------- INSTANCE VARIABLES --------------------------------- //

            /// Identifies the axis to which this mapper should contribute in the internal controller state data structure.
            const EAxis axis;

            /// Identifies the direction to which this mapper should contribute on its associated axis.
            /// If set to anything other than both directions, the contribution is to half of the axis only.
            const EAxisDirection direction;


        public:
            // -------- CONSTRUCTION AND DESTRUCTION ----------------------- //

            /// Initialization constructor.
            /// Specifies the axis and, optionally, the direction to which this mapper should contribute in the internal controller state data structure.
            inline constexpr AxisMapper(EAxis axis, EAxisDirection direction = EAxisDirection::Both) : IElementMapper(), axis(axis), direction(direction)
            {
                // Nothing to do here.
            }


            // -------- INSTANCE METHODS ----------------------------------- //

            /// Retrieves and returns the axis direction to which this mapper should contribute on its associated axis.
            /// Intended for tests.
            /// @return Target axis direction.
            inline EAxisDirection GetAxisDirection(void) const
            {
                return direction;
            }


            // -------- CONCRETE INSTANCE METHODS -------------------------- //

            std::unique_ptr<IElementMapper> Clone(void) const override;
            void ContributeFromAnalogValue(SState& controllerState, int16_t analogValue) const override;
            void ContributeFromButtonValue(SState& controllerState, bool buttonPressed) const override;
            void ContributeFromTriggerValue(SState& controllerState, uint8_t triggerValue) const override;
            int GetTargetElementCount(void) const override;
            std::optional<SElementIdentifier> GetTargetElementAt(int index) const override;
        };

        /// Maps a single XInput controller element such that it contributes to a button reading on a virtual controller.
        /// For analog sticks, if the axis displacement from neutral is greater than a threshold, the button is considered pressed.
        /// For triggers, if the magnitude of the trigger reading is greater than a threshold, the button is considered pressed.
        /// For buttons, the button state is mapped directly to the target button.
        class ButtonMapper : public IElementMapper
        {
        private:
            // -------- INSTANCE VARIABLES --------------------------------- //

            /// Identifies the button to which this mapper should contribute in the internal controller state data structure.
            const EButton button;


        public:
            // -------- CONSTRUCTION AND DESTRUCTION ----------------------- //

            /// Initialization constructor.
            /// Specifies the button to which all updates are contributed.
            inline constexpr ButtonMapper(EButton button) : IElementMapper(), button(button)
            {
                // Nothing to do here.
            }


            // -------- CONCRETE INSTANCE METHODS -------------------------- //

            std::unique_ptr<IElementMapper> Clone(void) const override;
            void ContributeFromAnalogValue(SState& controllerState, int16_t analogValue) const override;
            void ContributeFromButtonValue(SState& controllerState, bool buttonPressed) const override;
            void ContributeFromTriggerValue(SState& controllerState, uint8_t triggerValue) const override;
            int GetTargetElementCount(void) const override;
            std::optional<SElementIdentifier> GetTargetElementAt(int index) const override;
        };

        /// Maps a single XInput controller element to multiple underlying element mappers.
        class CompoundMapper : public IElementMapper
        {
        public:
            // -------- CONSTANTS ------------------------------------------ //

            /// Maximum number of underlying element mappers that can be present.
            static constexpr int kMaxUnderlyingElementMappers = 8;


            // -------- TYPE DEFINITIONS ----------------------------------- //

            /// Convenience alias for the type used to hold underlying element mappers.
            typedef std::array<std::unique_ptr<IElementMapper>, kMaxUnderlyingElementMappers> TElementMappers;


        private:
            // -------- INSTANCE VARIABLES --------------------------------- //

            /// Element mappers to which input is forwarded.
            const TElementMappers elementMappers;


        public:
            // -------- CONSTRUCTION AND DESTRUCTION ----------------------- //

            /// Initialization constructor.
            /// Requires an array of underlying element mappers.
            /// Takes ownership of the objects passed as parameters.
            inline CompoundMapper(TElementMappers&& elementMappers) : elementMappers(std::move(elementMappers))
            {
                // Nothing to do here.
            }

            /// Copy constructor.
            /// Simply clones its underlying element mapper array.
            inline CompoundMapper(const CompoundMapper& other) : elementMappers(CopyElementMappers(other.GetElementMappers()))
            {
                // Nothing to do here.
            }


        private:
            // -------- CLASS METHODS -------------------------------------- //

            /// Copies the underlying element mapper data structure.
            /// For internal use only.
            /// @param [in] elementMappers Source element mapper data structure.
            /// @return Copy of the source element mapper data structure.
            static inline TElementMappers CopyElementMappers(const TElementMappers& elementMappers)
            {
                TElementMappers copyOfElementMappers;

                for (size_t i = 0; i < elementMappers.size(); ++i)
                {
                    if (nullptr == elementMappers[i])
                        copyOfElementMappers[i] = nullptr;
                    else
                        copyOfElementMappers[i] = elementMappers[i]->Clone();
                }

                return copyOfElementMappers;
            }


        public:
            // -------- INSTANCE METHODS ----------------------------------- //

            /// Retrieves and returns a read-only reference to the underlying element mapper array.
            /// Intended for tests.
            /// @return Read-only reference to the underlying element mapper array.
            inline const TElementMappers& GetElementMappers(void) const
            {
                return elementMappers;
            }


            // -------- CONCRETE INSTANCE METHODS -------------------------- //

            std::unique_ptr<IElementMapper> Clone(void) const override;
            void ContributeFromAnalogValue(SState& controllerState, int16_t analogValue) const override;
            void ContributeFromButtonValue(SState& controllerState, bool buttonPressed) const override;
            void ContributeFromTriggerValue(SState& controllerState, uint8_t triggerValue) const override;
            void ContributeNeutral(SState& controllerState) const override;
            int GetTargetElementCount(void) const override;
            std::optional<SElementIdentifier> GetTargetElementAt(int index) const override;
        };

        /// Maps a single XInput controller element such that it contributes to an axis value on a virtual controller, but removes analog functionality. Values contributed are either zero or extreme.
        /// For analog sticks, the value read is mapped to either neutral or an extreme axis value. In whole-axis mode, the possible values are negative extreme, neutral, and positive extreme. In half-axis mode, possible values are neutral and extreme (input in the inactive direction is ignored).
        /// For triggers, which unlike analog sticks do not have a centered neutral position, possible values depend on the axis mode. In whole-axis mode, the possible values are negative extreme and positive extreme. In half-axis mode, the possible values are neutral and extreme.
        /// For buttons, the behavior is the same as the standard button-to-axis mapping behavior.
        class DigitalAxisMapper : public AxisMapper
        {
        public:
            // -------- CONSTRUCTION AND DESTRUCTION ----------------------- //

            /// Initialization constructor.
            /// Specifies the axis and, optionally, the direction to which this mapper should contribute in the internal controller state data structure.
            inline constexpr DigitalAxisMapper(EAxis axis, EAxisDirection direction = EAxisDirection::Both) : AxisMapper(axis, direction)
            {
                // Nothing to do here.
            }


            // -------- CONCRETE INSTANCE METHODS -------------------------- //

            std::unique_ptr<IElementMapper> Clone(void) const override;
            void ContributeFromAnalogValue(SState& controllerState, int16_t analogValue) const override;
            void ContributeFromTriggerValue(SState& controllerState, uint8_t triggerValue) const override;
        };

        /// Inverts the input reading from an XInput controller element and then forwards it to another element mapper.
        class InvertMapper : public IElementMapper
        {
        private:
            // -------- INSTANCE VARIABLES --------------------------------- //

            /// Mapper to which inverted input is forwarded.
            const std::unique_ptr<const IElementMapper> elementMapper;


        public:
            // -------- CONSTRUCTION AND DESTRUCTION ----------------------- //

            /// Initialization constructor.
            /// Requires an underlying element mapper.
            /// Takes ownership of the objects passed as parameters.
            inline InvertMapper(std::unique_ptr<const IElementMapper>&& elementMapper) : elementMapper(std::move(elementMapper))
            {
                // Nothing to do here.
            }

            /// Copy constructor.
            /// Simply clones its underlying element mapper.
            inline InvertMapper(const InvertMapper& other) : elementMapper((other.elementMapper != nullptr) ? other.elementMapper->Clone() : nullptr)
            {
                // Nothing to do here.
            }


            // -------- INSTANCE METHODS ----------------------------------- //

            /// Retrieves and returns a raw read-only pointer to the underlying element mapper.
            /// This object maintains ownership over the returned pointer.
            /// Intended for tests.
            /// @return Read-only pointer to the underlying element mapper.
            inline const IElementMapper* GetElementMapper(void) const
            {
                return elementMapper.get();
            }


            // -------- CONCRETE INSTANCE METHODS -------------------------- //

            std::unique_ptr<IElementMapper> Clone(void) const override;
            void ContributeFromAnalogValue(SState& controllerState, int16_t analogValue) const override;
            void ContributeFromButtonValue(SState& controllerState, bool buttonPressed) const override;
            void ContributeFromTriggerValue(SState& controllerState, uint8_t triggerValue) const override;
            void ContributeNeutral(SState& controllerState) const override;
            int GetTargetElementCount(void) const override;
            std::optional<SElementIdentifier> GetTargetElementAt(int index) const override;
        };
        
        /// Maps a single XInput controller element to a keyboard key.
        /// For analog sticks, if the axis displacement from neutral is greater than a threshold, the keyboard key is considered pressed.
        /// For triggers, if the magnitude of the trigger reading is greater than a threshold, the keyboard key is considered pressed.
        /// For buttons, the button state is mapped directly to the target keyboard key.
        class KeyboardMapper : public IElementMapper
        {
        private:
            // -------- INSTANCE VARIABLES --------------------------------- //

            /// Identifies the keyboard key to which this mapper should contribute on the virtual keyboard.
            const Keyboard::TKeyIdentifier key;


        public:
            // -------- CONSTRUCTION AND DESTRUCTION ----------------------- //

            /// Initialization constructor.
            /// Specifies the keyboard key to which to contribute.
            inline constexpr KeyboardMapper(Keyboard::TKeyIdentifier key) : key(key)
            {
                // Nothing to do here.
            }


            // -------- INSTANCE METHODS ----------------------------------- //

            /// Retrieves and returns the target keyboard key to which this object contributes.
            /// Intended for tests.
            /// @return Target keyboard key identifier.
            inline Keyboard::TKeyIdentifier GetTargetKey(void) const
            {
                return key;
            }


            // -------- CONCRETE INSTANCE METHODS -------------------------- //

            std::unique_ptr<IElementMapper> Clone(void) const override;
            void ContributeFromAnalogValue(SState& controllerState, int16_t analogValue) const override;
            void ContributeFromButtonValue(SState& controllerState, bool buttonPressed) const override;
            void ContributeFromTriggerValue(SState& controllerState, uint8_t triggerValue) const override;
            void ContributeNeutral(SState& controllerState) const override;
            int GetTargetElementCount(void) const override;
            std::optional<SElementIdentifier> GetTargetElementAt(int index) const override;
        };

        /// Maps a single XInput controller element such that it contributes to a POV on a virtual controller.
        class PovMapper : public IElementMapper
        {
        private:
            // -------- INSTANCE VARIABLES --------------------------------- //

            /// Identifies the POV direction to which this mapper should contribute in the internal controller state data structure.
            const EPovDirection povDirection;


        public:
            // -------- CONSTRUCTION AND DESTRUCTION ----------------------- //

            /// Initialization constructor.
            /// Specifies the POV contribution direction.
            inline constexpr PovMapper(EPovDirection povDirection) : IElementMapper(), povDirection(povDirection)
            {
                // Nothing to do here.
            }


            // -------- INSTANCE METHODS ----------------------------------- //

            /// Retrieves and returns the direction used for contributions.
            /// Intended for tests.
            /// @return Target direction for positive contributions.
            inline EPovDirection GetDirection(void) const
            {
                return povDirection;
            }


            // -------- CONCRETE INSTANCE METHODS -------------------------- //

            std::unique_ptr<IElementMapper> Clone(void) const override;
            void ContributeFromAnalogValue(SState& controllerState, int16_t analogValue) const override;
            void ContributeFromButtonValue(SState& controllerState, bool buttonPressed) const override;
            void ContributeFromTriggerValue(SState& controllerState, uint8_t triggerValue) const override;
            int GetTargetElementCount(void) const override;
            std::optional<SElementIdentifier> GetTargetElementAt(int index) const override;
        };

        /// Maps a single XInput controller element to two underlying element mappers depending on its state, either positive or negative.
        /// For analog values, "positive" means that the axis value is greater than or equal to the netural value, and "negative" means it is less than the neutral value.
        /// For button values, "positive" means the button is pressed, and "negative" means it is not pressed.
        /// For trigger values, "positive" means the trigger value is greater than or equal to the midpoint, and "negative" means it is less than the midpoint.
        /// Whichever of the two contained element mapper is inactive during any given request for contributions is given an opportunity to contribute a neutral state.
        class SplitMapper : public IElementMapper
        {
        private:
            // -------- INSTANCE VARIABLES --------------------------------- //

            /// Underlying mapper that is asked for a contribution when the associated XInput controller element is in "positive" state.
            const std::unique_ptr<const IElementMapper> positiveMapper;

            /// Underlying mapper that is asked for a contribution when the associated XInput controller element is in "negative" state.
            const std::unique_ptr<const IElementMapper> negativeMapper;


        public:
            // -------- CONSTRUCTION AND DESTRUCTION ----------------------- //

            /// Initialization constructor.
            /// Requires both a positive and a negative mapper, one or both of which can be `nullptr`.
            /// Takes ownership of the objects passed as parameters.
            inline SplitMapper(std::unique_ptr<const IElementMapper>&& positiveMapper, std::unique_ptr<const IElementMapper>&& negativeMapper) : positiveMapper(std::move(positiveMapper)), negativeMapper(std::move(negativeMapper))
            {
                // Nothing to do here.
            }

            /// Copy constructor.
            /// Simply clones whichever of the two sub-mappers are present.
            inline SplitMapper(const SplitMapper& other) : positiveMapper((other.positiveMapper != nullptr) ? other.positiveMapper->Clone() : nullptr), negativeMapper((other.negativeMapper != nullptr) ? other.negativeMapper->Clone() : nullptr)
            {
                // Nothing to do here.
            }


            // -------- INSTANCE METHODS ----------------------------------- //

            /// Retrieves and returns a raw read-only pointer to the positive element mapper.
            /// This object maintains ownership over the returned pointer.
            /// Intended for tests.
            /// @return Read-only pointer to the positive element mapper.
            inline const IElementMapper* GetPositiveMapper(void) const
            {
                return positiveMapper.get();
            }

            /// Retrieves and returns a raw read-only pointer to the negative element mapper.
            /// This object maintains ownership over the returned pointer.
            /// Intended for tests.
            /// @return Read-only pointer to the negative element mapper.
            inline const IElementMapper* GetNegativeMapper(void) const
            {
                return negativeMapper.get();
            }


            // -------- CONCRETE INSTANCE METHODS -------------------------- //

            std::unique_ptr<IElementMapper> Clone(void) const override;
            void ContributeFromAnalogValue(SState& controllerState, int16_t analogValue) const override;
            void ContributeFromButtonValue(SState& controllerState, bool buttonPressed) const override;
            void ContributeFromTriggerValue(SState& controllerState, uint8_t triggerValue) const override;
            void ContributeNeutral(SState& controllerState) const override;
            int GetTargetElementCount(void) const override;
            std::optional<SElementIdentifier> GetTargetElementAt(int index) const override;
        };
    }
}

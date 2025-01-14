/*****************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 *****************************************************************************
 * Authored by Samuel Grossman
 * Copyright (c) 2016-2022
 *************************************************************************//**
 * @file MockElementMapper.h
 *   Mock element mapper interface that can be used for tests.
 *****************************************************************************/

#pragma once

#include "ForceFeedbackEffect.h"


namespace XidiTest
{
    using namespace ::Xidi::Controller::ForceFeedback;


    /// Mock version of a force feedback effect, used for testing purposes.
    /// Simply returns the received time as the output magnitude.
    class MockEffect : public Effect
    {
    public:
        // -------- CONCRETE INSTANCE METHODS ------------------------------ //

        std::unique_ptr<Effect> Clone(void) const override
        {
            return std::make_unique<MockEffect>(*this);
        }

    protected:
        TEffectValue ComputeRawMagnitude(TEffectTimeMs rawTime) const override
        {
            return (TEffectValue)rawTime;
        }
    };

    /// Type-specific parameter structure used for mock force feedback effects with type-specific parameters.
    struct SMockTypeSpecificParameters
    {
        /// Flag that specifies whether or not the contents of an instance of this structure should be considered valid type-specific parameter values.
        /// Tests that make instances of this structure should set this flag accordingly.
        bool valid;

        /// Integer parameter with no meaning.
        int param1;

        /// Floating-point parameter with no meaning.
        float param2;

        /// Simple check for equality.
        /// Primarily useful during testing.
        /// @param [in] other Object with which to compare.
        /// @return `true` if this object is equal to the other object, `false` otherwise.
        constexpr inline bool operator==(const SMockTypeSpecificParameters& other) const = default;
    };

    /// Mock version of a force feedback effect with type-specific parameters, used for testing purposes.
    /// Simply returns the received time as the output magnitude and uses a mock type-specific parameter structure.
    class MockEffectWithTypeSpecificParameters : public EffectWithTypeSpecificParameters<SMockTypeSpecificParameters>
    {
    private:
        // -------- INSTANCE VARIABLES ------------------------------------- //

        /// Specifies if whatever error might be present in a set of invalid type-specific parameters can be automatically fixed.
        bool canFixInvalidTypeSpecificParameters = false;


    public:
        // -------- INSTANCE METHODS --------------------------------------- //

        /// Retrieves whether or not this effect's type-specific parameters have an error that can automatically be fixed somehow.
        /// This value is intended to be set by tests exercising automatic fixing of type-specific parameter errors.
        /// @return `true` if so, `false` if not.
        inline bool GetCanFixInvalidTypeSpecificParameters(void)
        {
            return canFixInvalidTypeSpecificParameters;
        }

        /// Enables or disables this effect's ability to fix an error in type-specific parameters.
        /// This value is intended to be set by tests exercising automatic fixing of type-specific parameter errors.
        /// @param [in] newCanFixInvalidTypeSpecificParameters Whether or not an error should be considered fixable.
        inline void SetCanFixInvalidTypeSpecificParameters(bool newCanFixInvalidTypeSpecificParameters)
        {
            canFixInvalidTypeSpecificParameters = newCanFixInvalidTypeSpecificParameters;
        }


        // -------- CONCRETE INSTANCE METHODS ------------------------------ //

        bool AreTypeSpecificParametersValid(const SMockTypeSpecificParameters& newTypeSpecificParameters) const override
        {
            return newTypeSpecificParameters.valid;
        }

        void CheckAndFixTypeSpecificParameters(SMockTypeSpecificParameters& newTypeSpecificParameters) const override
        {
            if (true == canFixInvalidTypeSpecificParameters)
                newTypeSpecificParameters.valid = true;
        }

        std::unique_ptr<Effect> Clone(void) const override
        {
            return std::make_unique<MockEffectWithTypeSpecificParameters>(*this);
        }

    protected:
        TEffectValue ComputeRawMagnitude(TEffectTimeMs rawTime) const override
        {
            return (TEffectValue)rawTime;
        }
    };

    /// Mock version of a periodic force feedback effect.
    /// Returned waveform amplitude is simply the same as the input phase divided by the maximum possible phase value.
    class MockPeriodicEffect : public PeriodicEffect
    {
    public:
        // -------- CONSTANTS ---------------------------------------------- //

        /// Number of hundredths of a degree per waveform cycle.
        static constexpr unsigned int kDegreeHundredthsPerCycle = 36000;


        // -------- CONCRETE INSTANCE METHODS ------------------------------ //

        TEffectValue WaveformAmplitude(TEffectValue phase) const override
        {
            return (phase / 36000);
        }

        std::unique_ptr<Effect> Clone(void) const override
        {
            return std::make_unique<MockPeriodicEffect>(*this);
        }
    };
}

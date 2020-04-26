/*****************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 *****************************************************************************
 * Authored by Samuel Grossman
 * Copyright (c) 2016-2020
 *************************************************************************//**
 * @file Mapper/XInputSharedTriggers.h
 *   Declares a mapper that maps to the default configuration of an
 *   XInput controller when accessed via DirectInput, with the exception
 *   that the LT and RT triggers share the Z axis.
 *****************************************************************************/

#pragma once

#include "Mapper.h"


namespace Xidi
{
    namespace Mapper
    {
        /// Provides a mapping to the default button layout of an XInput controller when accessed via DirectInput, with the triggers sharing an axis.
        /// Right stick is mapped to Rx and Ry axes, and triggers are mapped to share the Z axis.
        class XInputSharedTriggers : public IMapper
        {
        public:
            // -------- TYPE DEFINITIONS ----------------------------------------------- //

            /// Identifies each button modelled by this mapper.
            /// Values specify DirectInput instance number.
            enum class EButton : TInstanceIdx
            {
                ButtonA = 0,
                ButtonB = 1,
                ButtonX = 2,
                ButtonY = 3,
                ButtonLB = 4,
                ButtonRB = 5,
                ButtonBack = 6,
                ButtonStart = 7,
                ButtonLeftStick = 8,
                ButtonRightStick = 9,

                ButtonCount = 10
            };

            /// Identifies each axis modelled by this mapper.
            /// Values specify DirectInput instance number.
            enum class EAxis : TInstanceIdx
            {
                AxisX = 0,
                AxisY = 1,
                AxisZ = 2,
                AxisRX = 3,
                AxisRY = 4,

                AxisCount = 5
            };

            /// Identifies each point-of-view controller modelled by this mapper.
            /// Values specify DirectInput instance number.
            enum class EPov : TInstanceIdx
            {
                PovDpad = 0,

                PovCount = 1
            };


            // -------- CONCRETE INSTANCE METHODS -------------------------------------- //
            // See "Mapper/Base.h" for documentation.

            virtual const TInstanceIdx AxisInstanceIndex(REFGUID axisGUID, const TInstanceIdx instanceNumber);
            virtual const TInstanceCount AxisTypeCount(REFGUID axisGUID);
            virtual const GUID AxisTypeFromInstanceNumber(const TInstanceIdx instanceNumber);
            virtual const TInstance MapXInputElementToDirectInputInstance(EXInputControllerElement element);
            virtual const TInstanceCount NumInstancesOfType(const EInstanceType type);
        };
    }
}

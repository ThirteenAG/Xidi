/***************************************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 ***************************************************************************************************
 * Authored by Samuel Grossman
 * Copyright (c) 2016-2025
 ***********************************************************************************************//**
 * @file SetHooksDirectInput.cpp
 *   Implementation of all functionality for setting DirectInput hooks.
 **************************************************************************************************/

#include <Hookshot/Hookshot.h>
#include <Infra/Core/Message.h>

#include "SetHooks.h"

namespace Xidi
{
  void SetHookCoCreateInstance(Hookshot::IHookshot* hookshot)
  {
    OutputSetHookResult(L"CoCreateInstance", StaticHook_CoCreateInstance::SetHook(hookshot));
  }
} // namespace Xidi

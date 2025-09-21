/***************************************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 ***************************************************************************************************
 * Authored by Samuel Grossman
 * Copyright (c) 2016-2025
 ***********************************************************************************************//**
 * @file ApiXidi.cpp
 *   Implementation of common parts of the internal API for communication between Xidi modules.
 **************************************************************************************************/

#include "ApiXidi.h"

#include <unordered_map>

namespace Xidi
{
  namespace Api
  {
    /// Contains and allows internal access to the interface object registry.
    /// This style of implementation ensures the registry is valid early during static
    /// initialization.
    /// @return Writable reference to the registry.
    static std::unordered_map<EClass, IXidi*>& GetInterfaceObjectRegistry(void)
    {
      static std::unordered_map<EClass, IXidi*> interfaceObjectRegistry;
      return interfaceObjectRegistry;
    }

    /// Looks up and returns a pointer to the interface object corresponding to the specified class
    /// enumerator.
    /// @param [in] apiClass API class enumerator.
    /// @return Pointer to the registered implementing object, or `nullptr` if the interface is not
    /// implemented.
    static inline IXidi* LookupInterfaceObjectForClass(EClass apiClass)
    {
      std::unordered_map<EClass, IXidi*>& interfaceObjectRegistry = GetInterfaceObjectRegistry();
      if (false == interfaceObjectRegistry.contains(apiClass)) return nullptr;
      return interfaceObjectRegistry.at(apiClass);
    }

    /// Registers an interface object as the implementing object for the Xidi API of the specified
    /// class. If another object is already registered, this function does nothing.
    /// @param [in] apiClass API class enumerator.
    /// @param [in] interfaceObject Pointer to the interface object to register.
    static inline void RegisterInterfaceObject(EClass apiClass, IXidi* interfaceObject)
    {
      std::unordered_map<EClass, IXidi*>& interfaceObjectRegistry = GetInterfaceObjectRegistry();

      if (false == interfaceObjectRegistry.contains(apiClass))
        interfaceObjectRegistry[apiClass] = interfaceObject;
    }

    IXidi::IXidi(EClass apiClass)
    {
      RegisterInterfaceObject(apiClass, this);
    }
  } // namespace Api
} // namespace Xidi

extern "C" __declspec(dllexport) void* XidiApiGetInterface(Xidi::Api::EClass apiClass)
{
  return LookupInterfaceObjectForClass(apiClass);
}

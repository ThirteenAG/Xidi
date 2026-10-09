/***************************************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 ***************************************************************************************************
 * Fork extension. Kept out of the upstream source files so that upstream changes merge cleanly.
 ***********************************************************************************************//**
 * @file ConditionEffectTest.cpp
 *   Unit tests for condition force feedback effects (spring, damper, inertia, friction), which
 *   racing games written for steering wheels create on every device.
 **************************************************************************************************/

#include <cstddef>
#include <memory>

#include <Infra/Test/TestCase.h>

#include "ApiDirectInput.h"
#include "ApiGUID.h"
#include "ElementMapper.h"
#include "ForceFeedbackDevice.h"
#include "Mapper.h"
#include "MockPhysicalController.h"
#include "VirtualController.h"
#include "VirtualDirectInputDevice.h"
#include "VirtualDirectInputEffect.h"

namespace XidiTest
{
  using namespace ::Xidi;
  using namespace ::Xidi::Controller;

  /// Controller identifier used for all test cases in this file.
  static constexpr TControllerIdentifier kConditionTestControllerIdentifier = 2;

  /// Mapper with force feedback actuators on the X and Y axes.
  static const Mapper kConditionTestMapper(
      {.stickLeftX = std::make_unique<AxisMapper>(EAxis::X),
       .stickLeftY = std::make_unique<AxisMapper>(EAxis::Y),
       .buttonA = std::make_unique<ButtonMapper>(EButton::B1)},
      {.leftMotor =
           {.isPresent = true,
            .mode = ForceFeedback::EActuatorMode::SingleAxis,
            .singleAxis = {.axis = EAxis::X, .direction = EAxisDirection::Both}},
       .rightMotor = {
           .isPresent = true,
           .mode = ForceFeedback::EActuatorMode::SingleAxis,
           .singleAxis = {.axis = EAxis::Y, .direction = EAxisDirection::Both}}});

  /// Data packet used with the test data format: two axes.
  struct SConditionTestDataPacket
  {
    LONG axisX;
    LONG axisY;
  };

  static DIOBJECTDATAFORMAT conditionTestObjectFormatSpec[] = {
      {.pguid = &GUID_XAxis,
       .dwOfs = offsetof(SConditionTestDataPacket, axisX),
       .dwType = DIDFT_AXIS | DIDFT_ANYINSTANCE},
      {.pguid = &GUID_YAxis,
       .dwOfs = offsetof(SConditionTestDataPacket, axisY),
       .dwType = DIDFT_AXIS | DIDFT_ANYINSTANCE}};

  static const DIDATAFORMAT kConditionTestFormatSpec = {
      .dwSize = sizeof(DIDATAFORMAT),
      .dwObjSize = sizeof(DIOBJECTDATAFORMAT),
      .dwFlags = DIDF_ABSAXIS,
      .dwDataSize = sizeof(SConditionTestDataPacket),
      .dwNumObjs = _countof(conditionTestObjectFormatSpec),
      .rgodf = conditionTestObjectFormatSpec};

  // Creates each condition effect the way NFS Underground creates its wheel centering spring:
  // exclusively acquired device, parameters supplied at creation time, one DICONDITION per axis.
  // Creation must succeed so the game stops retrying every frame, and the effect must not drive
  // any motor because a gamepad cannot render a position-dependent force.
  TEST_CASE(VirtualDirectInputDevice_ForceFeedback_CreateConditionEffects)
  {
    for (const GUID* effectGuid : {&GUID_Spring, &GUID_Damper, &GUID_Inertia, &GUID_Friction})
    {
      constexpr SPhysicalState kPhysicalState = {.deviceStatus = EPhysicalDeviceStatus::Ok};
      MockPhysicalController physicalController(
          kConditionTestControllerIdentifier, kConditionTestMapper, &kPhysicalState, 1);
      ForceFeedback::Device* const forceFeedbackDevice =
          &physicalController.GetForceFeedbackDevice();

      VirtualDirectInputDevice<EDirectInputVersion::k8A> diController(
          std::make_unique<VirtualController>(kConditionTestControllerIdentifier));
      TEST_ASSERT(DI_OK == diController.SetDataFormat(&kConditionTestFormatSpec));
      TEST_ASSERT(
          DI_OK ==
          diController.SetCooperativeLevel(nullptr, DISCL_EXCLUSIVE | DISCL_FOREGROUND));
      TEST_ASSERT(DI_OK == diController.Acquire());

      DWORD axes[] = {
          offsetof(SConditionTestDataPacket, axisX), offsetof(SConditionTestDataPacket, axisY)};
      LONG direction[] = {0, 0};
      DICONDITION conditions[2] = {
          {.lOffset = 0,
           .lPositiveCoefficient = 5000,
           .lNegativeCoefficient = 5000,
           .dwPositiveSaturation = 10000,
           .dwNegativeSaturation = 10000,
           .lDeadBand = 100},
          {.lPositiveCoefficient = 2000, .lNegativeCoefficient = 2000}};

      const DIEFFECT effectParameters = {
          .dwSize = sizeof(DIEFFECT),
          .dwFlags = (DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS),
          .dwDuration = INFINITE,
          .dwGain = DI_FFNOMINALMAX,
          .cAxes = 2,
          .rgdwAxes = axes,
          .rglDirection = direction,
          .cbTypeSpecificParams = sizeof(conditions),
          .lpvTypeSpecificParams = conditions};

      VirtualDirectInputEffect<EDirectInputVersion::k8A>* diEffect = nullptr;
      TEST_ASSERT(
          DI_OK ==
          diController.CreateEffect(
              *effectGuid, &effectParameters, (LPDIRECTINPUTEFFECT*)&diEffect, nullptr));
      TEST_ASSERT(nullptr != diEffect);

      GUID createdEffectGuid = {};
      TEST_ASSERT(DI_OK == diEffect->GetEffectGuid(&createdEffectGuid));
      TEST_ASSERT(*effectGuid == createdEffectGuid);
      TEST_ASSERT(
          true == forceFeedbackDevice->IsEffectOnDevice(diEffect->UnderlyingEffect().Identifier()));

      // The first axis's parameters are reported back.
      DICONDITION readBack = {};
      DIEFFECT readParameters = {
          .dwSize = sizeof(DIEFFECT),
          .cbTypeSpecificParams = sizeof(readBack),
          .lpvTypeSpecificParams = &readBack};
      TEST_ASSERT(DI_OK == diEffect->GetParameters(&readParameters, DIEP_TYPESPECIFICPARAMS));
      TEST_ASSERT(5000 == readBack.lPositiveCoefficient);
      TEST_ASSERT(100 == readBack.lDeadBand);

      TEST_ASSERT(DI_OK == diEffect->Start(1, 0));
      const ForceFeedback::TOrderedMagnitudeComponents magnitude =
          forceFeedbackDevice->PlayEffects();
      for (const auto component : magnitude)
        TEST_ASSERT(0 == component);

      diEffect->Release();
    }
  }

  // Enumerating condition effects reports all four with the condition type, consistent with
  // GetEffectInfo.
  TEST_CASE(VirtualDirectInputDevice_ForceFeedback_ConditionEffectInfo)
  {
    MockPhysicalController physicalController(
        kConditionTestControllerIdentifier, kConditionTestMapper);
    VirtualDirectInputDevice<EDirectInputVersion::k8W> diController(
        std::make_unique<VirtualController>(kConditionTestControllerIdentifier));

    int conditionEffectsSeen = 0;
    TEST_ASSERT(
        DI_OK ==
        diController.EnumEffects(
            [](LPCDIEFFECTINFOW pdei, LPVOID pvRef) -> BOOL
            {
              TEST_ASSERT(DIEFT_CONDITION == DIEFT_GETTYPE(pdei->dwEffType));
              *static_cast<int*>(pvRef) += 1;
              return DIENUM_CONTINUE;
            },
            &conditionEffectsSeen,
            DIEFT_CONDITION));
    TEST_ASSERT(4 == conditionEffectsSeen);

    DIEFFECTINFOW effectInfo = {.dwSize = sizeof(effectInfo)};
    TEST_ASSERT(DI_OK == diController.GetEffectInfo(&effectInfo, GUID_Spring));
    TEST_ASSERT(DIEFT_CONDITION == DIEFT_GETTYPE(effectInfo.dwEffType));
  }
} // namespace XidiTest

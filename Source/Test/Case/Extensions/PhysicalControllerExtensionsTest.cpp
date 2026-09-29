/***************************************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 ***************************************************************************************************
 * Fork extension. Kept out of the upstream source files so that upstream changes merge cleanly.
 ***********************************************************************************************//**
 * @file PhysicalControllerExtensionsTest.cpp
 *   Unit tests for switching mappers at runtime under application control and for
 *   application-driven vibration.
 **************************************************************************************************/

#include <cstdint>
#include <memory>

#include <Infra/Test/TestCase.h>

#include "ElementMapper.h"
#include "ForceFeedbackTypes.h"
#include "Mapper.h"
#include "MockKeyboard.h"
#include "MockMouse.h"
#include "MockPhysicalController.h"
#include "PhysicalControllerTypes.h"
#include "VirtualControllerTypes.h"

#include "../../../Extensions/PhysicalController.inl"

namespace XidiTest
{
  using namespace ::Xidi::Controller;
  using ::Xidi::Controller::Extensions::ProfileSwitcher;

  /// Controller identifier used for all profile switching test cases in this file.
  static constexpr TControllerIdentifier kTestControllerIdentifier = 0;

  /// Opaque source identifier used for all profile switching test cases in this file.
  static constexpr uint32_t kTestSourceIdentifier = 0;

  /// Keyboard key used for test cases that involve keyboard side effects.
  static constexpr ::Xidi::Keyboard::TKeyIdentifier kTestKey = 55;

  /// Name of the mapper requested by the test profile callback, or `nullptr` for no preference.
  static const wchar_t* testRequestedProfileName = nullptr;

  /// Registers the test profile callback, if not already registered, and sets the name it returns.
  /// @param [in] profileName Name the test profile callback should return.
  static void RequestProfile(const wchar_t* profileName)
  {
    static const bool kCallbackRegistered = XidiRegisterProfileCallback(
        []() -> const wchar_t*
        {
          return testRequestedProfileName;
        });
    TEST_ASSERT(true == kCallbackRegistered);

    testRequestedProfileName = profileName;
  }

  /// Creates a physical controller state with the controller connected and all elements neutral.
  static SPhysicalState NeutralPhysicalState(void)
  {
    return {.deviceStatus = EPhysicalDeviceStatus::Ok};
  }

  /// Creates a physical controller state with the controller connected and one button pressed.
  static SPhysicalState PhysicalStateWithButton(EPhysicalButton button)
  {
    SPhysicalState physicalState = NeutralPhysicalState();
    physicalState[button] = true;
    return physicalState;
  }

  /// Maps a physical state using the mapper currently selected by a profile switcher.
  static SState MapWith(const ProfileSwitcher& profileSwitcher, SPhysicalState physicalState)
  {
    return profileSwitcher.GetMapper()->MapStatePhysicalToVirtual(
        physicalState, kTestSourceIdentifier);
  }

  // With no profile requested, the configured mapper is used and nothing changes.
  TEST_CASE(ProfileSwitcher_NoPreference_UsesConfiguredMapper)
  {
    RequestProfile(nullptr);

    ProfileSwitcher profileSwitcher(kTestControllerIdentifier, kTestSourceIdentifier);
    TEST_ASSERT(false == profileSwitcher.Refresh(NeutralPhysicalState()));
    TEST_ASSERT(profileSwitcher.GetMapper() == Mapper::GetConfigured(kTestControllerIdentifier));
  }

  // With nothing held, a requested mapper takes effect immediately and completely.
  TEST_CASE(ProfileSwitcher_NothingHeld_SwitchesImmediately)
  {
    const Mapper mapperA(
        L"ExtensionsTestMapperA", {.buttonA = std::make_unique<ButtonMapper>(EButton::B1)});

    ProfileSwitcher profileSwitcher(kTestControllerIdentifier, kTestSourceIdentifier);

    RequestProfile(L"ExtensionsTestMapperA");
    TEST_ASSERT(true == profileSwitcher.Refresh(NeutralPhysicalState()));
    TEST_ASSERT(profileSwitcher.GetMapper() == &mapperA);
    TEST_ASSERT(false == profileSwitcher.Refresh(NeutralPhysicalState()));
    TEST_ASSERT(profileSwitcher.GetMapper() == &mapperA);

    RequestProfile(nullptr);
    TEST_ASSERT(true == profileSwitcher.Refresh(NeutralPhysicalState()));
    TEST_ASSERT(profileSwitcher.GetMapper() == Mapper::GetConfigured(kTestControllerIdentifier));
  }

  // A button held while switching to a mapper that does not map it keeps being reported as pressed
  // until it is released, while all other buttons immediately use the new mapper. This is the case
  // of an in-game menu that is kept open by holding a button and that uses its own mapper.
  TEST_CASE(ProfileSwitcher_HeldButton_KeepsMappingUntilReleased)
  {
    const Mapper mapperA(
        L"ExtensionsTestMapperA",
        {.buttonA = std::make_unique<ButtonMapper>(EButton::B1),
         .buttonRB = std::make_unique<ButtonMapper>(EButton::B6)});
    const Mapper mapperB(
        L"ExtensionsTestMapperB", {.buttonA = std::make_unique<ButtonMapper>(EButton::B2)});

    ProfileSwitcher profileSwitcher(kTestControllerIdentifier, kTestSourceIdentifier);
    RequestProfile(L"ExtensionsTestMapperA");
    profileSwitcher.Refresh(NeutralPhysicalState());

    const SPhysicalState holdingRB = PhysicalStateWithButton(EPhysicalButton::RB);
    TEST_ASSERT(false == profileSwitcher.Refresh(holdingRB));
    TEST_ASSERT(true == MapWith(profileSwitcher, holdingRB)[EButton::B6]);

    RequestProfile(L"ExtensionsTestMapperB");
    TEST_ASSERT(true == profileSwitcher.Refresh(holdingRB));
    TEST_ASSERT(true == MapWith(profileSwitcher, holdingRB)[EButton::B6]);

    SPhysicalState holdingRBPressingA = holdingRB;
    holdingRBPressingA[EPhysicalButton::A] = true;
    TEST_ASSERT(false == profileSwitcher.Refresh(holdingRBPressingA));
    const SState virtualStateHoldingRBPressingA = MapWith(profileSwitcher, holdingRBPressingA);
    TEST_ASSERT(true == virtualStateHoldingRBPressingA[EButton::B6]);
    TEST_ASSERT(true == virtualStateHoldingRBPressingA[EButton::B2]);
    TEST_ASSERT(false == virtualStateHoldingRBPressingA[EButton::B1]);

    TEST_ASSERT(true == profileSwitcher.Refresh(NeutralPhysicalState()));
    TEST_ASSERT(profileSwitcher.GetMapper() == &mapperB);
    TEST_ASSERT(false == MapWith(profileSwitcher, holdingRB)[EButton::B6]);
  }

  // A button held while switching between two mappers that both map it identically is never
  // reported as released.
  TEST_CASE(ProfileSwitcher_HeldButton_SameMappingNeverReleased)
  {
    const Mapper mapperA(
        L"ExtensionsTestMapperA", {.buttonRB = std::make_unique<ButtonMapper>(EButton::B6)});
    const Mapper mapperB(
        L"ExtensionsTestMapperB",
        {.buttonA = std::make_unique<ButtonMapper>(EButton::B2),
         .buttonRB = std::make_unique<ButtonMapper>(EButton::B6)});

    ProfileSwitcher profileSwitcher(kTestControllerIdentifier, kTestSourceIdentifier);
    RequestProfile(L"ExtensionsTestMapperA");
    profileSwitcher.Refresh(NeutralPhysicalState());

    const SPhysicalState holdingRB = PhysicalStateWithButton(EPhysicalButton::RB);
    for (const wchar_t* profileName :
         {L"ExtensionsTestMapperB", L"ExtensionsTestMapperA", L"ExtensionsTestMapperB"})
    {
      RequestProfile(profileName);
      profileSwitcher.Refresh(holdingRB);
      TEST_ASSERT(true == MapWith(profileSwitcher, holdingRB)[EButton::B6]);
    }

    profileSwitcher.Refresh(NeutralPhysicalState());
    TEST_ASSERT(profileSwitcher.GetMapper() == &mapperB);
    TEST_ASSERT(true == MapWith(profileSwitcher, holdingRB)[EButton::B6]);
  }

  // Switching back to the original mapper while an element is held over restores the original
  // mapper completely.
  TEST_CASE(ProfileSwitcher_HeldButton_SwitchBack)
  {
    const Mapper mapperA(
        L"ExtensionsTestMapperA", {.buttonRB = std::make_unique<ButtonMapper>(EButton::B6)});
    const Mapper mapperB(
        L"ExtensionsTestMapperB", {.buttonA = std::make_unique<ButtonMapper>(EButton::B2)});

    ProfileSwitcher profileSwitcher(kTestControllerIdentifier, kTestSourceIdentifier);
    RequestProfile(L"ExtensionsTestMapperA");
    profileSwitcher.Refresh(NeutralPhysicalState());

    const SPhysicalState holdingRB = PhysicalStateWithButton(EPhysicalButton::RB);
    RequestProfile(L"ExtensionsTestMapperB");
    TEST_ASSERT(true == profileSwitcher.Refresh(holdingRB));
    TEST_ASSERT(profileSwitcher.GetMapper() != &mapperA);
    TEST_ASSERT(profileSwitcher.GetMapper() != &mapperB);

    RequestProfile(L"ExtensionsTestMapperA");
    TEST_ASSERT(true == profileSwitcher.Refresh(holdingRB));
    TEST_ASSERT(profileSwitcher.GetMapper() == &mapperA);
    TEST_ASSERT(false == profileSwitcher.Refresh(NeutralPhysicalState()));
  }

  // A button that presses a keyboard key keeps the key pressed while held across a switch. Once it
  // is released and handed over, the key is released and the button uses the new mapper.
  TEST_CASE(ProfileSwitcher_HeldButton_KeyboardKeyStaysPressed)
  {
    const Mapper mapperA(
        L"ExtensionsTestMapperA", {.buttonB = std::make_unique<KeyboardMapper>(kTestKey)});
    const Mapper mapperB(
        L"ExtensionsTestMapperB", {.buttonB = std::make_unique<ButtonMapper>(EButton::B2)});

    MockKeyboard expectedKeyboardStateUnpressed;
    MockKeyboard expectedKeyboardStatePressed;
    expectedKeyboardStatePressed.SubmitKeyPressedState(kTestKey);

    MockKeyboard actualKeyboardState;
    actualKeyboardState.BeginCapture();

    ProfileSwitcher profileSwitcher(kTestControllerIdentifier, kTestSourceIdentifier);
    RequestProfile(L"ExtensionsTestMapperA");
    profileSwitcher.Refresh(NeutralPhysicalState());

    const SPhysicalState holdingB = PhysicalStateWithButton(EPhysicalButton::B);
    MapWith(profileSwitcher, holdingB);
    TEST_ASSERT(actualKeyboardState == expectedKeyboardStatePressed);

    RequestProfile(L"ExtensionsTestMapperB");
    profileSwitcher.Refresh(holdingB);
    const SState virtualStateHoldingB = MapWith(profileSwitcher, holdingB);
    TEST_ASSERT(actualKeyboardState == expectedKeyboardStatePressed);
    TEST_ASSERT(false == virtualStateHoldingB[EButton::B2]);

    profileSwitcher.Refresh(NeutralPhysicalState());
    MapWith(profileSwitcher, NeutralPhysicalState());
    TEST_ASSERT(actualKeyboardState == expectedKeyboardStateUnpressed);
    TEST_ASSERT(profileSwitcher.GetMapper() == &mapperB);

    TEST_ASSERT(true == MapWith(profileSwitcher, holdingB)[EButton::B2]);
    TEST_ASSERT(actualKeyboardState == expectedKeyboardStateUnpressed);

    actualKeyboardState.EndCapture();
  }

  // Both axes of a deflected stick are held over together, even if only one of them is deflected
  // far enough on its own.
  TEST_CASE(ProfileSwitcher_HeldStick_BothAxesHeldOver)
  {
    const Mapper mapperA(
        L"ExtensionsTestMapperA",
        {.stickLeftX = std::make_unique<AxisMapper>(EAxis::X),
         .stickLeftY = std::make_unique<AxisMapper>(EAxis::Y)});
    const Mapper mapperB(
        L"ExtensionsTestMapperB",
        {.stickLeftX = std::make_unique<AxisMapper>(EAxis::RotX),
         .stickLeftY = std::make_unique<AxisMapper>(EAxis::RotY)});

    ProfileSwitcher profileSwitcher(kTestControllerIdentifier, kTestSourceIdentifier);
    RequestProfile(L"ExtensionsTestMapperA");
    profileSwitcher.Refresh(NeutralPhysicalState());

    SPhysicalState deflectedRight = NeutralPhysicalState();
    deflectedRight[EPhysicalStick::LeftX] = 20000;

    RequestProfile(L"ExtensionsTestMapperB");
    TEST_ASSERT(true == profileSwitcher.Refresh(deflectedRight));

    SPhysicalState deflectedRightAndUp = deflectedRight;
    deflectedRightAndUp[EPhysicalStick::LeftY] = 5000;
    TEST_ASSERT(false == profileSwitcher.Refresh(deflectedRightAndUp));

    const SState virtualStateDeflected = MapWith(profileSwitcher, deflectedRightAndUp);
    TEST_ASSERT(0 != virtualStateDeflected[EAxis::X]);
    TEST_ASSERT(0 != virtualStateDeflected[EAxis::Y]);
    TEST_ASSERT(0 == virtualStateDeflected[EAxis::RotX]);
    TEST_ASSERT(0 == virtualStateDeflected[EAxis::RotY]);

    SPhysicalState nearlyCentered = NeutralPhysicalState();
    nearlyCentered[EPhysicalStick::LeftX] = 1000;
    TEST_ASSERT(true == profileSwitcher.Refresh(nearlyCentered));
    TEST_ASSERT(profileSwitcher.GetMapper() == &mapperB);

    const SState virtualStateNearlyCentered = MapWith(profileSwitcher, nearlyCentered);
    TEST_ASSERT(0 == virtualStateNearlyCentered[EAxis::X]);
    TEST_ASSERT(0 != virtualStateNearlyCentered[EAxis::RotX]);
  }

  // Handing over an element that is not held undoes side effects of the outgoing element mapper,
  // in this case mouse movement from a slightly deflected stick.
  TEST_CASE(ProfileSwitcher_Handover_StopsMouseMovement)
  {
    const Mapper mapperA(
        L"ExtensionsTestMapperA",
        {.stickRightX = std::make_unique<MouseAxisMapper>(EMouseAxis::X)});
    const Mapper mapperB(
        L"ExtensionsTestMapperB", {.stickRightX = std::make_unique<AxisMapper>(EAxis::Z)});

    MockMouse actualMouseState;
    actualMouseState.BeginCapture();

    ProfileSwitcher profileSwitcher(kTestControllerIdentifier, kTestSourceIdentifier);
    RequestProfile(L"ExtensionsTestMapperA");
    profileSwitcher.Refresh(NeutralPhysicalState());

    SPhysicalState slightlyDeflected = NeutralPhysicalState();
    slightlyDeflected[EPhysicalStick::RightX] = 6000;
    MapWith(profileSwitcher, slightlyDeflected);

    const uint32_t kStickRightXSourceIdentifier =
        (kTestSourceIdentifier << 8) + ELEMENT_MAP_INDEX_OF(stickRightX);
    TEST_ASSERT(
        0 !=
        actualMouseState
            .GetMovementContributionFromSource(EMouseAxis::X, kStickRightXSourceIdentifier)
            .value_or(0));

    RequestProfile(L"ExtensionsTestMapperB");
    TEST_ASSERT(true == profileSwitcher.Refresh(slightlyDeflected));
    TEST_ASSERT(profileSwitcher.GetMapper() == &mapperB);
    TEST_ASSERT(
        0 ==
        actualMouseState
            .GetMovementContributionFromSource(EMouseAxis::X, kStickRightXSourceIdentifier)
            .value_or(0));

    actualMouseState.EndCapture();
  }

  // An unknown mapper name falls back to the configured mapper.
  TEST_CASE(ProfileSwitcher_UnknownProfile_UsesConfiguredMapper)
  {
    const Mapper mapperA(
        L"ExtensionsTestMapperA", {.buttonA = std::make_unique<ButtonMapper>(EButton::B1)});

    ProfileSwitcher profileSwitcher(kTestControllerIdentifier, kTestSourceIdentifier);
    RequestProfile(L"ExtensionsTestMapperA");
    profileSwitcher.Refresh(NeutralPhysicalState());
    TEST_ASSERT(profileSwitcher.GetMapper() == &mapperA);

    RequestProfile(L"ExtensionsTestMapperThatDoesNotExist");
    TEST_ASSERT(true == profileSwitcher.Refresh(NeutralPhysicalState()));
    TEST_ASSERT(profileSwitcher.GetMapper() == Mapper::GetConfigured(kTestControllerIdentifier));
  }

  // Disconnecting the controller releases everything that was held over, and the application is
  // not asked for a mapper while the controller is disconnected.
  TEST_CASE(ProfileSwitcher_Disconnected_ReleasesHeldOverElements)
  {
    const Mapper mapperA(
        L"ExtensionsTestMapperA", {.buttonRB = std::make_unique<ButtonMapper>(EButton::B6)});
    const Mapper mapperB(
        L"ExtensionsTestMapperB", {.buttonA = std::make_unique<ButtonMapper>(EButton::B2)});

    ProfileSwitcher profileSwitcher(kTestControllerIdentifier, kTestSourceIdentifier);
    RequestProfile(L"ExtensionsTestMapperA");
    profileSwitcher.Refresh(NeutralPhysicalState());

    RequestProfile(L"ExtensionsTestMapperB");
    profileSwitcher.Refresh(PhysicalStateWithButton(EPhysicalButton::RB));
    TEST_ASSERT(profileSwitcher.GetMapper() != &mapperB);

    RequestProfile(L"ExtensionsTestMapperA");
    TEST_ASSERT(
        true == profileSwitcher.Refresh({.deviceStatus = EPhysicalDeviceStatus::NotConnected}));
    TEST_ASSERT(profileSwitcher.GetMapper() == &mapperB);
  }

  // The mapper requested by the application is published for force feedback purposes.
  TEST_CASE(ProfileSwitcher_ForceFeedbackMapper_FollowsRequestedMapper)
  {
    const Mapper mapperA(
        L"ExtensionsTestMapperA", {.buttonRB = std::make_unique<ButtonMapper>(EButton::B6)});
    const Mapper mapperB(
        L"ExtensionsTestMapperB", {.buttonA = std::make_unique<ButtonMapper>(EButton::B2)});

    ProfileSwitcher profileSwitcher(kTestControllerIdentifier, kTestSourceIdentifier);
    RequestProfile(L"ExtensionsTestMapperA");
    profileSwitcher.Refresh(NeutralPhysicalState());
    TEST_ASSERT(Extensions::ForceFeedbackMapper(kTestControllerIdentifier, nullptr) == &mapperA);

    RequestProfile(L"ExtensionsTestMapperB");
    profileSwitcher.Refresh(PhysicalStateWithButton(EPhysicalButton::RB));
    TEST_ASSERT(Extensions::ForceFeedbackMapper(kTestControllerIdentifier, nullptr) == &mapperB);

    RequestProfile(nullptr);
    profileSwitcher.Refresh(NeutralPhysicalState());
  }

  // Vibration requested through the API is combined with force feedback effects, each motor running
  // at the stronger of the two, and applies to the first connected controller when requested using
  // index -1.
  TEST_CASE(RequestedVibration_CombinedWithForceFeedback)
  {
    const Mapper mapper(Mapper::SElementMap{});
    const SPhysicalState kConnectedPhysicalState = NeutralPhysicalState();
    MockPhysicalController physicalController(0, mapper, &kConnectedPhysicalState, 1);

    const ForceFeedback::SPhysicalActuatorComponents kEffectValues = {
        .leftMotor = 500, .rightMotor = 3000};
    ForceFeedback::SPhysicalActuatorComponents actualValues = kEffectValues;

    Extensions::ApplyRequestedVibration(0, actualValues);
    TEST_ASSERT(actualValues == kEffectValues);

    TEST_ASSERT(true == XidiSendVibration(-1, 1000, 2000));
    actualValues = kEffectValues;
    Extensions::ApplyRequestedVibration(0, actualValues);
    TEST_ASSERT(1000 == actualValues.leftMotor);
    TEST_ASSERT(3000 == actualValues.rightMotor);

    TEST_ASSERT(true == XidiSendVibration(0, 4000, 0));
    actualValues = kEffectValues;
    Extensions::ApplyRequestedVibration(0, actualValues);
    TEST_ASSERT(4000 == actualValues.leftMotor);
    TEST_ASSERT(3000 == actualValues.rightMotor);

    TEST_ASSERT(true == XidiSendVibration(-1, 0, 0));
    TEST_ASSERT(true == XidiSendVibration(0, 0, 0));
    actualValues = kEffectValues;
    Extensions::ApplyRequestedVibration(0, actualValues);
    TEST_ASSERT(actualValues == kEffectValues);

    TEST_ASSERT(false == XidiSendVibration(-2, 1000, 1000));
    TEST_ASSERT(false == XidiSendVibration(1000, 1000, 1000));
  }
} // namespace XidiTest

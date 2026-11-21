// OneDrop3D (F12) : service de la poulie piloté par le registre 0x24
#include "catch2/catch_test_macros.hpp"
#include "catch2/generators/catch_generators.hpp"

#include "../../../../src/modules/globals.h"
#include "../../../../src/modules/idler.h"
#include "../../../../src/modules/motion.h"
#include "../../../../src/modules/pulley.h"

#include "../../../../src/logic/pulley_service.h"
#include "../../../../src/registers.h"

#include "../stubs/main_loop_stub.h"
#include "../stubs/stub_motion.h"

#include <new>

using logic::PulleyService;

namespace {

constexpr uint8_t pulleyServiceRegister = 0x24;
constexpr int32_t stepsPer10mm = int32_t(config::pulley.stepsPerUnit * 256 + 0.5) * 10 / 256;

void Init() {
    ForceReinitAllAutomata();
    new (&logic::pulleyService) PulleyService();
    HomeIdlerAndSelector();
    REQUIRE(mi::idler.Disengaged());
}

uint16_t Request(uint8_t slot, int8_t dist) {
    return uint16_t(slot) << 8 | uint8_t(dist);
}

void Loop() {
    main_loop();
    logic::pulleyService.Step();
}

/// Runs the service until it ends, records whether the idler was engaged on `slot` while the pulley was moving
bool RunUntilIdle(int8_t engagedSlot, bool &engagedWhileMoving, bool &disengagedWhileMoving, uint32_t maxLoops) {
    engagedWhileMoving = false;
    disengagedWhileMoving = false;
    while (logic::pulleyService.Active() && --maxLoops) {
        Loop();
        if (!mm::motion.QueueEmpty(mm::Pulley)) {
            if (mi::idler.Engaged() && mi::idler.Slot() == engagedSlot) {
                engagedWhileMoving = true;
            }
            if (mi::idler.Disengaged()) {
                disengagedWhileMoving = true;
            }
        }
    }
    return maxLoops > 0;
}

void WaitIdlerReady() {
    for (uint32_t i = 0; i < idlerEngageDisengageMaxSteps && mi::idler.State() != mi::Idler::Ready; ++i) {
        Loop();
    }
}

} // namespace

TEST_CASE("pulley_service::jog", "[pulley_service]") {
    const uint8_t slot = GENERATE(0, 2, 4);
    const int8_t dist = GENERATE(10, -10);
    Init();

    const int32_t start = mm::motion.Position(mm::Pulley);
    REQUIRE(WriteRegister(pulleyServiceRegister, Request(slot, dist)));
    uint16_t reg = 0;
    REQUIRE(ReadRegister(pulleyServiceRegister, reg));
    REQUIRE(reg == PulleyService::Jog);

    // déjà occupé : refusé
    REQUIRE_FALSE(WriteRegister(pulleyServiceRegister, Request(slot, dist)));

    bool engaged, disengaged;
    REQUIRE(RunUntilIdle(slot, engaged, disengaged, 100000));
    REQUIRE(engaged); // galet serré sur l'emplacement pendant le mouvement
    REQUIRE_FALSE(disengaged);
    REQUIRE(mm::motion.Position(mm::Pulley) - start == (dist > 0 ? stepsPer10mm : -stepsPer10mm));

    // BUG-1 : galet relâché et poulie coupée à la fin
    WaitIdlerReady();
    REQUIRE(mi::idler.Disengaged());
    REQUIRE_FALSE(mm::motion.Enabled(mm::Pulley));
    REQUIRE(ReadRegister(pulleyServiceRegister, reg));
    REQUIRE(reg == PulleyService::Idle);
}

TEST_CASE("pulley_service::clean_stops_by_itself", "[pulley_service]") {
    Init();
    const int32_t start = mm::motion.Position(mm::Pulley);
    REQUIRE(WriteRegister(pulleyServiceRegister, Request(5, -10)));
    REQUIRE(logic::pulleyService.Mode() == PulleyService::Clean);

    bool engaged, disengaged;
    REQUIRE(RunUntilIdle(-1, engaged, disengaged, 200000));
    REQUIRE(disengaged); // galet libéré pendant toute la rotation
    REQUIRE_FALSE(engaged);
    // 10 mm/s pendant 60 s, puis arrêt sans ordre
    REQUIRE(start - mm::motion.Position(mm::Pulley) == stepsPer10mm * PulleyService::cleanSeconds);
    REQUIRE(mi::idler.Disengaged());
    REQUIRE_FALSE(mm::motion.Enabled(mm::Pulley));
}

TEST_CASE("pulley_service::stop", "[pulley_service]") {
    const uint8_t slot = GENERATE(1, 5);
    Init();
    const int32_t start = mm::motion.Position(mm::Pulley);
    REQUIRE(WriteRegister(pulleyServiceRegister, Request(slot, 100)));
    // laisser tourner un peu
    for (uint32_t i = 0; i < 2000 + idlerEngageDisengageMaxSteps && mm::motion.Position(mm::Pulley) - start < 500; ++i) {
        Loop();
    }
    REQUIRE(mm::motion.Position(mm::Pulley) - start >= 500);

    REQUIRE(WriteRegister(pulleyServiceRegister, 0)); // arrêt
    bool engaged, disengaged;
    REQUIRE(RunUntilIdle(slot, engaged, disengaged, 1000));
    const int32_t stoppedAt = mm::motion.Position(mm::Pulley);
    WaitIdlerReady();
    REQUIRE(mm::motion.Position(mm::Pulley) == stoppedAt);
    REQUIRE(stoppedAt - start < 100 * stepsPer10mm / 10);
    REQUIRE(mi::idler.Disengaged());
    REQUIRE_FALSE(mm::motion.Enabled(mm::Pulley));

    // un arrêt sans rien en cours est accepté et ne fait rien
    REQUIRE(WriteRegister(pulleyServiceRegister, 0));
    REQUIRE_FALSE(logic::pulleyService.Active());
}

TEST_CASE("pulley_service::refused", "[pulley_service]") {
    Init();
    // emplacement inconnu
    REQUIRE_FALSE(WriteRegister(pulleyServiceRegister, Request(6, 10)));
    // filament dans le sélecteur
    mg::globals.SetFilamentLoaded(mg::globals.ActiveSlot(), mg::FilamentLoadState::InSelector);
    REQUIRE_FALSE(WriteRegister(pulleyServiceRegister, Request(0, 10)));
    REQUIRE_FALSE(WriteRegister(pulleyServiceRegister, Request(5, 10)));
    REQUIRE_FALSE(logic::pulleyService.Active());
    REQUIRE(mi::idler.Disengaged());
}

// OneDrop3D (F12) : service de la poulie piloté par le registre 0x24
#include "catch2/catch_test_macros.hpp"
#include "catch2/generators/catch_generators.hpp"

#include "../../../../src/modules/globals.h"
#include "../../../../src/modules/idler.h"
#include "../../../../src/modules/motion.h"
#include "../../../../src/modules/pulley.h"

#include "../../../../src/logic/pulley_service.h"
#include "../../../../src/logic/tool_change.h"
#include "../../../../src/application.h"
#include "../../../../src/registers.h"

#include "../stubs/main_loop_stub.h"
#include "../stubs/stub_motion.h"

#include <new>

using logic::PulleyService;

namespace {

constexpr uint8_t pulleyServiceRegister = 0x24;
constexpr int32_t stepsPerMm_x256 = int32_t(config::pulley.stepsPerUnit * 256 + 0.5);
/// Même calcul entier que le firmware (arrondi vers le bas : +10 mm = 835 pas, -10 mm = -836 pas)
constexpr int32_t Steps(int8_t mm) {
    return (int32_t(mm) * stepsPerMm_x256) >> 8;
}
constexpr int32_t stepsPer10mm = Steps(10);

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
    REQUIRE(mm::motion.Position(mm::Pulley) - start == Steps(dist));

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
    REQUIRE(mm::motion.Position(mm::Pulley) - start == Steps(-10) * PulleyService::cleanSeconds);
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
    // le service finit quand le galet est relâché (une commande ne doit pas le trouver en marche)
    REQUIRE(RunUntilIdle(slot, engaged, disengaged, 1000 + idlerEngageDisengageMaxSteps));
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

// ---- F11 : assistance pendant l'impression, registre 0x25 ----

namespace {

constexpr uint8_t assistRegister = 0x25;
/// Même calcul entier que le firmware : pas/s pour des centièmes de mm/s
constexpr int32_t AssistRate(uint16_t v) {
    return int32_t((uint32_t(v) * uint32_t(config::pulley.stepsPerUnit * 256 / 100 + 0.5)) >> 8);
}

uint16_t ReadAssist() {
    uint16_t reg = 0;
    REQUIRE(ReadRegister(assistRegister, reg));
    return reg;
}

/// Filament jusqu'à la buse sur slot, assistance lancée à v et poulie en marche
void StartAssist(uint8_t slot, uint16_t v) {
    ForceReinitAllAutomata();
    new (&logic::pulleyService) PulleyService();
    REQUIRE(EnsureActiveSlotIndex(slot, mg::FilamentLoadState::InNozzle));
    REQUIRE(mi::idler.Disengaged());
    REQUIRE(WriteRegister(assistRegister, v));
    REQUIRE(ReadAssist() == (PulleyService::assistPresent | v));
    for (uint32_t i = 0; i < idlerEngageDisengageMaxSteps && mm::motion.QueueEmpty(mm::Pulley); ++i) {
        Loop();
    }
    REQUIRE(mi::idler.Engaged());
    REQUIRE(mi::idler.Slot() == slot);
    REQUIRE_FALSE(mm::motion.QueueEmpty(mm::Pulley));
}

/// Pas restants du mouvement de la poulie (cible - position)
int32_t PulleyStepsAhead() {
    return mm::AxisNearestTargetPos(mm::Pulley) - mm::motion.Position(mm::Pulley);
}

void RequireReleased() {
    WaitIdlerReady();
    REQUIRE(mi::idler.Disengaged());
    REQUIRE_FALSE(mm::motion.Enabled(mm::Pulley));
    REQUIRE(ReadAssist() == PulleyService::assistPresent);
}

} // namespace

TEST_CASE("pulley_service::assist_runs_and_follows_speed", "[pulley_service]") {
    const uint8_t slot = GENERATE(0, 2, 4);
    StartAssist(slot, 500); // 5 mm/s
    REQUIRE(logic::pulleyService.Mode() == PulleyService::Assist);
    // un ordre vaut assistSeconds à la vitesse demandée
    REQUIRE(PulleyStepsAhead() == AssistRate(500) * PulleyService::assistSeconds);

    // nouvelle vitesse en marche : le reste du mouvement est remplacé
    for (int i = 0; i < 100; ++i) {
        Loop();
    }
    REQUIRE(WriteRegister(assistRegister, 1200));
    REQUIRE(ReadAssist() == (PulleyService::assistPresent | 1200));
    REQUIRE(PulleyStepsAhead() == AssistRate(1200) * PulleyService::assistSeconds);
    REQUIRE(mi::idler.Engaged());

    // le galet et l'assistance ne sont pas touchés par un ordre refusé (trop rapide)
    REQUIRE_FALSE(WriteRegister(assistRegister, PulleyService::assistMax + 1));
    REQUIRE(ReadAssist() == (PulleyService::assistPresent | 1200));

    // arrêt demandé : galet relâché, moteur coupé
    REQUIRE(WriteRegister(assistRegister, 0));
    bool engaged, disengaged;
    REQUIRE(RunUntilIdle(slot, engaged, disengaged, idlerEngageDisengageMaxSteps));
    RequireReleased();
}

TEST_CASE("pulley_service::assist_stops_by_itself", "[pulley_service]") {
    StartAssist(1, 300);
    const int32_t start = mm::motion.Position(mm::Pulley);
    bool engaged, disengaged;
    REQUIRE(RunUntilIdle(1, engaged, disengaged, 100000 + idlerEngageDisengageMaxSteps));
    REQUIRE(engaged); // galet serré pendant tout le mouvement
    REQUIRE_FALSE(disengaged);
    REQUIRE(mm::motion.Position(mm::Pulley) - start == AssistRate(300) * PulleyService::assistSeconds);
    RequireReleased();
}

TEST_CASE("pulley_service::assist_stopped_by_command", "[pulley_service]") {
    StartAssist(3, 800);
    // l'imprimante envoie un changement d'outil : refusé, et l'assistance s'arrête
    application.PlanCommand(mp::RequestMsg(mp::RequestMsgCodes::Tool, 1));
    REQUIRE(application.currentCommand != &logic::toolChange);
    REQUIRE(mm::motion.QueueEmpty(mm::Pulley));
    // refusé tant que le galet n'est pas relâché ; l'imprimante renvoie la commande
    bool engaged, disengaged;
    REQUIRE(RunUntilIdle(3, engaged, disengaged, idlerEngageDisengageMaxSteps));
    REQUIRE_FALSE(engaged); // poulie arrêtée pendant tout le relâchement
    RequireReleased();
    REQUIRE(logic::pulleyService.Mode() == PulleyService::Idle);
}

TEST_CASE("pulley_service::assist_refused", "[pulley_service]") {
    ForceReinitAllAutomata();
    new (&logic::pulleyService) PulleyService();
    REQUIRE(EnsureActiveSlotIndex(2, mg::FilamentLoadState::InFSensor));
    // filament pas jusqu'à la buse
    REQUIRE_FALSE(WriteRegister(assistRegister, 500));
    mg::globals.SetFilamentLoaded(2, mg::FilamentLoadState::InNozzle);
    // trop rapide
    REQUIRE_FALSE(WriteRegister(assistRegister, PulleyService::assistMax + 1));
    // le jog du F12 est refusé pendant l'assistance
    REQUIRE(WriteRegister(assistRegister, 500));
    REQUIRE_FALSE(WriteRegister(pulleyServiceRegister, Request(5, 10)));
    bool engaged, disengaged;
    // l'arrêt sans assistance est accepté et ne fait rien ; refus = lecture sans vitesse
    REQUIRE(WriteRegister(assistRegister, 0));
    REQUIRE(RunUntilIdle(2, engaged, disengaged, idlerEngageDisengageMaxSteps));
    REQUIRE(WriteRegister(assistRegister, 0));
    REQUIRE(ReadAssist() == PulleyService::assistPresent);
    REQUIRE(mi::idler.Disengaged());

    // galet sans prise d'origine (MMU allumé avec le filament dans la buse) : accepté, la prise d'origine d'abord
    mi::idler.InvalidateHoming();
    REQUIRE(WriteRegister(assistRegister, 500));
    REQUIRE(mi::idler.State() == mi::Idler::HomeForward);
}

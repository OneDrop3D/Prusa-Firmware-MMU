// OneDrop3D : appuis rapides sur Gauche/Droite du MMU, cumulés en une seule cible du sélecteur
#include "catch2/catch_test_macros.hpp"
#include "application.h"
#include "registers.h"

#include "modules/globals.h"
#include "modules/motion.h"
#include "modules/selector.h"
#include "modules/user_input.h"

#include "logic/move_selector.h"
#include "logic/no_command.h"
#include "logic/pulley_service.h"

#include "../logic/stubs/main_loop_stub.h"
#include "../logic/stubs/stub_motion.h"
#include "../modules/stubs/stub_serial.h"

#include <new>

namespace {

void Init(uint8_t slot) {
    ForceReinitAllAutomata();
    new (&logic::moveSelector) logic::MoveSelector();
    new (&logic::pulleyService) logic::PulleyService();
    modules::serial::ClearRX();
    REQUIRE(EnsureActiveSlotIndex(slot, mg::FilamentLoadState::AtPulley));
    application.currentCommand = &logic::noCommand;
    application.lastCommandProcessedMs = mt::timebase.Millis() - 2000; // plus d'une seconde sans commande
}

void Loop() {
    main_loop();
    application.Step();
}

void Press(mui::Event e) {
    mui::userInput.ProcessMessage(e);
    Loop();
}

/// Jusqu'à la fin du déplacement manuel ; faux si le sélecteur s'est arrêté sur `forbidden` en chemin
bool RunToEnd(uint8_t forbidden = 0xff) {
    for (uint32_t i = 0; i < selectorMoveMaxSteps; ++i) {
        if (logic::moveSelector.State() == ProgressCode::OK) {
            return true;
        }
        if (ms::selector.Slot() == forbidden) {
            return false;
        }
        Loop();
    }
    return false;
}

constexpr mm::pos_t SlotPos(uint8_t slot) {
    return ms::Selector::SlotPosition(slot).v;
}

} // namespace

TEST_CASE("application::manual_selector_double_press", "[application]") {
    Init(1);
    Press(mui::Event::Right);
    REQUIRE(application.currentCommand == &logic::moveSelector);
    REQUIRE(ms::selector.PlannedSlot() == 2);
    for (int i = 0; i < 100; ++i) {
        Loop();
    }
    REQUIRE(ms::selector.State() == ms::Selector::Moving);

    // 2e appui pendant le déplacement : un seul mouvement, prolongé jusqu'à la case 3
    Press(mui::Event::Right);
    REQUIRE(ms::selector.PlannedSlot() == 3);
    REQUIRE(mm::motion.PlannedMoves(mm::Selector) == 1);
    REQUIRE(mm::AxisNearestTargetPos(mm::Selector) == SlotPos(3));
    REQUIRE(RunToEnd(2));
    REQUIRE(ms::selector.Slot() == 3);
    REQUIRE(mg::globals.ActiveSlot() == 3);

    // Dans la seconde qui suit : l'appui n'est plus perdu
    Press(mui::Event::Right);
    REQUIRE(logic::moveSelector.State() == ProgressCode::MovingSelector);
    REQUIRE(ms::selector.PlannedSlot() == 4);

    // Butée : la position de parcage, comme chez Prusa
    Press(mui::Event::Right);
    Press(mui::Event::Right);
    Press(mui::Event::Right);
    REQUIRE(ms::selector.PlannedSlot() == config::toolCount);
    REQUIRE(RunToEnd());
    REQUIRE(ms::selector.Slot() == config::toolCount);

    // Dans l'autre sens
    Press(mui::Event::Left);
    Press(mui::Event::Left);
    REQUIRE(ms::selector.PlannedSlot() == config::toolCount - 2);
    REQUIRE(RunToEnd(config::toolCount - 1));
    REQUIRE(ms::selector.Slot() == config::toolCount - 2);
}

TEST_CASE("application::manual_selector_reverse_and_bounds", "[application]") {
    Init(1);
    // Aller-retour : Droite puis Gauche revient à la case de départ (le trajet inverse s'ajoute à la suite)
    Press(mui::Event::Right);
    Press(mui::Event::Left);
    REQUIRE(ms::selector.PlannedSlot() == 1);
    REQUIRE(RunToEnd());
    REQUIRE(ms::selector.Slot() == 1);

    // Butée à 0
    Press(mui::Event::Left);
    Press(mui::Event::Left);
    Press(mui::Event::Left);
    REQUIRE(ms::selector.PlannedSlot() == 0);
    REQUIRE(RunToEnd());
    REQUIRE(ms::selector.Slot() == 0);
}

TEST_CASE("application::manual_selector_middle_ignored", "[application]") {
    Init(1);
    Press(mui::Event::Right);
    Press(mui::Event::Middle); // ignoré pendant le déplacement, comme chez Prusa
    REQUIRE(application.currentCommand == &logic::moveSelector);
    REQUIRE(ms::selector.PlannedSlot() == 2);
    REQUIRE(RunToEnd());
    Press(mui::Event::Middle); // et dans la seconde qui suit
    REQUIRE(application.currentCommand == &logic::moveSelector);
    REQUIRE(logic::moveSelector.State() == ProgressCode::OK);
    REQUIRE_FALSE(mui::userInput.AnyEvent());
}

TEST_CASE("application::selector_register_retarget", "[application]") {
    Init(1);
    // Écran « Moteurs du MMU » de l'imprimante : M708 A0x1b, deux positions de suite
    REQUIRE(WriteRegister(0x1b, 2));
    for (int i = 0; i < 100; ++i) {
        main_loop();
    }
    REQUIRE(ms::selector.State() == ms::Selector::Moving);
    REQUIRE(WriteRegister(0x1b, 4));
    REQUIRE(ms::selector.PlannedSlot() == 4);
    REQUIRE(mm::motion.PlannedMoves(mm::Selector) == 1);
    while (ms::selector.State() != ms::Selector::Ready) {
        REQUIRE(ms::selector.Slot() != 2);
        main_loop();
    }
    REQUIRE(ms::selector.Slot() == 4);

    // Pendant une commande (ici un déplacement par les boutons, depuis la case active 1 comme chez Prusa), le registre
    // garde le comportement de Prusa : refusé pendant le mouvement
    Press(mui::Event::Left);
    REQUIRE(logic::moveSelector.State() == ProgressCode::MovingSelector);
    REQUIRE(ms::selector.PlannedSlot() == 0);
    REQUIRE(WriteRegister(0x1b, 3));
    REQUIRE(ms::selector.PlannedSlot() == 0);
}

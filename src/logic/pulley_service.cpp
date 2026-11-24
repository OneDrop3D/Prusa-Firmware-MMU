/// @file pulley_service.cpp
#include "pulley_service.h"
#include "progress_codes.h"
#include "../application.h"
#include "../modules/globals.h"
#include "../modules/idler.h"
#include "../modules/motion.h"
#include "../modules/pulley.h"

namespace logic {

PulleyService pulleyService;

namespace {
    constexpr int32_t stepsPerMm_x256 = int32_t(config::pulley.stepsPerUnit * 256 + 0.5);
    /// Vitesse du jog : la vitesse lente de Prusa (20 mm/s), celle des petits mouvements de la poulie
    constexpr mm::P_speed_t jogSpeed = mm::unitToAxisUnit<mm::P_speed_t>(config::pulleySlowFeedrate);
    /// F11 : pas/s = centièmes de mm/s * assistRate_x256 / 256 (précision 0,04 %)
    constexpr uint32_t assistRate_x256 = uint32_t(config::pulley.stepsPerUnit * 256 / 100 + 0.5);
} // namespace

bool PulleyService::Write(uint16_t value) {
    const int8_t d = int8_t(value & 0xff);
    const uint8_t slot = value >> 8;

    if (d == 0) { // stop
        if (mode != Idle) {
            mm::motion.AbortPlannedMoves(mm::Pulley);
            phase = Release;
        }
        return true;
    }

    if (mode != Idle
        || application.CurrentProgressCode() != uint8_t(ProgressCode::OK)
        || mg::globals.FilamentLoaded() > mg::FilamentLoadState::AtPulley
        || slot > config::toolCount) {
        return false;
    }

    // Galet serré sur l'emplacement (jog) ou libéré (nettoyer) ; le module fait la prise d'origine si besoin
    if ((slot < config::toolCount ? mi::idler.Engage(slot) : mi::idler.Disengage()) == mi::Idler::OperationResult::Refused) {
        return false;
    }
    mode = slot < config::toolCount ? Jog : Clean;
    distance = d;
    phase = WaitIdler;
    return true;
}

bool PulleyService::WriteAssist(uint16_t value) {
    if (value == 0) {
        if (mode == Assist) {
            speed = 0;
            Write(0);
        }
        return true;
    }
    if (value > assistMax) {
        return false;
    }
    if (mode == Assist && phase != Release) { // nouvelle vitesse
        speed = value;
        if (phase == Moving) {
            RunAssist();
        }
        return true;
    }
    if (mode != Idle
        || application.CurrentProgressCode() != uint8_t(ProgressCode::OK)
        || mg::globals.FilamentLoaded() != mg::FilamentLoadState::InNozzle
        || !mi::idler.HomingValid()
        || mi::idler.Engage(mg::globals.ActiveSlot()) == mi::Idler::OperationResult::Refused) {
        return false;
    }
    mode = Assist;
    speed = value;
    phase = WaitIdler;
    return true;
}

void PulleyService::RunAssist() {
    const mm::steps_t rate = mm::steps_t((speed * assistRate_x256) >> 8);
    // La vitesse change en marche : le reste du mouvement est remplacé (halt = false garde la vitesse actuelle)
    mm::motion.AbortPlannedMoves(mm::Pulley, false);
    mpu::pulley.PlanMove(mm::P_pos_t({ int32_t(rate) * assistSeconds }), mm::P_speed_t({ rate }));
}

void PulleyService::Step() {
    if (mode == Idle) {
        return;
    }
    const uint8_t idlerState = mi::idler.State();
    // Au-delà de HomeBack : TMCFailed, HomingFailed ou OnHold, le galet ne bougera plus
    const bool idlerFailed = idlerState > mi::Idler::HomeBack;

    switch (phase) {
    case WaitIdler:
        if (idlerState == mi::Idler::Ready) {
            if (mode == Assist) {
                // Courant réduit : la poulie ne pousse qu'avec une petite force et décroche quand elle est en avance
                mm::motion.InitAxis(mm::Pulley, hal::tmc2130::MotorCurrents(assistIRun, 0));
                RunAssist();
                phase = Moving;
                break;
            }
            mpu::pulley.InitAxis();
            // Calcul entier (pas de flottants) : 1 mm = stepsPerMm_x256 / 256 pas, précision 0,02 %
            const int32_t steps = (int32_t(distance) * stepsPerMm_x256) >> 8;
            if (mode == Jog) {
                mpu::pulley.PlanMove(mm::P_pos_t({ steps }), jogSpeed);
            } else {
                mpu::pulley.PlanMove(mm::P_pos_t({ steps * cleanSeconds }), mm::P_speed_t({ mm::steps_t(steps < 0 ? -steps : steps) }));
            }
            phase = Moving;
        } else if (idlerFailed) {
            phase = Release;
        }
        break;
    case Moving:
        if (mm::motion.QueueEmpty(mm::Pulley)) {
            phase = Release;
        }
        break;
    default: // Release : on attend la fin d'un mouvement du galet en cours, on le relâche, puis la fin de ce mouvement
        // (une commande ne doit pas trouver le galet en marche : elle reste refusée jusque-là)
        if (idlerFailed || (idlerState == mi::Idler::Ready && mi::idler.Disengaged())) {
            mpu::pulley.InitAxis(); // courant de Prusa (F11 l'a baissé) ; moteur coupé
            mpu::pulley.Disable();
            mode = Idle;
        } else if (idlerState == mi::Idler::Ready) {
            mi::idler.Disengage();
        }
        break;
    }
}

} // namespace logic

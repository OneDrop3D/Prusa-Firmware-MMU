/// @file pulley_service.h
#pragma once
#include <stdint.h>

namespace logic {

/// OneDrop3D (F12) : petits mouvements de la poulie demandés par l'imprimante par le registre 0x24 (M708 A0x24 Xn),
/// en dehors des commandes de Prusa (l'imprimante ne voit aucune commande en cours).
///
/// Valeur écrite : emplacement * 256 + distance en mm (int8_t, -127 à 127).
/// - distance 0 : arrêt de ce qui tourne, galet relâché (toujours accepté) ;
/// - emplacement 0 à 4 (« jog ») : galet serré sur cet emplacement, poulie de ±N mm à la vitesse lente, puis galet
///   relâché (incident BUG-1 : un galet serré garde 530 mA, il ne doit jamais le rester) ;
/// - emplacement 5 (« nettoyer ») : galet libéré, poulie en rotation à |N| mm/s dans le sens du signe ; le mouvement
///   planifié dure cleanSeconds, elle s'arrête donc seule même si l'ordre d'arrêt ne vient jamais.
/// Refusé (réponse Rejected) pendant une commande, pendant un autre mouvement de ce service, ou si un filament est
/// dans le sélecteur ou plus loin.
///
/// Lecture : 0 = rien, 1 = jog en cours, 2 = nettoyage en cours.
class PulleyService {
public:
    enum : uint8_t {
        Idle = 0,
        Jog = 1,
        Clean = 2,
    };

    /// Durée maximale de la rotation libre (« nettoyer ») [s]
    static constexpr uint8_t cleanSeconds = 60;

    /// @returns false if the request has been refused
    bool Write(uint16_t value);

    /// Advances the state machine, called from the main loop
    void Step();

    inline uint8_t Mode() const { return mode; }
    inline bool Active() const { return mode != Idle; }

private:
    enum : uint8_t {
        WaitIdler = 0,
        Moving,
        Release,
    };

    // Zéro au démarrage (.bss) : pas de constructeur, pas de code d'initialisation
    uint8_t mode;
    uint8_t phase;
    int8_t distance;
};

/// The one and only instance of PulleyService in the FW
extern PulleyService pulleyService;

} // namespace logic

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
/// Lecture : 0 = rien, 1 = jog en cours, 2 = nettoyage en cours, 3 = assistance (F11).
///
/// OneDrop3D (F11) : assistance pendant l'impression, registre 0x25 (M708 A0x25 Xn), n = vitesse de la poulie en
/// centièmes de mm/s, envoyée par l'imprimante quelques fois par seconde (vitesse du filament × rapport > 1) :
/// - n > 0 : galet serré sur l'emplacement chargé, poulie à n/100 mm/s avec un courant réduit (assistIRun) : un peu
///   plus vite que l'extrudeur, elle pousse avec une petite force et décroche (pas perdus, sans dommage) dès qu'elle
///   est en avance. Chaque ordre vaut assistSeconds : sans nouvel ordre, elle s'arrête et le galet se relâche ;
/// - n = 0 : arrêt, courant de Prusa, galet relâché (toujours accepté).
/// Refusé si n > assistMax, pendant une commande ou un jog/nettoyage, sans filament jusqu'à la buse, ou si le galet
/// n'a pas fait sa prise d'origine (MMU redémarré pendant l'impression : elle balaierait le filament). Une commande
/// de l'imprimante arrête l'assistance et reste refusée jusqu'au galet relâché : l'imprimante la renvoie.
/// Lecture : 0x8000 | vitesse appliquée (0 hors assistance) ; le bit 15 dit à l'imprimante que le registre existe.
class PulleyService {
public:
    enum : uint8_t {
        Idle = 0,
        Jog = 1,
        Clean = 2,
        Assist = 3,
    };

    /// Durée maximale de la rotation libre (« nettoyer ») [s]
    static constexpr uint8_t cleanSeconds = 60;
    /// F11 : durée de validité d'un ordre d'assistance [s]
    static constexpr uint8_t assistSeconds = 4;
    /// F11 : vitesse maximale acceptée [0,01 mm/s] (30 mm/s)
    static constexpr uint16_t assistMax = 3000;
    /// F11 : courant de la poulie pendant l'assistance (TMC2130 iRun ; 13 = 230 mA chez Prusa, ~17 mA par cran)
    static constexpr uint8_t assistIRun = 6;
    static constexpr uint16_t assistPresent = 0x8000;

    /// @returns false if the request has been refused
    bool Write(uint16_t value);

    /// F11 : registre 0x25. @returns false if the request has been refused
    bool WriteAssist(uint16_t value);
    inline uint16_t ReadAssist() const { return assistPresent | (mode == Assist ? speed : 0); }
    /// F11 : une commande de l'imprimante arrête l'assistance
    inline void StopAssist() {
        if (mode == Assist) {
            WriteAssist(0);
        }
    }

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
    uint16_t speed; ///< F11 [0,01 mm/s]

    void RunAssist();
};

/// The one and only instance of PulleyService in the FW
extern PulleyService pulleyService;

} // namespace logic

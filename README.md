# RFID Detector — Flipper Zero FAP
### Metal Detector + Pokemon Card Finder

---

## Come funziona / How it works

Il carrier LF a **125 kHz** rimane attivo in continuazione.
Ogni oggetto metallico o foglio di alluminio nelle vicinanze
**carica l'antenna** cambiando l'impedenza: il comparatore HW
conta le transizioni del segnale; la deviazione dalla baseline
viene mostrata come barra grafica.

The **125 kHz** LF carrier stays on continuously.
Nearby metal or conductive foil **loads the antenna**,
changing its impedance: the hardware comparator counts
signal transitions; the deviation from baseline is shown
as a scrolling bar graph.

> ⚠️ **Strumento sperimentale** — Non è un rilevatore certificato.
> La risposta dipende dall'hardware, dall'ambiente e dalla distanza.
> Distanza di lavoro tipica: 0.5–3 cm.
> Experimental tool only. Working distance: ~0.5–3 cm.

---

## Controlli / Controls

| Tasto | Azione |
|-------|--------|
| **◄** | Sensibilità − (1 = minima) |
| **►** | Sensibilità + (8 = massima) |
| **OK** | Cicla feedback: Suono+Vibra → Solo Suono → Solo Vibra → Silenzioso |
| **▲** | Mostra / nascondi schermata aiuto |
| **▼** | Cambia modalità: **Metal Detector** ↔ **Card Finder** |
| **BACK** | Esci / Exit |

In modalità **Card Finder** (dopo la calibrazione):
- **OK** → ricalibrare

---

## Modalità 1 — Metal Detector

```
┌─────────────────────────────┐
│ RFID METAL DETECTOR         │
├─────────────────────────────┤
│ ║║ ║ ║║║  ║          ┌───┐ │  ← storia (22 campioni)
│ ║║ ║ ║║║  ║          │ 42│ │  ← livello attuale %
│ ║║ ║ ║║║  ║║         │   │ │
│ ║║║║║║║║  ║║║        │   │ │
├─────────────────────────────┤
│ S:4  S+V  DWN:Card  UP:Help│
└─────────────────────────────┘
```

- La barra a destra mostra il **livello corrente** (0–100 %)
- Il grafico a sinistra mostra la **storia** degli ultimi 22 campioni
- Il **suono** sale di tonalità (200→2000 Hz) con il segnale
- La **vibrazione** pulsa più velocemente con il segnale
- Il **LED rosso** si illumina proporzionalmente

**Calibrazione automatica** (12 frames × 80ms = ~1 s):
Tieni il Flipper fermo senza metallo vicino all'avvio.

---

## Modalità 2 — Pokemon Card Finder 🃏

Le carte **Holo** e **Reverse Holo** Pokémon contengono uno
strato di **foglio metallico** che interagisce con il campo RF.

Pokemon **Holo** and **Reverse Holo** cards contain a layer of
**metallic foil** that interacts with the RF field.

```
┌──────────────────────────────┐
│ * POKEMON CARD FINDER *      │
├──────────────────────────────┤
│ Segnale: 34%                 │
│ [████████████               ]│
│                              │
│       Scansione...           │
│  Muovi piano / Scan slowly   │
├──────────────────────────────┤
│ S:7  OK:Recal  S+V  DWN:Metal│
└──────────────────────────────┘
```

### Come scansionare una bustina / Scanning a booster pack

1. Entra in **Card Finder** (pulsante ▼)
2. **Attendi la calibrazione** automatica (~2 s): tieni il
   Flipper fermo, lontano dalla bustina
3. Premi la faccia anteriore del Flipper **contro la bustina**
4. **Muovi lentamente** (1–2 cm/sec) lungo la bustina
5. Quando il segnale supera la soglia per >350 ms:
   → **"CARTA TROVATA!"** + suono + vibrazione

### Suggerimenti / Tips

- Usa **sensibilità 7–8** per le carte (default automatico)
- Avvicina l'antenna (lato inferiore del Flipper) alla bustina
- Mantieni il Flipper **piatto** sulla bustina, non inclinato
- Se troppe false positività: abbassa la sensibilità o ricalibrare (OK)
- La **bustina chiusa** funziona meglio di carte sciolte
- Le carte **Full Art** e **Secret Rare** hanno più foglio = segnale più forte

---

## Build & Install

### Requisiti / Requirements

- Flipper Zero con firmware ≥ 0.85.x (ufficiale, Unleashed o RogueMaster)
- Python 3.8+
- `ufbt` (universal Flipper Build Tool)

### Compilare / Build

```bash
# Installa ufbt (una sola volta)
pip install ufbt

# Entra nella cartella
cd rfid_detector

# Compila e carica via USB (Flipper connesso)
ufbt launch

# Solo compilare (produce il .fap)
ufbt build

# Copia manuale: il .fap si trova in
#   .ufbt/build/f7-firmware/rfid_detector.fap
# Copialo su SD:  /apps/Tools/rfid_detector.fap
```

### Alternativa: compilare dal firmware sorgente

```bash
# Clona il firmware ufficiale
git clone --recursive https://github.com/flipperdevices/flipperzero-firmware
cd flipperzero-firmware

# Copia la cartella del progetto in applications_user/
cp -r /path/to/rfid_detector applications_user/

# Build FAP
./fbt fap_rfid_detector
```

---

## Calibrazione del segnale / Signal Calibration

Se la barra è **sempre al massimo** o **sempre a zero**,
modifica la costante `NORM_SCALE` nel sorgente:

```c
// Aumenta se la barra è sempre piena (segnale troppo amplificato)
// Diminuisci se la barra non si muove mai
#define NORM_SCALE  3000.0f
```

**Procedura di taratura:**
1. Avvia l'app in modalità Metal Detector, sensibilità 4
2. Avvicina un oggetto metallico a ~1 cm dall'antenna
3. La barra dovrebbe salire al 50–80%
4. Se è sempre 100%: raddoppia `NORM_SCALE`
5. Se non si muove mai: dimezzala

---

## Note tecniche / Technical Notes

- Il carrier 125 kHz consuma energia extra — esci con BACK per
  spegnere il campo e risparmiare batteria
- L'app usa direttamente l'HAL (`furi_hal_rfid_*`) senza
  interfacciarsi con il servizio LFRFID di sistema
- Il comparatore riceve transizioni dalla via di ritorno
  dell'antenna (segnale accoppiato); metalli e foil alterano
  il numero di transizioni rispetto alla baseline in aria libera
- Bus hardware usato: TIM1 (carrier), Comparatore RFID

---

## Licenza / License

MIT — libero di modificare e ridistribuire.

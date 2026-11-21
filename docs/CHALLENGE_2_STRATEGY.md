# Challenge 2: einfache autonome Strategie

Recherche vom 21.11.2026. Grundlage: offizielles Regelwerk, aktueller Simulator und unten verlinkte Primärquellen. Der Nutzer hat VL53L5CX + ESP32-C6 mit späterer serieller Verbindung zum Zumo bestätigt. Dies ist ein Architekturvorschlag, keine implementierte oder auf dem Roboter nachgewiesene Lösung. Zahlen für Regler, Abstände und Zeitbudgets sind Startwerte für Versuche.

## Empfehlung

Einen kleinen Zustandsautomaten bauen: **suchen → Ziel auswählen → hinter den Würfel fahren → ausrichten → kurz schieben → Ergebnis prüfen**. Für die Eigenposition reichen zunächst Encoder, ein um den Stillstandsoffset korrigierter Gyro und gelegentliche Korrekturen an bekannten Markierungen. Der ToF liefert lokale Würfelbeobachtungen. Die Arena ist bekannt; eine allgemeine SLAM-Lösung ist dafür zunächst unnötig.

Zuerst einen beliebig platzierten, gut erreichbaren Würfel zuverlässig ins große Quadrat bringen. Danach mehrere Würfel und belegte Ablageplätze berücksichtigen. Präzise Ablagen im kleinen Quadrat kommen hinzu, sobald die gemessenen Fehler klein genug sind. Reinforcement Learning bleibt ein mögliches späteres Experiment für eine eng begrenzte Teilaufgabe.

Diese Priorisierung ist unsere technische Einschätzung für diese Aufgabe, kein experimenteller Vergleich von RL und klassischen Verfahren auf diesem Zumo.

## 1. Was die Aufgabe vorgibt

Ein Roboter startet in der Mitte einer ungefähr 160 × 160 cm großen Arena in vorgegebener Richtung. Fünf reflektierende Würfel mit 6 cm Kantenlänge liegen in für alle Teams gleichen Positionen. Die Runde dauert höchstens 120 Sekunden. Ein Würfel vollständig im kleinen Quadrat bringt drei Punkte, mindestens halb im großen einen Punkt. Start ist Taste A; grünes Blinken beendet die Runde. Bei Punktgleichheit entscheidet die Zeit. [Regelwerk](https://makernight.spark-hpi.de/en/docs/challenge-2/)

**Korrigierte Tape-Regel, vom Nutzer bestätigt:** 12 und 30 cm sind die Innenmaße; das 2,5 cm breite Band außen zählt zur Wertungsfläche. Damit gelten effektiv **17 × 17 cm für drei Punkte** und **35 × 35 cm für einen Punkt**. Der Simulator bildet das mit `ZONE.small` und `ZONE.home` bereits richtig ab, siehe [push-pull.ts](../src/shared/push-pull.ts). Die zuvor gelesene Webseite enthielt noch die veraltete Aussage, nicht alle fünf Würfel passten hinein. Vier passen mit Reserve; fünf sind bei besonderer Anordnung geometrisch möglich (15 Punkte), aber noch kein nachgewiesenes autonomes Ablageverfahren. Diese Klarstellung ersetzt die frühere Unsicherheit in dieser Notiz.

Weitere Konsequenzen unserer Planung:

- Gleiche Aufstellung für alle Teams bedeutet nicht, dass unser Programm die Würfelkoordinaten kennt. Sie werden autonom gesucht.
- Bereits abgelegte Würfel sind Hindernisse. Ein einzelner Zielpunkt im Ursprung funktioniert für mehrere Würfel schlecht.
- Das Schieben eines Würfels und die freie Fahrt des Roboters sind verschiedene Planungsprobleme.
- Würfel direkt an Wänden oder in Ecken können ohne Zugvorrichtung für einen direkten Schub zur Mitte unerreichbar sein. Vorerst zurückstellen; eine spätere Bergungsstrategie separat entwickeln.

## 2. Vergleich der Ansätze

| Ansatz | Vorteil für uns | Hauptaufwand | Entscheidung |
|---|---|---|---|
| Nur feste Motorzeiten und Wegpunkte | Schnell für eine Demonstration | Fehler durch Schlupf und unbekannte Würfelpositionen | Nur als erster Funktionstest |
| Zustandsautomat, Geometrie, Rückmeldung | Jede Teilfunktion lässt sich einzeln messen und verbessern | Würfelerkennung, Lokalisierung und zuverlässiger Kontakt | Empfohlener Ausgangspunkt |
| Allgemeines SLAM + umfassender Manipulationsplaner | Könnte wesentlich allgemeinere Umgebungen behandeln | Zusätzliche Modelle und Zustände | Für die bekannte Arena zunächst zu viel |
| RL für die gesamte Runde | Kann komplexes Verhalten lernen | Beobachtungen, Belohnung, Training, Transfer und Fehlersuche | Vorerst zurückstellen |

Die großen Unsicherheiten des Simulators betreffen gerade Kontakt und Wahrnehmung: Reibung ist angenommen, der ToF ist kein validiertes optisches Modell. Eine in Simulation erfolgreiche gelernte Strategie müsste gegen solche Abweichungen robust werden. Das betrifft auch klassische Regler; bei diesen können wir Schwachstellen zunächst gezielter isolieren. Grundlage der Einschätzung: [Simulator-README](../README.md), [ToF-Modellgrenzen](VL53L5CX.md).

Als Hintergrund zeigt Tobin et al. die Bedeutung von Variabilität beim Transfer gelernter Wahrnehmung aus Simulation. Diese Arbeit behandelt visuelle Objektlokalisierung, nicht das Training unserer Fahrstrategie; sie ist kein Beleg, dass RL hier scheitern würde. [Tobin et al., 2017](https://arxiv.org/abs/1703.06907)

## 3. Position tracken

Der Roboter hält die Schätzung `(x, y, theta)` und eine grobe Unsicherheit. Ursprung ist die Arena-Mitte, die bekannte Startrichtung definiert Winkel null. Die Startplatzierung hat trotzdem Toleranzen.

### Kurze Bewegungen: Encoder + Gyro

Im Stillstand den Gyrooffset `bias` bestimmen. Während der Fahrt fortlaufend messen:

```text
ds = (Weg_links + Weg_rechts) / 2
dtheta = (Drehrate_z - bias) * dt
x += ds * cos(theta + dtheta/2)
y += ds * sin(theta + dtheta/2)
theta += dtheta
```

Wegdifferenzen und Winkel in passenden Einheiten verwenden. Beim Abbremsen ebenfalls weiter integrieren. Als Start einen festen Regelzyklus von etwa 10–20 ms versuchen und dessen tatsächliche Laufzeit messen. Die vorhandene [Ein-Würfel-Demo](../src/examples/push_pull.py) setzt diese Grundidee bereits um, aber ohne explizite Stillstandskalibrierung des Gyrooffsets.

Encoder messen Kettenbewegung, nicht zuverlässig Bodenbewegung. Gerade beim Schieben kann der Zumo rutschen, obwohl die Encoder Fortschritt anzeigen. Der Gyro verbessert die Winkelmessung, beseitigt aber keinen seitlichen oder translatorischen Schlupf. Deshalb gemessene Würfelbewegung und bekannte Referenzen einbeziehen; nicht nur aus Encoderweg auf erfolgreichen Transport schließen. Das ist auch eine explizite Grenze der vorhandenen Demo.

### Drift korrigieren: bekannte Referenzen

Die schwarzen Quadrate sind mögliche Landmarken. Bei einem geplanten, ungefähr senkrechten Überqueren einer bekannten Kante korrigieren wir die zur Kante normale Positionskomponente. Dazu Sensorversatz, Tapedicke, Fahrtrichtung und erwartete Kante berücksichtigen. Ein einzelner Treffer liefert weder eine vollständige Position noch automatisch einen neuen Winkel.

Mehrere Sensoren beziehungsweise wiederholte, kontrollierte Überfahrten können weitere Information liefern. Eine schwarze Messung allein ist mehrdeutig: kleines oder großes Quadrat, welche Seite, ein Ecktreffer oder ein angehobener Sensor. Deshalb Korrekturen nur in einem passenden Zustand und bei plausibler vorheriger Positionsschätzung zulassen. Abgelegte Würfel können Zugänge blockieren oder die Markierung verdecken; einen freien Referenzzugang erhalten.

ToF-Wandmessungen sind eine zusätzliche Möglichkeit: bei bekannter Orientierung und identifizierter Wand kann deren Abstand eine Positionskomponente korrigieren. Vordergrundwürfel, schwarze Oberflächen, ungültige Messungen und Unsicherheit der Wandzuordnung machen dies zu einer späteren Erweiterung. Kein Echo bedeutet nicht automatisch freie Fläche. Zunächst keine allgemeine Scan-Matching- oder Filterarchitektur einführen.

Einfache Unsicherheitsverwaltung genügt für den Anfang: Unsicherheit wächst mit Weg, Drehungen und Schieben; eine bestätigte Referenz reduziert nur die tatsächlich beobachtete Komponente. Wird die Ablagegenauigkeit zu schlecht, referenzieren oder das größere Ziel wählen. Ein EKF kann später sinnvoll sein, wenn Messdaten zeigen, dass diese einfache Lösung nicht ausreicht.

## 4. Würfel mit dem ToF finden

Der VL53L5CX hat 8×8 Zonen. Die maximale Bildrate dafür beträgt laut STs ULD-Anleitung 15 Hz; die beworbenen 60 Hz beziehen sich auf 4×4. Reichweite und Genauigkeit hängen von Messbedingungen ab. Insbesondere sind die Datenblattwerte kein Nachweis dafür, dass ein einzelner 6-cm-Würfel auf die volle nominelle Entfernung erkannt wird. [ST ULD-Anleitung](https://www.st.com/resource/en/user_manual/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-a-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf), [ST-Datenblatt](https://www.st.com/resource/en/datasheet/vl53l5cx.pdf)

Vorschlag für eine möglichst einfache erste Erkennung:

1. In kleinen Winkelschritten drehen, jeweils abbremsen und nach dem Stillstand ein neues Messbild abwarten. Für den ersten Versuch sind etwa 10° Schritte plausibel, anschließend auf Geschwindigkeit optimieren.
2. Zonen anhand Einbauhöhe, Neigung und Gültigkeitsstatus auswählen. Bodenrückgaben nicht pauschal als Objekt interpretieren. Eine gültige Einzelzone darf einen Kandidaten erzeugen; zwei benachbarte Zonen als Pflicht würden entfernte kleine Würfel übersehen.
3. Zusammenhängende beziehungsweise zeitlich konsistente Nahmessungen zu Kandidaten bündeln. Bekannte Wandentfernung und grobe Objektbreite als Plausibilitätsprüfung nutzen, nicht als sichere Klassifikation.
4. Kandidat anfahren und aus kürzerer Entfernung erneut beobachten. Erst dann seitliche Position und Schubansatz genauer bestimmen.
5. Ähnliche Beobachtungen räumlich zusammenführen. Pro Würfel grobe Position, Unsicherheit, letzten Sichtzeitpunkt und Zustand speichern.

Bei nominell ungefähr 45° horizontalem Sichtfeld ist eine Zone grob 5,6° breit. Bei 80 cm Entfernung entspricht das etwa 8 cm; ein 6-cm-Würfel ist dann kleiner als eine Zone. Diese geometrische Abschätzung erklärt, weshalb wir Fernerkennung als Kandidatensuche behandeln. Die reale Zonencharakteristik und Teilbelegung müssen am Sensor getestet werden. [Sichtfeld: ST-Datenblatt](https://www.st.com/resource/en/datasheet/vl53l5cx.pdf)

Ein Rundumblick aus der Mitte garantiert nicht, alle fünf Würfel zu sehen: Verdeckungen, Reichweite und Auflösung bleiben. Wenn gute Kandidaten fehlen, einige freie, verteilte Suchpositionen anfahren und erneut scannen. Suchpositionen müssen unbekannten Raum langsam und mit Hinderniserkennung erschließen. Abstände zwischen Suchpositionen aus der experimentell gemessenen Würfelerkennungsreichweite ableiten.

ToF misst zunächst eine sichtbare Oberfläche, nicht das Würfelzentrum. Auch die Drehung des Würfels ist anfangs unsicher. Messungen daher nicht als exakte Zentren verwenden; den geometrischen Versatz abschätzen und beim Annähern verfeinern. Koordinatenumrechnung muss STs Tiefenkonvention, Zonenorientierung und den Sensoreinbau berücksichtigen; siehe die bereits recherchierten Details in [VL53L5CX.md](VL53L5CX.md).

### Serielle Verbindung

Die Rollen einfach halten: ESP32 liefert Tiefenbilder, der Zumo führt Zustandsautomat und Regelung aus. Das vorhandene [spark_tof.py](../src/robot/python/spark_tof.py) ist nur ein Leser für einen vorgeschlagenen Vertrag, noch kein Beleg für passende ESP-Firmware.

Für den ersten Stop-and-scan-Ablauf alte UART-Daten kontrolliert abarbeiten und ein nach dem Stillstand aufgenommenes Bild verwenden. Sequenznummern und Aufnahmezeit helfen dabei; bei getrennten Geräteuhren deren Zuordnung beachten. Später beim Scannen während der Drehung die Orientierung zur Aufnahmezeit verwenden. Empfangszeit allein erzeugt durch Übertragungslatenz Winkelfehler. Die sichtbare Capture-Pose des Simulators steht echtem Roboter-Code nicht als Messung zur Verfügung.

## 5. Einen Würfel nach Hause bringen

Für Würfelposition `c` und einen freien Ablagepunkt `g` berechnen wir zunächst die Richtung:

```text
u = (g - c) / Abstand(g, c)
Anfahrpunkt = c - u * d
```

`d` enthält Würfelhalbausdehnung entlang der Schubrichtung, Abstand vom Roboterursprung zur Schaufel und eine Anfahrreserve. Es ist kein pauschaler Abstand vom Robotermittelpunkt zur Würfeloberfläche. Mit unsicherer Würfeldrehung konservativ rechnen und in der Nähe nachmessen.

Dann:

1. Prüfen, ob hinter dem Würfel genug Platz für den Roboter ist und der gesamte Schubkorridor frei ist.
2. Den Anfahrpunkt über freie Wegpunkte erreichen. Für die erste Version reichen direkte Fahrt und wenige Umfahrpunkte; A* erst ergänzen, wenn Hindernisse es erfordern.
3. Würfel erneut erfassen, mittig und möglichst flächig anfahren.
4. Langsam Kontakt herstellen und ungefähr gerade schieben. Kleine Kurskorrekturen begrenzen; bei größerem Versatz stoppen und neu ansetzen.
5. Zunächst nach etwa 5–10 cm Schub kontrollieren. Falls der Würfel im Kontakt nicht zuverlässig sichtbar ist, kontrolliert wenige Zentimeter zurückfahren und neu messen. Die nötige Distanz hängt von Montage, Schaufel und Nahbereich ab.
6. Nahe dem Ziel Geschwindigkeit und Schrittlänge reduzieren, absetzen, zurückziehen und Ablage aus einer freien Sicht überprüfen.

Schubschritte sind Regelparameter, keine belegten Optimalwerte. Größere Schritte nur nach erfolgreichen Versuchen verwenden. Ein Verlust des ToF-Signals im Nahbereich ist für sich kein sicherer Kontaktverlust. Ebenso beweist ein vorhandenes Echo keine stabile mechanische Führung.

Eine breite, passend geformte Schaufel beziehungsweise kurze seitliche Führung könnte seitliches Entkommen reduzieren. Das ist eine zu testende mechanische Verbesserung; Anbauteile dürfen den Sensor nicht verdecken und brauchen zusätzlichen Platz beim Rangieren. Stabile Mehrpunktkontakte werden in der Literatur untersucht, garantieren aber nicht, dass die aktuelle schräge Zumo-Schaufel jeden Würfel sicher führt. [Lynch und Mason](https://publications.ri.cmu.edu/stable-pushing-mechanics-controllability-and-planning), [Zhou und Mason, §4.2](https://www.cs.cmu.edu/~jiajiz/isrr17_contributed_new.pdf)

### Kleine Karte statt großer Planung

Speichern müssen wir zunächst nur Arena, Zielquadrate, Roboterpose, Würfelkandidaten und reservierte Ablagen. Für freie Fahrt Hindernisse um Roboterabmessungen und Fehlerreserve vergrößern; bei Drehungen die ausgeschwenkte Kontur berücksichtigen. Bei einem späteren Raster-A* etwa 4–5 cm Zellen ausprobieren und danach die reale Kontur entlang des Pfades prüfen. Auf MicroPython kompakte Arrays verwenden; ein Raster voller Python-Objekte kann den Heap belasten.

Für einen Schub zusätzlich die gesamte bewegte Roboter-Würfel-Kontur prüfen. Ein A*-Pfad für einen Punkt oder einen einzelnen Würfel erlaubt nicht automatisch dessen Transport. Für Version 1 nur freie, nahezu gerade Schübe zulassen. Geknickte Wege brauchen neue Kontaktseiten beziehungsweise erneutes Ansetzen und sind eine spätere Erweiterung.

## 6. Mehrere Würfel und Punkte

Ziele sind freie Ablageplätze mit erreichbarer Schubrichtung, nicht jedes Mal die Mitte. Eine bereits bestätigte Ablage anschließend als Hindernis behandeln. Reihenfolge, Zufahrt, Rückzug und Sicht zum Überprüfen gemeinsam betrachten; nicht erst alle Würfel ins Zentrum drücken und später versuchen, sie dort zu sortieren.

Eine einfache Auswahlheuristik bewertet pro erreichbarem Paar aus Würfel und Ablage:

```text
Nutzen = erwarteter zusätzlicher Punktgewinn / geschätzte benötigte Zeit
```

Unsicherer Kontakt, langer Anfahrweg, enge Zufahrt oder gefährdete vorhandene Punkte senken den erwarteten Nutzen. Das ist eine Heuristik ohne Optimalitätsgarantie. Anfangs reichen feste Regeln: nahe, gut sichtbare und frei schiebbare Würfel bevorzugen; schwierige Wandfälle zurückstellen; bestätigte Ablagen schützen.

Ein Würfel vollständig in der kleinen Wertungsfläche einschließlich Tape (17 cm) hat bei achsparalleler Ausrichtung höchstens 5,5 cm Mittelpunktspielraum je Achse. Ein gedrehter Würfel benötigt mehr Breite. Vier achsparallele 6-cm-Würfel benötigen zusammen 12 × 12 cm und passen damit mit Reserve. Die Herausforderung bleiben Zufahrt, Ablagereihenfolge und Rückzug. Die geometrisch mögliche Fünferanordnung ist deshalb eine spätere Optimierung.

Bei der Ablageentscheidung direkt einen Innenplatz verwenden, wenn Erreichbarkeit und aktuelle Genauigkeit dafür reichen. Sonst einen zugänglichen Außenplatz im großen Quadrat wählen. Bereits abgelegte Würfel später umzusetzen ist eine optionale, erneut zu bewertende Aktion und keine kostenlose Punkteverbesserung.

Für die letzten etwa 10–15 Sekunden als anfängliche Heuristik keine lange Bergung mehr beginnen. Nach bestätigtem Erfolg und ohne aussichtsreichen Zusatzgewinn grün blinken; andernfalls entscheiden Zeitbudget und Risiko. Der Bot schätzt den Ablageerfolg aus Sensorik. Die exakte Simulatorpunktzahl darf nur die Auswertung lesen.

## 7. Zustandsautomat und Fehlerbehandlung

```text
WAIT_A → SCAN → SELECT → APPROACH → ALIGN → PUSH → VERIFY
           ↑                                  ↓       |
           └──────────── RECOVER ←─────────────┘       |
           └──────────────────────────────────────────┘

Bei sinnvoll abgeschlossenem Lauf: DONE → grünes Blinken
```

Jeder Bewegungszustand bekommt eine maximale Dauer. Bei verlorener Beobachtung, unplausibler Bewegung oder blockierter Zufahrt stoppen, wenn möglich frei zurücksetzen, neu beobachten und höchstens wenige Wiederholungen versuchen. Rückwärtsfahrt braucht ebenfalls einen zuvor geprüften freien Korridor. Nach wiederholtem Fehler den Kandidaten zurückstellen.

Stillstand trotz Motorbefehl lässt sich teilweise an Encodern erkennen; durchdrehende Ketten brauchen zusätzliche Plausibilitätsprüfung mit beobachteten Objekten oder Referenzen. IMU und Encoder allein garantieren keine Erkennung jedes Festfahrens. Die Erholung sollte deutlich vor der offiziellen 10-Sekunden-Grenze ansetzen. [Grenze: Regelwerk](https://makernight.spark-hpi.de/en/docs/challenge-2/)

## 8. Was die Papers für uns hergeben

### Lynch & Mason: Stable Pushing: Mechanics, Controllability, and Planning, 1996

Gelesen: Forschungszusammenfassung der Autoreninstitution; der verlinkte Volltext war über den Recherchezugang nicht auswertbar. Mehrere Kontakte erlauben unter Bedingungen Schubrichtungen, in denen das Objekt relativ zum Schieber stabil bleibt. Für uns motiviert das eine passende Kontaktgeometrie. Es ist keine Stabilitätsgarantie für unsere Hardware. [Primärquelle](https://publications.ri.cmu.edu/stable-pushing-mechanics-controllability-and-planning)

### Zhou & Mason: Pushing revisited: Differential flatness, trajectory planning and stabilization

Gelesen: Einleitung, Modell, Regelung, insbesondere §4.2 und Versuchsaufbau im Autorenmanuskript. Die Autoren untersuchen modellgestützte Planung und mechanische Stabilisierung durch zwei Kontaktpunkte. Für uns sind Kontaktgeometrie und begrenzte Kurven beim Schieben relevant. Den vollständigen mathematischen Planer würde ich zunächst weglassen. Ihre Präzisionsergebnisse vom Manipulator lassen sich nicht auf einen Zumo mit Kettenschlupf und ToF übertragen. [Autorenmanuskript](https://www.cs.cmu.edu/~jiajiz/isrr17_contributed_new.pdf)

### Heins & Schoellig: Robust Single-Point Pushing with Force Feedback, 2023

Gelesen: Modell, Regelgesetz sowie Simulations- und Hardwareversuche. Ein vergleichsweise kompakter Rückkopplungsregler schiebt Objekte trotz unbekannter Eigenschaften entlang einer Geraden. Der Aufbau setzt einen Kraft-Momenten-Sensor und bekannte Roboterposition voraus; die Hardware nutzt eine omnidirektionale Basis. Für uns stützt das die Untersuchung einfacher Rückkopplung, liefert aber keinen unmittelbar nutzbaren ToF-Regler. [Volltext](https://arxiv.org/pdf/2305.11048)

### Ozdamar et al.: Pushing in the Dark, 2024

Gelesen: Wahrnehmung, Reactive Pushing Strategy, Zustandswechsel sowie Versuchsbedingungen und Ergebnisse. Kontaktposition und ein gesonderter Zustand zum Neuausrichten steuern das Schieben. Das passt als Architekturidee zu unserem Vorschlag. Die Arbeit verwendet jedoch eine taktile Sensorhaut und seitliche Bewegung der Basis. Der Zumo kann nicht unmittelbar seitwärts fahren. Deshalb bei ihm nötigenfalls stoppen, zurücksetzen und neu anfahren. Die veröffentlichten Erfolgsraten sind keine Prognose für unsere Runde. [Volltext](https://arxiv.org/pdf/2403.09305)

Gemeinsame Schlussfolgerung unserer Recherche: einfache Regelung und bewusste Kontaktgestaltung sind einen ersten Prototyp wert. Keines dieser Papers beweist bereits die Zuverlässigkeit unserer Kombination aus Zumo, 6-cm-Würfeln und ToF.

## 9. Konkrete Umsetzung in diesem Repository

1. **Bewegung messen:** vorhandene Demo um Stillstandskalibrierung, begrenzte Fahr-/Drehprimitive und nachvollziehbare Zustandslogs erweitern. Fahrweg und Winkelfehler auch nach wiederholten Drehungen prüfen.
2. **Nur Wahrnehmung:** Scan aus mehreren Positionen; erkannte Würfel und Unsicherheit loggen. Im Simulator geschätzte gegen wahre Position auswerten, ohne die Wahrheit an das Roboterprogramm zu geben.
3. **Ein-Würfel-Regelkreis:** zunächst freie Layouts mit unterschiedlicher Würfelposition, Orientierung und Masse. Großer Zielbereich, Ablage prüfen und Kontaktverlust behandeln.
4. **Mehrere Würfel:** Ablagen reservieren, Anfahr-/Schubkorridore und bereits gesammelte Würfel berücksichtigen.
5. **Punkte optimieren:** Innenplätze, Auswahlheuristik und engere Zeitbudgets erst auf der funktionierenden Basis.
6. **Hardware abgleichen:** Erkennung und Schieben auf der echten Oberfläche messen; danach Regler und Simulatorannahmen anpassen.

Auswerten: Punkte nach 120 s, Anteil der Läufe mit mindestens einem sicheren Transport, erfolgreiche Transporte, Positionsfehler, Kontaktverluste und Festfahren. Neben Durchschnitt auch schlechte Läufe betrachten. Eigene Layouts zur Entwicklung und unberührte Layouts zur späteren Bewertung trennen.

Verschiedene Physik-Seeds allein sind kein Test verschiedener Würfelpositionen. Der aktuelle Headless-Runner unterstützt separat `--cube-seed` oder `--map`; die Bench besitzt `cubeSeeds`. Beispiele für einen später implementierten Bot:

```bash
npx tsx tests/match.ts your_program.py --arena push-pull --tof --cube-seed 11 --seed 7 --press A@0.2 --seconds 121
npx tsx tests/match.ts your_program.py --arena push-pull --tof --cube-seed 23 --seed 7 --press A@0.2 --seconds 121
```

Diese Transportversuche wurden für die Recherche nicht ausgeführt: `your_program.py` ist ein Platzhalter für die kommende Implementierung. Als erster Schritt existiert inzwischen `src/examples/discovery.py` mit der portablen Hilfe `spark_mapping.py`: Encoder-/Gyro-Odometrie, kurze Testfahrten und eine ToF-Discovery-Map im Inspector. Noch keine map-basierte Positionskorrektur und kein autonomer Würfeltransport.

## Noch praktisch zu klären

- Tatsächliche ToF-Montage: Höhe, Winkel, Verdeckung durch Schaufel oder Würfel, zuverlässiger Nahbereich.
- Erkennungsqualität für die echten reflektierenden Würfel und schwarzen Wände; keine Vollflächen-Datenblattwerte als Würfelreichweite übernehmen.
- ESP-Firmware und serielles Nachrichtenformat mit Sequenznummer und Zeitbezug.
- Genauigkeit von Fahrt, Drehung und kurzen Schüben auf der vorgesehenen Oberfläche.
- Möglichkeiten einer mechanischen Führung beziehungsweise Wandbergung. Die Tapeauslegung ist inzwischen bestätigt: Band zählt mit.

Aktuelle Priorität nach Rücksprache: zuerst Positionsschätzung und Discovery Map beim Fahren überprüfen. SLAM bedeutet, dass die Karte auch zur Korrektur der Pose verwendet wird; diese Rückkopplung folgt erst nach der sichtbaren und messbaren Odometrie-/Mapping-Grundlage. Autonomer Würfeltransport kommt danach.

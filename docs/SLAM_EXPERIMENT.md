# Roboteransicht und einfacher SLAM-Testlauf

Stand: 21.11.2026. Die Roboteransicht steuert keine Motoren. Es gibt keine autonome Erkundung, Würfelsuche oder Routenwahl. Nur das ausdrücklich ausgewählte Beispiel `src/examples/discovery.py` fährt eine kurze, feste Folge aus `MOVES`: vorwärts, links drehen, vorwärts, rückwärts, rechts drehen, rückwärts, anhalten. Dauer und Motorgeschwindigkeiten sind dort direkt editierbar.

## Im Simulator ausprobieren

1. Rechts unter dem Tiefenbild **Testskript laden** wählen. Das aktiviert den ToF und wählt das Beispiel; eigener Code bleibt im Slot gespeichert.
2. **Run**, dann **A / Start**. Erst A gibt die Bewegungen frei. Das Beispiel beendet die Folge selbst und schaltet die Motoren ab.
3. **Discovery Map** zeigt geschätzten Fahrweg, unbekannte Zellen, freie Evidenz und Hindernishypothesen. **Aktueller Scan** zeigt die frischen Messstrahlen von oben.
4. **Messdaten exportieren** speichert `robot-scans.jsonl` für einen externen SLAM-Vergleich. Im Replay enthält der Export nur bis zu diesem Zeitpunkt empfangene Beobachtungen.

Ohne ein Programm, das `spark_mapping` verwendet und `@spark-map`-Telemetrie ausgibt, erfindet die Oberfläche keine Roboterkarte. Das Laden eines Beispiels startet noch keine Fahrt.

## Was die Live-Karte bereits macht

- Encoder plus kalibrierter Gyro schätzen die Pose relativ zur angenommenen Startpose `(0, 0, 0)`. Motorbefehle dienen nicht als Wegmessung.
- Die ToF-Messungen werden auch während der Bewegung verarbeitet. Ein kurzer Odometrieverlauf liefert durch Interpolation die geschätzte Pose zur Aufnahmezeit statt zur späteren UART-Empfangszeit.
- Für Mapping während der Fahrt müssen ESP- und Zumo-Uhren dieselbe Zeitbasis haben. In der Simulation ist das gegeben. Auf Hardware muss `ESP_CLOCK_OFFSET_US` im Beispiel passend bestimmt werden. Bilder außerhalb des verfügbaren Odometrieverlaufs werden verworfen.
- Die Kartenhilfe kennt keine echten Roboterposen, Würfelkoordinaten oder Arenageometrie. Das 240-cm-Fenster ist nur ein lokaler Zeichenbereich; alle Zellen starten unbekannt.
- Neue Messungen passen die Belegung an. Ein verschobener Würfel wird an seiner alten Stelle erst entfernt, wenn gültige neue Strahlen dort freie Fläche beobachten. Fehlende Rückgaben löschen keine Hindernisse. Nicht wieder beobachtete Bereiche bleiben gespeichert.

Das ist **Odometrie plus Belegungskarte**, noch kein vollständiges SLAM und keine Zuordnung individueller Würfelidentitäten. Die Live-Ansicht nutzt derzeit nicht Spline-SLAM. Scan Matching beziehungsweise Kartenkorrekturen würden ausdrücklich in dieser Ebene ergänzt, ohne die Bewegungsfolge zu ändern.

## Spline-SLAM separat ausprobieren

[C2SR/spline-slam](https://github.com/C2SR/spline-slam) ist Python-Code mit NumPy/SciPy. Der Test lädt einen externen Checkout; keine Upstream-Dateien werden ins Projekt kopiert oder gepatcht. Geprüfte Revision: `a1c77379d4f16fec695887b7972b37b517c3cb35`.

```bash
# Einmalig in einem frischen Checkout vorbereiten:
git clone https://github.com/C2SR/spline-slam.git .context/spline-slam
git -C .context/spline-slam checkout a1c77379d4f16fec695887b7972b37b517c3cb35
uv venv .context/slam-venv
uv pip install --python .context/slam-venv/bin/python numpy scipy

# Reproduzierbaren Test mit dem gleichen MicroPython-Skript aufzeichnen:
npx tsx scripts/record-discovery.ts .context/robot-scans.jsonl --seed 7 --truth .context/evaluation-only.json

# Oder robot-scans.jsonl aus dem Browser verwenden, dann --truth weglassen:
.context/slam-venv/bin/python scripts/evaluate-spline-slam.py .context/robot-scans.jsonl \
  --source .context/spline-slam --output .context/spline-result.json \
  --truth .context/evaluation-only.json
```

Der Sensor-Export enthält je Bild nur Aufnahmezeit, geschätzte Aufnahmepose und gemessene Strahlen. Der Adapter rechnet diese wieder in den lokalen Roboterrahmen und übergibt belegte/freie Punkte an die vorhandenen Spline-SLAM-Klassen. Er erzeugt keine zusätzlichen Laserstrahlen und macht ungültige Rückgaben nicht zu freier Fläche.

Die separate Datei `evaluation-only.json` enthält die tatsächliche Endpose für die nachgelagerte Auswertung. Sie ist **kein Eingang des Schätzalgorithmus**; der Vergleich liest sie erst nach der vollständigen Schätzung. `--truth` ist optional. Bei einem abgebrochenen Lauf wird kein irreführender Endfehler berechnet.

Standard sind wie im Upstream-Beispiel drei Auflösungsstufen mit 5 cm feinster Knotenweite. Für die kleine Szene verwenden wir ein lokales 8 × 8 m großes Spline-Fenster. `--levels 1` erlaubt einen einfacheren Vergleich. Das Ergebnis-JSON enthält Konfiguration, Source-Revision, Verlauf und entweder `status: completed` oder `status: failed` mit Fehlerstelle. Ein fertiger Lauf allein wäre noch kein Nachweis einer besseren Positionsschätzung.

## Erste tatsächliche Ergebnisse

Getestet mit Python 3.14.9, NumPy 2.5.5, SciPy 1.18.1, der Beispielaufstellung und unverändertem Upstream-Kern. Die Seeds verändern hier Sensorrauschen/Physik, nicht die Würfelaufstellung.

| Seed | Exportierte ToF-Bilder | Spline-Konfiguration | Ergebnis |
|---|---:|---|---|
| 7 | 24 | 3 Stufen | Abbruch beim 6. Bild, Aufnahmezeit 2,067 s: Optimierer fragt Spline-Indizes außerhalb des Kartenarrays ab |
| 13 | 23 | 3 Stufen | Abbruch beim 5. Bild, Aufnahmezeit 1,800 s: ebenfalls Array-Index außerhalb des Kartenbereichs |
| 7 | 24 | 1 Stufe | Abbruch beim 8. Bild, Aufnahmezeit 2,667 s: ebenfalls Index außerhalb des Kartenbereichs |

Das spricht gegen eine unveränderte Übernahme **in dieser getesteten Konfiguration**. Aus dem Abbruch lässt sich noch nicht ableiten, dass jede Anpassung von Spline-SLAM ungeeignet wäre. Der Upstream-Optimierer läuft ohne Begrenzung der Pose; unser schmaler, dünn besetzter ToF-Ausschnitt liefert wesentlich weniger Geometrie als die 180-Strahlen-Laserscans seines Beispiels. Eine schlecht bestimmte Ausgleichsrechnung ist deshalb eine plausible Erklärung, aber noch keine isoliert nachgewiesene Ursache.

Für die aktuelle Oberfläche bleibt die einfache, sensorbasierte Belegungskarte aktiv. Weitere SLAM-Versuche können mit denselben Exporten bewertet werden; fehlgeschlagene Schätzungen übernehmen weder Fahrsteuerung noch Live-Pose.

## Verifikation

`npm run test:discovery` prüft die pure Python-Schätzung, verzögerte Aufnahmen, das Entfernen erneut beobachteter alter Hindernisse, fragmentierte Telemetrie, zeitlich korrekten Replay/Export und das vollständige Sample in der echten WASM-MicroPython-Laufzeit. Der Test kontrolliert, dass vor A weder gefahren noch kartiert wird, dass die feste Folge beendet wird und dass die Karte während der Fahrt neue Messungen erhält. Die echte Pose wird ausschließlich zur Testauswertung verglichen.

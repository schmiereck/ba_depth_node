# PLAN.md — ba_depth_node

Perception-Seite der Arm-Control-Pipeline. Läuft auf dem **WSL2-Laptop**, nicht
auf dem Pi. Subscribt später die Overview-Kamera aus `ba_camera_bridge` und
liefert eine Tiefenkarte + Pixel→3D-Backprojection für den MoveIt-Goal-Generator.

Dieses Dokument beschreibt **Phase 1**: Depth Anything V2 (Small) lokal
installieren und mit einem Standbild testen. Noch keine ROS2-Integration.

---

## Scope Phase 1

**In Scope**
- venv auf dem WSL2-Laptop
- PyTorch (CUDA-Variante, falls GPU verfügbar; sonst CPU-Fallback)
- Depth Anything V2 Small über HuggingFace `transformers` laden
- Smoke-Test: Ein JPG rein, Depth-PNG raus, visuell prüfen
- Latenz und VRAM notieren (Grundlage für die Ziel-FPS der späteren ROS-Node)

**Nicht in Scope (spätere Phasen)**
- ROS2-Node, `rclpy`, `cv_bridge`, Image-Subscription
- Rectification (cv2.initUndistortRectifyMap) aus `camera_info`
- Pixel→3D-Backprojection mit den Intrinsics aus `ba_camera_bridge`
- VLM-Call / Zielpixel-Logik
- Hand-Eye-Transform

---

## Kontext

- **Host**: WSL2 `Ubuntu-22.04` auf dem Windows-Dev-Laptop
- **Compute**: **CPU only** — der Laptop hat keine dedizierte NVIDIA-GPU.
  Das ist bewusst so akzeptiert: DA V2 Small läuft auf CPU, nur langsamer
  (siehe Performance-Erwartung unten).
- **Zielmodell**: `depth-anything/Depth-Anything-V2-Small-hf` (HuggingFace)
  - ~25M Parameter, deutlich kleiner als Base/Large, reicht für 640×480 Frames
  - Output: relative inverse depth (nicht metrisch — das ist bewusst so;
    die Skalierung löst später der Perception-Node über die `camera_info`-
    Intrinsics + grobe Annahmen zur Tisch-/Regalhöhe)
- **Python**: 3.10 (WSL22 default) ist ausreichend

### Performance-Erwartung (CPU)

Grobe Richtwerte für DA V2 Small @ 640×480 auf einer modernen Laptop-CPU:

| Metrik | Erwartung |
|---|---|
| Latenz / Frame | 0.5–2 s |
| Durchsatz | 1–2 FPS |
| RAM-Peak | < 2 GB |

**Konsequenz für die spätere Architektur**: Der Perception-Node arbeitet
nicht kontinuierlich, sondern **trigger-basiert** ("Schnappschuss-Modus"):
auf Anforderung ein Frame ziehen, Depth berechnen, Ziel extrahieren, an
MoveIt weitergeben. 1 s Latenz vor einer Armbewegung ist unkritisch. Für
visuelles Servoing im Nahbereich (Gripper-Cam, Phase 4+) reicht DA V2 auf
CPU nicht — da muss entweder ein einfacheres Verfahren her (Marker-Tracking,
Pixel-Feedback) oder das Modell wandert auf eine Maschine mit GPU.

---

## Prerequisites

### 1. Python + venv-Paket

```bash
python3 --version      # ≥ 3.10
python3 -m venv --help # muss existieren
sudo apt install -y python3-venv python3-pip  # falls nicht vorhanden
```

### 2. System-Libs für OpenCV (nur falls `opencv-python` meckert)

Normalerweise nicht nötig, weil die `opencv-python`-Wheels statisch gelinkt
sind. Falls beim Import `libGL.so.1` fehlt:

```bash
sudo apt install -y libgl1
```

---

## venv-Setup

Der venv lebt **außerhalb** des Repos, unter `~/venvs/ba_depth_node/`. Grund:
Wenn `ba_depth_node` später ein ROS2-`ament_python`-Paket wird, baut `colcon`
den Code eh in einen eigenen Workspace — ein venv im Repo würde sich mit
`colcon build` beißen und müsste ohnehin in `.gitignore`. Ein zentraler venv
im Home ist sauberer und bleibt auch erhalten, wenn der Repo-Ordner mal neu
geklont wird.

```bash
mkdir -p ~/venvs
python3 -m venv ~/venvs/ba_depth_node
source ~/venvs/ba_depth_node/bin/activate
pip install --upgrade pip wheel
```

Nach jedem neuen Terminal: `source ~/venvs/ba_depth_node/bin/activate`.

> **Hinweis für später**: Der Wechsel von "pures venv" zu "ROS2-Node" ist
> nicht trivial. `rclpy` erwartet das System-Python von `/opt/ros/humble`,
> nicht den venv. Lösungsrichtung dann: entweder DA V2 System-weit via
> `pip install --user` installieren, oder den venv mit
> `python3 -m venv --system-site-packages` neu anlegen, damit er `rclpy`
> vom System sieht. Das entscheiden wir, wenn Phase 2 (ROS-Node) dran ist —
> für Phase 1 ist ein reiner venv einfacher.

---

## PyTorch installieren (CPU)

Da der Laptop keine GPU hat, installieren wir die CPU-Wheels direkt. Das
spart ~2 GB Download (keine CUDA-Runtime im Wheel) und ist die offizielle
PyTorch-CPU-Variante:

```bash
# im aktivierten venv
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

**Verifizieren**:

```bash
python -c "import torch; print(torch.__version__, 'cuda:', torch.cuda.is_available())"
```

Erwartet: `2.x.x+cpu cuda: False` — genau so soll es hier sein.

> Falls dieses Repo irgendwann auf einem GPU-Rechner (Workstation, Jetson)
> installiert wird: dann stattdessen `--index-url .../cu124` (oder passende
> CUDA-Version laut `nvidia-smi`). Für den Laptop ist das aber nicht
> relevant.

---

## Depth Anything V2 installieren

Zwei Wege existieren:

1. **HuggingFace `transformers`-Pipeline** — eine Zeile Code, lädt das
   Modell beim ersten Aufruf automatisch. **Für Phase 1 der richtige Weg.**
2. Upstream-Repo `LiheYoung/Depth-Anything-V2` klonen + Checkpoint manuell
   aus GitHub-Releases ziehen. Nur nötig, wenn wir später Training oder
   exotische Inferenz-Optionen brauchen — für einen Smoke-Test Overkill.

Wir nehmen Weg 1:

```bash
pip install transformers pillow
# opencv nur falls wir Frames aus dem ROS-JPG-Stream lesen/schreiben wollen
pip install opencv-python
```

`transformers` zieht `safetensors`, `huggingface_hub`, `tokenizers` etc. als
Sub-Dependencies. Der erste `pipeline(...)`-Aufruf lädt das Modell nach
`~/.cache/huggingface/hub/` (~100 MB für Small).

---

## Smoke-Test-Skript

Wegwerf-Skript im venv-Besitzer-Home, **nicht** im Repo committen:

```bash
mkdir -p ~/sandbox/ba_depth_node_test
cd ~/sandbox/ba_depth_node_test
```

`test_da_v2.py`:

```python
#!/usr/bin/env python3
"""
Phase-1-Smoke-Test für Depth Anything V2 Small (CPU only).
Liest ein JPG, läuft durch das Modell, speichert Depth als PNG.
"""
import sys
import time

import numpy as np
import torch
from PIL import Image
from transformers import pipeline

MODEL_ID = "depth-anything/Depth-Anything-V2-Small-hf"


def main(in_path: str, out_path: str) -> None:
    print(f"[init] torch={torch.__version__}  threads={torch.get_num_threads()}")

    t0 = time.perf_counter()
    pipe = pipeline(task="depth-estimation", model=MODEL_ID, device="cpu")
    print(f"[init] model loaded in {time.perf_counter()-t0:.2f}s")

    img = Image.open(in_path).convert("RGB")
    print(f"[input] {in_path} size={img.size}")

    # Warm-up: erster Lauf enthält Lazy-Init und ist nicht repräsentativ.
    _ = pipe(img)

    # Auf CPU reichen 3 Durchläufe — mehr kostet nur Wartezeit.
    t0 = time.perf_counter()
    N = 3
    for _ in range(N):
        result = pipe(img)
    dt = (time.perf_counter() - t0) / N
    print(f"[perf] avg {dt*1000:.0f} ms / frame  ({1/dt:.2f} FPS)")

    depth = result["depth"]  # PIL Image, 8-bit grayscale, relative inverse depth
    depth.save(out_path)
    print(f"[output] depth saved to {out_path}")

    arr = np.array(depth)
    print(f"[stats] min={arr.min()} max={arr.max()} mean={arr.mean():.1f}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: test_da_v2.py <input.jpg> <output_depth.png>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
```

### Testbild besorgen

Einfachster Weg: ein einzelnes Frame aus dem laufenden ROS-Stream rausholen.
Von einer frischen WSL-Shell aus (ROS-Env manuell, weil `bash -c` `.bashrc`
nicht sourced):

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/cyclonedds.xml

# Einmalig ein JPEG-Frame aus dem compressed Topic schnappen.
# image_transport liefert die Bytes im data-Feld direkt als JPEG — einfach
# rausschreiben.
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

rclpy.init()
node = Node("grab_one")
got = []

def cb(msg):
    with open("/tmp/frame.jpg", "wb") as f:
        f.write(bytes(msg.data))
    got.append(True)
    node.destroy_subscription(sub)

sub = node.create_subscription(
    CompressedImage, "/ba_overview_camera/image_raw/compressed", cb, 1)
while not got:
    rclpy.spin_once(node, timeout_sec=1.0)
print("saved /tmp/frame.jpg")
rclpy.shutdown()
PY
```

Dann in einem zweiten Shell-Fenster (mit venv aktiviert):

```bash
source ~/venvs/ba_depth_node/bin/activate
cd ~/sandbox/ba_depth_node_test
python test_da_v2.py /tmp/frame.jpg /tmp/frame_depth.png
```

### Visuelle Prüfung

`/tmp/frame_depth.png` im Windows-Explorer öffnen
(`\\wsl.localhost\Ubuntu-22.04\tmp\frame_depth.png`) oder via
`explorer.exe .` aus `/tmp`. Plausibel ist: näher = heller, weiter = dunkler
(oder umgekehrt, je nach Normalisierung). Hauptsache, die Kontur der
Regalebenen / Objekte ist erkennbar und nicht nur Rauschen.

---

## Verifikations-Checkliste

- [ ] `python -c "import torch; print(torch.__version__)"` gibt `...+cpu` zurück
- [ ] Modell lädt ohne Fehler (ca. 100 MB Download beim ersten Mal)
- [ ] Smoke-Test läuft durch, `depth_min != depth_max`
- [ ] Latenz pro Frame notiert (Erwartung auf CPU: 0.5–2 s bei 640×480)
- [ ] RAM-Peak während Inferenz notiert (Erwartung < 2 GB)
- [ ] Depth-PNG visuell plausibel

---

## Offene Fragen / Entscheidungen für Phase 2

Diese Punkte werden **nicht** in Phase 1 gelöst, aber beim Smoke-Test
schon mitdenken — die Messwerte fließen hier ein:

1. **venv vs. System-Python für den ROS-Node.** Sobald `rclpy` importiert
   werden muss, wird es unbequem. Kandidat-Lösungen in der Reihenfolge
   zunehmenden Aufwands:
   - venv mit `--system-site-packages` → sieht `rclpy` vom System
   - `pip install --user torch transformers ...` ins System-Python und
     venv weglassen
   - Docker-Container (nur wenn die beiden oberen Optionen scheitern)
2. **Rectification.** Die C930e hat merkliche Verzeichnung. Für DA V2 an
   sich nicht kritisch (Modell ist auf Web-Bildern trainiert), **aber** die
   spätere Pixel→3D-Backprojection braucht entweder rectified Input **oder**
   muss die Verzeichnung bei der Rückprojektion selbst rausrechnen. Sauberer:
   einmal rektifizieren, damit der ganze Rest nur noch mit Lochkamera-Modell
   arbeitet. Die LUT kommt aus `camera_info` über
   `cv2.initUndistortRectifyMap`.
3. **Metrische Tiefe.** DA V2 Small liefert relative inverse depth. Für
   Grasp-Planung brauchen wir eine metrische Schätzung. Zwei Wege:
   - Bekannte Ebene (Tischhöhe, Regalhöhe) + ArUco-Marker als Anker →
     Skalierung per Least-Squares
   - Separate kleine Stereo-/ToF-Referenz. Kein Thema für Phase 1.
4. **Topic-Namen für den Depth-Output.** Vorschlag analog zu
   `ba_camera_bridge`: `/ba_overview_camera/depth/image_raw` (float32) +
   `/ba_overview_camera/depth/image_raw/compressedDepth` (PNG-16-bit für
   WLAN-Transport, falls wir das Bild weiterreichen wollen).

---

## Status

- [x] Phase 1.1: venv `~/venvs/ba_depth_node/` angelegt (Python 3.10.12)
- [x] Phase 1.2: PyTorch CPU installiert — `torch 2.11.0+cpu`,
      `torchvision 0.26.0+cpu`, 6 Threads, `cuda: False`
- [x] Phase 1.3: `transformers 5.5.3` + `depth-anything/Depth-Anything-V2-Small-hf`
      heruntergeladen (~100 MB in HF-Cache)
- [x] Phase 1.4: Smoke-Test mit Standbild bestanden
      (`~/sandbox/ba_depth_node_test/test_da_v2.py`)
- [x] Phase 1.5: Echter 640×480-Kamera-Frame aus
      `/ba_overview_camera/image_raw/compressed` durchgejagt

**Phase 1 abgeschlossen.** Entscheidung über Phase-2-Struktur
(ROS-Node-Layout, venv-Strategie, Rectification-Ort) steht als nächstes an.

### Gemessene Performance (Phase 1)

CPU-Inferenz, 6 Threads, DA V2 Small, Warm-up + N=3 Durchläufe.

| Metrik | Gorilla 7.3 MP (3174×2304) | Kamera 640×480 |
|---|---|---|
| Modell-Load | 14.3 s (Kaltstart, inkl. ~100 MB Download) | 2.03 s (HF-Cache) |
| Latenz pro Frame | 1067 ms | **861 ms** |
| Durchsatz | 0.94 FPS | **1.16 FPS** |
| Output | uint8 PNG, Range 0–255 voll, mean 108 | uint8 PNG, Range 0–255 voll, mean 75.7 |
| Visuelle Prüfung | Fäuste hell (nah), Wand dunkel (fern), saubere Kanten | Kamera-Frame plausibel |

**Wichtige Erkenntnis**: Die Inferenzzeit ist bildgrößen-**fast-unabhängig**.
Meine Vorab-Schätzung "~300–500 ms für 640×480" war falsch — der Löwenanteil
steckt im Modell-Forward auf der internen festen Eingabegröße (~518 px
Kantenlänge), nicht im Pre-/Postprocessing. Realistische Baseline für
Phase 2: **~0.9 s pro Frame auf dieser CPU**, unabhängig von der
Kameraauflösung.

Das bestätigt die Trigger-Architektur: 1 FPS reicht nicht für Live-Depth,
aber für "Schnappschuss → Ziel berechnen → MoveIt-Goal" ist es unproblematisch.

# Projektplan: ROS2 USB-Kamera Integration (6DOF Robot Arm)

## 1. Zielsetzung
Anbindung einer USB-Kamera ("Overview") und optional einer zweiten Kamera ("Gripper") an den Raspberry Pi, um Bilddaten als ROS2-Topics für die Verarbeitung auf dem Laptop (VLM, Depth Anything V2, MoveIt) bereitzustellen.

---

## 2. Hardware-Konfiguration
* **Host:** Raspberry Pi (Ubuntu + ROS2 Humble/Iron/Jazzy).
* **Kamera:** Standard USB-Webcam (UVC-kompatibel).
* **Verbindung:** WLAN (5GHz empfohlen) oder Ethernet zum Laptop.
* **Einschränkung:** USB-Bus-Bandbreite beachten (beide Kameras an USB 3.0 Ports des Pi hängen).

---

## 3. Software-Stack & Treiber
Für maximale Kompatibilität und Performance wird der **v4l2_camera** Node verwendet.

* **Paket:** `v4l2_camera`
* **Grund:** Bessere Unterstützung für moderne ROS2-Konventionen als der alte `usb_cam` Node.
* **Installation (RPi):**
    ```bash
    sudo apt update
    sudo apt install ros-${ROS_DISTRO}-v4l2-camera ros-${ROS_DISTRO}-image-transport-plugins
    ```

---

## 4. Netzwerk-Optimierung (Kritisch)
Unkomprimierte HD-Bilder überlasten das WLAN sofort. Die Übertragung muss komprimiert erfolgen.

* **Strategie:** Der Pi publiziert das Raw-Image, das Plugin `image_transport` erzeugt automatisch ein `/compressed` Topic.
* **Transport-Typ:** JPEG (für geringste Latenz bei Video).
* **Laptop-Abonnement:** Der Laptop dekomprimiert das Bild erst bei Ankunft für Depth Anything V2.

---

## 5. Implementierungs-Schritte

### Phase 1: Kamera-Bringup (RPi)
Starten des Nodes mit reduziertem Footprint (z.B. 640x480 oder 1280x720 bei 15 FPS):
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="/dev/video0" \
  -p image_size:=[640,480] \
  -p frame_id:="camera_link_overview"

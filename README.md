# MDK Controller
Motion-Controller zur Ansteuerung von 3 Schrittmotoren für Foto-, Zeitraffer- und Videoanwendungen.

## Downloads

* [Update-Tool](https://github.com/milindur/MdkControllerUpdate/releases/download/v1.2/MdkControllerUpdate.exe)
* [Treiber](https://github.com/milindur/MdkControllerUpdate/releases/download/v1.2/drivers.zip)

## Firmware

1. Die Steuerung des Controllers erfolgt per MoCoBus-kompatiblen Protokoll. Damit werden neben der MDK Control App auch andere MoCoBus-kompatible Apps verwendet werden können. Hier denke ich insb. auch an qDDB mit der Motion-Control-Unterstützung, welche momentan entwickelt wird.
2. Es gibt ein paar Erweiterungen gegenüber MoCoBus für die Astronachführung und die Panoramafunktion. Diese können nur mit der MDK Control App benutzt werden.
3. Die Astronachführung funktioniert derzeit nur mit dem MDKv5, da die Geschwindigkeit genau auf die Getriebe-Untersetzung des MDKv5 abgestimmt ist. Um die notwendige extrem niedrige Geschwindigkeit erreichen zu können, ist die große Untersetzung von ca. 190:1 des MDKv5 erforderlich.
4. Momentan ist der Zeitraffer SMS Modus und ein Videomodus umgesetzt (und natürlich Panorama und Astro). Es fehlen allerdings noch Rampen (Lead-in/out; Ramp-in/out). Diese kommen jedoch kurzfristig mit dem nächsten Firmware-Update.
5. Es werden derzeit 2 Key-Frames (also Start und Ende) unterstützt. Da dies aber für manche Aufnahmen nicht ausreicht - wie ich selbst auch auf La Palma festgestellt habe -, ist geplant die Anzahl der Key-Frames auf min. 3 (besser noch mehr) zu erhöhen (sowohl für SMS als auch Video).

## Update der Firmware (Windows)

![Screenshot des Update-Tools](https://raw.githubusercontent.com/milindur/MdkControllerUpdate/master/images/mdk-controller-update.png)

1. Das Update erfolgt per USB, dazu muss der Controller mit dem rechten USB-Port des Controllers ("PROG") am PC angeschlossen werden. Eine weitere Spannungsversorgung per DC-Buchse ist dafür nicht notwendig.
2. Falls der Controller von Windows nicht automatisch erkannt wird, werden die entsprechenden Treiber benötigt.
   Diese können [hier](https://github.com/milindur/MdkControllerUpdate/releases/download/v1.1/drivers.zip) heruntergeladen werden.
3. Das eigentliche Update erfolgt mit Hilfe des Update-Tools. Die aktuellste Version kann [hier](https://github.com/milindur/MdkControllerUpdate/releases) heruntergeladen werden (MdkControllerUpdate.exe).
4. Das Update-Tool benötigt eine aktive Internetverbindung, um die Firmware herunterladen zu können.
5. Nach Auswahl des Controllers sowie des Firmware Releases, kann das eigentliche Update mit einem Klick auf "Update" durchgeführt werden.

## Hardware

1. Als Basisboard wird zwar ein Arduino verwendet, es kommt jedoch der Arduino Due zum Einsatz. Es handelt sich dabei um einen ARM Cortex-M3 32-Bit Mikrocontroller von Atmel, der deutlich mehr Performance als ein Arduino Mega 2560 hat.
2. Bedienung des Controllers per App via Bluetooth v4.0
3. Ansteuerung von 3 Schrittmotoren. Theoretisch sind auch 4 Schrittmotoren möglich, in das Gehäuse passen derzeit jedoch nur 3 Buchsen für die Motoren. In der Regel sollten 3 Achsen für Slider, Pan und Tilt reichen.
4. Die Schrittmotoren werden derzeit immer im 16-Mikroschritt-Betrieb angesteuert. Dies erlaubt einen sehr ruhigen und gleichmäßigen Lauf. Durch die gute Performance des Mikrocontrollers können trotzdem alle Motoren unabhängig voneinander mit verhältnismäßig hohen Geschwindigkeiten angesteuert werden.
5. Die Schrittmotoren werden standardmäßig im sogenannten Stealth-Mode betrieben, der bei niedrigen bis mittleren Geschwindigkeiten nahezu lautlos ist. Hier hört man dann nur noch die Mechanik und die Getriebe, aber nicht mehr das typische Surren/Pfeifen der Schrittmotoren.
6. Außerdem kommt ein intelligenter Stromsparmodus zum Einsatz. Bei Stillstand der Motoren wird der Motorstrom automatisch auf ca. 33% des Nennstroms abgesenkt. Dies reicht noch aus, um ein ausreichendes Haltemoment des Motors bei Stillstand zu erzeugen (insb. bei vertikalem Aufbau der Slider-Achse relevant), spart aber gegenüber einer dauerhaften Bestromung des Motors mit 100% deutlich Strom. Dies ist besonders bei mehrstündigen Zeitraffer-Aufnahmen mit langem Intervall etwa in der Nacht hilfreich.
7. Anschluss einer Kamera über galvanisch getrennte 2,5mm Klinkenbuchse.
8. Zwei weitere 2,5mm Klinkenbuchsen können für 4 Digital-Ein-/Ausgänge verwendet werden (derzeit noch nicht in der Firmware umgesetzt). Hier ist insb. ein Slave-Modus geplant.
9. Versorgung des Controllers mit 12V über eine DC-Buchse. Die Schrittmotortreiber können jeweils max. einen Dauerstrom von 1,2A, derzeit ist der Slider-Motor auf 0,8A und die Pan/Tilt-Motoren auf 0,72A begrenzt.

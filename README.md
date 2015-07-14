# MDK Controller
Motion-Controller zur Ansteuerung von 3 Schrittmotoren für Foto-, Zeitraffer- und Videoanwendungen.

Details zur Hardware:

1. Als Basisboard wird zwar ein Arduino verwendet, es kommt jedoch der Arduino Due zum Einsatz. Es handelt sich dabei um einen ARM Cortex-M3 32-Bit Mikrocontroller von Atmel, der deutlich mehr Performance als ein Arduino Mega 2560 hat.
2. Bedienung des Controllers per App via Bluetooth v4.0
3. Ansteuerung von 3 Schrittmotoren. Theoretisch sind auch 4 Schrittmotoren möglich, in das Gehäuse passen derzeit jedoch nur 3 Buchsen für die Motoren. In der Regel sollten 3 Achsen für Slider, Pan und Tilt reichen.
4. Die Schrittmotoren werden derzeit immer im 16-Mikroschritt-Betrieb angesteuert. Dies erlaubt einen sehr ruhigen und gleichmäßigen Lauf. Durch die gute Performance des Mikrocontrollers können trotzdem alle Motoren unabhängig voneinander mit verhältnismäßig hohen Geschwindigkeiten angesteuert werden.
5. Die Schrittmotoren werden standardmäßig im sogenannten Stealth-Mode betrieben, der bei niedrigen bis mittleren Geschwindigkeiten nahezu lautlos ist. Hier hört man dann nur noch die Mechanik und die Getriebe, aber nicht mehr das typische Surren/Pfeifen der Schrittmotoren.
6. Außerdem kommt ein intelligenter Stromsparmodus zum Einsatz. Bei Stillstand der Motoren wird der Motorstrom automatisch auf ca. 33% des Nennstroms abgesenkt. Dies reicht noch aus, um ein ausreichendes Haltemoment des Motors bei Stillstand zu erzeugen (insb. bei vertikalem Aufbau der Slider-Achse relevant), spart aber gegenüber einer dauerhaften Bestromung des Motors mit 100% deutlich Strom. Dies ist besonders bei mehrstündigen Zeitraffer-Aufnahmen mit langem Intervall etwa in der Nacht hilfreich.
7. Anschluss einer Kamera über galvanisch getrennte 2,5mm Klinkenbuchse.
8. Zwei weitere 2,5mm Klinkenbuchsen können für 4 Digital-Ein-/Ausgänge verwendet werden (derzeit noch nicht in der Firmware umgesetzt). Hier ist insb. ein Slave-Modus geplant.
9. Versorgung des Controllers mit 12V über eine DC-Buchse. Die Schrittmotortreiber können jeweils max. einen Dauerstrom von 1,2A, derzeit ist der Slider-Motor auf 0,8A und die Pan/Tilt-Motoren auf 0,72A begrenzt.

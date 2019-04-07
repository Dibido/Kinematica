# Kinematica
Kinematica opdracht WoR

## Uitvoeren
Om het programma uit te voeren moeten de webcam en de AL5D robotarm aangesloten zijn.
Het programma moet gebuild zijn met catkin_make
De highlevel driver moet gestart zijn om de commando's te kunnen verwerken
Nadat de highlevel driver is gestart kan de kinematics binary gestart worden (rosrun highlevel)
Dit wordt gedaan door rosrun kinematics 1 uit te voeren

Nadat kinematics gestart is moeten de kleuren worden gecalibreerd, dit wordt gedaan door de HSV sliders zo te zetten dat de kleur als een wit vlak in de mask te zien is.
Als de kleur gecalibreerd is kan er door op escape te drukken naar de volgende kleur gegaan worden
Wanneer het calibreren klaar is wordt er een rode rechthoek gedetecteerd die wordt gebruikt om het aantal pixels per centimeter te detecteren
Hierna wordt de basis van de robotarm gedetecteerd en de basis waar het blokje terecht moet komen.
Na deze waardes gedetecteerd te hebben wordt er gevraagd naar de omschrijving van het blokje dat gepakt moet worden
Deze omschijving is in de vorm [vorm][breedte][hoogte][kleur]
Na dit ingegeven te hebben worden er een aantal schermen getoond waarin te zien is of er een object gedetecteerd wordt
Als het object gedetecteerd is kan er op escape gedrukt worden en wordt het blokje opgepakt en bij de basis neergelegd

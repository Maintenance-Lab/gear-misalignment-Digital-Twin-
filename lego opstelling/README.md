# files
Het ontwerp van de logo opstelling is te vinden in de .io file deze kan worden geopend in Lego’s ontwerpprogramma en wordt gebruikt om onderdelen te bestellen.
Instructies voor het bestellen en bouwen zijn te vinden in de pdf’s. 
Een script voor het besturen van de motor en servo.
# To Do:
-	De motoren die bij de opstelling zitten zijn niet compatible met de BaseX. Voor het gebruik van de BaseX zal een EV3 motor moeten worden gebruikt in combinatie met een M5 stack servo voor het bewegen van het platform. 
-	Het eerste plan was de opstelling volledig te runnen van een BaseX en Core2 maar deze kunnen de ai voor de DT niet aan dus er wordt gebruik gemaakt van een pi4. Deze kan de zelfde code gebruiken als grote DT maar heeft een uitbreiding nodig om de motors te besturen. Een mogelijke oplossing is de motors door de BaseX te laten besturen en te communiceren met de BaseX via serial (de base x wacht op serial commandos om de motors aan te sturen en print meetwaardes voor de rotatie van de motors in serial). De andere mogelijke oplossing is over te gaan naar een pi schild dat Lego motoren kan aansturen dit zou makelijker te intergreren zijn met de bestaande code. 
-	De opstelling staat op het moment niet erg stabiel en dit komt door de plaatjes onder te pilaren. Deze zouden beter vervangen kunnen worden door een solide base plate. Ook zouden alle pilaren eigenlijk beter aan elkaar kunnen worden verbonden.

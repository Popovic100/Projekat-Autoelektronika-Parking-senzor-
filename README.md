# Projekat iz autoelektronike
-Parking senzor

## Tekst zadatka i periferije koje su korišćene
Napraviti minimum 4 taska: Task za prijem podataka od senzora, task za slanje i prijem podataka od PC-ja, task za obradu podataka i task za prikaz na displeju.
Dodati još taskova po potrebi. Sinhronizaciju između tajmera i taskova, kao i između taskova, realizovati pomoću semafora. Podatke između taskova slati pomoću redova (queue).
Taskovi koji služe za primanje i slanje podataka treba da budu što jednostavniji, samo da pokupe podatak i proslede ga taskovima koji obrađuju podatke.

## Senzori za parking
-Trenutnu vrednost levog i desnog ultrazvučnog senzora udaljenosti simulirati preko UniCom, simulatora serijske komunikacije na kanalu 0 i na kanalu 1.
-Trenutne vrednosti sa oba senzora se u program učitavaju svakih 200 ms (iskoristiti automatski odgovor pomoću Trigera). Simulirati senzore kao da šalju vrednost u milimetrima.

## Komunikacija sa PC-jem (UniCom kanal 2)
-Unos komandi i prikaz poruka simulirati pomoću UniCom kanala 2.
-Komande se završavaju karakterom carriage return (CR, broj 13 decimalno).
-Na svakih 5000 ms prikazati na kanalu 2 informacijame o udaljenostima u milimetrima, kao i trenutnoj zoni detekcije. Ove informacije slati odvojeno za oba senzora (kao dve poruke, po jedna za svaki senzor).

## Podržane komande na kanalu 2
-Kalibracija senzora
	-Komanda KALIBRACIJA<brojSenzora>_<udaljenostUMilimetrima>mm_<procenat>% (npr. KALIBRACIJA1_200mm_0% i KALIBRACIJA2_2000mm_100%).
	-Za svaki senzor poslati udaljenosti za 0% i za 100%.
-Uključivanje i isključivanje sistema:
	-Komande simuliraju automatski menjač: PARK/REVERSE/NEUTRAL/DRIVE
	-Parking senzori se uključuju samo pri vožnju unazad, odnosno, nakon prijema komande REVERSE, u ostalim slučajevima senzori se isključuju.

## LED bar
-Koristi se LED bar:
	-Jedan stub kao ulaz
	-Drugi stub kao izlaz
	-Treći stub kao izlaz
-Uključivanjem donje LED na ulaznom stubu → parking senzori se uključuju
-Ako su parking senzori uključeni, uključiti donju LED izlaznog stuba
-Treći LED stub služi za vizuelni signal udaljenosti gde su relevantni podaci sa senzora koji je izmerio manju udaljenost.  Vrednost koju je izmerio senzor se konvertuje u procente na osnovu kalibracije za taj senzor.
	-Ako je izmerena udaljenost u procentima veća od 100%, to je zona NEMA_DETEKCIJE i sve LED diode trećeg stuba su isključene.
	-Ako je izmerena udaljenost u procentima manja od 20%, to je zona KONTAKT_DETEKCIJA i sve LED diode trećeg stuba su uključene.
	-Svaka dioda predstavlja 10% intervala. Npr. Ako je udaljenost između 100% i 90%, uključena je jedna LED dioda, ako je između 90% i 80%, uključene su dve diode…

## Prikaz na LCD displeju (Seg7Mux)
-Displej se osvežava osvežava na svakih 250 ms.
-Prikazivati:
	-Udaljenosti oba senzora u format senzor1–senzor2

## Testiranje
-U Command Prompt je potrebno napisati cd<adresa foldera gde se nalaze periferija za serijsku komunikaciju>. Pošto su nam potrebna 3 kanala serijske komunikacije upisujemo AdvUniCom 0, AdvUniCom 1 i AdvUniCom 2.
-U Command Prompt je potrebno napisati cd<adresa foldera gde se nalaze periferija za 7Seg displej>. Potrebno nam je 9 cifara i zato pišemo Seg7_Mux.exe 9.
-U Command Prompt je potrebno napisati cd<adresa foldera gde se nalaze periferija za LEDbars>. Pošto nam je potreban jedan ulazni i dva izlazna stupca pišemo LED_bars_plus.exe bGR.

-Kanal 0 i kanal 1
Potrebno je u polje gde piše T1 za kanal 0 upisati S0, a za kanal 1 S1. Zatim u polje R1 na oba kanala upisujemo podatke u formatu: \05\00\0d (primer), pri tome mora biti čekirano polje auto.

-Kanal 2
Za početak je potrebno da izvršimo kalibraciju sistema. U polje pored dugmeta send code upisujemo pojedinačno sledeće komande u formatu:
	KALIBRACIJA1_200mm_0%\0d
	KALIBRACIJA2_200mm_0%\0d
	KALIBRACIJA1_1000mm_100%\0d
	KALIBRACIJA2_1000mm_100%\0d
Dok šaljemo komande za kalibraciju na kanalu 2 će se ispisivati poruka NEPOZNATA KOMANDA, sve dok ne uključimo sistem.
Nakon kalibracije potrebno je uključiti sistem i to možemo uraditi na 2 načina:
	-Slanjem komande REVERSE\0d
	-Pritiskom na prvu led diodu prvog stuba
Kada je sistem uključen na kanalu 2 ispisuju se poruke da je sistem uključen, vrednosti sa kanala 0 i 1 u milimetrima i zone detekcije.
Kada želimo da isključimo sistem saljemo komande:
	-PARK\0d
	-NEUTRAL\0d
	-DRIVE\0d
NAPOMENA: Ako je sistem uključen preko prekidača mora se isključiti ponovnim klikom na prvu led diodu, sistem neće reagovati na prethodne komande.

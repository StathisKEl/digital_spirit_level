/*
 x axis=roll(.)
 y axis=pitch(_)
 z axis=yaw(|)

 1)Μετατρέπουμε τις μετρήσεις από το επιταχυνσιόμετρο σε g και τις μετρήσεις από το γυροσκόπιο σε °/s.
 2)Εφαρμόζουμε ένα average filter στις μετρήσεις από το γυροσκόπιο ώστε το επίπεδο αναφοράς να αντιστοιχεί στις μηδέν μοίρες.
 3)Υπολογίζουμε την γωνία μεταξύ της ψηφιακής αεροστάθμης και του επιπέδου αναφοράς μετά από την περιστροφή της πρώτης ως προς τον άξονα περιστροφής X
 και την αντίστοιχη γωνία ως προς τον άξονα περιστροφής Y σύμφωνα και στις δυο περιπτώσεις με τις επεξεργασμένες μετρήσεις από το γυροσκόπιο.
 Οι τιμές των γωνιών αυτών είναι μια καλή προσέγγιση των πραγματικών αλλά καθώς περνάει ο χρόνος οι τιμές αυτές αρχίζουν και αποκλίνουν. 
 4)Εφαρμόζουμε ένα average filter στις μετρήσεις από το επιταχυνσιόμετρο ώστε το επίπεδο αναφοράς να αντιστοιχεί στις μηδέν μοίρες.
 5)Εφαρμόζουμε ένα median filter στις επεξεργασμένες μετρήσεις από το επιταχυνσιόμετρο ώστε να μειώσουμε τον θόρυβο καθώς αυτό είναι ευαίσθητο στις δονήσεις
 και σύμφωνα με τις τιμές που προκύπτουν από το παραπάνω φίλτρο υπολογίζουμε την γωνία που σχηματίζεται μεταξύ της ψηφιακής αεροστάθμης και του επιπέδου αναφοράς
 όταν η πρώτη περιστρέφεται ως προς τον άξονα περιστροφής X και την αντίστοιχη γωνία ως προς τον άξονα περιστροφής Y.
 Οι τιμές των γωνιών αυτών δεν αποκλίνουν καθώς περνάει ο χρόνος.
 6)Συνδυάζοντας τις τιμές των γωνιών που υπολογίσαμε σύμφωνα με τις επεξεργασμένες μετρήσεις από το γυροσκόπιο
 (οι τιμές που προκύπτουν από την εφαρμογή του average filter στις μετρήσεις από το γυροσκόπιο) και με τις τιμές που προκύπτουν από την εφαρμογή του median filter
 στις επεξεργασμένες μετρήσεις από το επιταχυνσιόμετρο (οι τιμές που προκύπτουν από την εφαρμογή του average filter στις μετρήσεις από το επιταχυνσιόμετρο)
 υπολογίζουμε την τελική γωνία που σχηματίζεται μεταξύ της ψηφιακής αεροστάθμης και του επιπέδου αναφοράς όταν η πρώτη περιστρέφεται ως προς τον άξονα 
 περιστροφής X και την αντίστοιχη γωνία ως προς τον άξονα περιστροφής Y. 

 Το gyro_roll αποκλίνει δηλαδή δεν επιστρέφει στο μηδέν μετά από περιστροφή της ψηφιακής αεροστάθμης και δεν έχει θόρυβο. Το accel_roll δεν αποκλίνει και έχει λίγο 
 θόρυβο. Το angle_roll δεν αποκλίνει και δεν έχει θόρυβο. Τα ίδια ισχύουν και για το gyro_pitch, accel_pitch και angle_pitch.
 
 Ο αριθμός 57.29577951 χρησιμοποιείται για την μετατροπή των rad σε deg.
 Ο αριθμός 1000000 χρησιμοποιείται για την μετατροπή των μs σε s.
*/

#include <Wire.h>//Για το I2C πρωτόκολλο.
#include <LiquidCrystal.h>// Για την οθόνη LCD

LiquidCrystal lcd(1,2,4,5,6,7);

unsigned long previoustime,spaceoftime=0;

int i,j=0,k;

long accelX,accelY,accelZ;
float gForceX,gForceY,gForceZ;

float agForceX,agForceY,agForceZ;
float mgForceX[11],mgForceY[11],mgForceZ[11];
float nmgForceX[11],nmgForceY[11],nmgForceZ[11];

long gyroX,gyroY,gyroZ;
float rotX,rotY;

float arotX,arotY;
float nrotX,nrotY;

float accel_roll,accel_pitch;
float angle_roll,angle_pitch;

boolean timeflag=true;

void setup(){
  float sgForceX=0,sgForceY=0,sgForceZ=0;
  float srotX=0,srotY=0;
  
  Wire.begin();
  lcd.begin(16,2);
  lcd.clear();
  
  setupMPU();
  
  //Υπολογίζουμε την μέση τιμή 2000 μετρήσεων από το γυροσκόπιο που αντιστοιχούν στον άξονα περιστροφής X και στον άξονα περιστροφής Y και την μέση τιμή 2000 μετρήσεων
  //από το επιταχυνσιόμετρο που αντιστοιχούν στον άξονα περιστροφής X, στον άξονα περιστροφής Y και στον άξονα περιστροφής Z. 
  for(i=1;i<=2000;i++){
    recordGyroRegisters();
    srotX=srotX+rotX;
    srotY=srotY+rotY;
    recordAccelRegisters();
    //Αποθηκεύουμε τις τελευταίες 11 μετρήσεις από το επιταχυνσιόμετρο που αντιστοιχούν στον άξονα περιστροφής X, στον άξονα περιστροφής Y και στον άξονα περιστροφής Z
    //σε τρεις διαφορετικούς πίνακες. 
    if(i>=1990){
      mgForceX[j]=gForceX;
      mgForceY[j]=gForceY;
      mgForceZ[j]=gForceZ;
      j=j+1;
    }
    sgForceX=sgForceX+gForceX;
    sgForceY=sgForceY+gForceY;
    sgForceZ=sgForceZ+gForceZ;
  }
  arotX=srotX/2000;
  arotY=srotY/2000;
  agForceX=sgForceX/2000;
  agForceY=sgForceY/2000;
  agForceZ=sgForceZ/2000;

  //Μετατοπίζουμε τις τιμές των τελευταίων 11 μετρήσεων από το επιταχυνσιόμετρο που αντιστοιχούν στον άξονα περιστροφής X, στον άξονα περιστροφής Y
  //και στον άξονα περιστροφής Z ώστε αυτές που αντιστοιχούν στους δυο πρώτους άξονες περιστροφής να κυμαίνονται γύρω από το μηδέν και αυτές που αντιστοιχούν στον
  //τρίτο άξονα περιστροφής γύρω από το ένα και αποθηκεύουμε τις καινούργιες τιμές στους ίδιους πίνακες που ήταν αποθηκευμένες και οι παλιές.
  for(j=0;j<=10;j++){
    mgForceX[j]=mgForceX[j]-agForceX;
    mgForceY[j]=mgForceY[j]-agForceY;
    mgForceZ[j]=(mgForceZ[j]-agForceZ)+1;
    //Αντιγράφουμε τα στοιχεία του κάθε πίνακα με τις μετατοπισμένες τιμές των τελευταίων 11 μετρήσεων από το επιταχυνσιόμετρο που αντιστοιχούν στον άξονα
    //περιστροφής X, στον άξονα περιστροφής Y και στον άξονα περιστροφής Z σε τρεις καινούργιους πίνακες.
    nmgForceX[j]=mgForceX[j];
    nmgForceY[j]=mgForceY[j];
    nmgForceZ[j]=mgForceZ[j];
  }

  //Ως αρχική τιμή της γωνίας του complimentary filter που αντιστοιχεί στον άξονα περιστροφής X θέτουμε την μετατοπισμένη τιμή της τελευταίας μέτρησης
  //από το επιταχυνσιόμετρο που αντιστοιχεί στον ίδιο άξονα περιστροφής και ως αρχική τιμή της γωνίας του complimentary filter που αντιστοιχεί στον άξονα περιστροφής Y
  //θέτουμε την μετατοπισμένη τιμή της τελευταίας μέτρησης από το επιταχυνσιόμετρο που αντιστοιχεί στον ίδιο άξονα περιστροφής. 
  angle_roll=mgForceX[10];
  angle_pitch=mgForceY[10];

  k=0;
  
  //Αποθηκεύουμε τον χρόνο από τον οποίο ξεκινάει η πρώτη μέτρηση.
  previoustime=micros();
}

void loop(){
  char arbuf[8],apbuf[8];
  int arntc_pos,apntc_pos;
  
  process_gyro_values();
  process_accel_values();

  angle_values(spaceoftime);

  //Μετατρέπουμε την τιμή της γωνίας του complimentary filter που αντιστοιχεί στον άξονα περιστροφής X από ακέραια τιμή τύπου float σε string και το ίδιο κάνουμε και 
  //για την τιμή της γωνίας του complimentary filter που αντιστοιχεί στον άξονα περιστροφής Y. 
  dtostrf(angle_roll, -4, 2, arbuf);
  dtostrf(angle_pitch, -4, 2, apbuf);

  //Βρίσκουμε την θέση του τερματικού χαρακτήρα στο string που αντιστοιχεί στην κάθε τιμή της γωνίας του complimentary filter. 
  for(i=0;i<8;i++){
    if(arbuf[i]=='\0'){
      arntc_pos=i;
      break;
    }
  }
  for(i=0;i<8;i++){ 
    if(apbuf[i]=='\0'){
      apntc_pos=i;
      break;
    }
  }

  //Εμφανίζουμε στην οθόνη LCD το string που αντιστοιχεί στην κάθε τιμή της γωνίας του complimentary filter διαγράφοντας παράλληλα ότι έχει απομείνει 
  //από το string που αντιστοιχεί στην προηγούμενη τιμή της κάθε γωνίας του complimentary filter .
  lcd.print("x_a:");
  lcd.setCursor(4,0);
  lcd.print(arbuf);
  for(i=4+arntc_pos;i<16;i++){
    lcd.setCursor(i,0);
    lcd.write(' ');
  }
  lcd.setCursor(0,1);
  lcd.print("y_a:");
  lcd.setCursor(4,1);
  lcd.print(apbuf);
  for(i=4+apntc_pos;i<16;i++){
    lcd.setCursor(i,1);
    lcd.write(' ');
  }
  lcd.setCursor(0,0);
  
  //Υπολογίζουμε τον χρόνο που μεσολάβησε μεταξύ δυο διαδοχικών μετρήσεων.
  if(timeflag){
    timeflag=false;
    spaceoftime=micros()-previoustime;
    previoustime=micros();
  }
  else{
    spaceoftime=micros()-previoustime;
    previoustime=micros();
  }
}

void setupMPU(){
  //Ρυθμίζουμε το MPU_6050.
  Wire.beginTransmission(0b1101000);//Ο αριθμός 1101000 στο δυαδικό σύστημα είναι η διεύθυνση του MPU-6050.
  //Το LSB αυτού του αριθμού μπορεί να είναι είτε μηδέν είτε ένα ανάλογα με το αν έχουμε συνδέσει το pin με το όνομα AD0 του MPU_6050 σε λογικό μηδέν (GND)
  //ή σε λογικό ένα (VCC) αντίστοιχα. Με αυτόν τον τρόπο μπορούμε να συνδέσουμε δυο MPU_6050 από τα οποία το ένα θα έχει διεύθυνση 1101000 και το άλλο
  //διεύθυνση 1101001.  
  Wire.write(0x6B);//Ο αριθμός 6B στο δεκαεξαδικό σύστημα είναι η διεύθυνση του καταχωρητή PWR_MGMT_1.
  Wire.write(0b00000000);//Η συσκευή αρχικά είναι σε sleep mode. Θέτωντας το bit6 μηδέν απενεργοποιούμε την λειτουργία sleep mode και "ξυπνάμε" την συσκευή.
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x1B);//Ο αριθμός 1B στο δεκαεξαδικό σύστημα είναι η διεύθυνση του καταχωρητή GYRO_CONFIG. 
  Wire.write(0b00001000);//Θέτωντας το bit4 μηδέν και το bit3 ένα ρυθμίζουμε το γυροσκόπιο ώστε να δουλεύει στην κλίμακα ±500°/s.
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x1C);//Ο αριθμός 1C στο δεκαεξαδικό σύστημα είναι η διεύθυνση του καταχωρητή ACCEL_CONFIG. 
  Wire.write(0b00010000);//Θέτωντας το bit4 ένα και το bit3 μηδέν ρυθμίζουμε το επιταχυνσιόμετρο ώστε να δουλεύει στην κλίμακα ±8g.
  Wire.endTransmission(); 
}

void recordAccelRegisters(){
  //Πέρνουμε τις μη επεξεργασμένες τιμές από το επιταχυνσιόμετρο.
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available()<6);
  accelX=Wire.read()<<8|Wire.read();//Αποθηκεύουμε τα περιεχόμενα των καταχωρητών ACCEL_XOUT_H και ACCEL_XOUT_L στην μεταβλητή accelX.   
  accelY=Wire.read()<<8|Wire.read();//Αποθηκεύουμε τα περιεχόμενα των καταχωρητών ACCEL_YOUT_H και ACCEL_YOUT_L στην μεταβλητή accelY. 
  accelZ=Wire.read()<<8|Wire.read();//Αποθηκεύουμε τα περιεχόμενα των καταχωρητών ACCEL_ZOUT_H και ACCEL_ZOUT_L στην μεταβλητή accelZ.    
  processAccelData();
}

void processAccelData(){
  //Μετατρέπουμε τις μη επεξεργασμένες τιμές από το επιταχυνσιόμετρο σε g. 
  gForceX=accelX/4096.0;
  gForceY=accelY/4096.0; 
  gForceZ=accelZ/4096.0;
}

void recordGyroRegisters(){
  //Πέρνουμε τις μη επεξεργασμένες τιμές από το γυροσκόπιο.
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x43); 
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available()<6);
  gyroX=Wire.read()<<8|Wire.read();//Αποθηκεύουμε τα περιεχόμενα των καταχωρητών GYRO_XOUT_H και GYRO_XOUT_L στην μεταβλητή gyroX.   
  gyroY=Wire.read()<<8|Wire.read();//Αποθηκεύουμε τα περιεχόμενα των καταχωρητών GYRO_YOUT_H και GYRO_YOUT_L στην μεταβλητή gyroY.   
  gyroZ=Wire.read()<<8|Wire.read();//Αποθηκεύουμε τα περιεχόμενα των καταχωρητών GYRO_ZOUT_H και GYRO_ZOUT_L στην μεταβλητή gyroZ.   
  processGyroData();
}

void processGyroData(){
  float rotZ;
  
  //Μετατρέπουμε τις μη επεξεργασμένες τιμές από το γυροσκόπιο σε °/s.
  rotX=gyroX/65.5;
  rotY=gyroY/65.5; 
  rotZ=gyroZ/65.5;
}

void process_gyro_values(){
  recordGyroRegisters();
 
  //Μετατοπίζουμε την τιμή της μέτρησης από το γυροσκόπιο που αντιστοιχεί στον άξονα περιστροφής X και την τιμή της μέτρησης από το γυροσκόπιο που αντιστοιχεί
  //στον άξονα περιστροφής Y ώστε οι τιμές αυτές να κυμαίνονται γύρω από το μηδέν. 
  nrotX=rotX-arotX;
  nrotY=rotY-arotY;
}

void process_accel_values(){
  int posX,posY,posZ;
  float ngForceX,ngForceY,ngForceZ;
  float swapvX,swapvY,swapvZ;
  
  //Ταξινομούμε τους πίνακες με τις τελευταίες 11 μετρήσεις από το επιταχυνσιόμετρο που αντιστοιχούν σε κάθε έναν άξονα περιστροφής σε αύξουσα σειρά.
  for(i=0;i<11;i++){
    posX=i;
    posY=i;
    posZ=i;
    for(j=i+1;j<11;j++){
      if(mgForceX[j]<mgForceX[posX]){
        posX=j;
      }
      if(mgForceY[j]<mgForceY[posY]){
        posY=j;
      }
      if(mgForceZ[j]<mgForceZ[posZ]){
        posZ=j;
      }
    }
    swapvX=mgForceX[i];
    mgForceX[i]=mgForceX[posX];
    mgForceX[posX]=swapvX;

    swapvY=mgForceY[i];
    mgForceY[i]=mgForceY[posY];
    mgForceY[posY]=swapvY;

    swapvZ=mgForceZ[i];
    mgForceZ[i]=mgForceZ[posZ];
    mgForceZ[posZ]=swapvZ;
  }
  
  //Υπολογίζουμε την γωνία που αντιστοιχεί στον άξονα περιστροφής X και την γωνία που αντιστοιχεί στον άξονα περιστροφής Y χρησιμοποιώντας το ενδιάμεσο στοιχείο
  //του κάθε ταξινομημένου πίνακα με τις τελευταίες 11 μετρήσεις από το επιταχυνσιόμετρο.
  accel_roll=atan2(mgForceY[5],mgForceZ[5])*57.29577951;
  accel_pitch=atan2((-1)*mgForceX[5],sqrt((mgForceY[5]*mgForceY[5])+(mgForceZ[5]*mgForceZ[5])))*57.29577951;
  
  recordAccelRegisters();

  //Μετατοπίζουμε τις τιμές των καινούργιων μετρήσεων από το επιταχυνσιόμετρο που αντιστοιχούν σε κάθε έναν άξονα περιστροφής ώστε αυτές που αντιστοιχούν στον άξονα
  //περιστροφής X και στον άξονα περιστροφής Y να κυμαίνονται γύρω από το μηδέν και αυτές που αντιστοιχούν στον άξονα περιστροφής Z γύρω από το ένα.
  ngForceX=gForceX-agForceX;
  ngForceY=gForceY-agForceY;
  ngForceZ=(gForceZ-agForceZ)+1;

  //Προσθέτουμε τις μετατοπισμένες τιμές των καινούργιων μετρήσεων από το επιταχυνσιόμετρο που αντιστοιχούν σε κάθε έναν άξονα περιστροφής στους αντίστοιχους πίνακες
  //με τις τελευταίες 11 μετατοπισμένες τιμές των μετρήσεων από το επιταχυνσιόμετρο που αντιστοιχούν σε κάθε έναν άξονα περιστροφής.
  nmgForceX[k]=ngForceX;
  nmgForceY[k]=ngForceY;
  nmgForceZ[k]=ngForceZ;

  //Αντιγράφουμε τα στοιχεία του κάθε πίνακα με τις τελευταίες 11 μετατοπισμένες τιμές των μετρήσεων από το επιταχυνσιόμετρο που αντιστοιχούν σε κάθε έναν άξονα
  //περιστροφής στον πίνακα που αντιστοιχεί στον ίδιο άξονα περιστροφής και πρόκειται να ταξινομηθεί.
  for(j=0;j<=10;j++){
    mgForceX[j]=nmgForceX[j];
    mgForceY[j]=nmgForceY[j];
    mgForceZ[j]=nmgForceZ[j];
  }

   k=k+1;
   if(k==11){
    k=0;
   }
}

void angle_values(unsigned long sotav){
  //Συνδιάζοντας τις τιμές των γωνιών που προέκυψαν από το ενδιάμεσο στοιχείο του κάθε ταξινομημένου πίνακα με τις τελευταίες 11 μετατοπισμένες τιμές των μετρήσεων
  //από το επιταχυνσιόμετρο με τις μετατοπισμένες τιμές των μετρήσεων από το γυροσκόπιο υπολογίζουμε την τελική τιμή της γωνίας που αντιστοιχεί στον άξονα
  //περιστροφής X και στον άξονα περιστροφής Y.
  angle_roll=0.98*(angle_roll+(nrotX*(sotav/(float(1000000)))))+(0.02*accel_roll);
  angle_pitch=0.98*(angle_pitch+(nrotY*(sotav/(float(1000000)))))+(0.02*accel_pitch);
}

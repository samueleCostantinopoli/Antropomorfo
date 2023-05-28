/*
Progetto ARL 2022/2023 
Autori: Samuele Costantinopoli, Simoni Leonardo, Carlo De Carolis, Gabriele Capparozza

Traccia:Utilizzando Processing, disegnare un robot ANTROPOMORFO operato in CINEMATICA INVERSA. Il valore desiderato (xd,yd,zd) per la posizione della pinza e il suo orientamento 
devono essere modificabili da tastiera. In particolare, per quanto riguarda la posizione della pinza, le coordinate xd, yd e zd possono essere modificate nel seguente modo: 
con x minuscolo si diminuisce la coordinata xd, con X maiuscolo la si aumenta. Analogamente si possono usare le lettere y e z per le coordinate yd e zd.
Per quanto riguarda l'orientamento desiderato della pinza, e cioè la matrice Re, procedere come segue. Individuare l'asse z6 della pinza mediante gli angoli di azimuth α e di elevazione 
β rispetto al sistema di base (x0,y0,z0) (fare riferimento a questa figura), e definire x6 e y6 facendo seguire una rotazione di un angolo θ intorno all'asse z6. 
Questo si può dimostrare corrisponde a una parametrizzazione di tipo ZYZ della matrice di rotazione desiderata Re, con angoli (α,90o-β,θ), coincidente quindi con l'espressione 
della matrice R36 del polso sferico in cui occorre sostituire α al posto di θ4, 90o-β al posto di θ5 e θ al posto di θ6. Per cambiare l'orientamento della pinza sarà quindi 
sufficiente agire sulle variabili α, β e θ mediante la pressione per esempio dei seguenti tasti: a minuscolo per diminuire α e A maiuscolo per aumentarlo, b minuscolo per diminuire β e B maiuscolo 
per aumentarlo, t minuscolo per diminuire θ e T maiuscolo per aumentarlo.
Scrivere lo sketch tenendo inoltre conto delle seguenti specifiche:

  1-Durante tutta l'esecuzione del programma deve essere riportato a schermo il valore delle coordinate desiderate (xd,yd,zd) per la posizione della pinza e quello (in gradi) degli angoli (α,β,θ) 
  che definiscono l'orientamento della pinza. Le coordinate della pinza vanno scritte SCEGLIENDO COME TERNA DI RIFERIMENTO (x0,y0,z0) DI BASE QUELLA CONSIDERATA A LEZIONE 
  (e non quella utilizzata da Processing).
  
  2-Scrivere a schermo anche la matrice Re desiderata con le colonne di TRE COLORI DIVERSI.
  
  3-DISEGNARE sia la TERNA (x0,y0,z0) della BASE sia quella (x6,y6,z6) della PINZA utilizzando per i tre assi x, y e z gli STESSI COLORI usati per le colonne di Re 
  (cioè l'asse x va disegnato dello stesso colore della prima colonna di Re, l'asse y dello stesso colore della seconda colonna e l'asse z come la terza colonna). 
  Per semplicità gli assi possono essere disegnati senza frecce.
  
  4-TRASCURARE per semplicità il problema delle COLLISIONI tra i vari link del robot.
  
  5-Per semplicità la PINZA può essere rappresentata come un semplice PARALLELEPIPEDO.
   
  6-Prevedere la possibilità da tastiera (per esempio con i tasti '+' e '-') di passare dalla soluzione GOMITO ALTO a quella GOMITO BASSO. Fissare invece arbitrariamente la soluzione per il polso sferico.
  
  7-Includere le funzionalità (già implementate nei vari sketch visti a lezione) che permettono: 
  1) di modificare l'altezza della vista, 
  2) di modificare il valore della costante Kp della legge di controllo,
  3) di spostare la base del robot con un click di mouse.
  
  Nota: La nostra implementazione aggiunge un controllo sulle rotazioni alpha beta e theta. Nel momento in cui il robot effettua una di queste ruotazioni, ed esce dallo spazio di lavoro, il robot si 
  fermerà e comparirà la scritta rossa "Fuori dallo spazio di lavoro!" 
*/

//Parametro per la funzione camera()
float eyeY = 0;

//Coordinate del centro del link 1 del robot che viene spostato con il mouse
float xBase;
float yBase;

/* variabile per compattare le condizioni di fine corsa
int segno = 1;*/

//Parametri gomito alto e basso 
int gomito = 1;
int errore = 0;

//Parametri posizione desiderata (q={xd,yd,zd})
float[] P = new float[3];
float[] Pe = new float[3];
float[] Pos = new float[3];


//Parametri ornetamento pinza desidreato (azimut={alfa, beta, theta})
float alfa = PI/2;
float beta = 0;
float gamma = 0;

//Soluzione cinematica inversa (theta={theta1,..., theta6})
float[] theta = new float[6];

//
float arg;

//Velocita di spostamento
float kp = 0.5;

//Dimensioni link 0:
float d0x = 50; // lungo x
float d0y = 40; // lungo y
float d0z = 50; // lungo z

// dimensioni link 1
float d1x = 50; // lungo x
float d1y = 60; // lungo y
float d1z = 50; // lungo z

//Dimensioni link 2
float d2x = 150; // lungo x
float d2y = 40; // lungo y
float d2z = 40; // lungo z

//Dimensioni link 3
float d3x = 75; // lungo x
float d3y = 40; // lungo y
float d3z = 40; // lungo z

//Dimensioni link 4
float d4x = 120;
float d4y = 30;
float d4z = 30;

//Dimensioni link 5
float d5x = 35;
float d5y = 30;
float d5z = 30;

//Dimensioni link 6 (pinza)
float d6x = 25;
float d6y = 15;
float d6z = 15;

// Dati 
float d1 = d0y+d1y;    //40+60
float d4 = d3x+d4x;    //75+120 = 195
float d6 = d5x+d6x;    //35+25 = 60 
float l2 = d2x;        //150
float l1 = 0;          //d1x/2;      //50/2 = 25

float[][] Re = new float[3][3];
float[] Pw = new float[3];
float A1 = 0;
float A2 = 0;
float[][] R03 = new float[3][3];
float[][] R36 = new float[3][3];


void setup() 
{
  size(1580, 1050, P3D);
  stroke(255);
  strokeWeight(2);
  xBase = width/2;
  yBase = height/2;
  Pe[0] = 0;  //Xe
  Pe[1] = 200;  //Ye
  Pe[2] = 0;  //Ze
}

void draw() 
{
  // Inizializzo lo sfondo le luci e la vista
  background(0);
  lights();
  camera((width/2.0), height/2 - eyeY, (height/2.0) / tan(PI*60.0 / 360.0), width/2.0, height/2.0, 0, 0, 1, 0);

  // Controllo tasti
  // Mouse
  
  if (mousePressed)
  {
    xBase = mouseX; // Base del roboto desiderata
    yBase = mouseY;
  }
  // Tastiera
  if (keyPressed)
  {
    //Movimento camera
    if (keyCode == DOWN)
    {
      eyeY -= 5;
    }
    if (keyCode == UP)
    {
      eyeY += 5;
    }
    
    //Velocita' di movimento kp
    if (key == 'k')
    {
      kp -= .05;
      if (kp < 0.01)
      {
        kp = 0.01;
      }
    }
    if (key == 'K')
    {
      kp += .05;
      if (kp > 3.0)
      {
        kp = 3.0;
      }
    }

    //Vettore posizione q={xd,yd,zd}
    if (key == 'x')
    {
      Pe[0] -= kp*0.3;
      if(Pe[0] < -(d2x+d3x+d4x-d1x/2)){
        Pe[0] = -(d2x+d3x+d4x-d1x/2);
      }
    }
    if (key == 'X')
    {
      Pe[0] += kp*0.3;
      if(Pe[0] > (d2x+d3x+d4x-d1x/2)){
        Pe[0] = (d2x+d3x+d4x-d1x/2);
      }
    }
    if (key == 'y')
    {
      Pe[1] -= kp*0.3;
      if(Pe[1] < -(d2x+d3x+d4x-d1x/2)){
        Pe[1] = -(d2x+d3x+d4x-d1x/2);
      }
    }
    if(key == 'Y')
    {
      Pe[1] += kp*0.3;
      if(Pe[1] > (d2x+d3x+d4x-d1x/2)){
        Pe[1] = (d2x+d3x+d4x-d1x/2);
      }
    }
    if(key == 'z')
    {
      Pe[2] -= kp*0.3;
      if(Pe[2] < -(d0y+d1y+d2x+d3x+d4x)){
        Pe[2] = -(d0y+d1y+d2x+d3x+d4x);
      }
    }
    if(key == 'Z')
    {
      Pe[2] += kp*0.3;
      if(Pe[2] > (d0y+d1y+d2x+d3x+d4x)){
        Pe[2] = (d0y+d1y+d2x+d3x+d4x);
      }
    }
    
    if(errore==0){
      // Vettore orientamento pinza
      if (key == 'a')
      {
        alfa -= kp*0.1;
        if(alfa < -2*PI){
          alfa = 0;
        }
      }
      if (key == 'A')
      {
        alfa += kp*0.1;
        if(alfa > 2*PI){
          alfa = 0;
        }
      }
      if (key == 'b')
      {
        beta -= kp*0.1;
        if(beta < -2*PI){
          beta = 0;
        }
      }
      if(key == 'B')
      {
        beta += kp*0.1;
        if(beta > 2*PI){
          beta = 0;
        }
      }
      if(key == 't')
      {
        gamma -= kp*0.1;
        if(gamma < -2*PI){
          gamma = 0;
        }
      }
      if(key == 'T')
      {
        gamma += kp*0.1;
        if(gamma > 2*PI){
          gamma = 0;
        }
      }
    }
    // Gomito
    if(key == '+')
    {
      gomito = 1;
    }
    if(key == '-')
    {
      gomito = -1;
    }
  }    
  
  // Risolvo la cinamatica inversa
  cinematicaInversa();
  
  //Prove:
  //theta[0]=PI/2;
  //theta[1]=PI/2;
  //theta[2]=PI/2;
  //theta[3]=PI/4;
  //theta[4]=PI/2;
  //theta[5]=PI/2;
  
  // Disegno il robot
  Antropomorfo();
  
  //testo a schermo
  textSize(25);
  fill(255);
  
  //testo theta
  fill(255,0,0);
  text("alfa = ",10,70); 
  text(alfa*180/PI,120,70);
  text("gradi",250,70);
  text("beta = ",10,120); 
  text(beta*180/PI,120,120);
  text("gradi",250,120);
  text("theta = ",10,170); 
  text(gamma*180/PI,120,170);
  text("gradi",250,170);
  fill(0,255,0);
  
  //testo vari theta
  fill(0, 255, 0);
  text("theta1 = ", 1350, 70);
  text(theta[0]*180/PI, 1450, 70);
  text("theta2 = ", 1350, 90);
  text(theta[1]*180/PI, 1450, 90);
  text("theta3 = ", 1350, 110);
  text(theta[2]*180/PI, 1450, 110);
  text("theta4 = ", 1350, 130);
  text(theta[3]*180/PI, 1450, 130);
  text("theta5 = ", 1350, 150);
  text(theta[4]*180/PI, 1450, 150);
  text("theta6 = ", 1350, 170);
  text(theta[5]*180/PI, 1450, 170);
  
  //testo gomito
  fill(255);
  text("gomito =", 10, 220);
  if(gomito == 1){
    text("alto", 120, 220);
  }
  else{
    text("basso", 120, 220);
  }
  
  //testo vista
  fill(0,255,0);
  text("coordinata y vista = ",width/2-140,30); 
  text(eyeY,width/2+65,30);
  
  if(errore == 1){
    fill(255, 0,0);
    text("Fuori Spazio di lavoro!", width/2-130, 55);
  }
  
  //testo kp
  fill(0, 100, 255);
  text("costante kp =", 10, 270);
  text(kp, 150, 270);
  
  
  //testo coordinate
  fill(255, 0, 0);
  text("coordinata Xe =", 10, 350);
  text(Pe[0], 175, 350);
  text("coordinata Ye =", 10, 400);
  text(Pe[1], 175, 400);
  text("coordinata Ze =", 10, 450);
  text(Pe[2], 175, 450);

  fill(255, 0, 0);
  text("coordinata X =", 1300, 350);
  text(Pos[0], 1455, 350);
  text("coordinata Y =", 1300, 400);
  text(Pos[1], 1455, 400);
  text("coordinata Z =", 1300, 450);
  text(Pos[2], 1455, 450); 
  
  
  //rappresento la matrice Re
  fill(255,255,255);
  text("Re = ",20,540);
  stroke(255);
  line(80,480,80,600); //1
  line(80,480,100,480);//2
  line(80,600,100,600);//3
  line(180+150,480,180+150,600);//4
  line(180+150,480,160+150,480);//5
  line(180+150,600,160+150,600);//6
  //colonna 1
  fill(0, 255, 255);
  text(cos(alfa)*cos(PI/2-beta)*cos(gamma)-sin(alfa)*sin(gamma), 95, 505); //1.1
  text(sin(alfa)*cos(PI/2-beta)*cos(gamma)+cos(alfa)*sin(gamma), 95, 545); //2.1
  text(-sin(PI/2-beta)*cos(gamma), 95, 585); //3.1
  //colonna2
  fill(255, 0, 0);
  text(-cos(alfa)*cos(PI/2-beta)*sin(gamma)-sin(alfa)*cos(gamma), 175, 505); //1.2
  text(-sin(alfa)*cos(PI/2-beta)*sin(gamma)+cos(alfa)*cos(gamma), 175, 545); //2.2
  text(sin(PI/2-beta)*sin(gamma), 175, 585); //3.2
  //colonna3
  fill(255, 255, 0);
  text(cos(alfa)*sin(PI/2-beta), 255, 505); //1.3
  text(sin(alfa)*sin(PI/2-beta), 255, 545); //2.3
  text(cos(PI/2-beta), 255, 585); //3.3
} 

float C3, S3;

// Funzione per la cinematica inversa
void cinematicaInversa() // aggiungere misure robot
{ 
  // Trovo Re matrice orientamento desiderato
  Re[0][0] = cos(alfa)*cos(PI/2-beta)*cos(gamma)-sin(alfa)*sin(gamma);
  Re[0][1] = -cos(alfa)*cos(PI/2-beta)*sin(gamma)-sin(alfa)*cos(gamma);
  Re[0][2] = cos(alfa)*sin(PI/2-beta);
  Re[1][0] = sin(alfa)*cos(PI/2-beta)*cos(gamma)+cos(alfa)*sin(gamma);
  Re[1][1] = -sin(alfa)*cos(PI/2-beta)*sin(gamma)+cos(alfa)*cos(gamma);
  Re[1][2] = sin(alfa)*sin(PI/2-beta);
  Re[2][0] = -sin(PI/2-beta)*cos(gamma);
  Re[2][1] = sin(PI/2-beta)*sin(gamma);
  Re[2][2] = cos(PI/2-beta);
  
  // trovo Pw
  Pw[0]= Pe[0]-(d6*Re[0][2]);
  Pw[1]= Pe[1]-(d6*Re[1][2]);
  Pw[2]= Pe[2]-(d6*Re[2][2]);
  
  // theta1
  theta[0] = atan2(Pw[1],Pw[0]);
  
  // Trovo A1, A2
  A1 = (Pw[0]*cos(theta[0]))+Pw[1]*sin(theta[0])-l1;
  A2 = d1-Pw[2];
  
  // theta3
  arg = (pow(A1,2)+pow(A2,2)-pow(d4,2)-pow(l2,2))/(2*l2*d4);
  if(abs(arg)<=1){
    if(gomito == -1){
      theta[2] = PI-asin(arg);
    }else{
      theta[2] = asin(arg);
    }
  
    C3 = cos(theta[2]);
    S3 = sin(theta[2]);
    // theta2
    theta[1] = atan2(d4*C3*A1-(d4*S3+l2)*A2,(d4*S3+l2)*A1+d4*C3*A2);
  
  
    // Trovo R03              
    R03[0][0] = cos(theta[0])*cos(theta[1] + theta[2]);
    R03[0][1] = sin(theta[0]);
    R03[0][2] = cos(theta[0])*sin(theta[1] + theta[2]);
    
    R03[1][0] = sin(theta[0])*cos(theta[1] + theta[2]);
    R03[1][1] = -cos(theta[0]);
    R03[1][2] = sin(theta[0])*sin(theta[1] + theta[2]);
    
    R03[2][0] = sin(theta[1] + theta[2]);
    R03[2][1] = 0;
    R03[2][2] = -cos(theta[1] + theta[2]);
                 
    // Trovo R36
    R36 = mProd(trasposta(R03),Re);
    
    // theta5
    theta[4]= atan2(pow(pow(R36[0][2], 2) + pow(R36[1][2], 2), 0.5), R36[2][2]);
  
    if (theta[4]<0 && theta[4]>-PI) {
      theta[3] = atan2(-R36[1][2], -R36[0][2]);
      theta[5] = atan2(-R36[2][1], R36[2][0]);
    } else {
      theta[3] = atan2(R36[1][2], R36[0][2]);
      theta[5] = atan2(R36[2][1], -R36[2][0]);
    }
    
    if(theta[4] == 0){
      theta[5] = PI/4; //theta6 arbitrario;
      theta[3] = atan2(R03[0][1], R03[0][0]) - theta[5];
    }
    
    if(theta[4] == PI || theta[4] == -PI){
      theta[5] = PI/4; //theta6 arbitrario;
      theta[3] = atan2(-R03[0][1], -R03[0][0]) + theta[5];
    }
    errore = 0;
  }else{
    errore = 1;
  }
    

  Pos[0]= l1*cos(theta[0]) + l2*cos(theta[0])*cos(theta[1]) + d4*cos(theta[0])*sin(theta[1]+theta[2]) + d6*(cos(theta[0])*(cos(theta[1]+theta[2])*cos(theta[3])*sin(theta[4])+sin(theta[1]+theta[2])*cos(theta[4])) + sin(theta[0])*sin(theta[3])*sin(theta[4]));
  Pos[1]= l1*sin(theta[0]) + l2*sin(theta[0])*cos(theta[1]) + d4*sin(theta[0])*sin(theta[1]+theta[2]) + d6*(sin(theta[0])*(cos(theta[1]+theta[2])*cos(theta[3])*sin(theta[4])+sin(theta[1]+theta[2])*cos(theta[4])) - cos(theta[0])*sin(theta[3])*sin(theta[4]));
  Pos[2]= d1 + l2*sin(theta[1]) - d4*cos(theta[1]+theta[2]) + d6*(sin(theta[1]+theta[2])*cos(theta[3])*sin(theta[4]) - cos(theta[1]+theta[2])*cos(theta[4]));
}


// Funzione per disegnare il robot
void Antropomorfo()
{
  pushMatrix(); // Memorizza il sistema di riferimento attuale
  
  fill(250,250,250);
  translate(xBase,yBase);
  
  //Coordinata X
  stroke(255,0,0);
  line(0, 0, 0, 300, 0, 0);
  
  //Coordinata Y
  stroke(255, 255, 0);
  line(0, 0, 0, 0, -300, 0);

  //Coordinata Z
  stroke(0, 255,255);
  line(0, 0, 0, 0, 0, 300);
  
  stroke(79,79,79);
  fill(35,35,35);
  
  // creazione Link 0 (base)
  
  translate(0, -d0y/2, 0);
  box(d0x,d0y,d0z);
  translate(0, -d0y/2, 0);
  
  rotateY(theta[0]+PI/2);
    
  fill(216,10,0);
    
  // Link 1
  translate(0, -d1y/2, 0);
  box(d1x, d1y, d1y);
  translate(0, -d1y/2, 0);
    
  fill(35,35,35);
  
  sphere(d2y);
    
  fill(216,10,0);
    
  rotateZ(theta[1]-PI);
    
  //Creazione Link2
  translate(d2x/2,0,0); 
  box(d2x,d2y,d2z);
  translate(d2x/2,0,0);
        
  rotateZ(theta[2]-PI/2);
    
  fill(35,35,35);
  
  sphere(d3y);
    
  fill(216,10,0);
    
  // Link 3
  translate(d3x/2,0,0);
  box(d3x,d3y,d3z);  
  translate(d3x/2,0,0);
    
  rotateX(-theta[3]-PI);
    
  fill(35,35,35);
  // Link 4
  translate(d4x/2, 0,0);
  box(d4x,d4y,d4z);
  translate(d4x/2, 0,0);
  
  rotateZ(-theta[4]);
    
  sphere(d4y/1.5);
    
  fill(216,10,0);
  // Link 5
  translate(d5x/2,0,0);
  box(d5x, d5y, d5z);
  translate(d5x/2,0,0);
    
  // Rotazione theta 6
  rotateX(-theta[5]);
    
  fill(35,35,35);
  
  //Link 6 (si muove con theta5 = q[5])
  translate(d6x/2,0,0);
  box(d6x,d6y,d6z);
  translate(d6x/2, 0, 0);
    
  //Coordinata X
  stroke(255, 255, 0);
  line(0, 0, 0, 300, 0, 0);
    
  //Coordinata Y
  stroke(0, 255,255);
  line(0, 0, 0, 0, -300, 0);
  
  //Coordinata Z
  stroke(255,0,0);
  line(0, 0, 0, 0, 0, 300);
    
  popMatrix();  // Ritorna al sistema di riferimento memorizzato
}




/////// LIBRERIA PROFESSORE ///////////

float[][] mProd(float[][] A,float[][] B) // Calcola prodotto di due matrici A e B
{
  int nA = A.length;
  int nAB = A[0].length;
  int nB = B[0].length;
  
  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nB; j++) 
    {  
      for (int k=0; k < nAB; k++) 
      {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return C;
}

float[][] mSum(float[][] A,float[][] B) // Calcola la somma di due matrici A e B
{
  int nA = A.length;
  int nB = A[0].length;
  
  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nB; j++) 
    {  
      C[i][j] = A[i][j] + B[i][j];
    }
  }
  return C;
}

float[][] trasposta(float[][] A) // Calcola la trasposta di una matrice A
{
  int nR = A.length;
  int nC = A[0].length; 
  
  float[][] C = new float[nC][nR]; 

  for (int i=0; i < nC; i++) 
  {
    for (int j=0; j < nR; j++) 
    {  
      C[i][j] = A[j][i];
    }
  }
  return C;
}


float[][] minore(float[][] A, int i, int j) // Determina il minore (i,j) di una matrice A
{
  int nA = A.length;
  float[][] C = new float[nA-1][nA-1];
  
  for (int iM = 0; iM < i; iM++)
  {
    for (int jM = 0; jM < j; jM++)
    {
      C[iM][jM] = A[iM][jM];
    } 
    for (int jM = j; jM < nA-1; jM++)
    {
      C[iM][jM] = A[iM][jM+1];
    } 
  }
  for (int iM = i; iM < nA-1; iM++)
  {
    for (int jM = 0; jM < j; jM++)
    {
      C[iM][jM] = A[iM+1][jM];
    } 
    for (int jM = j; jM < nA-1; jM++)
    {
      C[iM][jM] = A[iM+1][jM+1];
    } 
  }
  return C;
}


float det(float[][] A) // Calcola il determinante di A
{
  int nA = A.length;
  float determinante = 0;
  
  if (nA == 1)
  {
    determinante = A[0][0];
  }
  else
  {
    for (int j=0; j < nA; j++) 
    {
      determinante = determinante + A[0][j]*pow(-1,j)*det(minore(A,0,j));
    }
  }
  return determinante;
}


float[][] invMat(float[][] A) // Calcola l'inversa di una matrice A
{
  int nA = A.length;
  float[][] C = new float[nA][nA];
  float detA = det(A);

  if (nA == 1)
  {
    C[0][0] = 1/detA;
  }
  else
  {
    for (int i=0; i < nA; i++) 
    {
      for (int j=0; j < nA; j++) 
      {
        C[j][i] = pow(-1,i+j)*det(minore(A,i,j))/detA;
      }
    }
  }
  return C;
}

float[][] idMat(int nA, float sigma) // Assegna una matrice identità di ordine nA moltiplicata per una costante sigma
{
  float[][] I = new float[nA][nA]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nA; j++) 
    {  
      I[i][j] = 0;
    }
    I[i][i] = sigma;
  }
  return I;
}

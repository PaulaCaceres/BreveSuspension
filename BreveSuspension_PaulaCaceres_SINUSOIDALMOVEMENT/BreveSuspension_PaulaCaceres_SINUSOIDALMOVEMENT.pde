import processing.video.*;
import gab.opencv.*;
import java.awt.*;
import netP5.*;
import oscP5.*;

OpenCV opencv;
ArrayList<Contour> contours;

OscP5 oscP5;
NetAddress myRemoteLocation;

//CAPTURA VIDEO//
Capture video;
int camPixels;
PImage antesPixels, imagenDiferencia;
boolean yaTomoReferencia;
//CAPTURA VIDEO//

// SENSADO
// el algoritmo saca un promedio de color
// si su diferencia con el color de referencia es mayor
// que limiteDiferencia, lo dibuja y ademas lo cuenta en presenceSum
int limiteDiferencia = 30;
int limitePresencia = 15000000;
boolean hayPresencia = false;
boolean debug = false;

Mover m;
ArrayList<Mover> movers;
int cuantas = 40;
PImage fondo;
PVector mov = new PVector();

void setup() {
  size(1280, 720);
  hint(ENABLE_DEPTH_SORT);

  inicializaVideo();

  opencv = new OpenCV(this, width, height);
  
  /* start oscP5, listening for incoming messages at port 12000 */
  oscP5 = new OscP5(this,12000);
  /* myRemoteLocation is a NetAddress (takes 2 parameters, an ip address and a port number) and is used as parameter in oscP5.send()*/
  myRemoteLocation = new NetAddress("127.0.0.1",12001);

  fondo = loadImage ("fondo.png");

  movers = new ArrayList<Mover>();
  for (int i = 0; i < cuantas; i++) {
    movers.add(new Mover());
  }
}

void draw() {
  OscMessage myOscMessage = new OscMessage("/test");
   // Enviar el mensaje
  oscP5.send(myOscMessage, myRemoteLocation); 
  // Añadir la variable (int) cuantas al osc message 
  //myOscMessage.add(cuantas); 
  
  if (hayPresencia){
    int i = movers.size();
    myOscMessage.add(i+10); 
   //myOscMessage.add(5);
   oscP5.send(myOscMessage, myRemoteLocation);
   }  else {
   myOscMessage.add(0);
    oscP5.send(myOscMessage, myRemoteLocation);
    }
  
  procesaVideo(); 
  opencv.loadImage(imagenDiferencia); 
  contours = opencv.findContours();
  for (Contour contour : contours) {
    Rectangle r = contour.getBoundingBox();
    if (r.width > 200 &&  r.height > 200) {
      noFill(); 
      //strokeWeight(2); 
      //stroke(0, 255, 0);
      rect(r.x, r.y, r.width, r.height);
    }
  }
  image(fondo, 0, 0);
  dibujaParticulas();
}

void keyReleased() {
  if (key == 'd' || key == 'D') debug = !debug;
  else if (key == 't' || key == 'T') yaTomoReferencia = false;
}

// Articula movimiento interno de la partícula con un movimiento externo
void dibujaParticulas() {

  mov.x = sin(millis()/1000.0)*900 + width/2;

  if (hayPresencia==false) {
    mov.y = sin(millis()/500.0)*300 + 300;
  } else {
    // int bajar = 400; muy alto ese valor
    int bajar = 4;
    mov.y = mov.y + bajar;
    println("HAY ALGUIEN");
  }

  if (debug) ellipse(mov.x, mov.y, 10, 10);

  for (Mover m : movers) {  
    m.arrive(mov);        // Update the location
    m.update();
    m.display();          // Display the Mover
    m.separate(movers);   //if (mov.x < height/4)
  }
}

void tomarReferencia() {
  antesPixels.loadPixels();
  for (int i = 0; i < camPixels; i++) {
    antesPixels.pixels[i] = video.pixels[i];
  }
  antesPixels.updatePixels();
}

void procesaVideo() {
  if (video.available()) {
    // Leer nuevo frame de video
    video.read(); 
    // Hacer disponibles los pixels del video
    video.loadPixels();
    if (!yaTomoReferencia) {
      tomarReferencia();
      yaTomoReferencia = true;
    }
    sustraccionFondo();
  }
  // Dibuja el resultado
  if (debug) { // si se necesita verificar el contenido de las imagenes
    image(imagenDiferencia, 0, 0, width/2, height/2);
    image(antesPixels, width/2, 0, width/2, height/2);
    image(video, width/2, height/2, width/2, height/2);
  } else { // de lo contrario imagen de fondo camara
    image(imagenDiferencia, 0, 0);
  }
}

void sustraccionFondo() {
  int presenceSum = 0;
  // Diferencia entre el frame actual y el fondo almacenado
  // Límite para comparar si el cambio entre las dos imágenes es mayor a...
  imagenDiferencia.loadPixels();
  // Para cada pixel de video de la cámara, tomar el color actual y el anterior de ese pixel
  for (int i = 0; i < camPixels; i++) { 
    color currentColor = video.pixels[i];
    color backgroundColor = antesPixels.pixels[i];
    // Extraer los colores de los píxeles del frame actual
    int currentR = (currentColor >> 16) & 0xFF;
    int currentG = (currentColor >> 8) & 0xFF;
    int currentB = currentColor & 0xFF;
    // Extraer los colores de los píxeles del fondo
    int backgroundR = (backgroundColor >> 16) & 0xFF;
    int backgroundG = (backgroundColor >> 8) & 0xFF;
    int backgroundB = backgroundColor & 0xFF;
    // Computar la diferencia entre los colores
    int diffR = abs(currentR - backgroundR);
    int diffG = abs(currentG - backgroundG);
    int diffB = abs(currentB - backgroundB);
    float promedio = (diffR + diffG + diffB)/3.0;
    // si el pixel es diferente dibuja el mismo pixel
    if (promedio > limiteDiferencia) imagenDiferencia.pixels[i] = currentColor;
    // de lo contrario negro
    else imagenDiferencia.pixels[i] = 0;
    // Sumar las diferencias a la cuenta
    presenceSum += promedio;
  }
  imagenDiferencia.updatePixels();
  println(presenceSum);
  if (presenceSum > limitePresencia) hayPresencia = true;
  else hayPresencia = false;
}

void inicializaVideo() {
  String[] cameras = Capture.list();
  printArray(cameras);
  // Empezar la captura
  // video = new Capture(this, width, height);
  video = new Capture(this, 1280, 720, cameras[23]);
  video.start();
  // Almacenar píxeles de la cámara en variable camPixels
  camPixels = video.width * video.height;
  // Almacenar la imagen el fondo que servirá de referencia en una PImage
  antesPixels = new PImage (video.width, video.height);
  imagenDiferencia = new PImage (video.width, video.height);
  yaTomoReferencia = false;
}
class Mover {
  
  PImage img;
  PVector position;
  PVector velocity;
  PVector acceleration;
  float maxforce;
  float maxspeed;

  Mover() {
    img = loadImage ("pana2.png");
    acceleration = new PVector(0,0);
    position = new PVector(random(width),random(height));
    velocity = new PVector(0,0);
    maxspeed = random(10, 30);
    maxforce = random(0.5,1.5);
  }


  void update() {
    velocity.add(acceleration);
    velocity.limit(maxspeed);
    position.add(velocity);
    acceleration.mult(0);  
  }
  
  
   void applyForce(PVector force) {
     acceleration.add(force);
 }
  
  void arrive(PVector target) {
    PVector desired = PVector.sub(target,position);  // desired = vector desde la posición al target
    float d = desired.mag();
      // Si la distancia entre el objeto y el target es menor a 100 píxeles...
    if (d < 100) {
      float m = map(d,0,100,0,maxspeed);
      desired.setMag(m);
    } else {
      // Proceder al máximo de velocidad (maxspeed)
      desired.setMag(maxspeed);
    }

    // Fuerza de dirección (steering) = velocidad deseada (desired) - velocidad actual (velocity)
    PVector steer = PVector.sub(desired,velocity);
    steer.limit(maxforce);  // Limitar a la máxima fuerza de dirección
    applyForce(steer);
   }
  
  
  void separate (ArrayList<Mover> movers) {
    float desiredseparation = random (200, 500); 
    PVector sum = new PVector();
    int count = 0;
    // buscar todos los elementos en la lista "movers" y traer cada elemento,
    // uno atrás de otro, a la variable(objeto) "other"
    for (Mover other : movers) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < desiredseparation)) {
        PVector diff = PVector.sub(position, other.position);
        diff.normalize();
        // Mientras más cerca esta el vehículo de otro más se tiene que alejar 
        // (mayor la magnitud del PVector que los aleja), mientras nmás lejos, menos.
        // Dividimos por la distancia para calcular adecuadamente
        diff.div(d); 
        sum.add(diff);
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      sum.normalize();
      sum.mult(maxspeed);
      PVector steer = PVector.sub(sum, velocity);
      steer.limit(maxforce);
      applyForce(steer);
    }
  }
  
  
  void display() {
    image(img,position.x,position.y,100,100);
  }

}
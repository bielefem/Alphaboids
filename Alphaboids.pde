Flock flock;
// config able data
int BoidNumber = 20;
float desiredseparation = 25.0f;
float neighbordist = 100;
int alphavalue = 5;


void setup() {
  size(1024, 700);
  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < BoidNumber; i++) {
     // flock.addBoid(new Boid(300,300));
    flock.addBoid(new Boid(random(500),random(500)));
  }
}

void draw() {
  background(250);
  flock.run();
}

// The Flock (a list of Boid objects)
class Flock {
  ArrayList<Boid> boids; // An ArrayList for all the boids
  Flock() {
    boids = new ArrayList<Boid>(); // Initialize the ArrayList
  }
  void run() {
    for (Boid b : boids) {
      b.run(boids);  // Passing the entire list of boids to each boid individually
    }
  }
  void addBoid(Boid b) {
    boids.add(b);
  }
}

// The Boid class
class Boid {

  PVector location;
  PVector velocity;
  float r;            //size
  float maxforce;    // Maximum steering force
  float maxspeed;    // Maximum speed
  int chohensioncounter = 0;// to avoid idle
    Boid(float x, float y) {

    // Leaving the code temporarily this way so that this example runs in JS
    float angle = random(TWO_PI);
    velocity = new PVector(cos(angle), sin(angle));

    location = new PVector(x, y);
    r = 2.0;
    maxspeed = 1;
    maxforce = 0.03;
  }

  void run(ArrayList<Boid> boids) {
    flock(boids);
    update();
    borders();
    render_cohension();
    render_avoidiance();
    render_entety();
  }

  void applyForce(PVector force) {
    // We could add mass here if we want A = F / M
    velocity.add(force);
  }

  // We accumulate a new acceleration each time based on three rules
  void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids);   // Separation
    PVector coh = big_cohesion(boids);   // Cohesion
    // Arbitrarily weight these forces
    sep.mult(1.5);
    coh.mult(0.5);
    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(coh);
    velocity.mult(less_cohesion(boids));
  }

  // Method to update location
  void update() {
    // Update velocity
    // Limit speed
    velocity.div(velocity.mag());
    velocity.mult(0.5);
    // New Position
    location.add(velocity);
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, location);  // A vector pointing from the location to the target
    // Scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);
    desired.setMag(maxspeed);

    // Steering = Desired minus Velocity  Reynolds
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    return steer;
  }

  void render_cohension() {
    float theta = velocity.heading() + radians(90);
    pushMatrix();
    translate(location.x, location.y);
    rotate(theta);

    //fill cohensive cognition
    fill( #7FFF00,  20);
    stroke(#76FF00);
    ellipse(0, 0, 2*neighbordist, 2*neighbordist);

    popMatrix();
  }

  void render_avoidiance() {
    float theta = velocity.heading() + radians(90);
    pushMatrix();
    translate(location.x, location.y);
    rotate(theta);
    // avoidence red
    fill(#EE0000,  100);
    stroke(#FF0000);
    ellipse(0, 0, 2*desiredseparation, 2*desiredseparation);
    popMatrix();
  }
  void render_entety() {
    float theta = velocity.heading() + radians(90);
    pushMatrix();
    translate(location.x, location.y);
    rotate(theta);
    // entitys black
    fill(0);
    stroke(0);
    beginShape(TRIANGLES);
    vertex(0, -r*2);
    vertex(-r, r*2);
    vertex(r, r*2);
    endShape();

    popMatrix();
  }

  // Turnaround
  void borders() {
    if (location.x < -r) velocity.mult(-1);
    if (location.y < -r) velocity.mult(-1);;
    if (location.x > width+r) velocity.mult(-1);
    if (location.y > height+r) velocity.mult(-1);
  }

  // Done 
  // Separation
  // Method checks for nearby boids and steers away
  PVector separate (ArrayList<Boid> boids) {
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(location, other.location);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        count++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    return steer;
  }

  //less neighbours
  int less_cohesion (ArrayList<Boid> boids) {
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      if ((d > 0) && (d < neighbordist)) {
        count++;
      }
    }
      if ((count < alphavalue)&& (this.chohensioncounter > 200+ random(300))) {
        this.chohensioncounter = 0;
        return -1;
      }
    else {
    this.chohensioncounter++;
      return 1;
    }
  }



  // // Cohesion
  // // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
  PVector big_cohesion (ArrayList<Boid> boids) {

    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      if ((d > 0) && (d < neighbordist)) {
        count++;
      }
    }
    if (count >= alphavalue){
      return new PVector(random(height),random(width));
    } 
    else {
      return new PVector(0, 0);// Steer towards the location
    }
  }
}

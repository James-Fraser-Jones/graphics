#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

struct Intersection{
  vec3 position;
  float distance;
  int triangleIndex;
};

#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

float camX = 0.0;
float camY = 0.0;
float camZ = -3.0;

float yaw = 0.0;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen, const vector<Triangle>& triangles);

void Rotatate(float angX, float angY, float angZ, glm::mat3& dir){

}

bool ClosestIntersection(vec3 start, vec3 dir, const vector<Triangle>& triangles, Intersection& closestIntersection){
  bool intersection = false;

  for (uint32_t i=0; i < triangles.size(); i++){
    Triangle triangle = triangles[i];

    vec4 v0 = triangle.v0;
    vec4 v1 = triangle.v1;
    vec4 v2 = triangle.v2;
    vec3 e1 = vec3(v1.x-v0.x, v1.y-v0.y, v1.z-v0.z);
    vec3 e2 = vec3(v2.x-v0.x, v2.y-v0.y, v2.z-v0.z);
    vec3 b = vec3(start.x-v0.x, start.y-v0.y, start.z-v0.z);
    mat3 A(-dir, e1, e2);
    vec3 x = glm::inverse(A) * b;

    float t = x.x;
    float u = x.y;
    float v = x.z;

    if ((t >= 0) && (u >= 0) && (v >= 0) && (u + v <= 1)){ //use less than or equal to here, instead of less than
      intersection = true;

      if (t <= closestIntersection.distance){
        closestIntersection.position = (start + (dir * t));
        closestIntersection.distance = t;
        closestIntersection.triangleIndex = i;
      }
    }
  }

  return intersection;
}

int main(int argc, char* argv[]){
  //Initialize the screen
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  //Load the test model into an empty vector of triangles
  vector<Triangle> triangles;
  LoadTestModel(triangles);

  //Enter the rendering loop
  while(NoQuitMessageSDL()){
      Update();
      Draw(screen, triangles);
      SDL_Renderframe(screen);
  }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen, const vector<Triangle>& triangles){
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t)); //Clear the buffer

  float focalLength = 2; //since we're using xf and yf, focal length needs to be equal to width of screen which is the magnitude of the interval [-1:1] which is 2

  //Loop over all pixels in the image and calculate a ray for each one.
  for (int y = 0; y < SCREEN_HEIGHT; y++){ //don't use unsigned ints here!!!
    for (int x = 0; x < SCREEN_WIDTH; x++){

      //converting pixel values to the interval of the cornell box [-1:1]
      float xf = 2 * (float) x / (SCREEN_WIDTH-1) - 1; //goes from interval [0:SCREEN_WIDTH-1] to [-1:1]
      float yf = 2 * (float) y / (SCREEN_HEIGHT-1) - 1; //goes from interval [0:SCREEN_HEIGHT-1] to [-1:1]
      float f = focalLength; //temporary focalLength to be modified using polar co-ordinates

      float ang = atan2(xf, focalLength); //conversion to polar co-ordinates
      ang += yaw; //changing angle with yaw global variable
      vec3 dir(cos(ang), yf, sin(ang));

      vec3 start(camX, camY, camZ);
      Intersection closestIntersection = {start, std::numeric_limits<float>::max(), -1};

      if (ClosestIntersection(start, dir, triangles, closestIntersection)){
        vec3 colour = triangles[closestIntersection.triangleIndex].color;
        PutPixelSDL(screen, x, y, colour);
      }

    }
  }
}

/*Place updates of parameters here*/
void Update(){
  /* Compute frame time */
  static int t = SDL_GetTicks();
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;

  /* Display render time */
  std::cout << "Render time: " << dt << " ms." << std::endl;

  float speed = 0.05;

  /* Update variables*/
  const uint8_t* keystate = SDL_GetKeyboardState( 0 );
  if(keystate[SDL_SCANCODE_UP]){
    camY -= speed;
  }
  if(keystate[SDL_SCANCODE_DOWN]){
    camY += speed;
  }
  if(keystate[SDL_SCANCODE_LEFT]){
    camX -= speed;
  }
  if(keystate[SDL_SCANCODE_RIGHT]){
    camX += speed;
  }
  if(keystate[SDL_SCANCODE_A]){
    yaw -= speed/2;
  }
  if(keystate[SDL_SCANCODE_D]){
    yaw += speed/2;
  }
}

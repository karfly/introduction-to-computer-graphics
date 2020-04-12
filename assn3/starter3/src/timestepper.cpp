#include "timestepper.h"

#include <cstdio>

void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 3.1
   std::vector<Vector3f> state = particleSystem->getState();
   std::vector<Vector3f> f = particleSystem->evalF(state);

   std::vector<Vector3f> newState;
   for (size_t i = 0; i < state.size(); ++i) {
      newState.push_back(state[i] + stepSize * f[i]);
   }

   particleSystem->setState(newState);
}

void Trapezoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 3.1
   std::vector<Vector3f> state = particleSystem->getState();
   std::vector<Vector3f> f0 = particleSystem->evalF(state);
   size_t stateSize = state.size();
   std::vector<Vector3f> movedState;
   for (size_t i = 0; i < stateSize; ++i) {
      movedState.push_back(state[i] + stepSize * f0[i]);
   }

   std::vector<Vector3f> f1 = particleSystem->evalF(movedState);
   std::vector<Vector3f> newState;
   for (size_t i = 0; i < state.size(); ++i) {
      newState.push_back(state[i] + (stepSize / 2.0) * (f0[i] + f1[i]));
   }

   particleSystem->setState(newState);
}


void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   std::vector<Vector3f> state = particleSystem->getState();
   size_t stateSize = state.size();
   
   // k1
   std::vector<Vector3f> k1 = particleSystem->evalF(state);
   for (size_t i = 0; i < stateSize; ++i) {
      k1[i] *= stepSize;
   }

   // k2
   std::vector<Vector3f> k2Input;
   for (size_t i = 0; i < stateSize; ++i) {
      k2Input.push_back(state[i] + 0.5 * k1[i]);
   }
   std::vector<Vector3f> k2 = particleSystem->evalF(k2Input);
   for (size_t i = 0; i < stateSize; ++i) {
      k2[i] *= stepSize;
   }

   // k3
   std::vector<Vector3f> k3Input;
   for (size_t i = 0; i < stateSize; ++i) {
      k3Input.push_back(state[i] + 0.5 * k2[i]);
   }
   std::vector<Vector3f> k3 = particleSystem->evalF(k3Input);
   for (size_t i = 0; i < stateSize; ++i) {
      k3[i] *= stepSize;
   }

   // k4
   std::vector<Vector3f> k4Input;
   for (size_t i = 0; i < stateSize; ++i) {
      k4Input.push_back(state[i] + k3[i]);
   }
   std::vector<Vector3f> k4 = particleSystem->evalF(k4Input);
   for (size_t i = 0; i < stateSize; ++i) {
      k4[i] *= stepSize;
   }

   // update state
   std::vector<Vector3f> newState;
   for (size_t i = 0; i < state.size(); ++i) {
      newState.push_back(state[i] + (1.0 / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
   }

   particleSystem->setState(newState);
}


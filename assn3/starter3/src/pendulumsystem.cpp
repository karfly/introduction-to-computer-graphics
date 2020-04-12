#include "pendulumsystem.h"

#include <iostream>
#include <math.h>
#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const float EPS = 1e-6;

const int NUM_PARTICLES = 5;
const float VISCOUS_K = 0.1;
const float SPRING_K = 4.0;
const float SPRING_LENGTH = 0.1;
const Vector3f g = Vector3f(0.0, -9.81, 0.0);

PendulumSystem::PendulumSystem()
{

    // TODO 4.2 Add particles for simple pendulum
    // TODO 4.3 Extend to multiple particles

    // To add a bit of randomness, use e.g.
    // float f = rand_uniform(-0.5f, 0.5f);
    // in your initial conditions.
    m_viscous_k = VISCOUS_K;
    m_spring_k = SPRING_K;
    m_spring_length = SPRING_LENGTH;

    for (size_t i = 0; i < NUM_PARTICLES; ++i) {
        m_masses.push_back(0.1);

        float y = 2.0;
        if (i != 0) {
            // y = m_vVecState[2 * (i - 1)][1] - rand_uniform(0.1, 0.5);
            y = m_vVecState[2 * (i - 1)][1] - rand_uniform(0.1, 0.3);
        }
        m_vVecState.push_back(Vector3f(rand_uniform(-0.1, 0.1), y, rand_uniform(-0.1, 0.1)));  // positions
        m_vVecState.push_back(Vector3f(0.0, 0.0, 0.0));  // velocities
    }
}


std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;

    // TODO 4.1: implement evalF
    //  - gravity
    std::vector<Vector3f> forcesGravity(NUM_PARTICLES);
    for (size_t i = 0; i < NUM_PARTICLES; ++i) {
        forcesGravity[i] = m_masses[i] * g;
    }

    //  - viscous drag
    std::vector<Vector3f> forcesViscous(NUM_PARTICLES);
    for (size_t i = 0; i < NUM_PARTICLES; ++i) {
        forcesViscous[i] = -m_viscous_k * state[2 * i + 1];
    }

    //  - springs
    std::vector<Vector3f> forcesSpring(NUM_PARTICLES);
    for (size_t i = 0; i < NUM_PARTICLES; ++i) {
        Vector3f forceSpring(0.0, 0.0, 0.0);

        if (i > 0) {
            Vector3f d = state[2 * i] - state[2 * (i - 1)];
            forceSpring += -m_spring_k * (d.abs() - m_spring_length) * (d / (EPS + d.abs()));
        }

        if (i < NUM_PARTICLES - 1) {
            Vector3f d = state[2 * i] - state[2 * (i + 1)];
            forceSpring += -m_spring_k * (d.abs() - m_spring_length) * (d / (EPS + d.abs()));
        }
        // if (isnan(-m_spring_k * (d.abs() - m_spring_length) * (d / (EPS + d.abs())))) {
        //     d.print();
        // }

        forcesSpring[i] = forceSpring;
    }
    
    // sum up all
    std::vector<Vector3f> accelerations(NUM_PARTICLES);
    for (size_t i = 0; i < NUM_PARTICLES; ++i) {
        accelerations[i] = (forcesGravity[i] + forcesViscous[i] + forcesSpring[i]) / m_masses[i];
        if (i == 0) {
            accelerations[i] *= 0.0;  // fix point
        }
        // accelerations[i] = (forcesSpring[i]) / m_masses[i];
    }

    // calculate derivative
    for (size_t i = 0; i < NUM_PARTICLES; ++i) {
        f.push_back(state[2 * i + 1]);  // velocity
        f.push_back(accelerations[i]);  // acceleration
    }

    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    // TODO 4.2, 4.3

    // example code. Replace with your own drawing  code
    for (size_t i = 0; i < NUM_PARTICLES; ++i) {
        gl.updateModelMatrix(Matrix4f::translation(m_vVecState[2 * i]));
        drawSphere(0.075f, 10, 10);
    }
}

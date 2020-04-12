#include <iostream>

#include "clothsystem.h"
#include "camera.h"
#include "vertexrecorder.h"

 // your system should at least contain 8x8 particles.
const int W = 10;
const int H = 10;
const float CLOTH_CELL_SIZE = 0.2;

const float EPS = 1e-6;

const Vector3f G = Vector3f(0.0, -9.81, 0.0);
const float VISCOUS_K = 0.1;

const float STRUCTURAL_SPRING_K = 50.0;
const float STRUCTURAL_SPRING_LENGTH = 0.2;

const float SHEAR_SPRING_K = 2.0;
const float SHEAR_SPRING_LENGTH = 0.4;

const float FLEXION_SPRING_K = 2.0;
const float FLEXION_SPRING_LENGTH = 0.4;


ClothSystem::ClothSystem()
{
    // TODO 5. Initialize m_vVecState with cloth particles. 
    // You can again use rand_uniform(lo, hi) to make things a bit more interesting
    m_h = H;
    m_w = W;

    m_viscous_k = VISCOUS_K;
    
    m_structural_spring_k = STRUCTURAL_SPRING_K;
    m_structural_spring_length = STRUCTURAL_SPRING_LENGTH;

    m_shear_spring_k = SHEAR_SPRING_K;
    m_shear_spring_length = SHEAR_SPRING_LENGTH;

    m_flexion_spring_k = FLEXION_SPRING_K;
    m_flexion_spring_length = FLEXION_SPRING_LENGTH;

    for (int i = 0; i < m_h; ++i) {
        for (int j = 0; j < m_w; ++j) {
            m_masses.push_back(0.1);
            
            float x = 0.0 - j * CLOTH_CELL_SIZE;
            float y = 0.0 - i * CLOTH_CELL_SIZE;
            float z = 0.0;

            m_vVecState.push_back(Vector3f(x, y, z));  // positions
            m_vVecState.push_back(Vector3f(0.0, 0.0, 0.0));  // velocities
        }
    }
}


std::vector<Vector3f> ClothSystem::evalF(std::vector<Vector3f> state)
{
    // TODO 5. implement evalF
    int numParticles = m_h * m_w;
    std::vector<Vector3f> f;
    
    // - gravity
    std::vector<Vector3f> forcesGravity(numParticles);
    for (int i = 0; i < numParticles; ++i) {
        forcesGravity[i] = m_masses[i] * G;
    }

    // - viscous drag
    std::vector<Vector3f> forcesViscous(numParticles);
    for (int i = 0; i < numParticles; ++i) {
        forcesViscous[i] = -m_viscous_k * state[2 * i + 1];
    }

    // - structural springs
    std::vector<Vector3f> forcesStructuralSpring(numParticles);
    for (int i = 0; i < m_h; ++i) {
        for (int j = 0; j < m_w; ++j) {
            Vector3f forceStructuralSpring(0.0, 0.0, 0.0);

            // top
            if (i - 1 >= 0) {
                Vector3f d = state[2 * indexOf(i, j)] - state[2 * indexOf(i - 1, j)];
                forceStructuralSpring += -m_structural_spring_k * (d.abs() - m_structural_spring_length) * (d / (EPS + d.abs()));
            }

            // left
            if (j - 1 >= 0) {
                Vector3f d = state[2 * indexOf(i, j)] - state[2 * indexOf(i, j - 1)];
                forceStructuralSpring += -m_structural_spring_k * (d.abs() - m_structural_spring_length) * (d / (EPS + d.abs()));
            }

            // bottom
            if (i + 1 <= m_h - 1) {
                Vector3f d = state[2 * indexOf(i, j)] - state[2 * indexOf(i + 1, j)];
                forceStructuralSpring += -m_structural_spring_k * (d.abs() - m_structural_spring_length) * (d / (EPS + d.abs()));
            }

            // right
            if (j + 1 <= m_w - 1) {
                Vector3f d = state[2 * indexOf(i, j)] - state[2 * indexOf(i, j + 1)];
                forceStructuralSpring += -m_structural_spring_k * (d.abs() - m_structural_spring_length) * (d / (EPS + d.abs()));
            }

            forcesStructuralSpring[indexOf(i, j)] = forceStructuralSpring;
        }
    }

    // - shear springs
    std::vector<Vector3f> forcesShearSpring(numParticles);
    for (int i = 0; i < m_h; ++i) {
        for (int j = 0; j < m_w; ++j) {
            Vector3f forceShearSpring(0.0, 0.0, 0.0);

            int new_i, new_j;

            // bottom-right
            new_i = i + 1;
            new_j = j + 1;
            if ((new_i >= 0) && (new_i <= m_h - 1) && (new_j >= 0) && (new_j<= m_w - 1))  {
                Vector3f d = state[2 * indexOf(i, j)] - state[2 * indexOf(new_i, new_j)];
                forceShearSpring += -m_structural_spring_k * (d.abs() - m_structural_spring_length) * (d / (EPS + d.abs()));
            }

            // top-left
            new_i = i - 1;
            new_j = j - 1;
            if ((new_i >= 0) && (new_i <= m_h - 1) && (new_j >= 0) && (new_j<= m_w - 1))  {
                Vector3f d = state[2 * indexOf(i, j)] - state[2 * indexOf(new_i, new_j)];
                forceShearSpring += -m_structural_spring_k * (d.abs() - m_structural_spring_length) * (d / (EPS + d.abs()));
            }

            // bottom-left
            new_i = i + 1;
            new_j = j - 1;
            if ((new_i >= 0) && (new_i <= m_h - 1) && (new_j >= 0) && (new_j<= m_w - 1))  {
                Vector3f d = state[2 * indexOf(i, j)] - state[2 * indexOf(new_i, new_j)];
                forceShearSpring += -m_structural_spring_k * (d.abs() - m_structural_spring_length) * (d / (EPS + d.abs()));
            }

            // top-right
            new_i = i - 1;
            new_j = j + 1;
            if ((new_i >= 0) && (new_i <= m_h - 1) && (new_j >= 0) && (new_j<= m_w - 1))  {
                Vector3f d = state[2 * indexOf(i, j)] - state[2 * indexOf(new_i, new_j)];
                forceShearSpring += -m_structural_spring_k * (d.abs() - m_structural_spring_length) * (d / (EPS + d.abs()));
            }

            forcesShearSpring[indexOf(i, j)] = forceShearSpring;
        }
    }

    // - flexion springs
    std::vector<Vector3f> forcesFlexionSpring(numParticles);
    for (int i = 0; i < m_h; ++i) {
        for (int j = 0; j < m_w; ++j) {
            Vector3f forceFlexionSpring(0.0, 0.0, 0.0);

            int new_i, new_j;

            // bottom
            new_i = i + 2;
            new_j = j;
            if ((new_i >= 0) && (new_i <= m_h - 1) && (new_j >= 0) && (new_j<= m_w - 1))  {
                Vector3f d = state[2 * indexOf(i, j)] - state[2 * indexOf(new_i, new_j)];
                forceFlexionSpring += -m_flexion_spring_k * (d.abs() - m_flexion_spring_length) * (d / (EPS + d.abs()));
            }

            new_i = i;
            new_j = j + 2;
            if ((new_i >= 0) && (new_i <= m_h - 1) && (new_j >= 0) && (new_j<= m_w - 1))  {
                Vector3f d = state[2 * indexOf(i, j)] - state[2 * indexOf(new_i, new_j)];
                forceFlexionSpring += -m_flexion_spring_k * (d.abs() - m_flexion_spring_length) * (d / (EPS + d.abs()));
            }

            forcesFlexionSpring[indexOf(i, j)] = forceFlexionSpring;
        }
    }


    // sum up all
    std::vector<Vector3f> accelerations(numParticles);
    for (int i = 0; i < numParticles; ++i) {
        accelerations[i] = (
            forcesGravity[i] + \
            forcesViscous[i] + \
            forcesStructuralSpring[i] + \
            forcesShearSpring[i] + \
            forcesFlexionSpring[i] \
            ) / m_masses[i];
            
        if ((i == 0) or (i == m_w - 1)) {
            accelerations[i] *= 0.0;  // fix point
        }
    }

    // calculate derivative
    for (int i = 0; i < numParticles; ++i) {
        f.push_back(state[2 * i + 1]);  // velocity
        f.push_back(accelerations[i]);  // acceleration
    }
     
    return f;
}

int ClothSystem::indexOf(int i, int j) {
    return i * m_w + j;
}

void ClothSystem::draw(GLProgram& gl)
{
    //TODO 5: render the system 
    //         - ie draw the particles as little spheres
    //         - or draw the springs as little lines or cylinders
    //         - or draw wireframe mesh

    const Vector3f CLOTH_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(CLOTH_COLOR);

    // EXAMPLE for how to render cloth particles.
    //  - you should replace this code.
    for (int i = 0; i < m_h; ++i) {
        for (int j = 0; j < m_w; ++j) {
            Vector3f position = m_vVecState[2 * indexOf(i, j)];
            gl.updateModelMatrix(Matrix4f::translation(position));
            drawSphere(0.04f, 8, 8);
        }
    }
    
    // EXAMPLE: This shows you how to render lines to debug the spring system.
    //
    //          You should replace this code.
    //
    //          Since lines don't have a clearly defined normal, we can't use
    //          a regular lighting model.
    //          GLprogram has a "color only" mode, where illumination
    //          is disabled, and you specify color directly as vertex attribute.
    //          Note: enableLighting/disableLighting invalidates uniforms,
    //          so you'll have to update the transformation/material parameters
    //          after a mode change.
    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
    VertexRecorder rec;
    for (int i = 0; i < m_h; ++i) {
        for (int j = 0; j < m_w; ++j) {
            if ((j + 1) < m_w) {
                rec.record(m_vVecState[2 * indexOf(i, j)], CLOTH_COLOR);
                rec.record(m_vVecState[2 * indexOf(i, j + 1)], CLOTH_COLOR);
            }

            if ((i + 1) < m_h) {
                rec.record(m_vVecState[2 * indexOf(i, j)], CLOTH_COLOR);
                rec.record(m_vVecState[2 * indexOf(i + 1, j)], CLOTH_COLOR);
            }
        }
    }
    glLineWidth(3.0f);
    rec.draw(GL_LINES);
    gl.enableLighting(); // reset to default lighting model
}


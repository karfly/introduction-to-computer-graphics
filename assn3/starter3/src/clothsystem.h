#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

#include <vector>

#include "particlesystem.h"

class ClothSystem : public ParticleSystem
{
    ///ADD MORE FUNCTION AND FIELDS HERE
public:
    ClothSystem();

    // evalF is called by the integrator at least once per time step
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;

    // helper function
    int indexOf(int i, int j);

    // draw is called once per frame
    void draw(GLProgram& ctx);

    // inherits
    // std::vector<Vector3f> m_vVecState;

private:
    size_t m_h;
    size_t m_w;

    std::vector<float> m_masses;
    float m_viscous_k;

    float m_structural_spring_k;
    float m_structural_spring_length;

    float m_shear_spring_k;
    float m_shear_spring_length;

    float m_flexion_spring_k;
    float m_flexion_spring_length;
};


#endif

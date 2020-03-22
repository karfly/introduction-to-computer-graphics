#include "skeletalmodel.h"
#include <cassert>

#include "starter2_util.h"
#include "vertexrecorder.h"

using namespace std;

SkeletalModel::SkeletalModel() {
    program = compileProgram(c_vertexshader, c_fragmentshader_light);
    if (!program) {
        printf("Cannot compile program\n");
        assert(false);
    }
}

SkeletalModel::~SkeletalModel() {
    // destructor will release memory when SkeletalModel is deleted
    while (m_joints.size()) {
        delete m_joints.back();
        m_joints.pop_back();
    }

    glDeleteProgram(program);
}

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
    loadSkeleton(skeletonFile);

    m_mesh.load(meshFile);
    m_mesh.loadAttachments(attachmentsFile, (int)m_joints.size());

    computeBindWorldToJointTransforms();
    updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(const Camera& camera, bool skeletonVisible)
{
    // draw() gets called whenever a redraw is required
    // (after an update() occurs, when the camera moves, the window is resized, etc)

    m_matrixStack.clear();

    glUseProgram(program);
    updateShadingUniforms();
    if (skeletonVisible)
    {
        drawJoints(camera);
        drawSkeleton(camera);
    }
    else
    {
        // Tell the mesh to draw itself.
        // Since we transform mesh vertices on the CPU,
        // There is no need to set a Model matrix as uniform
        camera.SetUniforms(program, Matrix4f::identity());
        m_mesh.draw();
    }
    glUseProgram(0);
}

void SkeletalModel::updateShadingUniforms() {
    // UPDATE MATERIAL UNIFORMS
    GLfloat diffColor[] = { 0.4f, 0.4f, 0.4f, 1 };
    GLfloat specColor[] = { 0.9f, 0.9f, 0.9f, 1 };
    GLfloat shininess[] = { 50.0f };
    int loc = glGetUniformLocation(program, "diffColor");
    glUniform4fv(loc, 1, diffColor);
    loc = glGetUniformLocation(program, "specColor");
    glUniform4fv(loc, 1, specColor);
    loc = glGetUniformLocation(program, "shininess");
    glUniform1f(loc, shininess[0]);

    // UPDATE LIGHT UNIFORMS
    GLfloat lightPos[] = { 3.0f, 3.0f, 5.0f, 1.0f };
    loc = glGetUniformLocation(program, "lightPos");
    glUniform4fv(loc, 1, lightPos);

    GLfloat lightDiff[] = { 120.0f, 120.0f, 120.0f, 1.0f };
    loc = glGetUniformLocation(program, "lightDiff");
    glUniform4fv(loc, 1, lightDiff);
}

void SkeletalModel::loadSkeleton(const char* filename)
{
    // Load the skeleton from file here.
    std::cout << "Reading skeleton..." << std::endl;
    std::string line;

    std::ifstream infile(filename);
    while(std::getline(infile, line)) {
        std::istringstream lineStream(line);

        float x, y, z;
        int parentIndex;
        lineStream >> x >> y >> z >> parentIndex;

        Joint *joint = new Joint;
        joint->transform = Matrix4f::translation(x, y, z);
        m_joints.push_back(joint);

        if (parentIndex == -1) {
            m_rootJoint = joint;
        } else {
            m_joints[parentIndex]->children.push_back(joint);
        }
    }
}

void SkeletalModel::drawJoints_impl(const Camera& camera, const Joint * joint) {
    m_matrixStack.push(joint->transform);
    camera.SetUniforms(program, m_matrixStack.top());

    drawSphere(0.025f, 12, 12);

    for (auto& child : joint->children) {
        drawJoints_impl(camera, child);
    }

    m_matrixStack.pop();
}

void SkeletalModel::drawJoints(const Camera& camera)
{
    // Draw a sphere at each joint. You will need to add a recursive
    // helper function to traverse the joint hierarchy.
    //
    // We recommend using drawSphere( 0.025f, 12, 12 )
    // to draw a sphere of reasonable size.
    //
    // You should use your MatrixStack class. A function
    // should push it's changes onto the stack, and
    // use stack.pop() to revert the stack to the original
    // state.
    m_matrixStack.clear();
    drawJoints_impl(camera, m_rootJoint);
}

void SkeletalModel::drawSkeleton_impl(const Camera& camera, const Joint * joint) {
    m_matrixStack.push(joint->transform);
    
    // Traverse to children
    for (auto& child : joint->children) {
        Vector3f childTranslation = child->transform.getCol(3).xyz();
        float boneLength = childTranslation.abs();

        Matrix4f cylinderTransform = Matrix4f::translation(childTranslation);

        // Setup orthogonal coordinate system
        Vector3f y = -childTranslation.normalized();

        Vector3f vectorNotCollinearWithY = y + Vector3f(1.0, 0, 0);
        Vector3f x = Vector3f::cross(y, vectorNotCollinearWithY).normalized();
        Vector3f z = Vector3f::cross(x, y).normalized();

        // Setup transform
        Matrix3f cylinderRotation = Matrix3f(x, y, z);
        cylinderTransform.setSubmatrix3x3(0, 0, cylinderRotation);

        camera.SetUniforms(program, m_matrixStack.top() * cylinderTransform);
        drawCylinder(6, 0.02f, boneLength);

        drawSkeleton_impl(camera, child);
    }

    m_matrixStack.pop();
}

void SkeletalModel::drawSkeleton(const Camera& camera)
{
    // Draw cylinders between the joints. You will need to add a recursive 
    // helper function to traverse the joint hierarchy.
    //
    // We recommend using drawCylinder(6, 0.02f, <height>);
    // to draw a cylinder of reasonable diameter.


    m_matrixStack.clear();
    drawSkeleton_impl(camera, m_rootJoint);
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
    // Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
    m_joints[jointIndex]->transform.setSubmatrix3x3(
        0,
        0,
        Matrix3f::rotateX(rX) * Matrix3f::rotateY(rY) * Matrix3f::rotateZ(rZ)
    );
}

void SkeletalModel::computeBindWorldToJointTransforms_impl(Joint * joint) {
    m_matrixStack.push(joint->transform);
    joint->bindWorldToJointTransform = m_matrixStack.top().inverse();
    for (auto& child : joint->children) {
        computeBindWorldToJointTransforms_impl(child);
    }

    m_matrixStack.pop();
}

void SkeletalModel::computeBindWorldToJointTransforms()
{
    // 2.3.1. Implement this method to compute a per-joint transform from
    // world-space to joint space in the BIND POSE.
    //
    // Note that this needs to be computed only once since there is only
    // a single bind pose.
    //
    // This method should update each joint's bindWorldToJointTransform.
    // You will need to add a recursive helper function to traverse the joint hierarchy.
    m_matrixStack.clear();
    computeBindWorldToJointTransforms_impl(m_rootJoint);
}

void SkeletalModel::updateCurrentJointToWorldTransforms_impl(Joint * joint) {
    m_matrixStack.push(joint->transform);
    joint->currentJointToWorldTransform = m_matrixStack.top();
    for (auto& child : joint->children) {
        updateCurrentJointToWorldTransforms_impl(child);
    }

    m_matrixStack.pop();
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
    // 2.3.2. Implement this method to compute a per-joint transform from
    // joint space to world space in the CURRENT POSE.
    //
    // The current pose is defined by the rotations you've applied to the
    // joints and hence needs to be *updated* every time the joint angles change.
    //
    // This method should update each joint's currentJointToWorldTransform.
    // You will need to add a recursive helper function to traverse the joint hierarchy.
    m_matrixStack.clear();
    updateCurrentJointToWorldTransforms_impl(m_rootJoint);
}

void SkeletalModel::updateMesh()
{
    // 2.3.2. This is the core of SSD.
    // Implement this method to update the vertices of the mesh
    // given the current state of the skeleton.
    // You will need both the bind pose world --> joint transforms.
    // and the current joint --> world transforms.
    for (size_t i = 0; i < m_mesh.bindVertices.size(); ++i) {
        Vector4f bindVertex4f = Vector4f(m_mesh.bindVertices[i], 1.0);

        // Vector4f newCurrentVertex4f = Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
        Vector3f newCurrentVertex = Vector3f(0.0f, 0.0f, 0.0f);
        for (size_t j = 0; j < m_joints.size(); ++j) {
            float weight = m_mesh.attachments[i][j];
            newCurrentVertex += weight * (m_joints[j]->currentJointToWorldTransform * m_joints[j]->bindWorldToJointTransform * bindVertex4f).xyz();
        }
        
        m_mesh.currentVertices[i] = newCurrentVertex;
    }
}
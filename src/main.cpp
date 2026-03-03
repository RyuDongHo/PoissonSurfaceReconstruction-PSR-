/*
 * Poisson Surface Reconstruction - Main Application
 *
 * - GLB 파일 로딩 및 렌더링 (embedded texture 지원)
 * - Orbit 카메라 컨트롤 (좌클릭: 회전, 우클릭: 패닝, 스크롤: 줌)
 * - J/K 키: FOV 조절, ESC: 종료
 */

#define GLM_ENABLE_EXPERIMENTAL
#include <stdio.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <algorithm>
#include "shader.h"
#include "common.h"

// =============================================================================
// Global Variables
// =============================================================================

// Window
const GLuint WIN_W = 1600;
const GLuint WIN_H = 900;
const GLuint WIN_X = 800;
const GLuint WIN_Y = 450;
float aspectRatio = (float)WIN_W / (float)WIN_H;
GLFWwindow *window;

// Loaded Geometry
std::vector<glm::vec3> loadedVertices;
std::vector<glm::vec2> loadedUVs;
std::vector<glm::vec3> loadedNormals;

// OpenGL Resources
GLuint vao;
GLuint vbo;
GLuint textureID = 0;
GLuint programID;
size_t vertexCount = 0;

// Camera
float fov          = 80.f;
float cameraRadius = 100.0f;
float cameraTheta  = glm::radians(45.0f);
float cameraPhi    = glm::radians(45.0f);
glm::vec3 cameraTarget = glm::vec3(0.f);
const float rotSpeed  = 0.005f;
const float panSpeed  = 0.05f;
const float zoomSpeed = 8.0f;

// Mouse
enum MouseMode { NONE, ROTATE, PAN };
MouseMode mouseMode = NONE;
glm::vec2 lastMousePos = glm::vec2(0.f);

// MVP
glm::mat4 matModel = glm::mat4(1.f);
glm::mat4 matView  = glm::mat4(1.f);
glm::mat4 matProj  = glm::mat4(1.f);

// =============================================================================
// Upload geometry to GPU
// =============================================================================
void uploadToGPU()
{
    std::vector<glm::vec4> pos;
    std::vector<glm::vec2> uvs;

    pos.reserve(loadedVertices.size());
    uvs.reserve(loadedVertices.size());

    for (size_t i = 0; i < loadedVertices.size(); i++)
    {
        pos.push_back(glm::vec4(loadedVertices[i], 1.0f));
        uvs.push_back(i < loadedUVs.size() ? loadedUVs[i] : glm::vec2(0.f));
    }

    size_t posSize = pos.size() * sizeof(glm::vec4);
    size_t uvSize  = uvs.size() * sizeof(glm::vec2);

    glBufferData(GL_ARRAY_BUFFER, posSize + uvSize, NULL, GL_STATIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0,       posSize, pos.data());
    glBufferSubData(GL_ARRAY_BUFFER, posSize, uvSize,  uvs.data());

    // Layout 0: position (vec4)
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glEnableVertexAttribArray(0);
    // Layout 2: texCoord (vec2)
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, (void *)posSize);
    glEnableVertexAttribArray(2);

    vertexCount = pos.size();
    printf("GPU upload complete. Vertex count: %zu\n", vertexCount);
}

// =============================================================================
// OpenGL Init
// =============================================================================
void initGL()
{
    programID = LoadShader("../../shader/vertex.glsl", "../../shader/fragment.glsl");
    if (programID == 0)
    {
        printf("Shader loading failed!\n");
        return;
    }

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    uploadToGPU();

    glClearColor(0.5f, 0.8f, 0.8f, 1.0f);
    glClearDepthf(1.0f);
    glUseProgram(programID);
    glDisable(GL_CULL_FACE);

    printf("OpenGL initialization complete\n");
}

// =============================================================================
// Per-frame update (camera matrices)
// =============================================================================
void updateFunc()
{
    glm::vec3 cameraPos;
    cameraPos.x = cameraTarget.x + cameraRadius * sinf(cameraPhi) * cosf(cameraTheta);
    cameraPos.y = cameraTarget.y + cameraRadius * cosf(cameraPhi);
    cameraPos.z = cameraTarget.z + cameraRadius * sinf(cameraPhi) * sinf(cameraTheta);

    matView = glm::lookAt(cameraPos, cameraTarget, glm::vec3(0.f, 1.f, 0.f));
    matProj = glm::perspectiveRH(glm::radians(fov), aspectRatio, 0.1f, 5000.0f);
}

// =============================================================================
// Render
// =============================================================================
void drawFunc()
{
    GLint win_w, win_h;
    glfwGetWindowSize(window, &win_w, &win_h);
    aspectRatio = (float)win_w / (float)win_h;

    glClearColor(0.5f, 0.8f, 0.8f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glBindVertexArray(vao);
    glUseProgram(programID);

    GLuint loc;
    loc = glGetUniformLocation(programID, "modelMat");
    glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(matModel));
    loc = glGetUniformLocation(programID, "viewMat");
    glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(matView));
    loc = glGetUniformLocation(programID, "projMat");
    glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(matProj));

    if (textureID != 0)
    {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, textureID);
        loc = glGetUniformLocation(programID, "textureSampler");
        glUniform1i(loc, 0);
        loc = glGetUniformLocation(programID, "useTexture");
        glUniform1i(loc, 1);
    }
    else
    {
        loc = glGetUniformLocation(programID, "useTexture");
        glUniform1i(loc, 0);
    }

    glViewport(0, 0, win_w, win_h);
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)vertexCount);

    glFinish();
}

// =============================================================================
// Input Callbacks
// =============================================================================
void cursorPosFunc(GLFWwindow *win, double xscr, double yscr)
{
    glm::vec2 cur   = glm::vec2((float)xscr, (float)yscr);
    glm::vec2 delta = cur - lastMousePos;

    if (mouseMode == ROTATE)
    {
        cameraTheta -= delta.x * rotSpeed;
        cameraPhi   -= delta.y * rotSpeed;
        cameraPhi = glm::clamp(cameraPhi, 0.1f, glm::pi<float>() - 0.1f);
    }
    else if (mouseMode == PAN)
    {
        glm::vec3 cameraPos;
        cameraPos.x = cameraTarget.x + cameraRadius * sinf(cameraPhi) * cosf(cameraTheta);
        cameraPos.y = cameraTarget.y + cameraRadius * cosf(cameraPhi);
        cameraPos.z = cameraTarget.z + cameraRadius * sinf(cameraPhi) * sinf(cameraTheta);

        glm::vec3 forward = glm::normalize(cameraTarget - cameraPos);
        glm::vec3 right   = glm::normalize(glm::cross(forward, glm::vec3(0.f, 1.f, 0.f)));
        glm::vec3 up      = glm::normalize(glm::cross(right, forward));

        cameraTarget -= right * delta.x * panSpeed;
        cameraTarget += up    * delta.y * panSpeed;
    }

    lastMousePos = cur;
}

void mouseButtonFunc(GLFWwindow *win, int button, int action, int mods)
{
    GLdouble x, y;
    glfwGetCursorPos(win, &x, &y);
    lastMousePos = glm::vec2((float)x, (float)y);

    if (action == GLFW_PRESS)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT)  mouseMode = ROTATE;
        if (button == GLFW_MOUSE_BUTTON_RIGHT) mouseMode = PAN;
    }
    else if (action == GLFW_RELEASE)
        mouseMode = NONE;
}

void scrollFunc(GLFWwindow *win, double xoffset, double yoffset)
{
    cameraRadius -= (float)yoffset * zoomSpeed;
    cameraRadius  = glm::clamp(cameraRadius, 5.0f, 500.0f);
}

void keyFunc(GLFWwindow *win, int key, int scancode, int action, int mods)
{
    if (action != GLFW_PRESS) return;
    switch (key)
    {
    case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(win, GL_TRUE);
        break;
    case GLFW_KEY_J:
        fov = std::clamp(fov + 5.f, 1.f, 120.f);
        printf("FOV: %.1f\n", fov);
        break;
    case GLFW_KEY_K:
        fov = std::clamp(fov - 5.f, 1.f, 120.f);
        printf("FOV: %.1f\n", fov);
        break;
    }
}

// =============================================================================
// Main
// =============================================================================
int main(int argc, char *argv[])
{
    // 1. GLFW + window
    glfwInit();
    window = glfwCreateWindow(WIN_W, WIN_H, "Poisson Surface Reconstruction", NULL, NULL);
    glfwSetWindowPos(window, WIN_X, WIN_Y);
    glfwMakeContextCurrent(window);
    glewInit();

    glfwSetKeyCallback(window, keyFunc);
    glfwSetCursorPosCallback(window, cursorPosFunc);
    glfwSetMouseButtonCallback(window, mouseButtonFunc);
    glfwSetScrollCallback(window, scrollFunc);

    // 2. Load GLB
    bool res = loadGLB("../../resource/mesh.glb", loadedVertices, loadedUVs, loadedNormals, &textureID);
    if (!res)
    {
        printf("Failed to load GLB file!\n");
        return -1;
    }
    printf("Loaded %zu vertices\n", loadedVertices.size());
    if (textureID == 0)
        printf("No embedded texture found, using vertex colors\n");

    // 3. OpenGL init
    initGL();

    // 4. Render loop
    while (!glfwWindowShouldClose(window))
    {
        updateFunc();
        drawFunc();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

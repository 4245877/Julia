#ifndef MESH_H
#define MESH_H

#include <glad/glad.h> // содержит все объявления OpenGL-типов

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>



#include <string>
#include <vector>
using namespace std;

struct Vertex {

    // Позиция
    glm::vec3 Position;

    // Нормаль
    glm::vec3 Normal;

    // Текстурные координаты
    glm::vec2 TexCoords;

    // Касательный вектор
    glm::vec3 Tangent;

    // Вектор бинормали (вектор, перпендикулярный касательному вектору и вектору нормали)
    glm::vec3 Bitangent;
};

struct vertex {

    // Позиция
    glm::vec3 Position;

    // Нормаль
    glm::vec3 Normal;

    // Текстурные координаты
    glm::vec2 TexCoords;

    // Касательный вектор
    glm::vec3 Tangent;

    // Вектор бинормали (вектор, перпендикулярный касательному вектору и вектору нормали)
    glm::vec3 Bitangent;
};

struct Texture {
    unsigned int id;
    string type;
    string path;
};

class Mesh {
public:
    // Данные меша
    vector<Vertex> vertices;
    vector<unsigned int> indices;
    vector<Texture> textures;
    unsigned int VAO;

    // Конструктор
    Mesh(vector<Vertex> vertices, vector<unsigned int> indices, vector<Texture> textures)
    {
        this->vertices = vertices;
        this->indices = indices;
        this->textures = textures;

        // Теперь, когда у нас есть все необходимые данные, устанавливаем вершинные буферы и указатели атрибутов
        setupMesh();
    }

    // Рендеринг меша
    //void Draw(Shader& shader);

private:
    // Данные для рендеринга 
    unsigned int VBO, EBO;

    // Инициализируем все буферные объекты/массивы
    void setupMesh()
    {
        // Создаем буферные объекты/массивы
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);

        // Загружаем данные в вершинный буфер
        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        // Самое замечательное в структурах то, что расположение в памяти их внутренних переменных является последовательным.
        // Смысл данного трюка в том, что мы можем просто передать указатель на структуру, и она прекрасно преобразуется в массив данных с элементами типа glm::vec3 (или glm::vec2), который затем будет преобразован в массив данных float, ну а в конце – в байтовый массив
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

        // Устанавливаем указатели вершинных атрибутов

        // Координаты вершин
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

        // Нормали вершин
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));

        // Текстурные координаты вершин
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));

        // Касательный вектор вершины
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Tangent));

        // Вектор бинормали вершины
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Bitangent));

        glBindVertexArray(0);
    }
};
#endif
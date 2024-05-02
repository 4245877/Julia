// ResponseGenerator.h
#pragma once

#include "KnowledgeBase.h"

class ResponseGenerator {
public:
    ResponseGenerator(const KnowledgeBase& kb);
    std::string generateResponse(const std::string& input) const;

private:
    const KnowledgeBase& knowledgeBase;
};

// ResponseGenerator.cpp
#include "ResponseGenerator.h"

ResponseGenerator::ResponseGenerator(const KnowledgeBase& kb) : knowledgeBase(kb) {}

std::string ResponseGenerator::generateResponse(const std::string& input) const {
    return knowledgeBase.getFact(input); // это простейший пример, вы можете добавить сложную логику обработки ввода и поиска ответа
}

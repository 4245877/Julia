#pragma once

#include "KnowledgeBase.h"
#include "ResponseGenerator.h"

class AICore {
public:
    AICore();

    std::string processInput(const std::string& input);

private:
    KnowledgeBase knowledgeBase;
    ResponseGenerator responseGenerator;
};


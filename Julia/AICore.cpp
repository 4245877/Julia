#include "AICore.h"

AICore::AICore() : responseGenerator(knowledgeBase) {
    // Заполните базу данных примерами
    knowledgeBase.addFact("привет", "Привет, человек!");
    knowledgeBase.addFact("как дела?", "Я машина, у меня нет чувств, но я готов помогать!");
}

std::string AICore::processInput(const std::string& input) {
    return responseGenerator.generateResponse(input);
}

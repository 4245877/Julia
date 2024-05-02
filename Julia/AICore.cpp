#include "AICore.h"

AICore::AICore() : responseGenerator(knowledgeBase) {
    // ��������� ���� ������ ���������
    knowledgeBase.addFact("������", "������, �������!");
    knowledgeBase.addFact("��� ����?", "� ������, � ���� ��� ������, �� � ����� ��������!");
}

std::string AICore::processInput(const std::string& input) {
    return responseGenerator.generateResponse(input);
}

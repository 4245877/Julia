// KnowledgeBase.h
#pragma once

#include <string>
#include <map>

class KnowledgeBase {
public:
    void addFact(const std::string& key, const std::string& fact);
    std::string getFact(const std::string& key) const;

private:
    std::map<std::string, std::string> database;
};

// KnowledgeBase.cpp
#include "KnowledgeBase.h"

void KnowledgeBase::addFact(const std::string& key, const std::string& fact) {
    database[key] = fact;
}

std::string KnowledgeBase::getFact(const std::string& key) const {
    if (database.find(key) != database.end()) {
        return database.at(key);
    }
    return ""; // или вернуть сообщение об ошибке
}

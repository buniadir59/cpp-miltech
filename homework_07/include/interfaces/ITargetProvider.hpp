#pragma once

#include "dto/Target.hpp"

//Провайдер цілей: кількість та дані кожної цілі (позиція, швидкість)
class ITargetProvider {

public:
    virtual int getTargetCount() = 0;
    virtual dto::Target getTarget(int index) = 0;
    virtual ~ITargetProvider() {}
};
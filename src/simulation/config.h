#pragma once
#include <vector>
#include <variant>

struct ConfigFloatField { const char* name; float* value; float min, max; };
struct ConfigBoolField  { const char* name; bool*  value; };

using ConfigField = std::variant<ConfigFloatField, ConfigBoolField>;

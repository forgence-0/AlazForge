#pragma once
// Minimal, bağımlılıksız JSON okuyucu/yazıcı. Genel amaçlı bir JSON
// kütüphanesi DEĞİLDİR (nlohmann/rapidjson gibi bir dış bağımlılık
// eklemek yerine, AlazForge'un kendi sahne dosyası şemasını okuyup
// yazmaya yetecek kadar geniş bir alt küme): nesne, dizi, sayı, string,
// bool, null. Unicode kaçış dizileri (\\uXXXX) desteklenmez -- sahne
// dosyalarında gerek yok.

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace alazforge {

class JsonValue;
using JsonArray = std::vector<JsonValue>;
using JsonObject = std::map<std::string, JsonValue>;

enum class JsonType : uint8_t { Null, Bool, Number, String, Array, Object };

class JsonValue {
public:
    JsonValue() : type(JsonType::Null) {}
    JsonValue(bool b) : type(JsonType::Bool), boolValue(b) {}
    JsonValue(double n) : type(JsonType::Number), numberValue(n) {}
    JsonValue(int n) : type(JsonType::Number), numberValue(static_cast<double>(n)) {}
    JsonValue(const std::string& s) : type(JsonType::String), stringValue(s) {}
    JsonValue(const char* s) : type(JsonType::String), stringValue(s) {}
    JsonValue(const JsonArray& a) : type(JsonType::Array), arrayValue(a) {}
    JsonValue(const JsonObject& o) : type(JsonType::Object), objectValue(o) {}

    JsonType Type() const { return type; }
    bool IsNull() const { return type == JsonType::Null; }

    double AsNumber(double def = 0.0) const { return type == JsonType::Number ? numberValue : def; }
    bool AsBool(bool def = false) const { return type == JsonType::Bool ? boolValue : def; }
    const std::string& AsString() const { return stringValue; }
    const JsonArray& AsArray() const { return arrayValue; }
    const JsonObject& AsObject() const { return objectValue; }

    // Nesne alanı erişimi: yoksa Null JsonValue döner (fırlatmaz).
    const JsonValue& operator[](const std::string& key) const {
        static const JsonValue kNull;
        if (type != JsonType::Object) return kNull;
        auto it = objectValue.find(key);
        return it != objectValue.end() ? it->second : kNull;
    }

    // Yazım için: nesneye alan ekler (yalnızca Object tipinde anlamlıdır).
    void Set(const std::string& key, JsonValue value) {
        type = JsonType::Object;
        objectValue[key] = std::move(value);
    }

    static JsonValue MakeArray() {
        JsonValue v;
        v.type = JsonType::Array;
        return v;
    }
    void PushBack(JsonValue value) {
        type = JsonType::Array;
        arrayValue.push_back(std::move(value));
    }

    std::string Dump(int indent = 0) const;

private:
    JsonType type;
    bool boolValue = false;
    double numberValue = 0.0;
    std::string stringValue;
    JsonArray arrayValue;
    JsonObject objectValue;
};

// text'i ayrıştırır; başarısızsa false döner (outValue tanımsız kalır).
bool ParseJson(const std::string& text, JsonValue& outValue);

} // namespace alazforge

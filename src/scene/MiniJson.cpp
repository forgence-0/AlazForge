#include "scene/MiniJson.h"

#include <cctype>
#include <cstdio>
#include <sstream>

namespace alazforge {

namespace {

void DumpString(const std::string& s, std::ostringstream& out) {
    out << '"';
    for (char c : s) {
        switch (c) {
            case '"':
                out << "\\\"";
                break;
            case '\\':
                out << "\\\\";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                out << c;
        }
    }
    out << '"';
}

} // namespace

std::string JsonValue::Dump(int indent) const {
    std::ostringstream out;
    const std::string pad(static_cast<size_t>(indent) * 2, ' ');
    const std::string padIn(static_cast<size_t>(indent + 1) * 2, ' ');

    switch (type) {
        case JsonType::Null:
            out << "null";
            break;
        case JsonType::Bool:
            out << (boolValue ? "true" : "false");
            break;
        case JsonType::Number: {
            // Tam sayi degerleri (ornegin ObjectLayer, MaterialId) ondalikli
            // yazilmasin diye kontrol edilir; genel kayan-nokta degerler
            // icin yeterli hassasiyette yazilir.
            if (numberValue == static_cast<double>(static_cast<int64_t>(numberValue))) {
                out << static_cast<int64_t>(numberValue);
            } else {
                char buf[64];
                std::snprintf(buf, sizeof(buf), "%.9g", numberValue);
                out << buf;
            }
            break;
        }
        case JsonType::String:
            DumpString(stringValue, out);
            break;
        case JsonType::Array: {
            if (arrayValue.empty()) {
                out << "[]";
                break;
            }
            out << "[\n";
            for (size_t i = 0; i < arrayValue.size(); ++i) {
                out << padIn << arrayValue[i].Dump(indent + 1);
                if (i + 1 < arrayValue.size()) out << ",";
                out << "\n";
            }
            out << pad << "]";
            break;
        }
        case JsonType::Object: {
            if (objectValue.empty()) {
                out << "{}";
                break;
            }
            out << "{\n";
            size_t i = 0;
            for (const auto& [key, value] : objectValue) {
                out << padIn;
                DumpString(key, out);
                out << ": " << value.Dump(indent + 1);
                if (++i < objectValue.size()) out << ",";
                out << "\n";
            }
            out << pad << "}";
            break;
        }
    }
    return out.str();
}

namespace {

// Basit recursive-descent JSON ayrıştırıcı. Yalnızca AlazForge'un kendi
// yazdığı JSON'u okumak için yeterli olacak kadar sağlam -- hatalı girdide
// false döner, fırlatmaz.
class Parser {
public:
    explicit Parser(const std::string& inText) : text(inText) {}

    bool Parse(JsonValue& out) {
        SkipWhitespace();
        if (!ParseValue(out)) return false;
        SkipWhitespace();
        return true;
    }

private:
    const std::string& text;
    size_t pos = 0;

    bool AtEnd() const { return pos >= text.size(); }
    char Peek() const { return AtEnd() ? '\0' : text[pos]; }

    void SkipWhitespace() {
        while (!AtEnd() && std::isspace(static_cast<unsigned char>(text[pos])))
            ++pos;
    }

    bool ParseValue(JsonValue& out) {
        SkipWhitespace();
        if (AtEnd()) return false;
        const char c = Peek();
        if (c == '{') return ParseObject(out);
        if (c == '[') return ParseArray(out);
        if (c == '"') {
            std::string s;
            if (!ParseString(s)) return false;
            out = JsonValue(s);
            return true;
        }
        if (c == 't' || c == 'f') return ParseBool(out);
        if (c == 'n') return ParseNull(out);
        return ParseNumber(out);
    }

    bool ParseObject(JsonValue& out) {
        if (Peek() != '{') return false;
        ++pos;
        JsonObject obj;
        SkipWhitespace();
        if (Peek() == '}') {
            ++pos;
            out = JsonValue(obj);
            return true;
        }
        while (true) {
            SkipWhitespace();
            std::string key;
            if (Peek() != '"' || !ParseString(key)) return false;
            SkipWhitespace();
            if (Peek() != ':') return false;
            ++pos;
            JsonValue value;
            if (!ParseValue(value)) return false;
            obj[key] = std::move(value);
            SkipWhitespace();
            if (Peek() == ',') {
                ++pos;
                continue;
            }
            if (Peek() == '}') {
                ++pos;
                break;
            }
            return false;
        }
        out = JsonValue(obj);
        return true;
    }

    bool ParseArray(JsonValue& out) {
        if (Peek() != '[') return false;
        ++pos;
        JsonArray arr;
        SkipWhitespace();
        if (Peek() == ']') {
            ++pos;
            out = JsonValue(arr);
            return true;
        }
        while (true) {
            JsonValue value;
            if (!ParseValue(value)) return false;
            arr.push_back(std::move(value));
            SkipWhitespace();
            if (Peek() == ',') {
                ++pos;
                continue;
            }
            if (Peek() == ']') {
                ++pos;
                break;
            }
            return false;
        }
        out = JsonValue(arr);
        return true;
    }

    bool ParseString(std::string& out) {
        if (Peek() != '"') return false;
        ++pos;
        std::string s;
        while (!AtEnd() && text[pos] != '"') {
            char c = text[pos++];
            if (c == '\\' && !AtEnd()) {
                const char esc = text[pos++];
                switch (esc) {
                    case '"':
                        s += '"';
                        break;
                    case '\\':
                        s += '\\';
                        break;
                    case '/':
                        s += '/';
                        break;
                    case 'n':
                        s += '\n';
                        break;
                    case 't':
                        s += '\t';
                        break;
                    default:
                        s += esc;
                }
            } else {
                s += c;
            }
        }
        if (AtEnd()) return false; // kapanis tirnagi bulunamadi
        ++pos;                     // kapanis '"'
        out = std::move(s);
        return true;
    }

    bool ParseNumber(JsonValue& out) {
        const size_t start = pos;
        if (Peek() == '-') ++pos;
        while (!AtEnd() && std::isdigit(static_cast<unsigned char>(text[pos])))
            ++pos;
        if (!AtEnd() && text[pos] == '.') {
            ++pos;
            while (!AtEnd() && std::isdigit(static_cast<unsigned char>(text[pos])))
                ++pos;
        }
        if (!AtEnd() && (text[pos] == 'e' || text[pos] == 'E')) {
            ++pos;
            if (!AtEnd() && (text[pos] == '+' || text[pos] == '-')) ++pos;
            while (!AtEnd() && std::isdigit(static_cast<unsigned char>(text[pos])))
                ++pos;
        }
        if (pos == start) return false;
        out = JsonValue(std::stod(text.substr(start, pos - start)));
        return true;
    }

    bool ParseBool(JsonValue& out) {
        if (text.compare(pos, 4, "true") == 0) {
            pos += 4;
            out = JsonValue(true);
            return true;
        }
        if (text.compare(pos, 5, "false") == 0) {
            pos += 5;
            out = JsonValue(false);
            return true;
        }
        return false;
    }

    bool ParseNull(JsonValue& out) {
        if (text.compare(pos, 4, "null") == 0) {
            pos += 4;
            out = JsonValue();
            return true;
        }
        return false;
    }
};

} // namespace

bool ParseJson(const std::string& text, JsonValue& outValue) {
    Parser parser(text);
    return parser.Parse(outValue);
}

} // namespace alazforge

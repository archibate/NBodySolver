#pragma once

#include <string_view>
#include <functional>
#include <charconv>
#include <variant>
#include <stack>
#include <map>
#include "Optional.h"

struct ConfigParser {
    struct Variant {
        std::variant<std::string_view, std::map<std::string_view, std::vector<Variant>>> v;

        Variant() = default;

        explicit Variant(decltype(v) v) : v(std::move(v)) {}

        static Variant makeAtom(std::variant_alternative_t<0, decltype(v)> x) {
            return Variant{decltype(v){x}};
        }

        static Variant makeDict(std::variant_alternative_t<1, decltype(v)> x) {
            return Variant{decltype(v){x}};
        }

        static Variant makeAtom() {
            return Variant{decltype(v){std::in_place_index<0>}};
        }

        static Variant makeDict() {
            return Variant{decltype(v){std::in_place_index<1>}};
        }

        auto &getAtom() {
            return std::get<0>(v);
        }

        auto &getDict() {
            return std::get<1>(v);
        }

        auto &getAtom() const {
            return std::get<0>(v);
        }

        bool isDouble() const {
            auto sv = getAtom();
            stripStringViewAsNumeric(sv);
            double ret = 0.0;
            return std::from_chars(sv.data(), sv.data() + sv.size(), ret).ec == std::errc();
        }

        bool isInt() const {
            auto sv = getAtom();
            stripStringViewAsNumeric(sv);
            int ret = 0.0;
            return std::from_chars(sv.data(), sv.data() + sv.size(), ret).ec == std::errc();
        }

        double getDouble() const {
            auto sv = getAtom();
            stripStringViewAsNumeric(sv);
            double ret = 0.0;
            if (std::from_chars(sv.data(), sv.data() + sv.size(), ret).ec != std::errc())
                throw;
            return ret;
        }

        int getInt() const {
            auto sv = getAtom();
            stripStringViewAsNumeric(sv);
            int ret = 0;
            if (std::from_chars(sv.data(), sv.data() + sv.size(), ret).ec != std::errc())
                throw;
            return ret;
        }

        auto &getDict() const {
            return std::get<1>(v);
        }

        bool isAtom() const {
            return v.index() == 0;
        }

        bool isDict() const {
            return v.index() == 1;
        }

        auto &dictEntry(std::string_view key) const {
            auto &vs = getDict().at(key);
            //std::cout << "=====";
            //for (auto const &[k, vs]: getDict()) {
                //for (auto const &v: vs) {
                    //if (v.isAtom()) {
                        //std::cout << k << ' ' << v.getAtom() << '\n';
                    //}
                //}
            //}
            //std::cout << "=====";
            return vs.at(vs.size() - 1);
        }

        auto &dictEntry(std::string_view key, size_t index) const {
            return getDict().at(key).at(index);
        }

        Optional<Variant const *> dictEntrySafe(std::string_view key, size_t index = 0) const {
            auto it = getDict().find(key);
            if (it != getDict().end() && it->second.size() > index) {
                return {&it->second[index]};
            }
            return std::nullopt;
        }

        void dictEntryForEach(std::string_view key, std::function<void(size_t, Variant const &)> const &yield) const {
            auto it = getDict().find(key);
            if (it == getDict().end()) return;
            size_t index = 0;
            for (auto const &v: it->second) {
                yield(index, v);
                index++;
            }
        }

        size_t dictEntryCount(std::string_view key) const {
            auto it = getDict().find(key);
            return it != getDict().end() ? it->second.size() : 0;
        }

        void dictForEach(std::function<void(std::string_view, Variant const &)> const &yield) const {
            for (auto const &[k, vs]: getDict()) {
                for (auto const &v: vs) {
                    yield(k, v);
                }
            }
        }
    };

    Variant result = Variant::makeDict();

    static void parseInto(Variant &result, std::string_view sv) {
        std::map<std::string_view, std::vector<Variant>> &currDict = result.getDict();
        while (!sv.empty()) {
            size_t offset = sv.find_first_of("{=");
            if (offset == sv.npos) {
                break;
            }
            std::string_view key = sv;
            key.remove_suffix(sv.size() - offset);
            char ch = sv[offset];
            sv.remove_prefix(offset + 1);
            if (ch == '{') {
                size_t endpos = 0;
                size_t braces = 1;
                while (braces != 0) {
                    endpos = sv.find_first_of("{}", endpos);
                    if (endpos == sv.npos) {
                        break;
                    } else if (sv[endpos] == '}') {
                        braces--;
                    } else { // sv[endpos] == '{'
                        braces++;
                    }
                    endpos++;
                }
                std::string_view val = sv;
                if (endpos != sv.npos) {
                    val.remove_suffix(sv.size() - endpos);
                    sv.remove_prefix(endpos + 1);
                } else {
                    sv.remove_prefix(sv.size());
                }
                stripStringView(key);
                Variant out = Variant::makeDict();
                parseInto(out, val);
                currDict[key].push_back(std::move(out));
            } else { // ch == '='
                size_t endpos = sv.find_first_of("\r\n");
                std::string_view val = sv;
                if (endpos != sv.npos) {
                    val.remove_suffix(sv.size() - endpos);
                    sv.remove_prefix(endpos + 1);
                } else {
                    sv.remove_prefix(sv.size());
                }
                stripStringView(key);
                stripStringView(val);
                currDict[key].push_back(Variant::makeAtom(val));
            }
        }
    }

    static void stripStringViewAsNumeric(std::string_view &sv) {
        size_t begpos = sv.find_first_of("1234567890-.");
        size_t endpos = sv.find_first_not_of("1234567890e+-.");
        if (begpos == sv.npos) {
            sv.remove_prefix(sv.size());
        } else {
            if (endpos != sv.npos) {
                sv.remove_suffix(sv.size() - endpos - 1);
            }
            sv.remove_prefix(begpos);
        }
    }

    static void stripStringView(std::string_view &sv) {
        size_t begpos = sv.find_first_not_of(" \r\n\t\v\f");
        size_t endpos = sv.find_last_not_of(" \r\n\t\v\f");
        if (endpos == sv.npos || begpos == sv.npos) {
            sv.remove_prefix(sv.size());
        } else {
            sv.remove_suffix(sv.size() - endpos - 1);
            sv.remove_prefix(begpos);
        }
    }

    Variant const &getConfig() const {
        return result;
    }

    ConfigParser &parse(std::string_view sv) {
        parseInto(result, sv);
        return *this;
    }
};

    //config.dictForEach(SelfKnowingFunctor{[] (auto const &self, auto key, auto const &val) -> void {
        //if (val.isAtom()) {
            //if (val.isDouble()) {
                //std::cout << key << '=' << val.getDouble() << '\n';
            //} else {
                //std::cout << key << '=' << std::quoted(val.getAtom()) << '\n';
            //}
        //} else {
            //std::cout << key << '{' << '\n';
            //val.dictForEach(self);
            //std::cout << '}' << '\n';
        //}
    //}});

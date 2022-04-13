#define __DEGUB_ENABLE__
#ifdef __DEGUB_ENABLE__
template<int MAX = 100>
class DebugItem {
public:
    static DebugItem<MAX>* items[MAX];
    static long itemCount;
    virtual void print() = 0;
    static void printAll()
    {
        for (int i = 0; i < itemCount; i++) {
            items[i]->print();
        }
    }
};

template<int MAX>
long DebugItem<MAX>::itemCount = 0;

template<int MAX>
DebugItem<MAX>* DebugItem<MAX>::items[MAX];

template<typename T>
class TemplatedDebugItem : public DebugItem<> {
public:
    T* value;
    const char* name;
    TemplatedDebugItem(T* value, const char* name)
        : value(value), name(name)
    {
        DebugItem<>::items[DebugItem<>::itemCount++] = (DebugItem<>*)this;
    }

    virtual void print()
    {
        Serial.print(name);
        Serial.print(": ");
        Serial.print("addr=");
        Serial.print((unsigned long)value, HEX);
        if (value != nullptr) {
            Serial.print("value=");
            Serial.print(*value);
        }
        Serial.print("\n");
    }
};

#define BREAK Serial.print("-----------------------------------------------\n"); \
    Serial.print(__FILE__); \
    Serial.print(":"); \
    Serial.print(__LINE__); \
    Serial.println(": BREAK"); \
    DebugItem<>::printAll(); \
    Serial.println("-----------------------------------------------"); while(Serial.available() == 0){} while (Serial.available()) {Serial.read();} (void)0

#define REG_VAR(var) static auto __dbg_ ## var = new TemplatedDebugItem<decltype(var)>(&var, #var)
#else
#define BREAK
#define REG_VAR(var)
#endif

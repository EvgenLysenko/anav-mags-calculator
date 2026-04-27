#ifndef __CSV_PARSER_H__
#define __CSV_PARSER_H__

class CsvParser
{
public:
    typedef unsigned int (*ReaderFunction)(const char** buf, void* data);

    CsvParser(ReaderFunction readFunction, void* data);
    CsvParser(const char* buf, unsigned int size);

protected:
    ReaderFunction readFunction;
    void* readFunctionData;
    const char* buf;
    unsigned int left;

    void set(const char* buf, unsigned int bufSize);

    char getNextChar();

    const char* current;
    char getCurrentChar() {
        return *current;
    }

    bool endLineDetected;
    bool endDetected;
    bool wasAnyValue;

    char moveToNextValue();
    char moveToNonWhitespace();
    char moveToNextNonWhitespaceValue();

public:
    static bool isWhitespace(char c) {
        return c == ' ' || c == '\t';
    }

    static bool isEndline(char c) {
        return c == '\n' || c == '\r';
    }

    char nextLine();

    bool isEndLineDetected() const { return endLineDetected; }
    bool isEnd() const { return endDetected; }

    unsigned int nextString(char* buf, unsigned int bufSize);
    bool nextInt(int& value);
    bool nextFloat(float& value);
    bool nextDouble(double& value);

    int nextInt() {
        int value = 0;
        nextInt(value);
        return value;
    }

    float nextFloat() {
        float value = 0;
        nextFloat(value);
        return value;
    }

    double nextDouble() {
        double value = 0;
        nextDouble(value);
        return value;
    }
};

#endif

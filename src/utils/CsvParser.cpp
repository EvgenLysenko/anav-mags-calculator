#include "CsvParser.h"

static const float floatMultipliers[] = {
    0.1f,
    0.01f,
    0.001f,
    0.0001f,
    0.00001f,
    0.000001f,
    0.0000001f,
    0.00000001f,
    0.000000001f,
    0.0000000001f,
};

static const double doubleMultipliers[] = {
    0.1,
    0.01,
    0.001,
    0.0001,
    0.00001,
    0.000001,
    0.0000001,
    0.00000001,
    0.000000001,
    0.0000000001,
    0.00000000001,
    0.000000000001,
    0.0000000000001,
    0.00000000000001,
    0.000000000000001,
    0.0000000000000001,
    0.00000000000000001,
};

CsvParser::CsvParser(CsvParser::ReaderFunction readFunction, void* data)
    : readFunction(readFunction)
    , readFunctionData(data)
    , buf(0)
    , endLineDetected(false)
    , endDetected(false)
    , wasAnyValue(false)
{
    left = readFunction(&buf, data);
    current = buf;

    if (!left)
        endDetected = true;
}

CsvParser::CsvParser(const char* buf, unsigned int size)
    : readFunction(0)
    , readFunctionData(0)
    , buf(buf)
    , endLineDetected(false)
    , endDetected(false)
    , wasAnyValue(false)
{
    set(buf, size);
}

void CsvParser::set(const char* buf, unsigned int bufSize)
{
    this->buf = buf;
    this->current = buf;
    this->left = bufSize;
    this->endDetected = false;
    this->endLineDetected = false;
    this->wasAnyValue = false;

    if (!this->left)
        this->endDetected = true;
}

char CsvParser::getNextChar()
{
    if (left > 1) {
        --left;
        return *++current;
    }
    else if (left == 1) {
        if (readFunction) {
            left = readFunction(&buf, readFunctionData);
            current = buf;

            if (left) {
                return *current;
            }
            else {
                endDetected = true;
                return 0;
            }
        }
        else {
            left = 0;
            current = 0;
            endLineDetected = true;
            endDetected = true;
            return 0;
        }
    }
    else {
        endDetected = true;
        return 0;
    }
}

char CsvParser::moveToNextValue()
{
    char c = getCurrentChar();

    if (wasAnyValue) {
        if (c == ',')
            return getNextChar();

        while (left) {
            if (c == ',') {
                return getNextChar();
            }
            else if (isEndline(c)) {
                endLineDetected = true;
                return c;
            }

            c = getNextChar();
        }
    }

    return c;
}

char CsvParser::moveToNonWhitespace()
{
    char c = getCurrentChar();

    while (left && (isWhitespace(c) || c == '\0')) {
        c = getNextChar();
    }

    return c;
}

char CsvParser::moveToNextNonWhitespaceValue()
{
    if (endLineDetected || endDetected)
        return 0;

    char c = moveToNextValue();
    c = moveToNonWhitespace();

    return c;
}

char CsvParser::nextLine()
{
    if (endDetected)
        return 0;

    wasAnyValue = false;
    endLineDetected = false;

    char c = getCurrentChar();
    char endChar = 0;

    while (left) {
        if (isEndline(c)) {
            if (endChar == 0) {
                endChar = c;
            }
            // \r\n or \n\r - one line
            else if (endChar != c) {
                return getNextChar();
            }
            // \r\r or \n\n - two lines
            else {
                return c;
            }
        }
        else if (endChar) {
            return c;
        }
        else if (c == '\0') {
            return getNextChar();
        }

        c = getNextChar();
    }

    return c;
}

unsigned int CsvParser::nextString(char* buf, unsigned int bufSize)
{
    char c = moveToNextNonWhitespaceValue();
    if (endLineDetected || endDetected)
        return 0;

    unsigned int readSize = 0;
    unsigned int spacesCount = 0;

    while (c && bufSize) {
        if (c == ',') {
            break;
        }
        else if (c == '\n' || c == '\r') {
            endLineDetected = true;
            break;
        }
        else {
            if (isWhitespace(c))
                ++spacesCount;
            else
                spacesCount = 0;

            *buf++ = c;
            ++readSize;
            --bufSize;
        }

        c = getNextChar();
    }

    wasAnyValue = true;

    return readSize - spacesCount;
}

bool CsvParser::nextInt(int& value)
{
    char c = moveToNextNonWhitespaceValue();
    if (endLineDetected || endDetected)
        return false;

    const bool isNegative = c == '-';
    if (isNegative) {
        c = getNextChar();
    }

    bool valuePresent = false;

    while (c) {
        if (c >= '0' && c <= '9') {
            if (!valuePresent) {
                value = 0;
                valuePresent = true;
            }

            value *= 10;
            value += c - '0';
        }
        else {
            break;
        }

        c = getNextChar();
    }

    wasAnyValue = true;

    if (isNegative)
        value = -value;

    return valuePresent;
}

bool CsvParser::nextFloat(float& value)
{
    char c = moveToNextNonWhitespaceValue();
    if (endLineDetected || endDetected)
        return false;

    const bool isNegative = c == '-';
    if (isNegative) {
        c = getNextChar();
    }

    bool valuePresent = false;
    bool wasComma = false;
    float multiplier = 1;
    int multiplierIdx = 0;

    while (c) {
        if (c >= '0' && c <= '9') {
            if (!valuePresent) {
                value = 0;
                valuePresent = true;
            }

            if (wasComma) {
                if (multiplierIdx < sizeof(floatMultipliers) / sizeof(*floatMultipliers))
                    multiplier = floatMultipliers[multiplierIdx++];
                else
                    multiplier /= 10;
            }

            value *= 10;
            value += c - '0';
        }
        else if (c >= '.')
        {
            if (!valuePresent) {
                value = 0;
                valuePresent = true;
            }

            if (!wasComma) {
                wasComma = true;
            }
            else {
                break;
            }
        }
        else {
            break;
        }

        c = getNextChar();
    }

    if (multiplier != 1) {
        value *= multiplier;
    }

    wasAnyValue = true;

    if (isNegative)
        value = -value;

    return valuePresent;
}

bool CsvParser::nextDouble(double& value)
{
    char c = moveToNextNonWhitespaceValue();
    if (endLineDetected || endDetected)
        return false;

    const bool isNegative = c == '-';
    if (isNegative) {
        c = getNextChar();
    }

    bool valuePresent = false;
    bool wasComma = false;
    double multiplier = 1;
    int multiplierIdx = 0;

    while (c) {
        if (c >= '0' && c <= '9') {
            if (!valuePresent) {
                value = 0;
                valuePresent = true;
            }

            if (wasComma) {
                if (multiplierIdx < sizeof(doubleMultipliers) / sizeof(*doubleMultipliers))
                    multiplier = doubleMultipliers[multiplierIdx++];
                else
                    multiplier /= 10;
            }

            value *= 10;
            value += c - '0';
        }
        else if (c >= '.')
        {
            if (!valuePresent) {
                value = 0;
                valuePresent = true;
            }

            if (!wasComma) {
                wasComma = true;
            }
            else {
                break;
            }
        }
        else {
            break;
        }

        c = getNextChar();
    }

    if (multiplier != 1) {
        value *= multiplier;
    }

    wasAnyValue = true;

    if (isNegative)
        value = -value;

    return valuePresent;
}

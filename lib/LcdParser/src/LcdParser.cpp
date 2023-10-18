#include "LcdParser.h"

LcdParser::LcdParser() {}

void LcdParser::parseLcd() {
    for (unsigned digit = 0; digit < DIGITS_IN_LCD; digit++) {
        bool flag = false;
        for (unsigned wire = 0; wire < WIRES_IN_LCD; wire++)
            for (unsigned bit = 0; bit < BITS_IN_PACKAGE; bit++)
                if (wiresShift[digit][wire][bit] != 0xFF)
                    digits[digit] |= (wiresData[wire][bit] == LCD_HIGH)
                                     << wiresShift[digit][wire][bit];
        for (unsigned digitPrint = 0; digitPrint < sizeof(printableDigits);
             digitPrint++)
            if (digits[digit] == printableDigits[digitPrint]) {
                digits[digit] = digitPrint + '0';
                flag = true;
                break;
            }
        if (!flag)
            digits[digit] = 'X';
    }
    snprintf(Data.SYS, 4U, "%c%c%c", digits[0U], digits[1U], digits[2U]);
    snprintf(Data.DIA, 4U, "%c%c%c", digits[3U], digits[4U], digits[5U]);
    snprintf(Data.PUL, 4U, "%c%c%c", digits[6U], digits[7U], digits[8U]);
}

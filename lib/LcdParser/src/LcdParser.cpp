#include "LcdParser.h"

LcdParser::LcdParser() {}

void LcdParser::parseLcd() {
    for (unsigned counter = 0; counter < DIGITS_IN_LCD; counter++) {
        char symbol = ' ';
        bool flag = false;
        for (unsigned wire = 0; wire < WIRES_IN_LCD; wire++)
            for (unsigned bit = 0; bit < BITS_IN_PACKAGE; bit++)
                if (wiresShift[counter][wire][bit] != 0xFF)
                    symbols[counter] |= (wiresData[wire][bit] == LCD_HIGH)
                                        << wiresShift[counter][wire][bit];
        for (unsigned symbolPrint = 0; symbolPrint < sizeof(printableSymbols);
             symbolPrint++)
            if (symbols[counter] == printableSymbols[symbolPrint]) {
                symbol = alphabet[symbolPrint];
                break;
            }
        symbols[counter] = symbol;
    }
    snprintf(Data.SYS, 4U, "%c%c%c", symbols[0U], symbols[1U], symbols[2U]);
    snprintf(Data.DIA, 4U, "%c%c%c", symbols[3U], symbols[4U], symbols[5U]);
    snprintf(Data.PUL, 4U, "%c%c%c", symbols[6U], symbols[7U], symbols[8U]);
}

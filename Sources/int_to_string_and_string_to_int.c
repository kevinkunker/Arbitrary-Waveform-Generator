/* int_to_string_and_string_to_int.c
 * Author: Kevin Kunker
 * 4/10/21
 */

#include <stdint.h>
#include "int_to_string_and_string_to_int.h"

#define MAX_DIGITS 10

void uint32_to_ascii(uint32_t data, char *string){
        uint32_t digits[MAX_DIGITS] = {0};
        int currentDigit = MAX_DIGITS-1;
        
        do {
                digits[currentDigit--] = data % 10;
                data /= 10;
        } while(data != 0);
        
        int i, firstDigit = 9;
        for(i = 0; i < MAX_DIGITS; i++){
                if(digits[i] != 0){
                        firstDigit = i;                     
                        break;
                }
        }
        
        for(i = firstDigit; i < MAX_DIGITS; i++){
                string[i-firstDigit] = '0'+digits[i];
        }
        string[i-firstDigit] = '\0';
}

uint32_t ascii_to_uint32(char *string){
        char buffer[MAX_DIGITS] = {0};
        
        int i = 0, j = 0;
        while(j < MAX_DIGITS && string[i] != '\0'){
                if(string[i] >= '0' && string[i] <= '9'){
                        buffer[j++] = string[i];
                }
                i++;
        }
        int numDigits = j;
        
        uint32_t result = 0, multiplier = 1;
        for(i = numDigits-1; i >= 0; i--){
                int addend = buffer[i]-'0';
                result += addend*multiplier;
                multiplier *= 10;
        }
        return result;
}


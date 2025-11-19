#ifndef XJ_AES128_H
#define XJ_AES128_H


#include <stdint.h>
#include <stdio.h>
#include <string.h>


extern int aes128_encrypt(const uint8_t *key, uint32_t keyLen, const uint8_t *pt, uint8_t *ct, uint32_t len);
extern int aes128_decrypt(const uint8_t *key, uint32_t keyLen, const uint8_t *ct, uint8_t *pt, uint32_t len);
  

#endif  

#ifndef _AES_ZL_H
#define _AES_ZL_H
#include <stdbool.h>

#define RoundKeyLEN  300
#define BLUE_SOFTKEY 1

/**
 * 参数 mac_addr: 设备的MAC地址。
 */
bool InitAES(char* mac_addr);

/**
 * 参数 userplain: 明文数组。
 * 参数 cipher: 密文数组。
 */
void AES(char* userplain, char* ciphertext);

/**
 * 参数 ciphertext: 密文数组。
 * 参数 decipher: 解密后数组。
 */
void DeAES(char* decipher, char* ciphertext);
#endif

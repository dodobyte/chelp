#ifndef __CHELPER_H
#define __CHELPER_H

#include <windows.h>

// dns.c
BOOL RetrieveDNS(LPWSTR pszAdapterName, CHAR *dns);

// lz4.c
int LZ4_decompress_safe(const char* src, char* dst, int compressedSize, int dstCapacity);

// shortcut.cpp
HRESULT CreateShortcut(LPCWSTR lpszPathObj, LPCWSTR lpszPathLink);
HRESULT ResolveShortcut(LPCWSTR lpszLinkFile, LPWSTR lpszPath, DWORD dwPathLen);

#endif
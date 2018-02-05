#ifndef __CHELPER_H
#define __CHELPER_H

#include <windows.h>

// dns.c
BOOL RetrieveDNS(LPWSTR pszAdapterName, CHAR *dns);

// lz4.c
int LZ4_decompress_safe(const char* src, char* dst, int compressedSize, int dstCapacity);

// shortcut.cpp
HRESULT CreateShortcut(WCHAR *path, WCHAR *link);
HRESULT ResolveShortcut(
	WCHAR *link,
	WCHAR *target,
	int ntarget,
	WCHAR *args,
	int nargs,
	WCHAR *icon,
	int nicon
	);

#endif
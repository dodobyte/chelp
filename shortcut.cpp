#define UNICODE

#include <windows.h>
#include <shobjidl.h>
#include <shlguid.h>
#include <strsafe.h>

extern "C"
HRESULT CreateShortcut(LPCWSTR lpszPathObj, LPCWSTR lpszPathLink)
{
	HRESULT hres;
	BOOL init = FALSE;
	IShellLink* psl = NULL;
	IPersistFile* ppf = NULL;

	hres = CoInitialize(NULL);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}
	init = TRUE;

	// Get a pointer to the IShellLink interface.
	hres = CoCreateInstance(
			CLSID_ShellLink,
			NULL,
			CLSCTX_INPROC_SERVER,
			IID_IShellLink,
			(LPVOID*)&psl
		);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	// Set the path to the shortcut target.
	hres = psl->SetPath(lpszPathObj);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	// Query IShellLink for the IPersistFile interface,
	// used for saving the shortcut in persistent storage.
	hres = psl->QueryInterface(IID_IPersistFile, (LPVOID*)&ppf);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	// Save the link by calling IPersistFile::Save.
	hres = ppf->Save(lpszPathLink, TRUE);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

Exit:
	if (ppf) {
		ppf->Release();
	}
	if (psl) {
		psl->Release();
	}
	if (init) {
		CoUninitialize();
	}
	return hres;
}

extern "C"
HRESULT ResolveShortcut(LPCWSTR lpszLinkFile, LPWSTR lpszPath, DWORD dwPathLen)
{
	HRESULT hres;
	BOOL init = FALSE;
	IShellLink* psl = NULL;
	IPersistFile* ppf = NULL;
	WIN32_FIND_DATA wfd = {0};
	WCHAR szGotPath[MAX_PATH] = {0};

	hres = CoInitialize(NULL);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}
	init = TRUE;

	// Get a pointer to the IShellLink interface.
	hres = CoCreateInstance(
			CLSID_ShellLink,
			NULL,
			CLSCTX_INPROC_SERVER,
			IID_IShellLink,
			(LPVOID*)&psl
		);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	// Get a pointer to the IPersistFile interface.
	hres = psl->QueryInterface(IID_IPersistFile, (void**)&ppf);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	// Load the shortcut.
	hres = ppf->Load(lpszLinkFile, STGM_READ);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	// Resolve the link.
	hres = psl->Resolve(NULL, 0);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	// Get the path to the link target.
	hres = psl->GetPath(szGotPath, MAX_PATH, (WIN32_FIND_DATA*)&wfd, 0);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	hres = StringCbCopy(lpszPath, dwPathLen * sizeof(WCHAR), szGotPath);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

Exit:
	if (ppf) {
		ppf->Release();
	}
	if (psl) {
		psl->Release();
	}
	if (init) {
		CoUninitialize();
	}
	return hres;
}
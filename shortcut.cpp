#define UNICODE

#include <windows.h>
#include <shlobj.h>
#include <shobjidl.h>
#include <shlguid.h>

extern "C"
HRESULT CreateShortcut(WCHAR *path, WCHAR *link)
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
	hres = psl->SetPath(path);
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
	hres = ppf->Save(link, TRUE);
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
HRESULT ResolveShortcut(
	WCHAR *link,
	WCHAR *target,
	int ntarget,
	WCHAR *args,
	int nargs,
	WCHAR *icon,
	int nicon
	)
{
	HRESULT hres;
	int iconidx = 0;
	BOOL init = FALSE;
	IShellLink* psl = NULL;
	IPersistFile* ppf = NULL;
	WIN32_FIND_DATA wfd = {0};

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
	hres = ppf->Load(link, STGM_READ);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	// Resolve the link.
	hres = psl->Resolve(NULL, SLR_NO_UI);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	// Get the path to the link target.
	hres = psl->GetPath(target, ntarget, (WIN32_FIND_DATA*)&wfd, 0);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	// Get the arguments of the link.
	hres = psl->GetArguments(args, nargs);
	if (!SUCCEEDED(hres)) {
		goto Exit;
	}

	// Get the icon location of the shortcut.
	hres = psl->GetIconLocation(icon, nicon, &iconidx);
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
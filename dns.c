#include <windows.h>
#include <dhcpcsdk.h>

BOOL RetrieveDNS(LPWSTR pszAdapterName, CHAR *dns)
{
    DWORD dwError, dwSize;
	CHAR TmpBuffer[64];
	DHCPCAPI_PARAMS DNSParams = {
			0,							// Flags
			OPTION_DOMAIN_NAME_SERVERS,	// OptionId
			FALSE,						// vendor specific?
			NULL,						// data filled in on return
			0							// nBytes
		};
	DHCPCAPI_PARAMS_ARRAY RequestParams = {
			1,							// only one option to request
			&DNSParams
		};
	DHCPCAPI_PARAMS_ARRAY SendParams = {
			0,
			NULL
		};
	dwSize = sizeof(TmpBuffer);
	dwError = DhcpRequestParams(
			DHCPCAPI_REQUEST_SYNCHRONOUS,	// Flags
			NULL,							// Reserved
			pszAdapterName,					// Adapter Name
			NULL,							// not using class id
			SendParams,						// sent parameters
			RequestParams,					// requesting params
			(PBYTE) TmpBuffer,				// buffer
			&dwSize,						// buffer size
			NULL							// Request ID
		);
	if (dwError != NO_ERROR) {
		return FALSE;
	}
	if (DNSParams.nBytesData == 0) {
		return FALSE;
	}
	CopyMemory(dns, DNSParams.Data, DNSParams.nBytesData);
	return TRUE;
}

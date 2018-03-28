package chelp

//#cgo LDFLAGS: -ldhcpcsvc -static
//#include "chelp.h"
import "C"
import (
	"net"
	"syscall"
	"unsafe"
)

func RetrieveDNS(adapter string) []string {
	s, err := syscall.UTF16PtrFromString(adapter)
	if err != nil {
		return nil
	}
	dns := make([]byte, 64)
	ptr := unsafe.Pointer(&dns[0])
	ret := C.RetrieveDNS((*C.WCHAR)(s), (*C.CHAR)(ptr))
	if ret != C.BOOL(1) {
		return nil
	}
	return []string{net.IP(dns[:4]).String(), net.IP(dns[4:8]).String()}
}

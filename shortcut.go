package chelp

//#cgo LDFLAGS: -lole32 -loleaut32 -luuid
//#include "chelp.h"
import "C"
import (
	"errors"
	"fmt"
	"syscall"
)

func CreateShortcut(path, link string) error {
	p, err := syscall.UTF16PtrFromString(path)
	if err != nil {
		return err
	}
	l, err := syscall.UTF16PtrFromString(link)
	if err != nil {
		return err
	}
	hres := C.CreateShortcut((*C.WCHAR)(p), (*C.WCHAR)(l))
	if hres != C.S_OK {
		hr := uint32(hres)
		s := fmt.Sprintf("CreateShortcut: %s %s HRESULT=%X\n", path, link, hr)
		return errors.New(s)
	}
	return nil
}

func ResolveShortcut(link string) (string, error) {
	l, err := syscall.UTF16PtrFromString(link)
	if err != nil {
		return "", err
	}
	n := C.MAX_PATH
	path := make([]uint16, n)
	hres := C.ResolveShortcut((*C.WCHAR)(l), (*C.WCHAR)(&path[0]), C.DWORD(n))
	if hres != C.S_OK {
		hr := uint32(hres)
		s := fmt.Sprintf("ResolveShortcut: %s HRESULT=%X\n", link, hr)
		return "", errors.New(s)
	}
	return syscall.UTF16ToString(path), nil
}

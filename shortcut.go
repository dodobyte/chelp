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

func ResolveShortcut(link string) (target, icon, args string, err error) {
	l, err := syscall.UTF16PtrFromString(link)
	if err != nil {
		return
	}
	n := 1024
	trg := make([]uint16, n)
	arg := make([]uint16, n)
	ico := make([]uint16, n)
	hres := C.ResolveShortcut((*C.WCHAR)(l),
		(*C.WCHAR)(&trg[0]), C.int(n),
		(*C.WCHAR)(&arg[0]), C.int(n),
		(*C.WCHAR)(&ico[0]), C.int(n))
	if hres != C.S_OK {
		hr := uint32(hres)
		s := fmt.Sprintf("ResolveShortcut: %s HRESULT=%X\n", link, hr)
		err = errors.New(s)
		return
	}
	target = syscall.UTF16ToString(trg)
	icon = syscall.UTF16ToString(ico)
	args = syscall.UTF16ToString(arg)
	err = nil
	return
}

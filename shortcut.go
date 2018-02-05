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

func ShortcutTarget(link string) (string, error) {
	target, _, _, err := resolveShortcut(link)
	if err != nil {
		return "", err
	}
	return target, nil
}

func ShortcutArgs(link string) (string, error) {
	_, args, _, err := resolveShortcut(link)
	if err != nil {
		return "", err
	}
	return args, nil
}

func ShortcutIcon(link string) (string, error) {
	_, _, icon, err := resolveShortcut(link)
	if err != nil {
		return "", err
	}
	return icon, nil
}

func resolveShortcut(link string) (string, string, string, error) {
	l, err := syscall.UTF16PtrFromString(link)
	if err != nil {
		return "", "", "", err
	}
	n := 1024
	target := make([]uint16, n)
	args := make([]uint16, n)
	icon := make([]uint16, n)

	hres := C.ResolveShortcut((*C.WCHAR)(l),
		(*C.WCHAR)(&target[0]), C.int(n),
		(*C.WCHAR)(&args[0]), C.int(n),
		(*C.WCHAR)(&icon[0]), C.int(n))

	if hres != C.S_OK {
		hr := uint32(hres)
		s := fmt.Sprintf("ResolveShortcut: %s HRESULT=%X\n", link, hr)
		return "", "", "", errors.New(s)
	}
	starget := syscall.UTF16ToString(target)
	sargs := syscall.UTF16ToString(args)
	sicon := syscall.UTF16ToString(icon)
	return starget, sargs, sicon, nil
}

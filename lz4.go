package chelp

//#include "chelp.h"
import "C"
import (
	"unsafe"
)

func LZ4Decompress(in, out []byte) int {
	i, o := unsafe.Pointer(&in[0]), unsafe.Pointer(&out[0])
	isize, osize := C.int(len(in)), C.int(len(out))
	ret := C.LZ4_decompress_safe((*C.char)(i), (*C.char)(o), isize, osize)
	return int(ret)
}

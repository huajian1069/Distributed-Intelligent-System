;;
;; errno.s
;;
;; A weak definition of _errno to permit
;; stand-alone use of the math library.
;; 

        .section .nbss,near,bss
	.ifdef __dsPIC30ELF
	.type   _errno,@object
	.endif

        .weak _errno
	.align 2
_errno:
        .space 2

        .end

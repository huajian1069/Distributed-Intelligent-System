@echo off
set SAVE=
set SAVE=%PIC30_OMF%
:askagain
echo.
echo Building for Processor: %PROC%
echo.
echo Press C for COFF library
echo Press E for ELF library
echo Press Q to quit
echo.
set choice=
set /p choice=Which OMF do you use?
if '%choice%' == 'C' goto COFF
if '%choice%' == 'c' goto COFF
if '%choice%' == 'E' goto ELF
if '%choice%' == 'e' goto ELF
if '%choice%' == 'Q' goto EXIT
if '%choice%' == 'q' goto EXIT
echo.
goto askagain
:EXIT
  echo Not building a library.
  goto end
:COFF
  set PIC30_OMF=coff
  goto start1
:ELF
  set PIC30_OMF=elf
:start1

@del  crt0.o crt1.o _exit.s read.s open.s lseek.s close.s write.s remove.s rename.s brk.s sbrk.s getenv.s system.s time.s _exit.o read.o open.o lseek.o close.o write.o remove.o rename.o brk.o sbrk.o getenv.o system.o time.o _exit.lst read.lst open.lst lseek.lst close.lst write.lst remove.lst rename.lst brk.lst sbrk.lst getenv.lst system.lst time.lst crt0.lst crt1.lst libpic30-%PIC30_OMF%.a version.s version.o 
@echo .section .comment > version.s
@echo .asciz "User compiled library" >> version.s
@pic30-gcc -c version.s
@echo %PIC30_OMF%: _exit.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o _exit.o -Wa,-ahli=_exit.lst _exit.c
@echo %PIC30_OMF%: read.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o read.o -Wa,-ahli=read.lst read.c
@echo %PIC30_OMF%: open.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o open.o -Wa,-ahli=open.lst open.c
@echo %PIC30_OMF%: lseek.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o lseek.o -Wa,-ahli=lseek.lst lseek.c
@echo %PIC30_OMF%: close.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o close.o -Wa,-ahli=close.lst close.c
@echo %PIC30_OMF%: write.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o write.o -Wa,-ahli=write.lst write.c
@echo %PIC30_OMF%: remove.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o remove.o -Wa,-ahli=remove.lst remove.c
@echo %PIC30_OMF%: rename.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o rename.o -Wa,-ahli=rename.lst rename.c
@echo %PIC30_OMF%: brk.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o brk.o -Wa,-ahli=brk.lst brk.c
@echo %PIC30_OMF%: sbrk.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o sbrk.o -Wa,-ahli=sbrk.lst sbrk.c
@echo %PIC30_OMF%: getenv.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o getenv.o -Wa,-ahli=getenv.lst getenv.c
@echo %PIC30_OMF%: system.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o system.o -Wa,-ahli=system.lst system.c
@echo %PIC30_OMF%: time.c
@pic30-gcc -c -g -Os -I../include -I../support/h -mconst-in-data -fverbose-asm -dr -msmall-code -msmall-data -mtext=.libc -fomit-frame-pointer -mcpu=generic-16bit -dr -o time.o -Wa,-ahli=time.lst time.c
@echo %PIC30_OMF%: crt0.s
@pic30-%PIC30_OMF%-as -o crt0.o -I../support/inc -ahldi=crt0.lst crt0.s
@echo %PIC30_OMF%: crt1.s
@pic30-%PIC30_OMF%-as -o crt1.o -I../support/inc -ahldi=crt1.lst crt1.s
@echo Archiving libpic30-%PIC30_OMF%.a
@pic30-%PIC30_OMF%-ar -r libpic30-%PIC30_OMF%.a _exit.o read.o open.o lseek.o close.o write.o remove.o rename.o brk.o sbrk.o getenv.o system.o time.o crt0.o crt1.o version.o
set PIC30_OMF=%SAVE%


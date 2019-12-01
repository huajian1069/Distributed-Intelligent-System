##
##  Makefile for epuck_library
##
##  $Date: 2009-11-10 12:37:33 +0100 (Tue, 10 Nov 2009) $
##  $Revision: 102 $
##  $Author: cianci $
##  $HeadURL: https://grmapc10.epfl.ch/svn/students/Epuck/EpuckDevelopmentTree/library/Makefile $
##


COMPONENTS 	= btcom epfl ircom 


.PHONY: all
all:
	for i in $(COMPONENTS); do \
	    (cd $$i && $(MAKE)); \
	done


.PHONY:	clean
clean:
	for i in $(COMPONENTS); do \
	    (cd $$i && $(MAKE) clean); \
	done

#-------------------------------------------------------------------------------

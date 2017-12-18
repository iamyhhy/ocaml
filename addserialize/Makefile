
PATH_TO_CIL             = ~/.opam/system/lib/cil
PATH_TO_CIL_LIBS	= $(PATH_TO_CIL)/_build/src
PATH_TO_FRONTC_LIBS	= $(PATH_TO_CIL)/_build/src/frontc
PATH_TO_OCAMLUTIL_LIBS	= $(PATH_TO_CIL)/_build/src/ocamlutil
PATH_TO_EXT_LIBS	= $(PATH_TO_CIL)/_build/src/ext
OCAMLOPT  = ocamlopt -I $(PATH_TO_CIL) -I $(PATH_TO_CIL_LIBS) -I $(PATH_TO_FRONTC_LIBS) -I $(PATH_TO_OCAMLUTIL_LIBS) -I $(PATH_TO_EXT_LIBS) 

all: addserialize

addserialize: addserialize.ml
	$(OCAMLOPT) -o addserialize str.cmxa unix.cmxa nums.cmxa cil.cmxa addserialize.ml

test: addserialize
	$(RM) serialized.data hello.i hello_out.c hello_out
	gcc -E hello.c > hello.i
	./addserialize --variable lptr --serialize SERIALIZE --deserialize DESERIALIZE hello.i 
	gcc -o hello_out hello_out.c
	./hello_out < hello.test_input
	./hello_out < /dev/null

array: addserialize
	$(RM) serialized.data array.i  array_out
	gcc -E array.c > array.i
	./addserialize --variable lptr --serialize SERIALIZE --deserialize DESERIALIZE array.i 
	gcc -o array_out array_out.c
	./array_out < hello.test_input
	./array_out < /dev/null

clean:
	$(RM) addserialize *.cm? *.o a.out hello hello_out hello.i array array_out array.i serialized.data


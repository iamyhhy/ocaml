PATH_TO_CIL = ~/.opam/system/lib/cil
OCAMLOPT = ocamlopt -I $(PATH_TO_CIL)
ADDSERIALIZE = ./addserialize

add: addserialize

addserialize: ../addserialize/addserialize.ml
	$(OCAMLOPT) -o addserialize str.cmxa unix.cmxa nums.cmxa cil.cmxa ../addserialize/addserialize.ml

test: addserialize
	$(RM) yu.out
	$(ADDSERIALIZE) --variable myCopter --serialize SERIALIZE --deserialize DESERIALIZE yu.i

clean:
	$(RM) addserialize *.cm? 
	

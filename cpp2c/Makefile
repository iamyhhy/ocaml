PATH_TO_CIL = ~/.opam/system/lib/cil
OCAMLOPT = ocamlopt -I $(PATH_TO_CIL)
ADDSERIALIZE = ./addserialize
STRUCT = yu

add: addserialize

# compile addserialize, generate the addserialize ELF binaries
addserialize: ../addserialize/addserialize.ml
	$(OCAMLOPT) -o addserialize str.cmxa unix.cmxa nums.cmxa cil.cmxa ../addserialize/addserialize.ml

# generate the yu_out.c that contains all the __serialize__ and  __deserilize__ functions 
test: addserialize
	$(RM) $(STRUCT)_out.c
	$(ADDSERIALIZE) --variable myStruct --serialize SERIALIZE --deserialize DESERIALIZE $(STRUCT).i

clean:
	$(RM) addserialize *.cm? 
	

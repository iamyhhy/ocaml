(*
 * Add Serialization and Deserialization Code to an Existing C File 
 *
 *)
open Cil
open Cilint

(* Certain functions, like "open" and "read" and "close", are part of the C
 * standard library. We need to refer to them when generating new code, so
 * we store references to them here. *)
let globals = Hashtbl.create 255 

(* We emit one 'serialization function' per relevant type in the input
 * program. Each relevant type's serialization function has a unique name. 
 *) 
let rec innerName basetype = 
  match unrollType basetype with
  (*in c, uint8_t is the same as unsigned char*)
  | TInt(_,_)-> 
    let bits = bitsSizeOf basetype in 
    Printf.sprintf "scalarInt%d" bits 
  | TFloat(_,_) ->
    (* the only thing that matters in a scalar is its size *) 
    let bits = bitsSizeOf basetype in 
    Printf.sprintf "scalarFloat%d" bits 
(*TODO: change scalar to int/float*)
  | TComp(ci,_) -> ci.cname 

  | TPtr(bt,_) -> "ptr_" ^ (innerName bt) 
  

  | TArray(bt,Some(sizeExp),_) -> begin
    match getInteger sizeExp with
    | Some(ci) -> Printf.sprintf "array%s_%s" 
      (string_of_cilint ci) (innerName bt) 
    | None -> failwith "innerName: unknown array size" 
  end 
  
  | TEnum(ei,_)-> ei.ename

  | _ -> failwith "innerName: unhandled"  

let serializeFunctionName basetype = "__serialize_" ^ (innerName basetype) 
let deserializeFunctionName basetype = "__deserialize_" ^ (innerName basetype) 

(* We create all of our serialization functions at once during a recursive
 * traversal of the relevant types. We store them here until later, when
 * we add them all to the file (and print them out). 
 *)
let createdSerializeCode = Hashtbl.create 255 

(* This OCaml function creates new C functions that are added to the input
 * C file. The new C functions, when later run at run-time, serialize 
 * dynamic variable values. *)
let rec createSerializeCode typ = begin
  let typ = unrollType typ in 
  let name = serializeFunctionName typ in
  Printf.printf "%s: creating: %b\n" name (Hashtbl.mem createdSerializeCode name) ; 
  if not (Hashtbl.mem createdSerializeCode name) then begin
    let fd = emptyFunction name in 
    let ptr_va = makeFormalVar fd "ptr" (TPtr(typ,[])) in 
    let fd_va = makeFormalVar fd "fd" (intType) in 
    Hashtbl.add createdSerializeCode name fd ;
    let stmts = 
      match typ with
      | TInt(_,_) 
      | TEnum(_,_)
      | TFloat(_,_) ->
        let bits = bitsSizeOf typ in 
        let str = Printf.sprintf "write(fd, ptr, %d);" (bits/8) in 
        Formatcil.cStmts str 
          (fun n t -> makeTempVar fd ~name:n t) locUnknown
          [ ("write", Fv(Hashtbl.find globals "write")) ; 
            ("fd", Fv(fd_va)) ; 
            ("ptr", Fv(ptr_va)) ; ]
      | TPtr(basetype,_) -> 
        let str = Printf.sprintf "{ int i ; write(fd, ptr, sizeof(*ptr)); if (*ptr != 0) { i = 0; while (i < memoizeMax) { if (memoize[i] == *ptr) return; i = i + 1; } memoize[memoizeMax] = *ptr; memoizeMax = memoizeMax + 1 ; serialize(*ptr, fd); }} " in 
        createSerializeCode basetype ; 
        let serializeName = serializeFunctionName basetype in
        Formatcil.cStmts str 
          (fun n t -> makeTempVar fd ~name:n t) locUnknown
          [ ("write", Fv(Hashtbl.find globals "write")) ; 
            ("memoize", Fv(Hashtbl.find globals "memoize")) ; 
            ("memoizeMax", Fv(Hashtbl.find globals "memoizeMax")) ; 
            ("fd", Fv(fd_va)) ; 
            ("ptr", Fv(ptr_va)) ; 
            ("serialize", Fv( (Hashtbl.find createdSerializeCode serializeName).svar)) ;
            ]


      | TComp(ci,_) -> 
        List.map (fun fi ->
          let fname = fi.fname in 
          let ftype = fi.ftype in 
          createSerializeCode ftype ; 
          let str = Printf.sprintf "serialize(&(ptr->%s) ,fd);" fname in
          let serializeName = serializeFunctionName ftype in
          Formatcil.cStmt str 
            (fun n t -> makeTempVar fd ~name:n t) locUnknown
            [ ("fd", Fv(fd_va)) ; 
              ("ptr", Fv(ptr_va)) ; 
              ("serialize", Fv((Hashtbl.find createdSerializeCode serializeName).svar)) ;
              ] 
        ) ci.cfields 

      | TArray(basetype,Some(sizeExp),_) -> 
        createSerializeCode basetype ; 
        let serializeName = serializeFunctionName basetype in
        let str = "{ int i; i=0; while (i < %e:sizeExp) { \
        serialize( & ((*ptr)[i]), fd); i = i + 1; }}" in 
        Formatcil.cStmts str 
          (fun n t -> makeTempVar fd ~name:n t) locUnknown
          [ ("write", Fv(Hashtbl.find globals "write")) ; 
            ("fd", Fv(fd_va)) ; 
            ("ptr", Fv(ptr_va)) ; 
            ("sizeExp", Fe(sizeExp)) ; 
            ("serialize", Fv( (Hashtbl.find createdSerializeCode serializeName).svar)) ;
            ]


       (* createSerializeCode basetype;
        let serializeName = serializeFunctionName basetype in
        let items = ei.eitems in *)
      | _ -> failwith "createSerializedCode: unhandled"  

    in 
	let contains s1 s2 =
		let re = Str.regexp_string s2
		in
			try ignore (Str.search_forward re s1 0); true
			with Not_found -> false in
    let this_is_one_of_those_anonymous_types = contains name "anon" in

    let stmts = if this_is_one_of_those_anonymous_types then [] else stmts in 
    Printf.printf "%s: creating: %b\n" name (Hashtbl.mem createdSerializeCode name) ; 
    Printf.printf "is this anon:  %b\n" this_is_one_of_those_anonymous_types;
    Printf.printf "Size of stmt: %d\n" (List.length stmts); 
    let block = mkBlock (compactStmts stmts) in 
    fd.sbody <- block 

  end 
end 

(* This OCaml function creates new C functions that are added to the input
 * C file. The new C functions, when later run at run-time, de-serialize 
 * dynamic variable values. *)
let rec createDeserializeCode typ = begin
  let typ = unrollType typ in 
  let name = deserializeFunctionName typ in
  Printf.printf "%s: creating: %b\n" name (Hashtbl.mem createdSerializeCode name) ; 
  if not (Hashtbl.mem createdSerializeCode name) then begin
    let fd = emptyFunction name in 
    let ptr_va = makeFormalVar fd "ptr" (TPtr(typ,[])) in 
    let fd_va = makeFormalVar fd "fd" (intType) in 
    Hashtbl.add createdSerializeCode name fd ;
    let stmts = 
      match typ with
      | TInt(_,_) 
      | TEnum(_,_)
      | TFloat(_,_) ->
        let bits = bitsSizeOf typ in 
        let str = Printf.sprintf "read(fd, ptr, %d);" (bits/8) in 
        Formatcil.cStmts str 
          (fun n t -> makeTempVar fd ~name:n t) locUnknown
          [ ("read", Fv(Hashtbl.find globals "read")) ; 
            ("fd", Fv(fd_va)) ; 
            ("ptr", Fv(ptr_va)) ; ]

      | TPtr(basetype,_) -> 
        let str = "{int i; read(fd, ptr, sizeof(*ptr)); if (*ptr != 0) { i = 0; while (i < memoizeMax) { if (memoize[i] == *ptr) { *ptr = memoize[i+1]; return; }  i = i + 2; }  memoize[memoizeMax] = *ptr;  *ptr = malloc(%e:mysize);  memoize[memoizeMax+1] = *ptr; memoizeMax = memoizeMax + 2;  deserialize(*ptr, fd); }}" in 
        createDeserializeCode basetype ; 
        let deserializeName = deserializeFunctionName basetype in
        Formatcil.cStmts str 
          (fun n t -> makeTempVar fd ~name:n t) locUnknown
          [ ("read", Fv(Hashtbl.find globals "read")) ; 
            ("malloc", Fv(Hashtbl.find globals "malloc")) ; 
            ("memoize", Fv(Hashtbl.find globals "memoize")) ; 
            ("memoizeMax", Fv(Hashtbl.find globals "memoizeMax")) ; 
            ("fd", Fv(fd_va)) ; 
            ("mysize", Fe(sizeOf basetype)) ; 
            ("ptr", Fv(ptr_va)) ; 
            ("deserialize", Fv( (Hashtbl.find createdSerializeCode deserializeName).svar)) ;
            ]

      | TComp(ci,_) -> 
        List.map (fun fi ->
          let fname = fi.fname in 
          let ftype = fi.ftype in 
          createDeserializeCode ftype ; 
          let str = Printf.sprintf "deserialize(&(ptr->%s) ,fd);" fname in
          let deserializeName = deserializeFunctionName ftype in
          Formatcil.cStmt str 
            (fun n t -> makeTempVar fd ~name:n t) locUnknown
            [ ("fd", Fv(fd_va)) ; 
              ("ptr", Fv(ptr_va)) ; 
              ("deserialize", Fv((Hashtbl.find createdSerializeCode deserializeName).svar)) ;
              ] 
        ) ci.cfields 

      | TArray(basetype,Some(sizeExp),_) -> 
        createDeserializeCode basetype ; 
        let deserializeName = deserializeFunctionName basetype in
        let str = "{ int i; i=0; while (i < %e:sizeExp) { \
        deserialize( & ((*ptr)[i]), fd); i = i + 1; }}" in 
        Formatcil.cStmts str 
          (fun n t -> makeTempVar fd ~name:n t) locUnknown
          [ ("write", Fv(Hashtbl.find globals "write")) ; 
            ("fd", Fv(fd_va)) ; 
            ("ptr", Fv(ptr_va)) ; 
            ("sizeExp", Fe(sizeExp)) ; 
            ("deserialize", Fv( (Hashtbl.find createdSerializeCode deserializeName).svar)) ;
            ]
      | TEnum(_,_)
      | _ -> failwith "createDeserializedCode: unhandled" 

    in 

	let contains s1 s2 =
		let re = Str.regexp_string s2
		in
			try ignore (Str.search_forward re s1 0); true
			with Not_found -> false in
    let this_is_one_of_those_anonymous_types = contains name "anon" in

    let stmts = if this_is_one_of_those_anonymous_types then [] else stmts in 
    Printf.printf "%s: creating: %b\n" name (Hashtbl.mem createdSerializeCode name) ; 
    Printf.printf "is this anon:  %b\n" this_is_one_of_those_anonymous_types;
    let block = mkBlock (compactStmts stmts) in 
    fd.sbody <- block 

  end 
end 

(* Our visitor sweeps over the program until it finds the appropriate
 * labels. At those labels it inserts serialization and deserialization
 * code. 
 *
 * For convenience, we also keep track of the type of the target variable
 * and the enclosing fundec, since we'll need to know those later. *) 
class addSerializeVisitor target_variable_name 
                          target_serialize_label 
                          target_deserialize_label 
                          target_fundec (* we write this for our caller *)
  = object
  inherit nopCilVisitor 

  val target_varinfo = ref None 

  method vvdec varinfo =
    if varinfo.vname = target_variable_name then target_varinfo := Some(varinfo) ;
    DoChildren

  method vfunc fundec = target_fundec := Some(fundec) ; DoChildren

  method vstmt stmt = 
    if List.exists (fun l -> match l with 
      | Label(lname,_,_) -> target_serialize_label = lname
      | _ -> false
    ) stmt.labels then begin
      let va, typ, fundec = match !target_varinfo, !target_fundec with
      | Some(va), Some(fd) -> va, va.vtype, fd
      | _ -> failwith "serialize label reached but variable not found" 
      in 
      createSerializeCode typ ; 
      let serializeName = serializeFunctionName typ in
      let str = "{ int fd ; fd = open(%g:filename, 514, 0770); memoizeMax = 0; serialize(& myvar, fd); close(fd); }" in 
      let stmts = Formatcil.cStmts str 
        (fun n t -> makeTempVar fundec ~name:n t) locUnknown
        [ ("open", Fv(Hashtbl.find globals "open")) ; 
          ("close", Fv(Hashtbl.find globals "close")) ; 
          ("myvar", Fv(va)) ; 
          ("serialize", Fv( (Hashtbl.find createdSerializeCode serializeName).svar)) ;
          ("memoizeMax", Fv( (Hashtbl.find globals "memoizeMax"))) ;
          ("filename", Fg("serialized.data")) ; 
        ] 
      in 
      let block = mkBlock (stmt :: stmts) in 
      ChangeTo(mkStmt (Block(block)))
    end else if List.exists (fun l -> match l with 
      | Label(lname,_,_) -> target_deserialize_label = lname
      | _ -> false
    ) stmt.labels then begin
      let va, typ, fundec = match !target_varinfo, !target_fundec with
      | Some(va), Some(fd) -> va, va.vtype, fd
      | _ -> failwith "deserialize label reached but variable not found" 
      in 
      createDeserializeCode typ ; 
      let deserializeName = deserializeFunctionName typ in
      let str = "{ int fd ; fd = open(%g:filename, 514, 0770); memoizeMax = 0; deserialize(& myvar, fd); close(fd); }" in 
      let stmts = Formatcil.cStmts str 
        (fun n t -> makeTempVar fundec ~name:n t) locUnknown
        [ ("open", Fv(Hashtbl.find globals "open")) ; 
          ("close", Fv(Hashtbl.find globals "close")) ; 
          ("myvar", Fv(va)) ; 
          ("memoizeMax", Fv( (Hashtbl.find globals "memoizeMax"))) ;
          ("deserialize", Fv( (Hashtbl.find createdSerializeCode deserializeName).svar)) ;
          ("filename", Fg("serialized.data")) ; 
        ] 
      in 
      let block = mkBlock (stmt :: stmts) in 
      ChangeTo(mkStmt (Block(block)))
    end else 
      DoChildren

end 

let main () = begin
  let target_variable = ref "" in 
  let target_serialize_label = ref "" in 
  let target_deserialize_label = ref "" in 
  let files = ref [] in 
  let memoize_size = ref 4096 in 

  let args = [ 
    ("--variable", Arg.Set_string target_variable, "X add code to (de)serialize variable X") ;
    ("--serialize", Arg.Set_string target_serialize_label, "X serialize at program label X") ;
    ("--deserialize", Arg.Set_string target_deserialize_label, "X deserialize at program label X") ;
    ("--memoizeMax", Arg.Set_int memoize_size, "X size of memoize array") ;
  ] in 
  let usage_message = "add (de)serialization code to a post-processed C file" in 

  Arg.parse (Arg.align args) 
    (fun filename -> files := filename :: !files) usage_message ; 

  Cil.initCIL () ; 

  List.iter (fun filename ->
    Printf.printf "%s: parsing\n" filename ; 
    let ast = Frontc.parse filename () in 
    Printf.printf "%s: parsed\n" filename ; 

    (* Find a bunch of relevant global functions like "open" and "write" *)
    Hashtbl.add globals "open" (findOrCreateFunc ast "open" (TVoid [])) ; 
    let close_type = TFun(intType,
      Some [ ("fd", intType, []) ], false, []) in 
    Hashtbl.add globals "close" (findOrCreateFunc ast "close" close_type) ; 
    let write_type = TFun(intType, 
      Some [ ("fd", intType, []) ; 
        ("buf", voidPtrType, []); 
        ("count", intType, []) ], false, []) in 
    Hashtbl.add globals "write" (findOrCreateFunc ast "write" write_type) ; 
    Hashtbl.add globals "read" (findOrCreateFunc ast "read" write_type); 
    Hashtbl.add globals "malloc" (findOrCreateFunc ast "malloc" (TVoid [])); 

    let memoizeMax_va = (makeGlobalVar "_memoizeMax" intType) in 
    Hashtbl.add globals "memoizeMax" memoizeMax_va ; 
    let memoize_va = (makeGlobalVar "_memoize" 
      (TArray(voidPtrType, Some(integer !memoize_size), []))) in 
    Hashtbl.add globals "memoize" memoize_va ; 

    let target_fundec = ref None in 
    let visitor = new addSerializeVisitor !target_variable
      !target_serialize_label !target_deserialize_label 
      target_fundec 
    in
    visitCilFileSameGlobals visitor ast ;

    let rec addNewCode globals = match globals, !target_fundec with
    | [], _ -> []
    | (GFun(fd,x) :: tl), (Some target_fd) -> 
      if fd.svar = target_fd.svar then begin
        let lst = ref [] in 
        (* Since we are creating code for mutually recursive functions, 
         * we first emit a declaration for every function and then
         * we emit the code for the body. *) 
        Hashtbl.iter (fun n fd -> lst := (GFun(fd,locUnknown)) :: !lst 
        ) createdSerializeCode ; 
        Hashtbl.iter (fun n fd -> lst := (GVarDecl(fd.svar,locUnknown)) :: !lst 
        ) createdSerializeCode ; 
        !lst @ [(GFun(fd,x))] @ (addNewCode tl) 

      end else (GFun(fd,x) :: (addNewCode tl))
    | (other :: tl), _ -> other :: (addNewCode tl) 
    in 
    let newGlobals = addNewCode ast.globals in 
    let ast = { ast with globals = 
      (GVarDecl(memoize_va,locUnknown)) :: 
      (GVarDecl(memoizeMax_va,locUnknown)) :: newGlobals } in 

    let outname = (Filename.chop_extension filename) ^ "_out.c" in 
    let outchan = open_out outname in 
    dumpFile defaultCilPrinter outchan outname ast ;
    close_out outchan ; 

  ) !files ; 

end ;;
main () ;; 

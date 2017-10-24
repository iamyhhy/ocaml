(*
 * Example code for Yu Huang that demonstrates programmatically finding
 * and traversin a C type using CIL. 
 *
 * Usage is something like:
 *  ./walktype --type mystruct --label HERE hello.i 
 *)
open Cil

(*
 * Eventually you would want to change this so that it computes offsets and
 * prints out the data values at the given offsets. You'll also need to
 * handle pointers and arrays, etc. 
 *) 
let rec serialize typ = 
  match typ with
  | TInt(ikind, attr) -> 
    Printf.printf "TInt: %d bits in size, %d bytes alignment\n" 
      (bitsSizeOf typ) (alignOf_int typ) 
  | TFloat(fkind, attr) -> 
    Printf.printf "TFloat: %d bits in size, %d bytes alignment\n" 
      (bitsSizeOf typ) (alignOf_int typ) 
  | TComp(compinfo, attr) -> 
    Printf.printf "TComp: %d fields\n" (List.length compinfo.cfields); 
    List.iter (fun fi ->
      serialize fi.ftype 
    ) compinfo.cfields
  | TNamed(_,_)-> let typ2 = unrollType typ in 
    serialize typ2
  | TPtr(t, attr)-> Printf.printf "This is a TPtr\n";
    serialize t
  | TArray(element, len, attr)-> 
        let signature = typeSig element in (* debugging info *) 
        let signature_string = Pretty.sprint ~width:80 
          (d_typsig () signature) in (*d_typsig is a function*) 
        Printf.printf "TArray: %d bits in size, %d bytes alignment, type: %s\n"(*Check the size of arr*)
        (bitsSizeOf typ) (alignOf_int typ) signature_string

  | _ -> failwith "serialize: unhandled" 

(*
 * This visit walks over a C AST until it finds the given label.
 *
 * Eventually you'll want to change it so that it doesn't just find the
 * given label but also adds the serialization/deserialization instructions
 * there. For now we just print them to stdout.
 *)
class walktypeCilVisitor target_type_name target_label = object(self)
  inherit nopCilVisitor 
(*It is right now chaning the expression on the original exp. How can
 * you add some code?*)

(*
(*e.g.: add 7 to each integer in the c code*)
val target_type = ref None
  method vexpr exp =
      match exp with
      | Const(CInt64(ival,ikind,so)) ->
                let plus_seven = Int64.add ival (Int64.of_int 7) in 
                let new_exp = Const(CInt64(plus_seven, ikind, None)) in 
                ChangeTo(new_exp) 
      | _ -> DoChildren

   (*When visit the type, get the fields(including pointer), generate
    * new statement and insert it in AST!!!!*)
*)

(*for every lval, make lval++ --print this command in the c code*)
    method vlval lval = 
        let new_exp =BinOp(PlusA,Lval(lval), Cil.one, intType )in
        let new_assign = Set(lval, new_exp, !currentLoc) in
        self#queueInstr [new_assign];
        SkipChildren


(*for every pointer type, print "I am a pointer: %s", pointername *)
    method vtype typ =
        begin match typ with
        |TPtr(ptr_type, attr) ->
                let printf_varinfo = makeVarinfo true  "printf" (TVoid []) in
                let new_exp =Const(CStr("Hello, this is a pointer\n") ) in
                let printf_instr = Call(None, Lval(Var printf_varinfo, NoOffset), [new_exp], !currentLoc) in

                self#queueInstr [printf_instr];
(*
                let print_printf = Formatcil.cInstr
                "printf(%e);" !currentLoc
                [
                    ("printf", Fv printf_varinfo);
                ] in
                let skind_print = Instr ([print_printf]) in
                let new_stmt =  mkStmt (skind_print) in
                printStmt cilprinter () new_stmt
  *)
                SkipChildren
        |_ -> DoChildren        
        end


        
end 




let main () = begin
  let target_type = ref "" in 
  let target_label = ref "" in 
  let files = ref [] in 

  let args = [ 
    ("--type", Arg.Set_string target_type, "X walk and describe type X") ;
    ("--label", Arg.Set_string target_label, "X operate at program label X") ;
  ] in 
  let usage_message = "walk and describe a type in a post-processed C file" in 

  Arg.parse (Arg.align args) 
    (fun filename -> files := filename :: !files) usage_message ; 

  Cil.initCIL () ; 

  List.iter (fun filename ->
    Printf.printf "%s: parsing\n" filename ; 
    let ast = Frontc.parse filename () in 
    Printf.printf "%s: parsed\n" filename ; 

    let visitor = new walktypeCilVisitor !target_type !target_label in
    visitCilFileSameGlobals visitor ast ;
    dumpFile defaultCilPrinter stdout filename ast ; 

  ) !files ; 

end ;;
main () ;; 

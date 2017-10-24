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
class walktypeCilVisitor target_type_name target_label = object
  inherit nopCilVisitor 

  (* While we're traversing, if we come across the type, store it for later
   * use. *) 
  val target_type = ref None 

  method vtype typ = 
    begin match typ with
    | TNamed(ti,attr) -> 
      if ti.tname = target_type_name then begin
        target_type := Some(typ) 
      end 
    | _ -> () end ; DoChildren 

  (* Look for the given label ... *) 
  method vstmt stmt = 
    if List.exists (fun l -> 
      match l with 
      | Label(lname,_,_) -> target_label = lname
      | _ -> false
    ) stmt.labels then begin

      (* We found the label. Time to serialize/deserialize/whatever. *) 
      Printf.printf "%s: found\n" target_label ; 
      match !target_type with
      | None -> failwith "label reached but type not found" 
      | Some(typ) -> begin
        let typ = unrollType typ in 
        let signature = typeSig typ in (* debugging info *) 
        let signature_string = Pretty.sprint ~width:80 
          (d_typsig () signature) in (*d_typsig is a function*) 
        Printf.printf "%s: %s\n" target_type_name signature_string ;
        serialize typ 
      end 

    end ;
    DoChildren
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

  ) !files ; 

end ;;
main () ;; 

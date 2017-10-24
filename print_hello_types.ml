(* Starting practice2: generate a print_hello function for each type the
 * code has in C. eg. print_hello_int*)
open Cil

(*walk through the AST and add a correspoding print_hello_typ fuction in the
 * AST *)

let genFun :(string->GFun) = 
    let new_exp = Const(CStr("Hello World\n")) in 
        (* find the printf in ast and get the varinfo of it and replace the
         * printf_varinfo here: findOrCreateFunc*)
        let printf_varinfo = findOrCreateFunc ast "printf"
        (TFun( TVoid([]),
              Some([
                  ("to_print", 
                   TPtr(
                       Cil.charType,
                       []),
                       []
                   )
                   ]),
              false,
              [] )) in
        
        let printf_instr = Call(None, Lval(Var printf_varinfo, NoOffset),
        [new_exp],locUnknown) in
        let new_fundec = emptyFunction "print_hello" in
        setFunctionTypeMakeFormals new_fundec (TFun(TVoid([]), Some([]), false,[]));
        Cil.setMaxId new_fundec;
        new_fundec.sbody <- ( mkBlock [(mkStmt (Instr( [printf_instr] )))] );
        computeCFGInfo new_fundec true; 
       
        let hello_func = GFun(new_fundec, locUnknown)
       

class typeCilVisitor = object(self)
    inherit nopCilVisitor
    method vtype typ =
        match typ with 
        | TInt(ikind, attr) ->



let main () = begin
    let files = ref [] in

    let usage_message = "Just add a print_hello function in a C code" in
    let args = [] in
    Arg.parse (Arg.align args)
        (*"::" is "append" to a list*)
        (fun filename -> files := filename :: !files) usage_message;

    Cil.initCIL ();
    
    List.iter (fun filename -> 
        Printf.printf "%s: parsing\n" filename;
        let ast = Frontc.parse filename () in
        Printf.printf "%s: parsed\n" filename;

        (*construct the print_hello function and add it to AST*)
        (*fundec(svar, sformals, slocals, smaxid, sbody, smaxstmtid,
         * sallstmts)*)
        let new_exp = Const(CStr("Hello World\n")) in 
        (* find the printf in ast and get the varinfo of it and replace the
         * printf_varinfo here: findOrCreateFunc*)
        let printf_varinfo = findOrCreateFunc ast "printf"
        (TFun( TVoid([]),
              Some([
                  ("to_print", 
                   TPtr(
                       Cil.charType,
                       []),
                       []
                   )
                   ]),
              false,
              [] )) in
        
        let printf_instr = Call(None, Lval(Var printf_varinfo, NoOffset),
        [new_exp],locUnknown) in
        let new_fundec = emptyFunction "print_hello" in
        setFunctionTypeMakeFormals new_fundec (TFun(TVoid([]), Some([]), false,[]));
        Cil.setMaxId new_fundec;
        new_fundec.sbody <- ( mkBlock [(mkStmt (Instr( [printf_instr] )))] );
        computeCFGInfo new_fundec true; 
       
        let hello_func = GFun(new_fundec, locUnknown) in
        
        ast.globals <-  (GVarDecl(new_fundec.svar,locUnknown) :: ast.globals) @ [hello_func];


        
        dumpFile defaultCilPrinter stdout filename ast ;

    ) !files ;

end;;
main ();;

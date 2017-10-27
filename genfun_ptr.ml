(* Practice4: generate a if-else function for  a linked list (head node)*)
open Cil

(*walk through the AST and add a correspoding print_hello_typ fuction in the
 * AST *)
 let fun_list = ref [];
 let fun_decl_list = ref [];

let genFun :(string->Cil.file -> Cil.fundec) = fun fun_name ast ->
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
        let new_fundec = emptyFunction fun_name in
        setFunctionTypeMakeFormals new_fundec (TFun(TVoid([]), Some([]), false,[]));
        Cil.setMaxId new_fundec;
        new_fundec.sbody <- ( mkBlock [(mkStmt (Instr( [printf_instr] )))] );
        computeCFGInfo new_fundec true; 
        new_fundec

(*gen a function for a link list *head: if head.next!=NULL, then print
something*)
class typeCilVisitor ast = object(self)
    inherit nopCilVisitor
    method vtype typ =
        match typ with 
        | TPTr(t, arr) -> 
            (*let fundec_hello = genFun "print_hello_ptr" ast in
            let hello_func = GFun(new_fundec, locUnknown) in
            let hello_func_decl = GVarDecl(fundec_hello.svar,
            locUnknown) in
            fun_list := hello_func :: !fun_list;
            fun_decl_list := hell_func_decl :: !fun_decl_lsit;
            DoChildren*)

            (*new_exp => head->next!=NULL*)
            let new_exp = BinOp(Ne,head->next,NULL) in
            let if_varinfo = 
            (*if head->next!=NULL then ...*)
            let if_stmt = mkEmptyStmt () in

        | _ -> ()

end





        




let main () = begin
    let files = ref [] in

    let usage_message = "Just add a print_hello function for a pointer
    type in a C code" in
    let args = [] in
    Arg.parse (Arg.align args)
        (*"::" is "append" to a list*)
        (fun filename -> files := filename :: !files) usage_message;

    Cil.initCIL ();
    
    List.iter (fun filename -> 
        Printf.printf "%s: parsing\n" filename;
        let ast = Frontc.parse filename () in
        Printf.printf "%s: parsed\n" filename;

(*
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
  
  *)
        ast.globals <-  fun_decl_list @  ast.globals @ func_list;


        
        dumpFile defaultCilPrinter stdout filename ast ;

    ) !files ;

end;;
main ();;

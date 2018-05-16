(* HOW DOES CIL'S FORMATSTRING ACTUALLY WORK *) 

type cilvar = string (* "x" , "y" *) 

(*cilexp is like a superclass, Var/Plus/Int are like subclasses for
 * cilexp*)
type cilexp =
    | Var of cilvar
    | Plus of cilexp * cilexp
    | Int of int 

type passing_in_type = 
    | Fv of cilvar
    | Fe of cilexp 

    (* cstmt "my variable is %v in expression %e" myvar myexp *) 
    (* cstmt "my variable is myvar in expression myexp" [ (myvar, Fv * * ...) ]  *)

(* cilvar_to_str is a function: transfer a cilvar to string. but in this
 * example, cilvar is string already itself, so it just returns itself*)
let cilvar_to_str var = var 

(* cil_exp_to_str is a recursive function. It takes a exp, and print the
 * exp *)
let rec cilexp_to_str exp = 
    match exp with
    | Var(v) -> cilvar_to_str v
    | Plus(e1,e2) -> 
            let lhs_str = cilexp_to_str e1 in
            let rhs_str = cilexp_to_str e2 in 
            Printf.sprintf "(%s + %s)" lhs_str rhs_str 
    | Int(i) -> string_of_int i 

(* cstmt is a recursive fucntion. It takes two arguments: a string, and
 * a list-argValues*)
(* It splits a string by \t to a word list, for each word, check if it is one of the
 * special words.*)
(* *)
let rec cstmt (formatString : string ) argValues  = 
    let space_regexp = Str.regexp "[ \t]+" in 
    let (words : string list) = Str.split space_regexp formatString in 
    List.iter (fun word ->
        let is_special_argument_word = 
            List.exists (fun (name, f_value) -> word = name) argValues 
        in
        if is_special_argument_word then begin 
            List.iter (fun (name, f_value) -> 
                if name = word then begin
                    match (f_value : passing_in_type) with
                    | Fv (v : cilvar) -> Printf.printf "%s " (cilvar_to_str v)
                    | Fe (e : cilexp) -> Printf.printf "%s " (cilexp_to_str e)
                end 
            ) argValues 
        end else
            Printf.printf "%s " word ; 
    ) words 

    (* in main(), pass a string and a special-word list to the function
        * cstmt() *)
let main () = begin
    let my_variable : cilvar = "x" in 
    let my_expression : cilexp = Plus( (Int 5), (Var my_variable) ) in 

    cstmt "my variable is myvar in expression myexp" 
        [ ("myvar", Fv my_variable) ;
          ("myexp", Fe my_expression) ; ] 

end ;;
main () ;; 

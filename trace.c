#include <stdio.h>
#include <stdlib.h>
// practice "tracing in heap", also some other coding pratice too
// use a linked list, read in a bunch of numbers, insert to the linked
// list, then reverse it.
// Then trace the linked list from the global space to the heap. To make
// this happen:
// 1. pause the program
// 2. somehow save the address OR actual value of the linked list in the
// heap
// 3. restart the program, it can resume using the value of the linked
// list you saved before.

typedef struct node {
    int val;
    struct node * next;
} node_t;

//int gvar = 5;

void print_list(node_t * head){
    node_t * current = head;
    int idx =0;
    while(current!=NULL){

        printf("index = %d, addr=%p,val = %d\n",idx,current, current->val);
        current = current->next;
        
        idx+=1;
    }
}

int main(){
    // construct a linked list:0, 1, 2, 3
    node_t * test_list = malloc(sizeof(node_t));
    test_list->val=0;
    test_list->next = malloc(sizeof(node_t));
    test_list->next->val=1;
    test_list->next->next = malloc(sizeof(node_t));
    test_list->next->next->val=2;
    test_list->next->next->next = malloc(sizeof(node_t));
    test_list->next->next->next->val=3;
    test_list->next->next->next->next=NULL;

    //test print_list()
    print_list(test_list);
    
    return 0;
}


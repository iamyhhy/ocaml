#include <stdlib.h>
#include<stdio.h>
 
 
struct tree
 
{
 
          struct tree *left;
 
          int info;
 
          struct tree *right;
 
};
 
void insert(struct tree **ptr,int item)
 
{
 
          if(*ptr==NULL)
 
          {
 
                   *ptr=(struct tree *)malloc(sizeof(struct tree));
 
                   (*ptr)->left=(*ptr)->right=NULL;
 
                   (*ptr)->info=item;
 
                   return;
 
          }
 
          else
 
          {
 
                   if(item<(*ptr)->info)
 
                   {
 
                             insert(&((*ptr)->left),item);
 
                   }
 
                   else
 
                   {
 
                             insert(&((*ptr)->right),item);
 
                   }
 
                   return;
 
          }
 
}

void main(){
   struct tree ** pptree = (struct tree **)malloc(sizeof(struct tree*));
    
    insert(pptree,3);
}

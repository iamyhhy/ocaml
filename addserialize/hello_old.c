#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

struct yu{
    int x;
};

struct list_node { 
  int head;
  struct list_node * tail;
  struct list_node * self1, *self2;
  struct yu y;
};

int main () {
  struct list_node * lptr = NULL;

  while (1) {
      int data; 
      struct list_node * old = lptr; 
      scanf(" %d", &data); 
      if (data <= 0) break; 

      lptr = malloc(sizeof(*lptr));
      lptr->head = data;
      lptr->tail = old ; 
      lptr->self1 = lptr ; 
      lptr->self2 = lptr ; 
  } 

  if (lptr == NULL) {
DESERIALIZE: 
  } else {
SERIALIZE: 
  } 

  for (; lptr; lptr=lptr->tail) {
      printf("%d %p %p\n", lptr->head, lptr->self1, lptr->self2); 
  } 

  return 0; 

} 

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

struct inner {
  int x;
  char y;
} ;

struct list_node { 
  int head;
  struct list_node * tail;
  struct list_node * self1, *self2;
  struct inner array[2]; 
};

int main () {
  struct list_node * lptr = NULL;
  int counter = 0, i;

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
      for (i=0;i<2;i++) {
        lptr->array[i].x = counter++;
        lptr->array[i].y = 'a' + counter++;
      } 
  } 

  if (lptr == NULL) {
DESERIALIZE: ;
  } else {
SERIALIZE: ;
  } 

  for (; lptr; lptr=lptr->tail) {
      printf("%d %p %p %d %c %d %c\n", lptr->head, lptr->self1, lptr->self2,
          lptr->array[0].x, lptr->array[0].y,
          lptr->array[1].x, lptr->array[1].y); 
  } 

  return 0; 

} 

// Physical memory allocator, intended to allocate
// memory for user processes, kernel stacks, page table pages,
// and pipe buffers. Allocates 4096-byte pages.

#include "types.h"
#include "defs.h"
#include "param.h"
#include "mmu.h"
#include "spinlock.h"

struct run {
  struct run *next;
};

struct {
  struct spinlock lock;
  struct run *freelist;
} kmem;

extern char end[]; // first address after kernel loaded from ELF file

struct {
  int count; // the number of elements in allocated list
  int alloclist[(int)PHYSTOP / PGSIZE]; // the list of allocated pages
}al;

// check if there has been any neighbor of the page whose address is passed in allocated
int 
has_neighbor(int r) 
{
  for (int i = 0; i < al.count; i++) {
    if (r == al.alloclist[i] + PGSIZE || r == al.alloclist[i] - PGSIZE)
      return 1;
  }

  return 0;
}

// Initialize free list of physical pages.
void
kinit(void)
{
  char *p;

  initlock(&kmem.lock, "kmem");
  p = (char*)PGROUNDUP((uint)end);
  for(; p + PGSIZE <= (char*)PHYSTOP; p += PGSIZE) {
    kfree(p);
  }

  al.count = 0;
}

// Free the page of physical memory pointed at by v,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
void
kfree(char *v)
{
  struct run *r;

  if((uint)v % PGSIZE || v < end || (uint)v >= PHYSTOP) 
    panic("kfree");

  // Fill with junk to catch dangling refs.
  memset(v, 1, PGSIZE);

  acquire(&kmem.lock);
  r = (struct run*)v;
  r->next = kmem.freelist;
  kmem.freelist = r;
  release(&kmem.lock);

  // Remove the free page from allocated list
  for (int i = 0; i < al.count; i++) {
    if (al.alloclist[i] == (int)v) {
      for (int j = i; j < al.count - 1; j++)
        al.alloclist[j] = al.alloclist[j + 1];

      al.alloclist[al.count - 1] = 0;
      al.count--;
    }
  }
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
char*
kalloc(void)
{
  struct run *r;

  acquire(&kmem.lock);
  r = kmem.freelist;
  // update the head of linked list if r is the first element
  if (!has_neighbor((int)r)) {
    kmem.freelist = r->next;
    release(&kmem.lock);
    // add the allocated page to allocate list
    al.alloclist[al.count++] = (int) r;
    return (char*)r;  
  }
  release(&kmem.lock);

  while (r->next != 0) {
    if (!has_neighbor((int) r->next)) {
      char* alloc = (char*) r->next;
      r->next = r->next->next;
      // add the allocated page to allocate list
      al.alloclist[al.count++] = (int) alloc;
      return alloc;
    } else {
      r = r->next;
    }
  }
  
  return (char*)r;
}

// return the frame numbers of the frames which have been allocated
int dump_allocated(int *frames, int numframes) {
  if (numframes > al.count || numframes < 0) return -1;

  for (int i = 0; i < numframes; i++)
    frames[i] = al.alloclist[al.count - 1 - i];
  
  return 0;
}


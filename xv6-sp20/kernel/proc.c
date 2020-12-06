#include "types.h"
#include "defs.h"
#include "param.h"
#include "mmu.h"
#include "x86.h"
#include "proc.h"
#include "spinlock.h"
#include "pstat.h"

struct {
  struct spinlock lock;
  struct proc proc[NPROC];
} ptable;

static struct proc *initproc;

// the multi level feedback queue used for scheduler
struct {
    // the queues storing addresses of all processes
    struct proc *queue[4][NPROC];
    // the number of ticks of each
    int tick_cnt[4][NPROC];
    // the size of each queue
    int qsize[4];
} mlfq;

int nextpid = 1;
extern void forkret(void);
extern void trapret(void);

static void wakeup1(void *chan);

void
pinit(void)
{
  initlock(&ptable.lock, "ptable");
}

// Initialize the mlfq
void 
init_mlfq(void) 
{
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < NPROC; j++) mlfq.queue[i][j] = 0;
    mlfq.qsize[i] = 0;
  }
}

// Update the time slice of the process
void 
q_update_timeslice(struct proc *p) 
{
  if (p->priority == 3) {
    p->single_time_slice = 8;
    p->rr_time_slice = 1;
  } else if (p->priority == 2) {
    p->single_time_slice = 16;
    p->rr_time_slice = 2;
  } else if (p->priority == 1) {
    p->single_time_slice = 32;
    p->rr_time_slice = 4;
  } else {
    p->single_time_slice = 0;
    p->rr_time_slice = 64;
  }
}

// Add a new process to the MLFQ when it is created
void 
q_add_proc(struct proc *p) 
{
  mlfq.queue[3][mlfq.qsize[3]++] = p;

  p->ticks = 0;
  p->rr_ticks = 0;
  for (int i = 0; i < 3; i++) p->total_ticks[i] = 0;
  for (int i = 0; i < 3; i++) p->wait[i] = 0;
  p->priority = 3;
  q_update_timeslice(p);
}

// Remove a process from its current level
void 
q_remove_proc(struct proc *p) 
{
  for (int i = 0; i < mlfq.qsize[p->priority]; i++) {
    if (mlfq.queue[p->priority][i] == p) {
      // move forward all processes after the found one
      for (int j = i; j < mlfq.qsize[p->priority] - 1; j++) 
        mlfq.queue[p->priority][j] = mlfq.queue[p->priority][j + 1];

      // remove the process from the queue and update size
      mlfq.queue[p->priority][mlfq.qsize[p->priority] - 1] = 0;
      mlfq.qsize[p->priority]--;
      return;
    }
  }
}

// Downgrade a process. It does nothing if process is on the lowest level
void 
q_downgrade_proc(struct proc *p) 
{
  if (p->priority == 0) return;

  // Add process to the tail of the queue at lower level
  mlfq.queue[p->priority - 1][mlfq.qsize[p->priority - 1]++] = p;
  // Remove process from current level
  q_remove_proc(p);
  p->wait[p->priority] = 0;
  p->priority--;
  p->ticks = 0;
  p->rr_ticks = 0;
  p->wait[p->priority] = 0;
  q_update_timeslice(p);
}

// Boost up a process to prevent starvation
void
q_boost_proc(struct proc* p)
{
  if (p->priority < 3) {
    q_remove_proc(p);
    p->wait[(p->priority)++] = 0;
    mlfq.queue[p->priority][(mlfq.qsize[p->priority])++] = p;
    q_update_timeslice(p);
    p->wait[p->priority] = 0;
    p->ticks = 0;
    p->rr_ticks = 0;
  }
}

// Update wait ticks of all other RUNNABLE processes except for p.
// Boost up processes if needed
void 
q_wait_proc(struct proc *p) 
{
  for (int i = 3; i >= 0; i--) {
    for (int j = 0; j < mlfq.qsize[i]; j++) {
      struct proc *cur_p = mlfq.queue[i][j];

      if (cur_p != p && cur_p->state == RUNNABLE) cur_p->wait[i]++;
    }
  }
}

void
q_boost_all(void)
{
  for (int i = 3; i >= 0; i--) {
    int j = 0;

    while(j < mlfq.qsize[i]) {
      struct proc *cur_p = mlfq.queue[i][j];

      int wait_t = (i == 0) ? 640 : (10 * cur_p->single_time_slice);
      
      if (cur_p->wait[cur_p->priority] >= wait_t) {
        q_boost_proc(cur_p);       
        continue;
      }

      j++;
    }
  }
}

// Move the process to the end of its queue
void 
q_move_back_proc(struct proc *p) 
{
  for (int i = 0; i < mlfq.qsize[p->priority]; i++) {
    if (mlfq.queue[p->priority][i] == p) {
      // move forward all processes after the found one
      for (int j = i; j < mlfq.qsize[p->priority] - 1; j++) 
        mlfq.queue[p->priority][j] = mlfq.queue[p->priority][j + 1];

      mlfq.queue[p->priority][mlfq.qsize[p->priority] - 1] = p;
      p->rr_ticks = 0;

      return;
    }
  }
}

// Look in the process table for an UNUSED proc.
// If found, change state to EMBRYO and initialize
// state required to run in the kernel.
// Otherwise return 0.
static struct proc*
allocproc(void)
{
  struct proc *p;
  char *sp;

  acquire(&ptable.lock);
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    if(p->state == UNUSED)
      goto found;
  release(&ptable.lock);
  return 0;

found:
  p->state = EMBRYO;
  p->pid = nextpid++;
  release(&ptable.lock);

  // Allocate kernel stack if possible.
  if((p->kstack = kalloc()) == 0){
    p->state = UNUSED;
    return 0;
  }
  sp = p->kstack + KSTACKSIZE;
  
  // Leave room for trap frame.
  sp -= sizeof *p->tf;
  p->tf = (struct trapframe*)sp;
  
  // Set up new context to start executing at forkret,
  // which returns to trapret.
  sp -= 4;
  *(uint*)sp = (uint)trapret;

  sp -= sizeof *p->context;
  p->context = (struct context*)sp;
  memset(p->context, 0, sizeof *p->context);
  p->context->eip = (uint)forkret;

  return p;
}

// Set up first user process.
void
userinit(void)
{
  struct proc *p;
  extern char _binary_initcode_start[], _binary_initcode_size[];
  
  p = allocproc();
  acquire(&ptable.lock);
  initproc = p;
  if((p->pgdir = setupkvm()) == 0)
    panic("userinit: out of memory?");
  inituvm(p->pgdir, _binary_initcode_start, (int)_binary_initcode_size);
  p->sz = PGSIZE;
  memset(p->tf, 0, sizeof(*p->tf));
  p->tf->cs = (SEG_UCODE << 3) | DPL_USER;
  p->tf->ds = (SEG_UDATA << 3) | DPL_USER;
  p->tf->es = p->tf->ds;
  p->tf->ss = p->tf->ds;
  p->tf->eflags = FL_IF;
  p->tf->esp = PGSIZE;
  p->tf->eip = 0;  // beginning of initcode.S

  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  p->state = RUNNABLE;
  q_add_proc(p); // add first user process to the queue
  release(&ptable.lock);
}

// Grow current process's memory by n bytes.
// Return 0 on success, -1 on failure.
int
growproc(int n)
{
  uint sz;
  
  sz = proc->sz;
  if(n > 0){
    if((sz = allocuvm(proc->pgdir, sz, sz + n)) == 0)
      return -1;
  } else if(n < 0){
    if((sz = deallocuvm(proc->pgdir, sz, sz + n)) == 0)
      return -1;
  }
  proc->sz = sz;
  switchuvm(proc);
  return 0;
}

// Create a new process copying p as the parent.
// Sets up stack to return as if from system call.
// Caller must set state of returned proc to RUNNABLE.
int
fork(void)
{
  int i, pid;
  struct proc *np;

  // Allocate process.
  if((np = allocproc()) == 0)
    return -1;

  // Copy process state from p.
  if((np->pgdir = copyuvm(proc->pgdir, proc->sz)) == 0){
    kfree(np->kstack);
    np->kstack = 0;
    np->state = UNUSED;
    return -1;
  }
  np->sz = proc->sz;
  np->parent = proc;
  *np->tf = *proc->tf;

  // Clear %eax so that fork returns 0 in the child.
  np->tf->eax = 0;

  for(i = 0; i < NOFILE; i++)
    if(proc->ofile[i])
      np->ofile[i] = filedup(proc->ofile[i]);
  np->cwd = idup(proc->cwd);
 
  pid = np->pid;
  np->state = RUNNABLE;
  q_add_proc(np);
  safestrcpy(np->name, proc->name, sizeof(proc->name));
  return pid;
}

// Exit the current process.  Does not return.
// An exited process remains in the zombie state
// until its parent calls wait() to find out it exited.
void
exit(void)
{
  struct proc *p;
  int fd;

  if(proc == initproc)
    panic("init exiting");

  // Close all open files.
  for(fd = 0; fd < NOFILE; fd++){
    if(proc->ofile[fd]){
      fileclose(proc->ofile[fd]);
      proc->ofile[fd] = 0;
    }
  }

  iput(proc->cwd);
  proc->cwd = 0;

  acquire(&ptable.lock);

  // Parent might be sleeping in wait().
  wakeup1(proc->parent);

  // Pass abandoned children to init.
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->parent == proc){
      p->parent = initproc;
      if(p->state == ZOMBIE)
        wakeup1(initproc);
    }
  }

  // Jump into the scheduler, never to return.
  proc->state = ZOMBIE;
  q_remove_proc(proc);  // remove the process from mlfq
  //cprintf("process %d is removed\n", p->pid);
  sched();
  panic("zombie exit");
}

// Wait for a child process to exit and return its pid.
// Return -1 if this process has no children.
int
wait(void)
{
  struct proc *p;
  int havekids, pid;

  acquire(&ptable.lock);
  for(;;){
    // Scan through table looking for zombie children.
    havekids = 0;
    for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
      if(p->parent != proc)
        continue;
      havekids = 1;
      if(p->state == ZOMBIE){
        // Found one.
        pid = p->pid;
        kfree(p->kstack);
        p->kstack = 0;
        freevm(p->pgdir);
        p->state = UNUSED;
        p->pid = 0;
        p->parent = 0;
        p->name[0] = 0;
        p->killed = 0;
        release(&ptable.lock);
        return pid;
      }
    }

    // No point waiting if we don't have any children.
    if(!havekids || proc->killed){
      release(&ptable.lock);
      return -1;
    }

    // Wait for children to exit.  (See wakeup1 call in proc_exit.)
    sleep(proc, &ptable.lock);  //DOC: wait-sleep
  }
}

// Per-CPU process scheduler.
// Each CPU calls scheduler() after setting itself up.
// Scheduler never returns.  It loops, doing:
//  - choose a process to run
//  - swtch to start running that process
//  - eventually that process transfers control
//      via swtch back to the scheduler.
void
scheduler(void)
{
  struct proc *p;

  for(;;){
    // Enable interrupts on this processor.
    sti();

    // Loop over MLFQ looking for process to run.
    acquire(&ptable.lock);
    
    q_boost_all();

    for (int i = 3; i >= 0; i--) {
      if (mlfq.qsize[i] == 0) {
        continue;
      }
      
      for (int j = 0; j < mlfq.qsize[i]; j++) {
        p = mlfq.queue[i][j];

        if (p->state != RUNNABLE) continue;
      
        p->ticks++;
        p->rr_ticks++;
        p->total_ticks[p->priority]++;
        p->wait[p->priority] = 0;         
        q_wait_proc(p);
        
        if (i != 0 && p->ticks >= p->single_time_slice) { 
          q_downgrade_proc(p);
        } else if (p->rr_ticks >= p->rr_time_slice) {
          q_move_back_proc(p);
        }
        
        goto run;
      }
    }

  run:
    // Switch to chosen process.  It is the process's
    // job to release ptable.lock and then reacquire it
    // before jumping back to us.
    proc = p;
    switchuvm(p);
    p->state = RUNNING;
    swtch(&cpu->scheduler, proc->context);
    switchkvm();

    // Process is done running for now.
    // It should have changed its p->state before coming
    // back.
    proc = 0;
    release(&ptable.lock);
  }
}

// Enter scheduler.  Must hold only ptable.lock
// and have changed proc->state.
void
sched(void)
{
  int intena;

  if(!holding(&ptable.lock))
    panic("sched ptable.lock");
  if(cpu->ncli != 1)
    panic("sched locks");
  if(proc->state == RUNNING)
    panic("sched running");
  if(readeflags()&FL_IF)
    panic("sched interruptible");
  intena = cpu->intena;
  swtch(&proc->context, cpu->scheduler);
  cpu->intena = intena;
}

// Give up the CPU for one scheduling round.
void
yield(void)
{
  acquire(&ptable.lock);  //DOC: yieldlock
  proc->state = RUNNABLE;
  sched();
  release(&ptable.lock);
}

// A fork child's very first scheduling by scheduler()
// will swtch here.  "Return" to user space.
void
forkret(void)
{
  // Still holding ptable.lock from scheduler.
  release(&ptable.lock);
  
  // Return to "caller", actually trapret (see allocproc).
}

// Atomically release lock and sleep on chan.
// Reacquires lock when awakened.
void
sleep(void *chan, struct spinlock *lk)
{
  if(proc == 0)
    panic("sleep");

  if(lk == 0)
    panic("sleep without lk");

  // Must acquire ptable.lock in order to
  // change p->state and then call sched.
  // Once we hold ptable.lock, we can be
  // guaranteed that we won't miss any wakeup
  // (wakeup runs with ptable.lock locked),
  // so it's okay to release lk.
  if(lk != &ptable.lock){  //DOC: sleeplock0
    acquire(&ptable.lock);  //DOC: sleeplock1
    release(lk);
  }

  // Go to sleep.
  proc->chan = chan;
  proc->state = SLEEPING;
  sched();

  // Tidy up.
  proc->chan = 0;

  // Reacquire original lock.
  if(lk != &ptable.lock){  //DOC: sleeplock2
    release(&ptable.lock);
    acquire(lk);
  }
}

// Wake up all processes sleeping on chan.
// The ptable lock must be held.
static void
wakeup1(void *chan)
{
  struct proc *p;

  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    if(p->state == SLEEPING && p->chan == chan)
      p->state = RUNNABLE;
}

// Wake up all processes sleeping on chan.
void
wakeup(void *chan)
{
  acquire(&ptable.lock);
  wakeup1(chan);
  release(&ptable.lock);
}

// Kill the process with the given pid.
// Process won't exit until it returns
// to user space (see trap in trap.c).
int
kill(int pid)
{
  struct proc *p;

  acquire(&ptable.lock);
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->pid == pid){
      p->killed = 1;
      // Wake process from sleep if necessary.
      if(p->state == SLEEPING)
        p->state = RUNNABLE;
      release(&ptable.lock);
      return 0;
    }
  }
  release(&ptable.lock);
  return -1;
}

// Print a process listing to console.  For debugging.
// Runs when user types ^P on console.
// No lock to avoid wedging a stuck machine further.
void
procdump(void)
{
  static char *states[] = {
  [UNUSED]    "unused",
  [EMBRYO]    "embryo",
  [SLEEPING]  "sleep ",
  [RUNNABLE]  "runble",
  [RUNNING]   "run   ",
  [ZOMBIE]    "zombie"
  };
  int i;
  struct proc *p;
  char *state;
  uint pc[10];
  
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->state == UNUSED)
      continue;
    if(p->state >= 0 && p->state < NELEM(states) && states[p->state])
      state = states[p->state];
    else
      state = "???";
    cprintf("%d %s %s", p->pid, state, p->name);
    if(p->state == SLEEPING){
      getcallerpcs((uint*)p->context->ebp+2, pc);
      for(i=0; i<10 && pc[i] != 0; i++)
        cprintf(" %p", pc[i]);
    }
    cprintf("\n");
  }
}

// Extract useful information for each process
int 
getprocinfo(struct pstat *stats) 
{
  if (stats == 0) return -1;

  struct proc *p;

  for (p = ptable.proc; p < &ptable.proc[NPROC]; p++) {
    // the process's index in ptable
    int pindex = p - &ptable.proc[0];

    if (p->state != UNUSED) {
      stats->inuse[pindex] = 1;
      stats->pid[pindex] = p->pid;
      stats->state[pindex] = p->state;
      stats->priority[pindex] = p->priority;
      for (int i = 0; i < 4; i++) stats->ticks[pindex][i] = p->total_ticks[i];
      for (int i = 0; i < 4; i++) stats->wait_ticks[pindex][i] = p->wait[i];
    } else {
      stats->inuse[pindex] = 0;
    }
  }

  return 0;
}

// Boost up current process one level up if it is not in priority 3
int
boostproc(void)
{
  if (proc->priority < 3) q_boost_proc(proc);
  return 0;
}



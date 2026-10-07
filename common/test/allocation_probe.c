// Private validation only: LD_PRELOAD glibc allocation counters, no sampler
// thread and no allocation backtrace. Read the shared file from the test driver.
#define _GNU_SOURCE
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>

extern void *__libc_malloc(size_t);
extern void *__libc_calloc(size_t,size_t);
extern void *__libc_realloc(void *,size_t);
extern void __libc_free(void *);
struct counters { uint64_t tid, mallocs, callocs, reallocs, frees, bytes, publish_calls, publish_nanos, publish_max_nanos, publish_histogram[8], polls, poll_nanos, pad[5]; };
struct shared { uint64_t magic, count, capacity, overflow; struct counters threads[512]; };
static struct shared *stats;
static __thread struct counters *local;
static struct counters *current(void) {
  if (!stats) return NULL;
  if (!local) {
    uint64_t index=__atomic_fetch_add(&stats->count,1,__ATOMIC_RELAXED);
    if(index>=512){__atomic_fetch_add(&stats->overflow,1,__ATOMIC_RELAXED);return NULL;}
    local=&stats->threads[index];local->tid=syscall(SYS_gettid);
  }
  return local;
}
__attribute__((constructor)) static void initialize(void) {
  const char *root=getenv("ROBOT_SERVER_BUILD_ROOT");if(!root)root="/work";
  char path[512];snprintf(path,sizeof(path),"%s/alloc-%d.bin",root,getpid());
  int fd=open(path,O_CREAT|O_TRUNC|O_RDWR,0600);if(fd<0)return;
  if(ftruncate(fd,sizeof(struct shared))){close(fd);return;}
  void *mapping=mmap(NULL,sizeof(struct shared),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);close(fd);
  if(mapping==MAP_FAILED)return;
  stats=mapping;stats->magic=0x58474332414c4c4fULL;stats->capacity=512;
}
void *malloc(size_t size){void *p=__libc_malloc(size);struct counters *c=current();if(c){c->mallocs++;c->bytes+=size;}return p;}
void *calloc(size_t n,size_t size){void *p=__libc_calloc(n,size);struct counters *c=current();if(c){c->callocs++;c->bytes+=n*size;}return p;}
void *realloc(void *old,size_t size){void *p=__libc_realloc(old,size);struct counters *c=current();if(c){c->reallocs++;c->bytes+=size;}return p;}
void free(void *p){struct counters *c=p?current():NULL;if(c)c->frees++;__libc_free(p);}

void xgc2_probe_publish(uint64_t nanos) {
 struct counters *c=current();if(!c)return;
 c->publish_calls++;c->publish_nanos+=nanos;
 if(nanos>c->publish_max_nanos)c->publish_max_nanos=nanos;
 unsigned bin=0;while(bin<7 && nanos>(1000ULL<<bin))bin++;
 c->publish_histogram[bin]++;
}

int poll(struct pollfd *fds, nfds_t count, int timeout) {
 struct timespec begin,end;clock_gettime(CLOCK_MONOTONIC,&begin);
 struct timespec wait, *limit=NULL;
 if(timeout>=0){wait.tv_sec=timeout/1000;wait.tv_nsec=(timeout%1000)*1000000L;limit=&wait;}
 int result=syscall(SYS_ppoll,fds,count,limit,NULL,0);
 clock_gettime(CLOCK_MONOTONIC,&end);struct counters *c=current();
 if(c){c->polls++;c->poll_nanos+=(end.tv_sec-begin.tv_sec)*1000000000ULL+end.tv_nsec-begin.tv_nsec;}
 return result;
}

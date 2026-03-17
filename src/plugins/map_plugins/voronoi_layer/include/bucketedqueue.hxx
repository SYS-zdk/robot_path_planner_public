/**
 * @file: bucketedqueue.hxx
 * @brief: Bucketed priority queue template implementation (third-party, integrated)
 * @author Third-party (original authors unknown); integrated by Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#include "bucketedqueue.h"

#include "limits.h"
#include <stdio.h>
#include <stdlib.h>

template <class T>
BucketPrioQueue<T>::BucketPrioQueue() {
  clear();
}

template <class T>
bool BucketPrioQueue<T>::empty() {
  return (count==0);
}


template <class T>
void BucketPrioQueue<T>::push(int prio, T t) {
  buckets[prio].push(t);
  if (nextPop == buckets.end() || prio < nextPop->first) nextPop = buckets.find(prio);
  count++;
}

template <class T>
T BucketPrioQueue<T>::pop() {
  while (nextPop!=buckets.end() && nextPop->second.empty()) ++nextPop;

  T p = nextPop->second.front();
  nextPop->second.pop();
  if (nextPop->second.empty()) {
    typename BucketType::iterator it = nextPop;
    nextPop++;
    buckets.erase(it);
  }
  count--;
  return p;
}

/*
 * created by spikerheado1234.
 */

#ifndef TREE_CLOCK_H
#define TREE_CLOCK_H

// Forward declaration to circumvent mutual importing.
namespace rt_lib {
    template <class K, class V, size_t T>
        class array_map;
}

#include "bit_vector.h"
#include <stdlib.h>
#include <stdint.h>

namespace rtLocal {
    template <class K, class V>
        class pair {
            public:
                K k;
                V v;

                pair() = default;

                pair(K k, V v) : k(k), v(v) {  }

                K first() {
                    return this->k;
                }

                V second() {
                    return this->v;
                }


        };
    template <class V>
        class stack {
            public:
                // Makes my life easier to just have everything public.

                V s[1000]; // Maybe think of increasing the size later?
                int n = 0;
                stack() = default;

                void push(V value) {
                    this->s[n] = value;
                    this->n++;
                }

                V pop() {
                    V value = s[n-1];
                    this->n--;
                    return value;
                }

                V top() {
                    V value = s[n];
                    return value;
                }

                bool empty() {
                    return n == 0;
                }

                int size() {
                    return n + 1;
                }
        };

    class clock_data {
        private:
            static unsigned const int clock_bits = 32;
            static unsigned const int p_clock_bits = 32;
            static unsigned const long ones = -1;
            static unsigned const long p_clock_mask = (ones >> p_clock_bits) << p_clock_bits;
            static unsigned const long clock_mask = (ones << clock_bits) >> clock_bits;

        public:
            clock_data() = default;
            // In all the subsequent methods data is A 64-bit integer, 
            // where the first 32 bits is data for p_clock, 
            // and the last 32 bits is data for the current clock.

            // TODO, fill up necessary methods over here.

            static long zero_clock_data(long data) {
                data = data >> clock_data::clock_bits;

                return data << clock_data::clock_bits;
            }

            static long zero_pclock_data(long data) {
                data = data << clock_data::p_clock_bits;

                return data >> clock_data::p_clock_bits;
            }

            static long set_clock(long data, int clock_value) {
                data = clock_data::zero_clock_data(data);
                data |= (long)clock_value;

                return data;
            }

            static long set_pclock(long data, int pclock_value) {
                data = clock_data::zero_pclock_data(data);
                data |= ((long)pclock_value) << p_clock_bits;

                return data;
            }

            static int get_clock(long data) {
                return data & clock_data::clock_mask;
            }

            static int get_pclock(long data) {
                return data >> clock_data::p_clock_bits;
            }

            static long add_clock(long data, int incr) {
                int clk_val = clock_data::get_clock(data);
                clk_val += incr;
                data = clock_data::set_clock(data, clk_val);

                return data;
            }

            static long add_pclock(long data, int incr) {
                int pclk_val = clock_data::get_pclock(data);
                pclk_val += incr;
                data = clock_data::set_pclock(data, pclk_val);

                return data;
            }

            static long make_clock(int clock_data, int p_clock) {
                long data = 0L;
                data |= (((long)p_clock) << p_clock_bits);
                data |= ((long)clock_data);

                return data;
            }
    };

    class tree_data {
        private:
            static const unsigned long ones = -1;
            static const unsigned long diff_bits = 16;
            static const unsigned long parent_mask = (ones >> (diff_bits * 3)) << (diff_bits * 3);
            static const unsigned long child_mask = (ones >> (diff_bits * 3)) << (diff_bits * 2);
            static const unsigned long left_sib_mask = (ones >> (diff_bits * 3)) << (diff_bits * 1);
            static const unsigned long right_sib_mask = (ones >> (diff_bits * 3));

            // In all subsequent methods, data is a 64-bit integer, 
            // (parent, first child, left sibling, right sibling) 
            // each comprising of 16 bits.
        public:

            tree_data() = default;

            static long zero_next(long data) {
                return data & (~right_sib_mask);
            }

            static long zero_prev(long data) {
                return data & (~left_sib_mask);
            }

            static long zero_child(long data) {
                return data & (~child_mask);
            }

            static long zero_parent(long data) {
                return data & (~parent_mask);
            }

            static short get_parent(long data) {
                return (data & tree_data::parent_mask) >> (tree_data::diff_bits * 3);
            }

            static short get_child(long data) {
                return (data & tree_data::child_mask) >> (tree_data::diff_bits * 2);
            }

            static short get_prev(long data) {
                return (data & tree_data::left_sib_mask) >> (tree_data::diff_bits * 1);
            }

            static short get_next(long data) {
                return (data & tree_data::right_sib_mask); 
            }

            static long set_next(long data, short next_tid) {
                long next_data = next_tid;
                data = tree_data::zero_next(data);
                data |= next_data;
                return data;
            }

            static long set_prev(long data, short prev_tid) {
                long prev_data = prev_tid;
                prev_data = prev_data << diff_bits;
                data = tree_data::zero_prev(data);
                data |= prev_data;
                return data;
            }

            static long set_child(long data, short next_child) {
                long next_child_data = next_child;
                next_child_data = next_child_data << (diff_bits * 2);
                data = tree_data::zero_child(data);
                data |= next_child_data;
                return data;
            }

            static long set_parent(long data, short next_parent) {
                long next_parent_data = next_parent;
                next_parent_data = next_parent_data << (diff_bits * 3);
                data = tree_data::zero_parent(data);
                data |= next_parent_data;
                return data;
            }

            static long make_tnode(unsigned short parent, unsigned short child, unsigned short prev, unsigned short next) {
                long parent_data = ((long)parent) << (diff_bits * 3); 
                long child_data = ((long)child) << (diff_bits * 2);
                long prev_data = ((long)prev) << (diff_bits * 1);
                long next_data = ((long)next);

                long data = parent_data | child_data | prev_data | next_data;
                return data;
            }
    };
}

namespace rtLib {
    template <size_t T = 64> // We default the size of this to be 64.
        class tree_clock {
            public:
                // Some macro-ing for to make my life easier.
                typedef rtLocal::stack<rtLocal::pair<unsigned short, long>> stackType;

                long * clocks;
                long * tree;
                int root_id; // Stores the tid that is the current root "node" of the tree_clock.
                int dimension;
                stackType* s;

                tree_clock() { // This is used to initialize lock tree_clocks.
                    this->root_id = -1;
                    clocks = (long*)malloc(sizeof(long)*T);
                    tree = (long*)malloc(sizeof(long)*T);
                    s = (stackType*)malloc(sizeof(stackType));

                    for (int i = 0; i < T + 1; i++) {
                        this->clocks[i] = 0L;
                        this->tree[i] = 0L;
                    }
                }

                tree_clock(unsigned int tid, unsigned int dimension) { // This is used to initialize thread tree_clocks.
                    dimension++; // No OOB errors since we are converting tid from 0-based to 1-based indexing.
                    this->dimension = dimension;
                    clocks = (long*)malloc(sizeof(long) * dimension);
                    tree = (long*)malloc(sizeof(long) * dimension);
                    s = (stackType*)malloc(sizeof(stackType));


                    for (int i = 0; i < dimension + 1; i++) {
                        this->clocks[i] = 0L;
                        this->tree[i] = 0L;
                    }

                    // Legality is defined as having a tree_node where the parent is non-zero then.
                    // All of thtis is additional work that is to be done
                    // for thread TCs.
                    tid++; 
                    this->root_id = tid;
                    this->tree[tid] = rtLocal::tree_data::make_tnode(tid, 0, 0, 0);
                }

                void deep_copy(const tree_clock& other) {
                    this->root_id = other.root_id; 
                    // Copy over the the clock and tree values.
                    for (int i = 0; i < other.dimension; i++) {
                        this->clocks[i] = other.clocks[i];
                        this->tree[i] = other.tree[i];
                    } 
                    // Copy over the stack values.
                    for (int i = 0; i < other.s->n; i++) {
                        this->s->s[i] = other.s->s[i];
                    }
                    this->s->n = other.s->n;
                }

                // Operators for tree over here.
                bool operator<(tree_clock& other) {
                    unsigned int curr_clock = rtLocal::clock_data::get_clock(this->clocks[this->root_id]);
                    unsigned int other_clock = rtLocal::clock_data::get_clock(other.clocks[other.root_id]);

                    return curr_clock < other_clock;
                }

                bool operator>(tree_clock& other) {
                    unsigned int curr_clock = rtLocal::clock_data::get_clock(this->clocks[this->root_id]);
                    unsigned int other_clock = rtLocal::clock_data::get_clock(other.clocks[other.root_id]);

                    return curr_clock > other_clock; 
                }

                bool operator>=(tree_clock& other) {
                    unsigned int curr_clock = rtLocal::clock_data::get_clock(this->clocks[this->root_id]);
                    unsigned int other_clock = rtLocal::clock_data::get_clock(other.clocks[other.root_id]);

                    return curr_clock >= other_clock; 
                }

                bool operator<=(tree_clock& other) {
                    unsigned int curr_clock = rtLocal::clock_data::get_clock(this->clocks[this->root_id]);
                    unsigned int other_clock = rtLocal::clock_data::get_clock(other.clocks[other.root_id]);

                    return curr_clock <= other_clock; 
                }

                bool operator<=(rt_lib::array_map<long long int, short int, T>& other) {
                    for (int i = 0; i < T; i++) {
                        if (other.v[i] < rtLocal::clock_data::get_clock(this->clocks[i+1])) {
                            return false;
                        }
                    }
                    return true;
                }

                bool operator==(tree_clock& other) {
                    unsigned int curr_clock = rtLocal::clock_data::get_clock(this->clocks[this->root_id]);
                    unsigned int other_clock = rtLocal::clock_data::get_clock(other.clocks[other.root_id]);

                    return curr_clock == other_clock; 
                }

                // Some useful helper functions.

                // Get the clock for thread t.
                unsigned int get(int thr) {
                    return rtLocal::clock_data::get_clock(this->clocks[thr]);
                }

                void new_clock(int thr, int clock_value) {
                    thr++;
                    this->clocks[thr] = rtLocal::clock_data::set_clock(this->clocks[thr], clock_value);
                }

                tree_clock& operator=(const tree_clock& other) {
                    this->deep_copy(other);
                    return *this;
                }

                unsigned int operator[](int val) { // For code re-use we must convert val from 0-based to 1-based indexing.
                    val++; // TODO, check if required.
                    return get(val);
                }

                // Increment the clock of the root
                // thread.
                void increment(int incr) {
                    long root_data = this->clocks[this->root_id];
                    long new_data = rtLocal::clock_data::add_clock(root_data, incr);
                    this->clocks[this->root_id] = new_data;
                }

                // Helper methods for the join operation.
                void get_updated_nodes_join(stackType* s, unsigned short u_prime_tid, tree_clock& other) {
                    unsigned short child_tid = rtLocal::tree_data::get_child(other.tree[u_prime_tid]);
                    long child_clock = other.clocks[child_tid];
                    while (child_tid != 0) {
                        if (this->get(child_tid) < rtLocal::clock_data::get_clock(child_clock)) {
                            get_updated_nodes_join(s, child_tid, other);
                        } else if (rtLocal::clock_data::get_pclock(child_clock) <= this->get(u_prime_tid)) {
                            break;
                        }

                        // Update the child at the end.
                        // For the next iteration.
                        child_tid = rtLocal::tree_data::get_next(other.tree[child_tid]);
                        child_clock = other.clocks[child_tid];
                    }
                    s->push(rtLocal::pair<unsigned short, long>(u_prime_tid, other.clocks[u_prime_tid])); 
                }

                void detach_nodes(stackType* s) {
                    for (int i = s->n - 1; i >= 0; i--) {
                        unsigned short tid = (s->s)[i].k;

                        if ((rtLocal::tree_data::get_parent(this->tree[tid]) != 0) && (rtLocal::tree_data::get_parent(this->tree[tid]) != tid)) {
                            if (tid != this->root_id) {
                                short x_tid = rtLocal::tree_data::get_parent(this->tree[tid]);
                                if (rtLocal::tree_data::get_child(this->tree[x_tid]) == tid) {
                                    this->tree[x_tid] = rtLocal::tree_data::set_child(this->tree[x_tid], rtLocal::tree_data::get_next(this->tree[tid])); 
                                }

                                unsigned short prev_tid = rtLocal::tree_data::get_prev(this->tree[tid]);
                                unsigned short next_tid = rtLocal::tree_data::get_next(this->tree[tid]);

                                this->tree[prev_tid] = rtLocal::tree_data::set_next(tree[prev_tid], next_tid);
                                this->tree[next_tid] = rtLocal::tree_data::set_prev(tree[next_tid], prev_tid);
                            }
                        }
                    }
                }

                void attach_nodes(stackType* s, tree_clock& other) { // Must check correctness over here.
                    while(!s->empty()) {
                        rtLocal::pair<unsigned short, long> u_prime = s->pop();
                        unsigned short u_prime_tid = u_prime.k;
                        long u_prime_data = u_prime.v;
                        long u_data = 0L;
                        long u_tree_node = 0L;
                        if (rtLocal::tree_data::get_parent(this->tree[u_prime_tid]) == 0) {
                            // Create a new node then.
                            u_tree_node = rtLocal::tree_data::make_tnode(u_prime_tid, 0, 0, 0);
                            this->clocks[u_prime_tid] = u_data;
                            this->tree[u_prime_tid] = u_tree_node;
                        } else {
                            u_data = this->clocks[u_prime_tid];
                            u_tree_node = this->tree[u_prime_tid];
                        }
                        u_data = rtLocal::clock_data::set_clock(u_data, rtLocal::clock_data::get_clock(u_prime_data));
                        this->clocks[u_prime_tid] = u_data;
                        unsigned short y_prime_tid = rtLocal::tree_data::get_parent(other.tree[u_prime_tid]);
                        if (y_prime_tid != 0 && y_prime_tid != u_prime_tid) {
                            u_data = rtLocal::clock_data::set_pclock(u_data, rtLocal::clock_data::get_pclock(u_prime_data));
                            this->clocks[u_prime_tid] = u_data;
                            push_child(this->tree[u_prime_tid], this->tree[y_prime_tid], u_prime_tid, y_prime_tid);
                        }
                    }
                }

                void get_updated_nodes_copy(stackType* s, tree_clock& other, int u_prime_tid, int z_tid) {
                    unsigned short child_tid = rtLocal::tree_data::get_child(other.tree[u_prime_tid]);
                    long child_data = other.clocks[child_tid];

                    while (child_tid != 0 && rtLocal::tree_data::get_parent(other.tree[child_tid]) != 0) {
                        if (other.get(child_tid) < this->get(child_tid)) {
                            get_updated_nodes_copy(s, other, child_tid, z_tid);
                        } else {
                            if ((z_tid != 0) && (child_tid == z_tid)) {
                                s->push(rtLocal::pair<unsigned short, long>(child_tid, child_data));
                            } 
                            if (rtLocal::clock_data::get_pclock(child_data) <= this->get(u_prime_tid)) {
                                break;
                            }
                        } 
                        // Update to next child_tid and child_data.
                        child_tid = rtLocal::tree_data::get_next(other.tree[child_tid]);
                        child_data = other.clocks[child_tid];
                    }
                    s->push(rtLocal::pair<unsigned short, long>(u_prime_tid, other.clocks[u_prime_tid]));
                }

                void push_child(unsigned long u_data, unsigned long v_data, unsigned short u_tid, unsigned short v_tid) {
                    // set attributes of u_data.
                    u_data = rtLocal::tree_data::set_parent(u_data, v_tid);
                    u_data = rtLocal::tree_data::set_next(u_data, rtLocal::tree_data::get_child(v_data));
                    u_data = rtLocal::tree_data::set_prev(u_data, 0);
                    // set attributes of original child of v_data.
                    unsigned short v_orig_child_tid = rtLocal::tree_data::get_child(v_data);
                    this->tree[v_orig_child_tid] = rtLocal::tree_data::set_prev(this->tree[v_orig_child_tid], u_tid);
                    // set attributes of v_data.
                    v_data = rtLocal::tree_data::set_child(v_data, u_tid);
                    this->tree[u_tid] = u_data;
                    this->tree[v_tid] = v_data;
                }

                void monotone_copy(tree_clock& other) {
                    // This means we have lock joined with thread and the lock had no prior
                    // information contained within it.
                    if (this->root_id < 0) {
                        this->deep_copy(other);
                        return;
                    }
                    short z_prime_tid = other.root_id;
                    short z_tid = this->root_id;
                    stackType s;
                    get_updated_nodes_copy(&s, other, z_prime_tid, z_tid);
                    detach_nodes(&s);
                    attach_nodes(&s, other);

                    // Assign new root.
                    this->root_id = z_prime_tid;
                }

                // The implementation of the join operator.
                void operator|=(tree_clock& other) {
                    // This means we have a thread joined with a lock 
                    // and the lock has no information stored within it. 
                    // We can do nothing here and do all the work in 
                    // the first release of the lock then.
                    if (this->root_id == other.root_id || other.root_id < 0) { 
                        return;
                    }
                    unsigned short z_prime_tid = other.root_id;
                    if (other.get(z_prime_tid) <= this->get(z_prime_tid)) {
                        return;     
                    }
                    stackType stack;
                    get_updated_nodes_join(&stack, z_prime_tid, other);
                    detach_nodes(&stack);
                    attach_nodes(&stack, other);

                    // Place updated subtree under root of current tree.
                    long w_data = this->clocks[z_prime_tid];
                    unsigned short z_tid = this->root_id;
                    long z_data = this->clocks[z_tid];
                    w_data = rtLocal::clock_data::set_pclock(w_data, rtLocal::clock_data::get_clock(z_data));
                    this->clocks[z_prime_tid] = w_data;
                    push_child(this->tree[z_prime_tid], this->tree[z_tid], z_prime_tid, z_tid);
                }
        };
}

#endif // For the TREE_CLOCK_H declaration.

#ifndef DCP_INLINE_H
#define DCP_INLINE_H

#if PICO_RP2350

#include <stdint.h>
#include <string.h>

// Inline Assembly for RP2350 DCP (Double Coprocessor)
// Allows direct access to hardware double precision accumulator without function call overhead.

static inline double dcp_dmul(double da, double db) {
    uint64_t ua, ub, ur;
    uint32_t a, b, c, d, e, f, g;
    memcpy(&ua, &da, 8); memcpy(&ub, &db, 8);
    __asm__ volatile (
        "mcrr  p4,   #1,   %Q[ua],%R[ua], c0\n\t"    // WXUP ua
        "mcrr  p4,   #1,   %Q[ub],%R[ub], c1\n\t"    // WYUP ub
        "mrrc  p4,   #0,   %[a],  %[b],   c4\n\t"    // RXMS a, b
        "mrrc  p4,   #0,   %[c],  %[d],   c5\n\t"    // RYMS c, d
        "umull %[e], %[f], %[a],  %[c]\n\t"          // e,f = a * c
        "movs  %[g], #0\n\t"                         // g = 0
        "umlal %[f], %[g], %[a],  %[d]\n\t"          // f,g += a * d
        "umlal %[f], %[g], %[b],  %[c]\n\t"          // f,g += b * c
        "mcrr  p4,   #2,   %[e],  %[f],   c0\n\t"    // WXMS e,f
        "movs  %[e], #0\n\t"                         // e = 0
        "umlal %[g], %[e], %[b],  %[d]\n\t"          // g,e += b * d
        "mcrr  p4,   #3,   %[g],  %[e],   c0\n\t"    // WXMO g,e
        "cdp   p4,   #8,   c0,    c0,     c0,#1\n\t" // NRDD
        "mrrc  p4,   #5,   %Q[ur],%R[ur], c0\n\t"    // RDDM ur
        : [ur] "=r" (ur), [a] "=r" (a), [b] "=r" (b), [c] "=r" (c), [d] "=r" (d), [e] "=r" (e), [f] "=r" (f), [g] "=r" (g)
        : [ua] "r" (ua), [ub] "r" (ub)
    );
    double dr; memcpy(&dr, &ur, 8); return dr;
}

static inline double dcp_dadd(double a, double b) {
    uint64_t ua, ub, ur;
    memcpy(&ua, &a, 8); memcpy(&ub, &b, 8);
    __asm__ volatile (
        "mcrr p4, #1, %Q1, %R1, c0\n\t" // WXUP a
        "mcrr p4, #1, %Q2, %R2, c1\n\t" // WYUP b
        "cdp p4, #0, c0, c0, c1, #0\n\t" // ADD0
        "cdp p4, #1, c0, c0, c1, #0\n\t" // ADD1
        "cdp p4, #8, c0, c0, c0, #1\n\t" // NRDD
        "mrrc p4, #1, %Q0, %R0, c0\n\t" // RDDA result
        : "=r" (ur)
        : "r" (ua), "r" (ub)
    );
    double r; memcpy(&r, &ur, 8); return r;
}

static inline float dcp_dadd_d2f(double a, double b) {
    uint64_t ua, ub;
    uint32_t ur;
    memcpy(&ua, &a, 8); memcpy(&ub, &b, 8);
    __asm__ volatile (
        "mcrr p4, #1, %Q1, %R1, c0\n\t" // WXUP a
        "mcrr p4, #1, %Q2, %R2, c1\n\t" // WYUP b
        "cdp p4, #0, c0, c0, c1, #0\n\t" // ADD0
        "cdp p4, #1, c0, c0, c1, #0\n\t" // ADD1
        "cdp p4, #8, c0, c0, c2, #1\n\t" // NRDF
        "mrc p4, #0, %0, c0, c2, #0\n\t" // RDFA result
        : "=r" (ur)
        : "r" (ua), "r" (ub)
    );
    float r; memcpy(&r, &ur, 4); return r;
}

static inline double dcp_dsub(double a, double b) {
    uint64_t ua, ub, ur;
    memcpy(&ua, &a, 8); memcpy(&ub, &b, 8);
    __asm__ volatile (
        "mcrr p4, #1, %Q1, %R1, c0\n\t" // WXUP a
        "mcrr p4, #1, %Q2, %R2, c1\n\t" // WYUP b
        "cdp p4, #0, c0, c0, c1, #0\n\t" // ADD0
        "cdp p4, #1, c0, c0, c1, #1\n\t" // SUB1
        "cdp p4, #8, c0, c0, c0, #1\n\t" // NRDD
        "mrrc p4, #3, %Q0, %R0, c0\n\t" // RDDS result
        : "=r" (ur)
        : "r" (ua), "r" (ub)
    );
    double r; memcpy(&r, &ur, 8); return r;
}

static inline double dcp_f2d(float a) {
    uint32_t ua; memcpy(&ua, &a, 4);
    uint64_t ur;
    __asm__ volatile (
        "mcrr p4, #1, %1, %1, c2\n\t"    // WXYU a, a
        "cdp p4, #8, c0, c0, c0, #1\n\t" // NRDD
        "mrrc p4, #11, %Q0, %R0, c0\n\t" // RDDG result
        : "=r" (ur)
        : "r" (ua)
    );
    double r; memcpy(&r, &ur, 8); return r;
}

static inline float dcp_d2f(double a) {
    uint64_t ua; memcpy(&ua, &a, 8);
    uint32_t ur;
    __asm__ volatile (
        "mcrr p4, #1, %Q1, %R1, c0\n\t" // WXUP a
        "cdp p4, #8, c0, c0, c2, #1\n\t" // NRDF
        "mrc p4, #0, %0, c0, c2, #5\n\t" // RDFG result
        : "=r" (ur)
        : "r" (ua)
    );
    float r; memcpy(&r, &ur, 4); return r;
}

#endif // PICO_RP2350
#endif // DCP_INLINE_H

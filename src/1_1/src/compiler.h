// Compiler optimization hints for SiliCa
// JIS X 6319-4 compatible card implementation
//
// Shared across the physical and application layers so the hot paths use a
// single, consistent set of hints. These expand to GCC/Clang built-ins and
// have no effect on program behavior.

#pragma once

// Force a function to be inlined regardless of the optimizer's cost model.
#define FORCE_INLINE __attribute__((always_inline)) inline

// Branch prediction hints for hot conditionals.
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)

// Promise the compiler that a pointer does not alias others in scope.
#define RESTRICT __restrict__

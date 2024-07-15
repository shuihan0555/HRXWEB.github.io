---
title: NDEBUG模式下的assert断言不起作用
subtitle:
date: 2024-07-15 14:45:23 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: C/C++
show_tags: true
---

# assert 定义

```cpp
#ifdef  NDEBUG

# define assert(expr)           (__ASSERT_VOID_CAST (0))

# if defined __cplusplus
#  define assert(expr)                                                  \
     (static_cast <bool> (expr)                                         \
      ? void (0)                                                        \
      : __assert_fail (#expr, __FILE__, __LINE__, __ASSERT_FUNCTION))
# elif !defined __GNUC__ || defined __STRICT_ANSI__
#  define assert(expr)                                                  \
    ((expr)                                                             \
     ? __ASSERT_VOID_CAST (0)                                           \
     : __assert_fail (#expr, __FILE__, __LINE__, __ASSERT_FUNCTION))
# else
/* The first occurrence of EXPR is not evaluated due to the sizeof,
   but will trigger any pedantic warnings masked by the __extension__
   for the second occurrence.  The ternary operator is required to
   support function pointers and bit fields in this context, and to
   suppress the evaluation of variable length arrays.  */
#  define assert(expr)                                                  \
  ((void) sizeof ((expr) ? 1 : 0), __extension__ ({                     \
      if (expr)                                                         \
        ; /* empty */                                                   \
      else                                                              \
        __assert_fail (#expr, __FILE__, __LINE__, __ASSERT_FUNCTION);   \
    }))
# endif
```

在NDEBUG模式下，assert宏被定义为一个空操作。这意味着在NDEBUG模式下，assert宏不会执行任何操作，也不会产生任何错误或警告。

# 如何确定是否定义了NDEBUG

可以在项目中的 `CMakeCache.txt` 文件中找到是否有NDEBUG的定义。

一般 `CMAKE_BUILD_TYPE` 设置为 `Release` 或者 `MinSizeRel` 时，NDEBUG 会被定义。
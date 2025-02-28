/*
MIT License

Copyright (c) 2021 Meng Rao <raomeng1@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#pragma once
// #define FMT_HEADER_ONLY
#ifndef FMTLOG_TSCNS
  #include "fmtlog-tscns.h"
#endif
#include <fmt/format.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <type_traits>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#ifdef _WIN32
#define FAST_THREAD_LOCAL thread_local
#else
#define FAST_THREAD_LOCAL __thread
#endif

#ifndef FMT_NOEXCEPT
#define FMT_NOEXCEPT
#endif

// define FMTLOG_BLOCK=1 if log statement should be blocked when queue is full, instead of discarding the msg
#ifndef FMTLOG_BLOCK
#define FMTLOG_BLOCK 0
#endif

#define FMTLOG_LEVEL_DBG 0
#define FMTLOG_LEVEL_INF 1
#define FMTLOG_LEVEL_STRATEGY 2
#define FMTLOG_LEVEL_WRN 3
#define FMTLOG_LEVEL_ERR 4
#define FMTLOG_LEVEL_OFF 5

// define FMTLOG_ACTIVE_LEVEL to turn off low log level in compile time
#ifndef FMTLOG_ACTIVE_LEVEL
#define FMTLOG_ACTIVE_LEVEL FMTLOG_LEVEL_DBG
#endif

#ifndef FMTLOG_QUEUE_SIZE
#define FMTLOG_QUEUE_SIZE (1 << 20)
#endif

namespace fmtlogdetail
{
  template<typename Arg>
  struct UnrefPtr : std::false_type
  {
    using type = Arg;
  };

  template<>
  struct UnrefPtr<char*> : std::false_type
  {
    using type = char*;
  };

  template<>
  struct UnrefPtr<void*> : std::false_type
  {
    using type = void*;
  };

  template<typename Arg>
  struct UnrefPtr<std::shared_ptr<Arg>> : std::true_type
  {
    using type = Arg;
  };

  template<typename Arg, typename D>
  struct UnrefPtr<std::unique_ptr<Arg, D>> : std::true_type
  {
    using type = Arg;
  };

  template<typename Arg>
  struct UnrefPtr<Arg*> : std::true_type
  {
    using type = Arg;
  };

};  // namespace fmtlogdetail

template<int __ = 0>
class fmtlogT
{
 public:
  enum LogLevel : uint8_t
  {
    DBG = 0,
    INF,
    STRATEGY,
    WRN,
    ERR,
    OFF
  };

  // Preallocate thread queue for current thread
  static void preallocate() noexcept;

  // Set the file for logging
  static void setLogFile(const char* filename, bool truncate = false);

  // Set an existing FILE* for logging, if manageFp is false fmtlog will not buffer log internally
  // and will not close the FILE*
  static void setLogFile(FILE* fp, bool manageFp = false);

  // Collect log msgs from all threads and write to log file
  // If forceFlush = true, internal file buffer is flushed
  // User need to call poll() repeatedly if startPollingThread is not used
  static void poll(bool forceFlush = false);

  // Set flush delay in nanosecond
  // If there's msg older than ns in the buffer, flush will be triggered
  static void setFlushDelay(int64_t ns) noexcept;

  // If current msg has level >= flushLogLevel, flush will be triggered
  static void flushOn(LogLevel flushLogLevel) noexcept;

  static void forceFlushLogFile() noexcept;
  
  // If file buffer has more than specified bytes, flush will be triggered
  static void setFlushBufSize(uint32_t bytes) noexcept;

  // callback signature user can register
  // ns: nanosecond timestamp
  // level: logLevel
  // location: full file path with line num, e.g: /home/raomeng/fmtlog/fmtlog.h:45
  // basePos: file base index in the location
  // threadName: thread id or the name user set with setThreadName
  // msg: full log msg with header
  // bodyPos: log body index in the msg
  // logFilePos: log file position of this msg
  typedef void (*LogCBFn)(
      int64_t ns,
      LogLevel level,
      fmt::string_view location,
      size_t basePos,
      fmt::string_view threadName,
      fmt::string_view msg,
      size_t bodyPos,
      size_t logFilePos);

  // Set a callback function for all log msgs with a mininum log level
  static void setLogCB(LogCBFn cb, LogLevel minCBLogLevel) noexcept;

  typedef void (*LogQFullCBFn)(void* userData);
  static void setLogQFullCB(LogQFullCBFn cb, void* userData) noexcept;

  // Close the log file and subsequent msgs will not be written into the file,
  // but callback function can still be used
  static void closeLogFile() noexcept;

  // Set log header pattern with fmt named arguments
  static void setHeaderPattern(const char* pattern);

  // Set a name for current thread, it'll be shown in {t} part in header pattern
  static void setThreadName(const char* name) noexcept;

  // Set current log level, lower level log msgs will be discarded
  static inline void setLogLevel(LogLevel logLevel) noexcept;

  // Get current log level
  static inline LogLevel getLogLevel() noexcept;

  // return true if passed log level is not lower than current log level
  static inline bool checkLogLevel(LogLevel logLevel) noexcept;

  // Run a polling thread in the background with a polling interval in ns
  // Note that user must not call poll() himself when the thread is running
  static void startPollingThread(int64_t pollInterval = 1000000000) noexcept;

  static void handleLoggerEnding() noexcept;
  
  // Stop the polling thread
  static void stopPollingThread() noexcept;

  // https://github.com/MengRao/SPSC_Queue
  class SPSCVarQueueOPT
  {
   public:
    struct MsgHeader
    {
      inline void push(uint32_t sz)
      {
        *(volatile uint32_t*)&size = sz + sizeof(MsgHeader);
      }

      uint32_t size;
      uint32_t logId;
    };
    static constexpr uint32_t BLK_CNT = FMTLOG_QUEUE_SIZE / sizeof(MsgHeader);

    MsgHeader* allocMsg(uint32_t size) noexcept;

    MsgHeader* alloc(uint32_t size)
    {
      size += sizeof(MsgHeader);
      uint32_t blk_sz = (size + sizeof(MsgHeader) - 1) / sizeof(MsgHeader);
      if (blk_sz >= free_write_cnt)
      {
        uint32_t read_idx_cache = *(volatile uint32_t*)&read_idx;
        if (read_idx_cache <= write_idx)
        {
          free_write_cnt = BLK_CNT - write_idx;
          if (blk_sz >= free_write_cnt && read_idx_cache != 0)
          {  // wrap around
            blk[0].size = 0;
            blk[write_idx].size = 1;
            write_idx = 0;
            free_write_cnt = read_idx_cache;
          }
        }
        else
        {
          free_write_cnt = read_idx_cache - write_idx;
        }
        if (free_write_cnt <= blk_sz)
        {
          return nullptr;
        }
      }
      MsgHeader* ret = &blk[write_idx];
      write_idx += blk_sz;
      free_write_cnt -= blk_sz;
      blk[write_idx].size = 0;
      return ret;
    }

    inline const MsgHeader* front()
    {
      uint32_t size = blk[read_idx].size;
      if (size == 1)
      {  // wrap around
        read_idx = 0;
        size = blk[0].size;
      }
      if (size == 0)
        return nullptr;
      return &blk[read_idx];
    }

    inline void pop()
    {
      uint32_t blk_sz = (blk[read_idx].size + sizeof(MsgHeader) - 1) / sizeof(MsgHeader);
      *(volatile uint32_t*)&read_idx = read_idx + blk_sz;
    }

   private:
    alignas(64) MsgHeader blk[BLK_CNT] = {};
    uint32_t write_idx = 0;
    uint32_t free_write_cnt = BLK_CNT;

    alignas(128) uint32_t read_idx = 0;
  };

  struct ThreadBuffer
  {
    SPSCVarQueueOPT varq;
    bool shouldDeallocate = false;
    char name[32];
    size_t nameSize;
  };

  using TSCNS = FMTLOG_TSCNS;

  void init() {
    tscns.init();
    currentLogLevel = INF;
  }

  using Context = fmt::format_context;
  using MemoryBuffer = fmt::basic_memory_buffer<char, 10000>;
  typedef const char* (*FormatToFn)(
      fmt::string_view format,
      const char* data,
      MemoryBuffer& out,
      int& argIdx,
      std::vector<fmt::basic_format_arg<Context>>& args);

  static void
  registerLogInfo(uint32_t& logId, FormatToFn fn, const char* location, LogLevel level, std::string fmtString) noexcept;

  static void vformat_to(MemoryBuffer& out, fmt::string_view fmt, fmt::format_args args);

  static size_t formatted_size(fmt::string_view fmt, fmt::format_args args);

  static void vformat_to(char* out, fmt::string_view fmt, fmt::format_args args);

  static typename SPSCVarQueueOPT::MsgHeader* allocMsg(uint32_t size, bool logQFullCB) noexcept;

  TSCNS tscns;

  volatile LogLevel currentLogLevel;
  static FAST_THREAD_LOCAL ThreadBuffer* threadBuffer;

  template<typename Arg>
  static inline constexpr bool isNamedArg()
  {
    return fmt::detail::is_named_arg<fmt::remove_cvref_t<Arg>>::value;
  }

  template<typename Arg>
  struct unNamedType
  {
    using type = Arg;
  };

  template<typename Arg>
  struct unNamedType<fmt::detail::named_arg<char, Arg>>
  {
    using type = Arg;
  };

#if FMT_USE_NONTYPE_TEMPLATE_ARGS
  template<typename Arg, size_t N, fmt::detail::fixed_string<char, N> Str>
  struct unNamedType<fmt::detail::static_named_arg<Arg, char, N, Str>>
  {
    using type = Arg;
  };
#endif

  template<typename Arg>
  static inline constexpr bool isCstring()
  {
    return fmt::detail::mapped_type_constant<Arg, char>::value == fmt::detail::type::cstring_type;
  }

  template<typename Arg>
  static inline constexpr bool isString()
  {
    return fmt::detail::mapped_type_constant<Arg, char>::value == fmt::detail::type::string_type;
  }

  template<typename Arg>
  static inline constexpr bool needCallDtor()
  {
    using ArgType = fmt::remove_cvref_t<Arg>;
    if constexpr (isNamedArg<Arg>())
    {
      return needCallDtor<typename unNamedType<ArgType>::type>();
    }
    if constexpr (isString<Arg>())
      return false;
    return !std::is_trivially_destructible<ArgType>::value;
  }

  template<size_t CstringIdx>
  static inline constexpr size_t getArgSizes(size_t* cstringSize)
  {
    return 0;
  }

  template<size_t CstringIdx, typename Arg, typename... Args>
  static inline constexpr size_t getArgSizes(size_t* cstringSize, const Arg& arg, const Args&... args)
  {
    if constexpr (isNamedArg<Arg>())
    {
      return getArgSizes<CstringIdx>(cstringSize, arg.value, args...);
    }
    else if constexpr (isCstring<Arg>())
    {
      size_t len = strlen(arg) + 1;
      cstringSize[CstringIdx] = len;
      return len + getArgSizes<CstringIdx + 1>(cstringSize, args...);
    }
    else if constexpr (isString<Arg>())
    {
      size_t len = arg.size() + 1;
      return len + getArgSizes<CstringIdx>(cstringSize, args...);
    }
    else
    {
      return sizeof(Arg) + getArgSizes<CstringIdx>(cstringSize, args...);
    }
  }

  template<size_t CstringIdx>
  static inline constexpr char* encodeArgs(size_t* cstringSize, char* out)
  {
    return out;
  }

  template<size_t CstringIdx, typename Arg, typename... Args>
  static inline constexpr char* encodeArgs(size_t* cstringSize, char* out, Arg&& arg, Args&&... args)
  {
    if constexpr (isNamedArg<Arg>())
    {
      return encodeArgs<CstringIdx>(cstringSize, out, arg.value, std::forward<Args>(args)...);
    }
    else if constexpr (isCstring<Arg>())
    {
      memcpy(out, arg, cstringSize[CstringIdx]);
      return encodeArgs<CstringIdx + 1>(cstringSize, out + cstringSize[CstringIdx], std::forward<Args>(args)...);
    }
    else if constexpr (isString<Arg>())
    {
      size_t len = arg.size();
      memcpy(out, arg.data(), len);
      out[len] = 0;
      return encodeArgs<CstringIdx>(cstringSize, out + len + 1, std::forward<Args>(args)...);
    }
    else
    {
      // If Arg has alignment >= 16, gcc could emit aligned move instructions(e.g. movdqa) for
      // placement new even if the *out* is misaligned, which would cause segfault. So we use memcpy
      // when possible
      if constexpr (std::is_trivially_copyable_v<fmt::remove_cvref_t<Arg>>)
      {
        memcpy(out, &arg, sizeof(Arg));
      }
      else
      {
        new (out) fmt::remove_cvref_t<Arg>(std::forward<Arg>(arg));
      }
      return encodeArgs<CstringIdx>(cstringSize, out + sizeof(Arg), std::forward<Args>(args)...);
    }
  }

  template<size_t Idx, size_t NamedIdx>
  static inline constexpr void storeNamedArgs(fmt::detail::named_arg_info<char>* named_args_store)
  {
  }

  template<size_t Idx, size_t NamedIdx, typename Arg, typename... Args>
  static inline constexpr void
  storeNamedArgs(fmt::detail::named_arg_info<char>* named_args_store, const Arg& arg, const Args&... args)
  {
    if constexpr (isNamedArg<Arg>())
    {
      named_args_store[NamedIdx] = { arg.name, Idx };
      storeNamedArgs<Idx + 1, NamedIdx + 1>(named_args_store, args...);
    }
    else
    {
      storeNamedArgs<Idx + 1, NamedIdx>(named_args_store, args...);
    }
  }

  template<bool ValueOnly, size_t Idx, size_t DestructIdx>
  static inline const char* decodeArgs(const char* in, fmt::basic_format_arg<Context>* args, const char** destruct_args)
  {
    return in;
  }

  template<bool ValueOnly, size_t Idx, size_t DestructIdx, typename Arg, typename... Args>
  static inline const char* decodeArgs(const char* in, fmt::basic_format_arg<Context>* args, const char** destruct_args)
  {
    using namespace fmtlogdetail;
    using ArgType = fmt::remove_cvref_t<Arg>;
    if constexpr (isNamedArg<ArgType>())
    {
      return decodeArgs<ValueOnly, Idx, DestructIdx, typename unNamedType<ArgType>::type, Args...>(in, args, destruct_args);
    }
    else if constexpr (isCstring<Arg>() || isString<Arg>())
    {
      size_t size = strlen(in);
      fmt::string_view v(in, size);
      if constexpr (ValueOnly)
      {
        fmt::detail::value<Context>& value_ = *(fmt::detail::value<Context>*)(args + Idx);
        value_ = v;
      }
      else
      {
        args[Idx] = v;
      }
      return decodeArgs<ValueOnly, Idx + 1, DestructIdx, Args...>(in + size + 1, args, destruct_args);
    }
    else
    {
      if constexpr (ValueOnly)
      {
        fmt::detail::value<Context>& value_ = *(fmt::detail::value<Context>*)(args + Idx);
        if constexpr (UnrefPtr<ArgType>::value)
        {
          value_ = **(ArgType*)in;
        }
        else
        {
          value_ = *(ArgType*)in;
        }
      }
      else
      {
        if constexpr (UnrefPtr<ArgType>::value)
        {
          args[Idx] = **(ArgType*)in;
        }
        else
        {
          args[Idx] = *(ArgType*)in;
        }
      }

      if constexpr (needCallDtor<Arg>())
      {
        destruct_args[DestructIdx] = in;
        return decodeArgs<ValueOnly, Idx + 1, DestructIdx + 1, Args...>(in + sizeof(ArgType), args, destruct_args);
      }
      else
      {
        return decodeArgs<ValueOnly, Idx + 1, DestructIdx, Args...>(in + sizeof(ArgType), args, destruct_args);
      }
    }
  }

  template<size_t DestructIdx>
  static inline void destructArgs(const char** destruct_args)
  {
  }

  template<size_t DestructIdx, typename Arg, typename... Args>
  static inline void destructArgs(const char** destruct_args)
  {
    using ArgType = fmt::remove_cvref_t<Arg>;
    if constexpr (isNamedArg<ArgType>())
    {
      destructArgs<DestructIdx, typename unNamedType<ArgType>::type, Args...>(destruct_args);
    }
    else if constexpr (needCallDtor<Arg>())
    {
      ((ArgType*)destruct_args[DestructIdx])->~ArgType();
      destructArgs<DestructIdx + 1, Args...>(destruct_args);
    }
    else
    {
      destructArgs<DestructIdx, Args...>(destruct_args);
    }
  }

  template<typename... Args>
  static const char* formatTo(
      fmt::string_view format,
      const char* data,
      MemoryBuffer& out,
      int& argIdx,
      std::vector<fmt::basic_format_arg<Context>>& args)
  {
    constexpr size_t num_args = sizeof...(Args);
    constexpr size_t num_dtors = fmt::detail::count<needCallDtor<Args>()...>();
    const char* dtor_args[std::max(num_dtors, (size_t)1)];
    const char* ret;
    if (argIdx < 0)
    {
      argIdx = (int)args.size();
      args.resize(argIdx + num_args);
      ret = decodeArgs<false, 0, 0, Args...>(data, args.data() + argIdx, dtor_args);
    }
    else
    {
      ret = decodeArgs<true, 0, 0, Args...>(data, args.data() + argIdx, dtor_args);
    }
    vformat_to(out, format, fmt::basic_format_args(args.data() + argIdx, num_args));
    destructArgs<0, Args...>(dtor_args);

    return ret;
  }

  template<bool Reorder, typename... Args>
  static std::string unNameFormat(std::string in, uint32_t* reorderIdx, const Args&... args)
  {
    constexpr size_t num_named_args = fmt::detail::count<isNamedArg<Args>()...>();
    if constexpr (num_named_args == 0)
    {
      return in;
    }
    const char* begin = in.data();
    const char* p = begin;
    std::size_t size_unnamed_str = in.size() + 1 + num_named_args * 5;
    auto unnamed_str = std::make_unique<char[]>(size_unnamed_str);
    fmt::detail::named_arg_info<char> named_args[std::max(num_named_args, (size_t)1)];
    storeNamedArgs<0, 0>(named_args, args...);

    char* out = (char*)unnamed_str.get();
    uint8_t arg_idx = 0;
    while (true)
    {
      auto c = *p++;
      if (!c)
      {
        size_t copy_size = p - begin - 1;
        memcpy(out, begin, copy_size);
        out += copy_size;
        break;
      }
      if (c != '{')
        continue;
      size_t copy_size = p - begin;
      memcpy(out, begin, copy_size);
      out += copy_size;
      begin = p;
      c = *p++;
      if (!c)
        fmt::report_error("invalid format string");
      if (fmt::detail::is_name_start(c))
      {
        while ((fmt::detail::is_name_start(c = *p) || ('0' <= c && c <= '9')))
        {
          ++p;
        }
        fmt::string_view name(begin, p - begin);
        int id = -1;
        for (size_t i = 0; i < num_named_args; ++i)
        {
          if (named_args[i].name == name)
          {
            id = named_args[i].id;
            break;
          }
        }
        if (id < 0)
          fmt::report_error("invalid format string");
        if constexpr (Reorder)
        {
          reorderIdx[id] = arg_idx++;
        }
        else
        {
          out = fmt::format_to(out, "{}", id);
        }
      }
      else
      {
        *out++ = c;
      }
      begin = p;
    }
    std::string result;
    result.reserve(size_unnamed_str);
    std::string_view ptr_span(unnamed_str.get(), size_unnamed_str);
    std::copy_n(ptr_span.begin(), out - unnamed_str.get(), std::back_inserter(result));
    return result;
  }

 public:
  template<typename... Args>
  inline void log(
      uint32_t& logId,
      int64_t tsc,
      const char* location,
      LogLevel level,
      fmt::format_string<typename fmtlogdetail::UnrefPtr<fmt::remove_cvref_t<Args>>::type...> format,
      Args&&... args) noexcept
  {
    if (!logId)
    {
      fmt::string_view format_str_v = format.get();
      auto unnamed_format = unNameFormat<false>(std::string{ format_str_v.begin(), format_str_v.end() }, nullptr, args...);
      registerLogInfo(logId, formatTo<Args...>, location, level, unnamed_format);
    }
    constexpr size_t num_cstring = fmt::detail::count<isCstring<Args>()...>();
    size_t cstringSizes[std::max(num_cstring, (size_t)1)];
    uint32_t alloc_size = 8 + (uint32_t)getArgSizes<0>(cstringSizes, args...);
    bool q_full_cb = true;
    do
    {
      if (auto header = allocMsg(alloc_size, q_full_cb))
      {
        header->logId = logId;
        char* out = (char*)(header + 1);
        *(int64_t*)out = tsc;
        out += 8;
        encodeArgs<0>(cstringSizes, out, std::forward<Args>(args)...);
        header->push(alloc_size);
        break;
      }
      q_full_cb = false;
    } while (FMTLOG_BLOCK);
  }

  template<typename... Args>
  inline void logOnce(const char* location, LogLevel level, fmt::format_string<Args...> format, Args&&... args)
  {

    logOnceV(location, level, format, fmt::make_format_args(std::forward<Args>(args)...));
  }

  template<typename... Args>
  inline void logOnceV(const char* location, LogLevel level, const fmt::format_string<Args...>& format, fmt::format_args&& fmt_args)
  {
    fmt::string_view sv(format);
    uint32_t fmt_size = formatted_size(sv, fmt_args);
    uint32_t alloc_size = 8 + 8 + fmt_size;
    bool q_full_cb = true;
    do
    {
      if (auto header = allocMsg(alloc_size, q_full_cb))
      {
        header->logId = (uint32_t)level;
        char* out = (char*)(header + 1);
        *(int64_t*)out = tscns.rdtsc();
        out += 8;
        *(const char**)out = location;
        out += 8;
        vformat_to(out, sv, fmt_args);
        header->push(alloc_size);
        break;
      }
      q_full_cb = false;
    } while (FMTLOG_BLOCK);
  }
};

using fmtlog = fmtlogT<>;

template<int _>
FAST_THREAD_LOCAL typename fmtlogT<_>::ThreadBuffer* fmtlogT<_>::threadBuffer;

template<int __ = 0>
struct fmtlogWrapper
{
  static fmtlog impl;
};

template<int _>
fmtlog fmtlogWrapper<_>::impl;

template<int _>
inline void fmtlogT<_>::setLogLevel(LogLevel logLevel) noexcept
{
  fmtlogWrapper<>::impl.currentLogLevel = logLevel;
}

template<int _>
inline typename fmtlogT<_>::LogLevel fmtlogT<_>::getLogLevel() noexcept
{
  return fmtlogWrapper<>::impl.currentLogLevel;
}

template<int _>
inline bool fmtlogT<_>::checkLogLevel(LogLevel logLevel) noexcept
{
#ifdef FMTLOG_NO_CHECK_LEVEL
  return true;
#else
  return logLevel >= fmtlogWrapper<>::impl.currentLogLevel;
#endif
}

#define __FMTLOG_S1(x)    #x
#define __FMTLOG_S2(x)    __FMTLOG_S1(x)
#define __FMTLOG_LOCATION __FILE__ ":" __FMTLOG_S2(__LINE__)

#define FMTLOG(level, format, ...)                                                                                          \
  do                                                                                                                        \
  {                                                                                                                         \
    static uint32_t logId = 0;                                                                                              \
    if (!fmtlog::checkLogLevel(level))                                                                                      \
      break;                                                                                                                \
    fmtlogWrapper<>::impl.log(logId, fmtlogWrapper<>::impl.tscns.rdtsc(), __FMTLOG_LOCATION, level, format, ##__VA_ARGS__); \
  } while (0)

#define FMTLOG_LIMIT(min_interval, level, format, ...)                                      \
  do                                                                                        \
  {                                                                                         \
    static uint32_t logId = 0;                                                              \
    static int64_t limitNs = 0;                                                             \
    if (!fmtlog::checkLogLevel(level))                                                      \
      break;                                                                                \
    int64_t tsc = fmtlogWrapper<>::impl.tscns.rdtsc();                                      \
    int64_t ns = fmtlogWrapper<>::impl.tscns.tsc2ns(tsc);                                   \
    if (ns < limitNs)                                                                       \
      break;                                                                                \
    limitNs = ns + min_interval;                                                            \
    fmtlogWrapper<>::impl.log(logId, tsc, __FMTLOG_LOCATION, level, format, ##__VA_ARGS__); \
  } while (0)

//
// Logs a formatted message with the specified source location.
// 
// The location string is expected to be a concatenation of the
// source file and line number, exactly as `__FMTLOG_LOCATION`
// macro would generate.
//
#define FMTLOG_ONCE_LOCATION(level, location, format, ...)                                         \
  do                                                                                \
  {                                                                                 \
    if (!fmtlog::checkLogLevel(level))                                              \
      break;                                                                        \
    fmtlogWrapper<>::impl.logOnce(location, level, format, ##__VA_ARGS__);                         \
  } while (0)

#define FMTLOG_ONCE(level, format, ...)                                                            \
    FMTLOG_ONCE_LOCATION(level, __FMTLOG_LOCATION, format, ##__VA_ARGS__);

#if FMTLOG_ACTIVE_LEVEL <= FMTLOG_LEVEL_DBG
#define log_debug(format, ...) FMTLOG(fmtlog::DBG, format, ##__VA_ARGS__)
#define log_debug_once(format, ...) FMTLOG_ONCE(fmtlog::DBG, format, ##__VA_ARGS__)
#define log_debug_limit(min_interval, format, ...) FMTLOG_LIMIT(min_interval, fmtlog::DBG, format, ##__VA_ARGS__)
#else
#define log_debug(format, ...) (void)0
#define log_debug_once(format, ...) (void)0
#define log_debug_limit(min_interval, format, ...) (void)0
#endif

#if FMTLOG_ACTIVE_LEVEL <= FMTLOG_LEVEL_INF
#define log_info(format, ...) FMTLOG(fmtlog::INF, format, ##__VA_ARGS__)
#define log_info_once(format, ...) FMTLOG_ONCE(fmtlog::INF, format, ##__VA_ARGS__)
#define log_info_limit(min_interval, format, ...) FMTLOG_LIMIT(min_interval, fmtlog::INF, format, ##__VA_ARGS__)
#else
#define log_info(format, ...) (void)0
#define log_info_once(format, ...) (void)0
#define log_info_limit(min_interval, format, ...) (void)0
#endif

#if FMTLOG_ACTIVE_LEVEL <= FMTLOG_LEVEL_STRATEGY
#define log_strategy(format, ...) FMTLOG(fmtlog::STRATEGY, format, ##__VA_ARGS__)
#define log_strategy_once(format, ...) FMTLOG_ONCE(fmtlog::STRATEGY, format, ##__VA_ARGS__)
#define log_strategy_limit(min_interval, format, ...) FMTLOG_LIMIT(min_interval, fmtlog::STRATEGY, format, ##__VA_ARGS__)
#else
#define log_strategy(format, ...) (void)0
#define log_strategy_once(format, ...) (void)0
#define log_strategy_limit(min_interval, format, ...) (void)0
#endif

#if FMTLOG_ACTIVE_LEVEL <= FMTLOG_LEVEL_WRN
#define log_warning(format, ...) FMTLOG(fmtlog::WRN, format, ##__VA_ARGS__)
#define log_warning_once(format, ...) FMTLOG_ONCE(fmtlog::WRN, format, ##__VA_ARGS__)
#define log_warning_limit(min_interval, format, ...) FMTLOG_LIMIT(min_interval, fmtlog::WRN, format, ##__VA_ARGS__)
#else
#define log_warning(format, ...) (void)0
#define log_warning_once(format, ...) (void)0
#define log_warning_limit(min_interval, format, ...) (void)0
#endif

#if FMTLOG_ACTIVE_LEVEL <= FMTLOG_LEVEL_ERR
#define log_error(format, ...) FMTLOG(fmtlog::ERR, format, ##__VA_ARGS__)
#define log_error_once(format, ...) FMTLOG_ONCE(fmtlog::ERR, format, ##__VA_ARGS__)
#define log_error_limit(min_interval, format, ...) FMTLOG_LIMIT(min_interval, fmtlog::ERR, format, ##__VA_ARGS__)
#else
#define log_error(format, ...) (void)0
#define log_error_once(format, ...) (void)0
#define log_error_limit(min_interval, format, ...) (void)0
#endif

#include "fmtlog/internal/fmtlog-inl.h"

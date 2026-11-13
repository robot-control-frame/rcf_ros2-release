#pragma once
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <type_traits>

namespace rcf_common {
namespace shm_tools {

template <typename T>
class ShmSwapBuffer {
  static_assert(std::is_trivially_copyable<T>::value,
                "ShmSwapBuffer<T> requires T to be trivially copyable for shared-memory memcpy.");

 private:
  static constexpr uint32_t kMagic = 0x53535042;  // "SSPB"
  // v3: only layout checks (size/align); no typeid / type-id string.
  static constexpr uint32_t kVersion = 3;
  static constexpr int kNoReadableBuffer = -1;
  static constexpr int kBufferCount = 2;
  static constexpr int kMaxReadRetries = 8;

  struct ShmData {
    uint32_t magic;
    uint32_t version;
    size_t shm_size;
    size_t type_size;
    size_t type_align;
    uint64_t type_hash;                              // unused since v3; kept for binary layout compatibility
    std::atomic<bool> writer_active;                 // 防止多个发布者交错写入同一缓冲
    std::atomic<int> readable_idx;                   // 当前可读缓冲索引
    std::atomic<uint64_t> buffer_seq[kBufferCount];  // 奇数表示该缓冲正在写入
    T buf[kBufferCount];                             // 双缓冲
  };
  std::string shm_name_;
  int shm_fd_ = -1;
  void* shm_addr_ = nullptr;
  ShmData* shm_data_ = nullptr;

 public:
  ShmSwapBuffer() = default;
  ShmSwapBuffer(const ShmSwapBuffer&) = delete;
  ShmSwapBuffer& operator=(const ShmSwapBuffer&) = delete;

  ~ShmSwapBuffer() {
    if (shm_addr_) munmap(shm_addr_, sizeof(ShmData));
    if (shm_fd_ >= 0) close(shm_fd_);
  }

  bool init(const std::string& name, bool creator = false) {
    shm_name_ = name;
    if (!shm_name_.empty() && shm_name_[0] == '/') {
      shm_name_.erase(0, 1);
    }
    // 将剩余的'/'替换为'_'
    for (size_t i = 0; i < shm_name_.size(); ++i) {
      if (shm_name_[i] == '/') {
        shm_name_[i] = '_';
      }
    }
    shm_name_ = "shm_swap_buffer_" + shm_name_;

    const size_t shm_size = sizeof(ShmData);
    bool created = false;

    if (creator) {
      shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR | O_CREAT | O_EXCL, 0666);
      if (shm_fd_ >= 0) {
        created = true;
      } else if (errno == EEXIST) {
        shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0666);
      }
    } else {
      shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0666);
      if (shm_fd_ < 0 && errno == ENOENT) {
        shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR | O_CREAT | O_EXCL, 0666);
        created = shm_fd_ >= 0;
        if (shm_fd_ < 0 && errno == EEXIST) {
          shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0666);
        }
      }
    }
    if (shm_fd_ < 0) {
      std::cerr << "shm_open failed: " << strerror(errno) << std::endl;
      return false;
    }

    if (created && ftruncate(shm_fd_, shm_size) == -1) {
      std::cerr << "ftruncate failed: " << strerror(errno) << std::endl;
      cleanup();
      return false;
    }

    struct stat shm_stat {};
    if (!created && fstat(shm_fd_, &shm_stat) == -1) {
      std::cerr << "fstat failed: " << strerror(errno) << std::endl;
      cleanup();
      return false;
    }
    if (!created && static_cast<size_t>(shm_stat.st_size) != shm_size) {
      std::cerr << "shared memory size mismatch: shared=" << shm_stat.st_size << ", current=" << shm_size << std::endl;
      cleanup();
      return false;
    }

    shm_addr_ = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (shm_addr_ == MAP_FAILED) {
      std::cerr << "mmap failed: " << strerror(errno) << std::endl;
      shm_addr_ = nullptr;
      cleanup();
      return false;
    }

    shm_data_ = static_cast<ShmData*>(shm_addr_);
    if (created) {
      new (shm_addr_) ShmData();
      init_metadata();
    } else {
      if (!validate_metadata()) {
        cleanup();
        return false;
      }
    }
    return true;
  }

  bool write(const T& data) {
    if (!shm_data_) return false;
    bool expected = false;
    if (!shm_data_->writer_active.compare_exchange_strong(expected, true, std::memory_order_acquire,
                                                          std::memory_order_relaxed)) {
      return false;
    }

    const int readable_idx = shm_data_->readable_idx.load(std::memory_order_acquire);
    if (readable_idx != kNoReadableBuffer && (readable_idx < 0 || readable_idx >= kBufferCount)) {
      shm_data_->writer_active.store(false, std::memory_order_release);
      return false;
    }
    const int writable_idx = readable_idx == kNoReadableBuffer ? 0 : 1 - readable_idx;

    shm_data_->buffer_seq[writable_idx].fetch_add(1, std::memory_order_acq_rel);
    std::memcpy(&shm_data_->buf[writable_idx], &data, sizeof(T));
    shm_data_->buffer_seq[writable_idx].fetch_add(1, std::memory_order_release);
    shm_data_->readable_idx.store(writable_idx, std::memory_order_release);
    shm_data_->writer_active.store(false, std::memory_order_release);
    return true;
  }

  bool read(T& data) {
    if (!shm_data_) return false;
    for (int retry = 0; retry < kMaxReadRetries; ++retry) {
      const int ridx = shm_data_->readable_idx.load(std::memory_order_acquire);
      if (ridx < 0 || ridx >= kBufferCount) return false;

      const uint64_t seq_before = shm_data_->buffer_seq[ridx].load(std::memory_order_acquire);
      if ((seq_before & 1U) != 0U) continue;

      std::memcpy(&data, &shm_data_->buf[ridx], sizeof(T));
      std::atomic_thread_fence(std::memory_order_acquire);

      const uint64_t seq_after = shm_data_->buffer_seq[ridx].load(std::memory_order_acquire);
      if (seq_before == seq_after && (seq_after & 1U) == 0U) return true;
    }
    return false;
  }

 private:
  void init_metadata() {
    shm_data_->magic = kMagic;
    shm_data_->version = kVersion;
    shm_data_->shm_size = sizeof(ShmData);
    shm_data_->type_size = sizeof(T);
    shm_data_->type_align = alignof(T);
    shm_data_->type_hash = 0;
    shm_data_->writer_active.store(false, std::memory_order_release);
    shm_data_->readable_idx.store(kNoReadableBuffer, std::memory_order_release);
    for (int i = 0; i < kBufferCount; ++i) {
      shm_data_->buffer_seq[i].store(0, std::memory_order_release);
    }
  }

  bool validate_metadata() const {
    bool valid = true;
    if (shm_data_->magic != kMagic || shm_data_->version != kVersion) {
      std::cerr << "shared memory metadata mismatch: " << shm_name_ << " magic/version" << " shared=("
                << shm_data_->magic << "," << shm_data_->version << ")" << " current=(" << kMagic << "," << kVersion
                << ")" << std::endl;
      valid = false;
    }
    if (shm_data_->shm_size != sizeof(ShmData)) {
      std::cerr << "shared memory layout size mismatch: " << shm_name_ << " shared=" << shm_data_->shm_size
                << ", current=" << sizeof(ShmData) << std::endl;
      valid = false;
    }
    if (shm_data_->type_size != sizeof(T)) {
      std::cerr << "shared memory type_size mismatch: " << shm_name_ << " shared=" << shm_data_->type_size
                << ", current=" << sizeof(T) << std::endl;
      valid = false;
    }
    if (shm_data_->type_align != alignof(T)) {
      std::cerr << "shared memory type_align mismatch: " << shm_name_ << " shared=" << shm_data_->type_align
                << ", current=" << alignof(T) << std::endl;
      valid = false;
    }
    return valid;
  }

  void cleanup() {
    if (shm_addr_) {
      munmap(shm_addr_, sizeof(ShmData));
      shm_addr_ = nullptr;
      shm_data_ = nullptr;
    }
    if (shm_fd_ >= 0) {
      close(shm_fd_);
      shm_fd_ = -1;
    }
  }
};

}  // namespace shm_tools
}  // namespace rcf_common

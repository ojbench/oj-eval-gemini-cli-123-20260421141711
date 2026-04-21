#include "fstream.h"
#include <vector>
#include <algorithm>

// 磁盘事件类型：正常、故障、更换
enum class EventType {
  NORMAL,  // 正常：所有磁盘工作正常
  FAILED,  // 故障：指定磁盘发生故障（文件被删除）
  REPLACED // 更换：指定磁盘被更换（文件被清空）
};

class RAID5Controller {
private:
  std::vector<sjtu::fstream *> drives_; // 磁盘文件对应的 fstream 对象
  int blocks_per_drive_;               // 每个磁盘的块数
  int block_size_;                     // 每个块的大小
  int num_disks_;                      // 磁盘数
  int failed_disk_id_;                 // 故障磁盘编号
  EventType current_event_;            // 当前事件类型

  void read_from_disk(int disk_id, int row, char *dest) {
    drives_[disk_id]->seekg(row * block_size_);
    drives_[disk_id]->read(dest, block_size_);
  }

  void write_to_disk(int disk_id, int row, const char *src) {
    drives_[disk_id]->seekp(row * block_size_);
    drives_[disk_id]->write(src, block_size_);
    drives_[disk_id]->flush();
  }

  void rebuild_disk(int drive_id) {
    std::vector<char> buffer(block_size_);
    std::vector<char> row_data(block_size_);
    for (int r = 0; r < blocks_per_drive_; ++r) {
      std::fill(row_data.begin(), row_data.end(), 0);
      for (int i = 0; i < num_disks_; ++i) {
        if (i == drive_id) continue;
        read_from_disk(i, r, buffer.data());
        for (int j = 0; j < block_size_; ++j) {
          row_data[j] ^= buffer[j];
        }
      }
      write_to_disk(drive_id, r, row_data.data());
    }
  }

public:
  RAID5Controller(std::vector<sjtu::fstream *> drives, int blocks_per_drive,
                  int block_size = 4096)
      : drives_(drives), blocks_per_drive_(blocks_per_drive),
        block_size_(block_size), num_disks_(drives.size()),
        failed_disk_id_(-1), current_event_(EventType::NORMAL) {}

  /**
   * @brief 启动 RAID5 系统
   * @param event_type_ 磁盘事件类型
   * @param drive_id 发生事件的磁盘编号（如果是 NORMAL 则忽略）
   *
   * 如果是 FAILED，对应的磁盘文件会被删除。此时不可再对该文件进行读写。
   * 如果是 REPLACED，对应的磁盘文件会被清空（但文件依然存在）
   * 如果是 NORMAL，所有磁盘正常工作
   * 注：磁盘被替换之前不一定损坏。
   */
  void Start(EventType event_type_, int drive_id) {
    current_event_ = event_type_;
    if (event_type_ == EventType::NORMAL) {
      failed_disk_id_ = -1;
    } else {
      failed_disk_id_ = drive_id;
      if (event_type_ == EventType::REPLACED) {
        rebuild_disk(drive_id);
        failed_disk_id_ = -1;
        current_event_ = EventType::NORMAL;
      }
    }
  }

  void Shutdown() {
    for (auto drive : drives_) {
      if (drive && drive->is_open()) {
        drive->close();
      }
    }
  }

  void ReadBlock(int block_id, char *result) {
    int r = block_id / (num_disks_ - 1);
    int k = block_id % (num_disks_ - 1);
    int P = (num_disks_ - 1) - (r % num_disks_);
    int D = (P + 1 + k) % num_disks_;

    if (D == failed_disk_id_) {
      std::vector<char> buffer(block_size_);
      std::fill(result, result + block_size_, 0);
      for (int i = 0; i < num_disks_; ++i) {
        if (i == failed_disk_id_) continue;
        read_from_disk(i, r, buffer.data());
        for (int j = 0; j < block_size_; ++j) {
          result[j] ^= buffer[j];
        }
      }
    } else {
      read_from_disk(D, r, result);
    }
  }

  void WriteBlock(int block_id, const char *data) {
    int r = block_id / (num_disks_ - 1);
    int k = block_id % (num_disks_ - 1);
    int P = (num_disks_ - 1) - (r % num_disks_);
    int D = (P + 1 + k) % num_disks_;

    if (failed_disk_id_ == -1) {
      std::vector<char> old_data(block_size_);
      std::vector<char> old_parity(block_size_);
      read_from_disk(D, r, old_data.data());
      read_from_disk(P, r, old_parity.data());
      for (int i = 0; i < block_size_; ++i) {
        old_parity[i] ^= old_data[i] ^ data[i];
      }
      write_to_disk(D, r, data);
      write_to_disk(P, r, old_parity.data());
    } else if (failed_disk_id_ == D) {
      std::vector<char> parity(block_size_);
      std::copy(data, data + block_size_, parity.begin());
      std::vector<char> buffer(block_size_);
      for (int i = 0; i < num_disks_; ++i) {
        if (i == D || i == P) continue;
        read_from_disk(i, r, buffer.data());
        for (int j = 0; j < block_size_; ++j) {
          parity[j] ^= buffer[j];
        }
      }
      write_to_disk(P, r, parity.data());
      if (current_event_ == EventType::REPLACED) {
        write_to_disk(D, r, data);
      }
    } else if (failed_disk_id_ == P) {
      write_to_disk(D, r, data);
      if (current_event_ == EventType::REPLACED) {
        std::vector<char> parity(block_size_, 0);
        std::vector<char> buffer(block_size_);
        for (int i = 0; i < num_disks_; ++i) {
          if (i == P) continue;
          if (i == D) {
            for (int j = 0; j < block_size_; ++j) parity[j] ^= data[j];
          } else {
            read_from_disk(i, r, buffer.data());
            for (int j = 0; j < block_size_; ++j) parity[j] ^= buffer[j];
          }
        }
        write_to_disk(P, r, parity.data());
      }
    } else {
      std::vector<char> old_data(block_size_);
      std::vector<char> old_parity(block_size_);
      read_from_disk(D, r, old_data.data());
      read_from_disk(P, r, old_parity.data());
      for (int i = 0; i < block_size_; ++i) {
        old_parity[i] ^= old_data[i] ^ data[i];
      }
      write_to_disk(D, r, data);
      write_to_disk(P, r, old_parity.data());
    }
  }

  int Capacity() {
    return (num_disks_ - 1) * blocks_per_drive_;
  }
};

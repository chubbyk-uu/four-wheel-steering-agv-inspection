#pragma once
#include <cstddef>
#include <stdexcept>
namespace agv_linescan {
inline bool DiscardShortTail(size_t rows,bool full,size_t minimum) {
  if(minimum>16384)throw std::invalid_argument("min_tail_rows exceeds supported frame size");
  return rows>0 && !full && rows<minimum;
}
}

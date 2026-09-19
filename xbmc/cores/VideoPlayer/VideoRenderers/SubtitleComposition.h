/*
 *  Copyright (C) 2026 Team CoreELEC
 *  This file is part of CoreELEC - https://coreelec.org
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace OVERLAY
{

class CSubtitleComposition
{
public:
  struct Rect
  {
    int x0{0};
    int y0{0};
    int x1{0};
    int y1{0};

    bool Empty() const { return x0 >= x1 || y0 >= y1; }
  };

  struct Submission
  {
    uint64_t id{0};
    uint64_t revision{0};
    int dstX{0};
    int dstY{0};
    int dstW{0};
    int dstH{0};
    Rect content;
    const uint32_t* pixels{nullptr};
    int stride{0};
  };

  enum class Result
  {
    IDLE,
    CHANGED,
  };

  void Reset(unsigned int width, unsigned int height);
  void BeginFrame(bool hasContent);
  void Submit(const Submission& submission);
  Result Compose();
  Rect CopyRect(unsigned int bufferIndex) const;
  void Commit(unsigned int bufferIndex);

  const std::vector<uint32_t>& Pixels() const { return m_pixels; }
  unsigned int Width() const { return m_width; }
  bool DisablePending() const { return m_disablePending; }
  bool HasPresentedContent() const { return !m_presented.empty(); }

private:
  struct Key
  {
    uint64_t id{0};
    uint64_t revision{0};
    int dstX{0};
    int dstY{0};
    int dstW{0};
    int dstH{0};
    Rect content;

    bool operator==(const Key& other) const;
  };

  static Rect Union(Rect first, Rect second);
  static bool Intersects(Rect first, Rect second);
  static Rect ContentUnion(const std::vector<Key>& keys);
  void Clear(Rect rect);
  void Fold(const Submission& submission, Rect clip);

  unsigned int m_width{0};
  unsigned int m_height{0};
  std::vector<uint32_t> m_pixels;
  std::vector<Submission> m_current;
  std::vector<Key> m_presented;
  std::vector<Key> m_pending;
  std::vector<Key> m_buffers[2];
  bool m_acceptCurrent{false};
  bool m_composed{false};
  bool m_disablePending{false};
  int m_emptyFrames{0};
};

} // namespace OVERLAY

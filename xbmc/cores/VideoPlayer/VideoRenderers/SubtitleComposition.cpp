/*
 *  Copyright (C) 2026 Team CoreELEC
 *  This file is part of CoreELEC - https://coreelec.org
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "SubtitleComposition.h"

#include <algorithm>
#include <cstring>

namespace OVERLAY
{
namespace
{
constexpr int REMOVAL_DEBOUNCE_FRAMES = 3;
}

bool CSubtitleComposition::Key::operator==(const Key& other) const
{
  return id == other.id && revision == other.revision && dstX == other.dstX && dstY == other.dstY &&
         dstW == other.dstW && dstH == other.dstH && content.x0 == other.content.x0 &&
         content.y0 == other.content.y0 && content.x1 == other.content.x1 &&
         content.y1 == other.content.y1;
}

CSubtitleComposition::Rect CSubtitleComposition::Union(Rect first, Rect second)
{
  if (first.Empty())
    return second;
  if (second.Empty())
    return first;
  return {std::min(first.x0, second.x0), std::min(first.y0, second.y0),
          std::max(first.x1, second.x1), std::max(first.y1, second.y1)};
}

bool CSubtitleComposition::Intersects(Rect first, Rect second)
{
  return !first.Empty() && !second.Empty() && first.x0 < second.x1 && second.x0 < first.x1 &&
         first.y0 < second.y1 && second.y0 < first.y1;
}

CSubtitleComposition::Rect CSubtitleComposition::ContentUnion(const std::vector<Key>& keys)
{
  Rect result;
  for (const auto& key : keys)
    result = Union(result, key.content);
  return result;
}

void CSubtitleComposition::Reset(unsigned int width, unsigned int height)
{
  m_width = width;
  m_height = height;
  m_pixels.assign(static_cast<size_t>(width) * height, 0);
  m_current.clear();
  m_presented.clear();
  m_pending.clear();
  m_buffers[0].clear();
  m_buffers[1].clear();
  m_acceptCurrent = false;
  m_composed = false;
  m_disablePending = false;
  m_emptyFrames = 0;
}

void CSubtitleComposition::BeginFrame(bool hasContent)
{
  m_current.clear();
  m_pending.clear();
  m_composed = false;

  if (hasContent)
  {
    m_emptyFrames = 0;
    m_disablePending = false;
    m_acceptCurrent = true;
  }
  else if (!m_presented.empty())
  {
    m_acceptCurrent = ++m_emptyFrames >= REMOVAL_DEBOUNCE_FRAMES;
    if (m_acceptCurrent)
      m_emptyFrames = 0;
  }
  else
  {
    m_acceptCurrent = true;
  }
}

void CSubtitleComposition::Submit(const Submission& submission)
{
  m_current.push_back(submission);
}

CSubtitleComposition::Result CSubtitleComposition::Compose()
{
  if (!m_acceptCurrent)
    return Result::IDLE;

  m_pending.reserve(m_current.size());
  for (const auto& submission : m_current)
  {
    m_pending.push_back({submission.id, submission.revision, submission.dstX, submission.dstY,
                         submission.dstW, submission.dstH, submission.content});
  }

  if (m_pending == m_presented)
    return Result::IDLE;

  const Rect dirty = Union(ContentUnion(m_presented), ContentUnion(m_pending));
  Clear(dirty);
  for (const auto& submission : m_current)
  {
    if (Intersects(submission.content, dirty))
      Fold(submission, dirty);
  }
  m_composed = true;
  return Result::CHANGED;
}

CSubtitleComposition::Rect CSubtitleComposition::CopyRect(unsigned int bufferIndex) const
{
  if (!m_composed || bufferIndex >= 2)
    return {};
  return Union(ContentUnion(m_buffers[bufferIndex]), ContentUnion(m_pending));
}

void CSubtitleComposition::Commit(unsigned int bufferIndex)
{
  if (!m_composed || bufferIndex >= 2)
    return;
  m_presented = m_pending;
  m_buffers[bufferIndex] = m_pending;
  m_disablePending = m_presented.empty();
  m_composed = false;
}

void CSubtitleComposition::Clear(Rect rect)
{
  rect.x0 = std::max(rect.x0, 0);
  rect.y0 = std::max(rect.y0, 0);
  rect.x1 = std::min(rect.x1, static_cast<int>(m_width));
  rect.y1 = std::min(rect.y1, static_cast<int>(m_height));
  if (rect.Empty())
    return;

  for (int y = rect.y0; y < rect.y1; ++y)
  {
    std::memset(m_pixels.data() + static_cast<size_t>(y) * m_width + rect.x0, 0,
                static_cast<size_t>(rect.x1 - rect.x0) * sizeof(uint32_t));
  }
}

void CSubtitleComposition::Fold(const Submission& submission, Rect clip)
{
  const Rect rect{std::max({submission.content.x0, clip.x0, 0}),
                  std::max({submission.content.y0, clip.y0, 0}),
                  std::min({submission.content.x1, clip.x1, static_cast<int>(m_width)}),
                  std::min({submission.content.y1, clip.y1, static_cast<int>(m_height)})};
  if (rect.Empty())
    return;

  for (int y = rect.y0; y < rect.y1; ++y)
  {
    const uint32_t* src = submission.pixels +
                          static_cast<size_t>(y - submission.dstY) * submission.stride +
                          (rect.x0 - submission.dstX);
    uint32_t* dst = m_pixels.data() + static_cast<size_t>(y) * m_width + rect.x0;
    for (int x = rect.x0; x < rect.x1; ++x, ++src, ++dst)
    {
      const uint32_t source = *src;
      const uint32_t alpha = source >> 24;
      if (alpha == 0)
        continue;
      if (alpha == 255)
      {
        *dst = source;
        continue;
      }

      const uint32_t target = *dst;
      const uint32_t inverseAlpha = 255 - alpha;
      const uint32_t red = std::min<uint32_t>(
          255, ((source >> 16) & 0xff) + (((target >> 16) & 0xff) * inverseAlpha >> 8));
      const uint32_t green = std::min<uint32_t>(
          255, ((source >> 8) & 0xff) + (((target >> 8) & 0xff) * inverseAlpha >> 8));
      const uint32_t blue =
          std::min<uint32_t>(255, (source & 0xff) + ((target & 0xff) * inverseAlpha >> 8));
      const uint32_t outputAlpha =
          std::min<uint32_t>(255, alpha + (((target >> 24) * inverseAlpha) >> 8));
      *dst = (outputAlpha << 24) | (red << 16) | (green << 8) | blue;
    }
  }
}

} // namespace OVERLAY

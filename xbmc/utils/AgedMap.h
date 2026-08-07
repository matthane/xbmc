/*
 *  Copyright (C) 2005-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <iterator>
#include <map>
#include <utility>

inline constexpr size_t AGED_MAP_DEFAULT_MAX_SIZE = 512;

// bounded map that evicts the oldest entry once MaxSize is exceeded. not thread
// safe, callers serialise access themselves
template<typename K, typename V, size_t MaxSize = AGED_MAP_DEFAULT_MAX_SIZE>
class CAgedMap
{
public:
  void insert(K key, V value)
  {
    const auto [it, inserted] = m_map.insert_or_assign(std::move(key), std::move(value));
    if (!inserted)
      return;

    m_ages.push_back(it->first);
    if (m_map.size() > MaxSize)
    {
      m_map.erase(m_ages.front());
      m_ages.pop_front();
    }
  }

  void erase(const K& key)
  {
    if (m_map.erase(key) == 0)
      return;

    const auto it = std::find(m_ages.begin(), m_ages.end(), key);
    if (it != m_ages.end())
      m_ages.erase(it);
  }

  auto find(const K& key) const { return m_map.find(key); }

  // the entry whose key is closest to `key`, or end() only when the map is empty.
  // keys and lookups can come from clocks that merely track each other, so an
  // exact match is not the normal case
  auto findNearest(const K& key) const
  {
    const auto next = m_map.lower_bound(key);
    if (next == m_map.begin())
      return next;

    const auto prev = std::prev(next);
    if (next == m_map.end())
      return prev;

    return (key - prev->first) <= (next->first - key) ? prev : next;
  }

  auto end() const { return m_map.end(); }

  size_t size() const { return m_map.size(); }

private:
  std::map<K, V> m_map;
  std::deque<K> m_ages;
};

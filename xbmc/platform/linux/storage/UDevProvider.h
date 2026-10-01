/*
 *  Copyright (C) 2012-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "storage/IStorageProvider.h"

#include <string>
#include <vector>

struct udev;
struct udev_monitor;

class CUDevProvider : public IStorageProvider
{
public:
  CUDevProvider();
  ~CUDevProvider() override = default;

  void Initialize() override;
  void Stop() override;

  void GetLocalDrives(std::vector<CMediaSource>& localDrives) override;
  void GetRemovableDrives(std::vector<CMediaSource>& removableDrives) override;

  bool Eject(const std::string& mountpath) override;

  std::vector<std::string> GetDiskUsage() override;

  bool PumpDriveChangeEvents(IStorageEventsCallback *callback) override;

private:
  void GetDisks(std::vector<CMediaSource>& devices, bool removable);
  bool DisksChanged();
  void UpdateDisks(bool removable);

  struct udev         *m_udev;
  struct udev_monitor *m_udevMon;

  std::vector<CMediaSource> m_localDisks;
  std::vector<CMediaSource> m_removableDisks;
  unsigned int m_disksGeneration{1};
  unsigned int m_localGeneration{0};
  unsigned int m_removableGeneration{0};
  int m_mountsFd{-1};
};

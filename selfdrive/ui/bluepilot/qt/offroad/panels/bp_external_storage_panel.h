/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 * Copyright (c) 2024-, BluePilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and BluePilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include <QGroupBox>
#include <QVBoxLayout>

#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_panel_base.h"
#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_ui_helpers.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/external_storage.h"

class BPExternalStoragePanel : public QWidget {
  Q_OBJECT

public:
  explicit BPExternalStoragePanel(QWidget *parent = nullptr);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private:
  void setupUI();
  void createStorageControlGroup();

  QGroupBox *createStyledGroupBox(const QString &title);

private:
  QVBoxLayout *mainLayout;

  // Storage Control Group
  QGroupBox *storageGroup;
  ExternalStorageControl *externalStorageControl;
};
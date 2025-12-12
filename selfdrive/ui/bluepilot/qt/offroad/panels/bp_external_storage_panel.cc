/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 * Copyright (c) 2024-, BluePilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and BluePilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_external_storage_panel.h"

#include <QVBoxLayout>

#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_ui_helpers.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/external_storage.h"
#include "system/hardware/hw.h"

BPExternalStoragePanel::BPExternalStoragePanel(QWidget *parent) : QWidget(parent) {
  setupUI();
}

void BPExternalStoragePanel::setupUI() {
  mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(40, 40, 40, 40);
  mainLayout->setSpacing(30);

  createStorageControlGroup();

  mainLayout->addStretch();

  setStyleSheet(R"(
    BPExternalStoragePanel {
      background-color: #1B1B1B;
    }
    BPExternalStoragePanel QGroupBox {
      background-color: transparent;
    }
  )");
}

QGroupBox *BPExternalStoragePanel::createStyledGroupBox(const QString &title) {
  QGroupBox *group = new QGroupBox(title, this);
  group->setStyleSheet(R"(
    QGroupBox {
      background-color: #242424;
      border: none;
      border-radius: 40px;
      margin-top: 50px;
      padding: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    QGroupBox::title {
      subcontrol-origin: margin;
      subcontrol-position: top left;
      padding: 5px 15px;
      border-top-left-radius: 15px;
      border-top-right-radius: 15px;
      border-bottom: none;
      margin-left: 35px;
      margin-top: 0px;
      background-color: #242424;
      color: #2196F3;
    }
    QGroupBox > QWidget {
      background-color: transparent;
    }
    QGroupBox::indicator {
      width: 0px;
    }
  )");
  group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  return group;
}

void BPExternalStoragePanel::createStorageControlGroup() {
  storageGroup = createStyledGroupBox(tr("External Storage Control"));
  QVBoxLayout *layout = new QVBoxLayout(storageGroup);
  layout->setSpacing(20);
  layout->setContentsMargins(25, 25, 25, 25);

  // Add the ExternalStorageControl widget from SunnyPilot
  // It is a ButtonControl subclass with a parameterless constructor
  #ifndef __APPLE__
  externalStorageControl = new ExternalStorageControl();
  layout->addWidget(externalStorageControl);
  #endif

  mainLayout->addWidget(storageGroup);
}

void BPExternalStoragePanel::showEvent(QShowEvent *event) {
  QWidget::showEvent(event);
  // ExternalStorageControl likely handles its own updates/refresh if needed
}

void BPExternalStoragePanel::hideEvent(QHideEvent *event) {
  QWidget::hideEvent(event);
}
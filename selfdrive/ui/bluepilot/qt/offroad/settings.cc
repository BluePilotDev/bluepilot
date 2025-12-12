/**
 * BluePilot Settings Window
 * Modern, streamlined settings interface with consistent BP styling
 */

#include "selfdrive/ui/bluepilot/qt/offroad/settings.h"

#include "selfdrive/ui/sunnypilot/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/offroad/firehose.h"
#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_panel_controls.h"

// BP Panel includes
#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_base_view.h"
#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_nav_bar_view.h"
#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_statistics_panel.h"
#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_network_panel.h"
#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_software_panel.h"
#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_models_panel.h"
#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_osm_panel.h"
#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_portal_panel.h"
#include "selfdrive/ui/bluepilot/qt/offroad/panels/bp_external_storage_panel.h"

// Sunnypilot panels still needed
#include "selfdrive/ui/sunnypilot/qt/offroad/settings/sunnylink_panel.h"
#include "selfdrive/ui/sunnypilot/qt/offroad/settings/trips_panel.h"

BPSettingsWindow::BPSettingsWindow(QWidget *parent) : SettingsWindow(parent) {
  // Main horizontal layout
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setContentsMargins(0, 0, 0, 0);
  main_layout->setSpacing(0);

  // === SIDEBAR SECTION ===
  sidebar_widget = new QWidget;
  sidebar_widget->setObjectName("bpSidebar");
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setContentsMargins(30, 30, 20, 30);
  sidebar_layout->setSpacing(0);

  // Close button at top
  QPushButton *close_btn = new QPushButton("×");
  close_btn->setObjectName("bpCloseButton");
  close_btn->setFixedSize(100, 100);
  QObject::connect(close_btn, &QPushButton::clicked, this, &BPSettingsWindow::closeSettings);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignLeft);
  sidebar_layout->addSpacing(20);

  // Navigation buttons container
  QWidget *buttons_widget = new QWidget;
  QVBoxLayout *buttons_layout = new QVBoxLayout(buttons_widget);
  buttons_layout->setContentsMargins(0, 0, 30, 0);
  buttons_layout->setSpacing(12);

  // Main panel widget
  panel_widget = new QStackedWidget();
  panel_widget->setObjectName("bpPanelWidget");

  // === SETUP PANELS ===
  // BP Network Panel (Native C++)
  auto bpNetworkPanel = new BPNetworkPanel(this);
  QObject::connect(uiState()->prime_state, &PrimeState::changed, bpNetworkPanel, &BPNetworkPanel::setPrimeType);

  // JSON-based panels
  BPBaseView *bpDeviceView = new BPBaseView(this);
  bpDeviceView->initialize("/selfdrive/ui/bluepilot/menus/bp_device_panel.json");
  QObject::connect(bpDeviceView, &BPBaseView::showDriverView, this, &BPSettingsWindow::showDriverView);
  QObject::connect(bpDeviceView, &BPBaseView::reviewTrainingGuide, this, &BPSettingsWindow::reviewTrainingGuide);

  BPBaseView *bpVisualsView = new BPBaseView(this);
  bpVisualsView->initialize("/selfdrive/ui/bluepilot/menus/bp_visuals_panel.json");

  BPBaseView *bpDisplayView = new BPBaseView(this);
  bpDisplayView->initialize("/selfdrive/ui/bluepilot/menus/bp_display_panel.json");

  BPBaseView *bpTogglesView = new BPBaseView(this);
  bpTogglesView->initialize("/selfdrive/ui/bluepilot/menus/bp_toggles_panel.json");

  BPBaseView *bpCruiseView = new BPBaseView(this);
  bpCruiseView->initialize("/selfdrive/ui/bluepilot/menus/bp_cruise_panel.json");

  BPBaseView *bpSteeringView = new BPBaseView(this);
  bpSteeringView->initialize("/selfdrive/ui/bluepilot/menus/bp_steering_panel.json");

  BPBaseView *bpDeveloperView = new BPBaseView(this);
  bpDeveloperView->initialize("/selfdrive/ui/bluepilot/menus/bp_developer_panel.json");

  BPBaseView *bpVehicleView = new BPBaseView(this);
  bpVehicleView->initialize("/selfdrive/ui/bluepilot/menus/bp_vehicle_panel.json");

  // Define panels in order
  QList<PanelInfo> panels = {
    PanelInfo(tr("Device"), bpDeviceView, "../../sunnypilot/selfdrive/assets/offroad/icon_home.svg"),
    PanelInfo(tr("Network"), bpNetworkPanel, "../assets/icons/network.png"),
    PanelInfo(tr("BP Portal"), new BPPortalPanel(this), "../assets/offroad/icon_routes.png"),
    PanelInfo(tr("Toggles"), bpTogglesView, "../../sunnypilot/selfdrive/assets/offroad/icon_toggle.png"),
    PanelInfo(tr("Steering"), bpSteeringView, "../../sunnypilot/selfdrive/assets/offroad/icon_lateral.png"),
    PanelInfo(tr("Cruise"), bpCruiseView, "../assets/icons/speed_limit.png"),
    PanelInfo(tr("Visuals"), bpVisualsView, "../../sunnypilot/selfdrive/assets/offroad/icon_visuals.png"),
    PanelInfo(tr("Display"), bpDisplayView, "../../sunnypilot/selfdrive/assets/offroad/icon_display.png"),
    PanelInfo(tr("Software"), new BPSoftwarePanel(this), "../../sunnypilot/selfdrive/assets/offroad/icon_software.png"),
    PanelInfo(tr("Models"), new BPModelsPanel(this), "../../sunnypilot/selfdrive/assets/offroad/icon_models.png"),
    PanelInfo(tr("Storage"), new BPExternalStoragePanel(this), "../assets/offroad/icon_statistics.png"),
    PanelInfo(tr("OSM"), new BPOsmPanel(this), "../../sunnypilot/selfdrive/assets/offroad/icon_map.png"),
    PanelInfo(tr("Trips"), new TripsPanel(this), "../../sunnypilot/selfdrive/assets/offroad/icon_trips.png"),
    PanelInfo(tr("Vehicle"), bpVehicleView, "../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png"),
    PanelInfo(tr("Firehose"), new FirehosePanel(this), "../../sunnypilot/selfdrive/assets/offroad/icon_firehose.svg"),
    PanelInfo(tr("sunnylink"), new SunnylinkPanel(this), "../assets/icons/wifi_strength_full.svg"),
    PanelInfo(tr("Developer"), bpDeveloperView, "../assets/icons/shell.png"),
    PanelInfo(tr("Statistics"), new BPStatisticsPanel(this), "../assets/offroad/icon_statistics.png"),
  };

  // === CREATE NAVIGATION BUTTONS ===
  nav_btns = new QButtonGroup(this);
  nav_btns->setExclusive(true);

  for (int i = 0; i < panels.size(); ++i) {
    auto &[name, panel, icon] = panels[i];

    // Add spacing between icon and text
    QPushButton *btn = new QPushButton("  " + name);
    btn->setObjectName("bpNavButton");
    btn->setCheckable(true);
    btn->setChecked(i == 0);
    btn->setIcon(QIcon(QPixmap(icon)));
    btn->setIconSize(QSize(50, 50));
    btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    btn->setMinimumHeight(70);

    nav_btns->addButton(btn, i);
    buttons_layout->addWidget(btn);

    // Set panel margins (Network panel handles its own)
    const int lr_margin = (name != tr("Network")) ? 50 : 0;
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    // Use BPScrollView for BP panels, ScrollViewSP for others
    QScrollArea *panel_frame;
    if (qobject_cast<BPBaseView*>(panel) || qobject_cast<BPNavBarView*>(panel) ||
        qobject_cast<BPPortalPanel*>(panel) || qobject_cast<BPStatisticsPanel*>(panel) ||
        qobject_cast<BPNetworkPanel*>(panel)) {
      panel_frame = new BPScrollView(panel, this);
    } else {
      panel_frame = new ScrollViewSP(panel, this);
    }
    panel_widget->addWidget(panel_frame);
  }

  QObject::connect(nav_btns, QOverload<QAbstractButton *>::of(&QButtonGroup::buttonClicked), [=](QAbstractButton *btn) {
    panel_widget->setCurrentIndex(nav_btns->id(btn));
    updateButtonStyles(btn);
  });

  // Set initial state
  if (nav_btns->buttons().size() > 0) {
    updateButtonStyles(nav_btns->buttons().at(0));
  }

  // Add buttons to scrollable area
  ScrollViewSP *buttons_scrollview = new ScrollViewSP(buttons_widget, this);
  sidebar_layout->addWidget(buttons_scrollview);

  // Set sidebar width - narrower than stock for more content space
  sidebar_widget->setFixedWidth(440);

  // Add sidebar and panel to main layout
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  // Set a global stylesheet for the settings window to ensure consistency
  // across all panels, especially those inherited from sunnypilot that
  // may not have their own styling.
  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    BPSettingsWindow {
      background-color: black;
    }
    QStackedWidget, BPScrollView, ScrollViewSP {
      background-color: black;
      border-radius: 30px;
    }
  )");

  // === MODERN BP STYLING ===
  // Only style the sidebar, let panels use their own styling
  sidebar_widget->setStyleSheet(QString(R"(
    /* Sidebar background */
    QWidget#bpSidebar {
      background-color: %1;
      border-right: 1px solid rgba(255, 255, 255, 0.1);
      border-radius: 20px;
    }

    /* Close button - modern circular design */
    QPushButton#bpCloseButton {
      font-size: 90px;
      padding-bottom: 8px;
      border-radius: 50px;
      background-color: %2;
      border: 2px solid rgba(255, 255, 255, 0.1);
      font-weight: 300;
      color: %3;
    }
    QPushButton#bpCloseButton:hover {
      background-color: %4;
      border-color: rgba(255, 255, 255, 0.15);
    }
    QPushButton#bpCloseButton:pressed {
      background-color: %5;
      color: %6;
    }

    /* Navigation buttons - modern flat design */
    QPushButton#bpNavButton {
      border-radius: 12px;
      background-color: transparent;
      border: 2px solid transparent;
      color: %7;
      font-size: 48px;
      font-weight: 500;
      text-align: left;
      padding-left: 18px;
      padding-right: 10px;
    }
    QPushButton#bpNavButton:hover {
      background-color: %4;
      border: 2px solid transparent;
    }
    QPushButton#bpNavButton:pressed {
      background-color: %5;
      color: %7;
      border: 2px solid transparent;
    }
  )").arg(bp_background.name())          // %1
     .arg(bp_card_background.name())     // %2
     .arg(bp_text_secondary.name())      // %3
     .arg(bp_button_hover.name())        // %4
     .arg(bp_button_pressed.name())      // %5
     .arg(bp_text_primary.name())        // %6
     .arg(bp_text_secondary.name())      // %7
  );
}

void BPSettingsWindow::updateButtonStyles(QAbstractButton *active_btn) {
  for (auto btn : nav_btns->buttons()) {
    if (btn == active_btn) {
      btn->setStyleSheet("background-color: #0084FF; color: white;");
    } else {
      btn->setStyleSheet(""); // Revert to default stylesheet
    }
  }
}

void BPSettingsWindow::showEvent(QShowEvent *event) {
  // Call parent implementation which resets to panel 0
  SettingsWindow::showEvent(event);

  // Sync button styles with the reset panel selection
  if (nav_btns && nav_btns->buttons().size() > 0) {
    updateButtonStyles(nav_btns->buttons().at(0));
  }
}

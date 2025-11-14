// selfdrive/ui/BP/qt/offroad/panels/bp_panel_conditions.cc

#include "selfdrive/ui/bluepilot/bp_logging.h"
#include "bp_panel_conditions.h"
#include "bp_panel_controls.h"
#include "common/swaglog.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/sunnypilot/ui.h"
#include "system/hardware/hw.h"

#include <QCoreApplication>
#include <capnp/dynamic.h>
#include "cereal/messaging/messaging.h"

bool PanelConditions::validateSingleCondition(const QString &conditionType, const QJsonValue &condition) {
  if (conditionType == "paramValueEquals") {
    QJsonObject equals = condition.toObject();
    for (auto it = equals.begin(); it != equals.end(); ++it) {
      std::string paramName = it.key().toStdString();
      std::string paramVal = params.get(paramName);
      QString expected = it.value().toString();

      if (paramVal.empty()) {
        return false;
      }

      bool isNumeric;
      double expectedNum = expected.toDouble(&isNumeric);

      if (isNumeric) {
        double actualNum = QString::fromStdString(paramVal).toDouble(&isNumeric);
        if (!isNumeric || std::abs(actualNum - expectedNum) > 1e-6) {
          return false;
        }
      } else if (paramVal != expected.toStdString()) {
        return false;
      }
    }
  } else if (conditionType == "paramValueGreaterThan") {
    QJsonObject greaterThan = condition.toObject();
    for (auto it = greaterThan.begin(); it != greaterThan.end(); ++it) {
      std::string paramName = it.key().toStdString();
      std::string paramVal = params.get(paramName);
      double compareNum = it.value().toDouble();

      if (paramVal.empty()) {
        return false;
      }

      double paramNum = std::stod(paramVal);
      if (paramNum <= compareNum) {
        return false;
      }
    }
  } else if (conditionType == "paramValueLessThan") {
    QJsonObject lessThan = condition.toObject();
    for (auto it = lessThan.begin(); it != lessThan.end(); ++it) {
      std::string paramName = it.key().toStdString();
      std::string paramVal = params.get(paramName);
      double compareNum = it.value().toDouble();

      if (paramVal.empty()) {
        return false;
      }

      double paramNum = std::stod(paramVal);
      if (paramNum >= compareNum) {
        return false;
      }
    }
  } else if (conditionType == "paramValueInRange") {
    QJsonObject range = condition.toObject();
    for (auto it = range.begin(); it != range.end(); ++it) {
      std::string paramName = it.key().toStdString();
      std::string paramVal = params.get(paramName);
      QJsonObject rangeValues = it.value().toObject();
      double min = rangeValues["min"].toDouble();
      double max = rangeValues["max"].toDouble();

      if (paramVal.empty()) {
        return false;
      }

      double paramNum = std::stod(paramVal);
      if (paramNum < min || paramNum > max) {
        return false;
      }
    }
  } else if (conditionType == "git_remote") {
    QJsonArray remotes = condition.toArray();
    std::vector<std::string> searchStrs;
    for (const auto &remote : remotes) {
      searchStrs.push_back(remote.toString().toStdString());
    }
    return isGitRemoteValid(searchStrs, {});
  } else if (conditionType == "git_branch") {
    QJsonArray branches = condition.toArray();
    std::vector<std::string> branchStrs;
    for (const auto &branch : branches) {
      branchStrs.push_back(branch.toString().toStdString());
    }
    return isGitRemoteValid({}, branchStrs);
  } else if (conditionType == "onlyWhenTheseParams") {
    QJsonArray requiredParams = condition.toArray();
    for (const auto &param : requiredParams) {
      std::string paramName = param.toString().toStdString();
      if (!params.getBool(paramName)) {
        return false;
      }
    }
  } else if (conditionType == "paramIsTrue") {
    if (condition.isString()) {
      // Single param: "paramIsTrue": "ParamName"
      QString paramName = condition.toString();
      return params.getBool(paramName.toStdString());
    } else if (condition.isArray()) {
      // Multiple params: "paramIsTrue": ["Param1", "Param2"]
      QJsonArray paramArray = condition.toArray();
      for (const auto &param : paramArray) {
        std::string paramName = param.toString().toStdString();
        if (!params.getBool(paramName)) {
          return false;
        }
      }
    } else if (condition.isObject()) {
      // Object format: "paramIsTrue": {"ParamName": true}
      QJsonObject paramObj = condition.toObject();
      for (auto it = paramObj.begin(); it != paramObj.end(); ++it) {
        std::string paramName = it.key().toStdString();
        if (!params.getBool(paramName)) {
          return false;
        }
      }
    }
  } else if (conditionType == "paramIsFalse") {
    if (condition.isString()) {
      // Single param: "paramIsFalse": "ParamName"
      QString paramName = condition.toString();
      return !params.getBool(paramName.toStdString());
    } else if (condition.isArray()) {
      // Multiple params: "paramIsFalse": ["Param1", "Param2"]
      QJsonArray paramArray = condition.toArray();
      for (const auto &param : paramArray) {
        std::string paramName = param.toString().toStdString();
        if (params.getBool(paramName)) {
          return false;
        }
      }
    } else if (condition.isObject()) {
      // Object format: "paramIsFalse": {"ParamName": true}
      QJsonObject paramObj = condition.toObject();
      for (auto it = paramObj.begin(); it != paramObj.end(); ++it) {
        std::string paramName = it.key().toStdString();
        if (params.getBool(paramName)) {
          return false;
        }
      }
    }
  } else if (conditionType == "paramLocked") {
    QString paramName = condition.toString();
    std::string lockParam = (paramName + "Lock").toStdString();
    return params.getBool(lockParam);
  } else if (conditionType == "paramNotLocked") {
    QString paramName = condition.toString();
    std::string lockParam = (paramName + "Lock").toStdString();
    return !params.getBool(lockParam);
  } else if (conditionType == "paramExists") {
    QString paramName = condition.toString();
    std::string paramVal = params.get(paramName.toStdString());
    return !paramVal.empty();
  } else if (conditionType == "paramNotExists") {
    QString paramName = condition.toString();
    std::string paramVal = params.get(paramName.toStdString());
    return paramVal.empty();
  } else if (conditionType == "hasCarParams") {
    auto cp_bytes = params.get("CarParamsPersistent");
    return !cp_bytes.empty();
  } else if (conditionType == "hasLongitudinalControl") {
    auto cp_bytes = params.get("CarParamsPersistent");
    if (cp_bytes.empty()) {
      return false;
    }
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
    bool has_long = hasLongitudinalControl(CP);
    bool expected = condition.toBool();
    return (has_long == expected);
  } else if (conditionType == "hasAlphaLongitudinalAvailable") {
    auto cp_bytes = params.get("CarParamsPersistent");
    if (cp_bytes.empty()) {
      return false;
    }
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
    bool has_alpha_long = CP.getAlphaLongitudinalAvailable();
    bool expected = condition.toBool();
    return (has_alpha_long == expected);
  } else if (conditionType == "isReleaseBranch") {
    return params.getBool("IsReleaseBranch") == condition.toBool();
  } else if (conditionType == "isTestedBranch") {
    return params.getBool("IsTestedBranch") == condition.toBool();
  } else if (conditionType == "isDevelopmentBranch") {
    return params.getBool("IsDevelopmentBranch") == condition.toBool();
  } else if (conditionType == "isNotReleaseBranch") {
    return !params.getBool("IsReleaseBranch") == condition.toBool();
  } else if (conditionType == "disableUpdates") {
    return params.getBool("DisableUpdates");
  } else if (conditionType == "isOffroad") {
    return !uiState()->scene.started;
  } else if (conditionType == "isOnroad") {
    return uiState()->scene.started;
  } else if (conditionType == "isEngaged") {
    return uiState()->engaged();
  } else if (conditionType == "isNotEngaged") {
    return !uiState()->engaged();
  } else if (conditionType == "isPcmCruise") {
    auto cp_bytes = params.get("CarParamsPersistent");
    if (cp_bytes.empty()) {
      return false;
    }
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
    return CP.getPcmCruise();
  } else if (conditionType == "hasIntelligentCruiseButtonManagement") {
    auto cp_sp_bytes = params.get("CarParamsSPPersistent");
    if (cp_sp_bytes.empty()) {
      return false;
    }
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_sp_bytes.data(), cp_sp_bytes.size()));
    cereal::CarParamsSP::Reader CP_SP = cmsg.getRoot<cereal::CarParamsSP>();
    bool has_icbm = CP_SP.getIntelligentCruiseButtonManagementAvailable();
    bool expected = condition.toBool();
    return (has_icbm == expected);
  } else if (conditionType == "isTiciHardware") {
    return Hardware::TICI();
  } else if (conditionType == "isPcHardware") {
    return Hardware::PC();
  } else if (conditionType == "isMadsLimitedBrand") {
    auto cp_bytes = params.get("CarParamsPersistent");
    if (cp_bytes.empty()) {
      return false;
    }
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
    std::string brand = CP.getBrand();

    // Determine if brand is limited
    bool is_limited = false;

    // Rivian always has limited MADS settings
    if (brand == "rivian") {
      is_limited = true;
    }
    // Tesla only has limited MADS settings if it doesn't have vehicle bus access
    else if (brand == "tesla") {
      auto cp_sp_bytes = params.get("CarParamsSPPersistent");
      if (!cp_sp_bytes.empty()) {
        AlignedBuffer aligned_buf_sp;
        capnp::FlatArrayMessageReader cmsg_sp(aligned_buf_sp.align(cp_sp_bytes.data(), cp_sp_bytes.size()));
        cereal::CarParamsSP::Reader CP_SP = cmsg_sp.getRoot<cereal::CarParamsSP>();
        bool has_vehicle_bus = CP_SP.getFlags() & 1;  // 1 == TeslaFlagsSP.HAS_VEHICLE_BUS
        is_limited = !has_vehicle_bus;
      } else {
        is_limited = true;  // Default to limited if we can't check
      }
    } else {
      is_limited = false;
    }

    // Compare the result with the expected value from JSON
    bool expected = condition.toBool();
    return (is_limited == expected);
  } else if (conditionType == "hasBlindSpotMonitoring") {
    auto cp_bytes = params.get("CarParamsPersistent");
    if (cp_bytes.empty()) {
      return false;
    }
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
    bool has_bsm = CP.getEnableBsm();
    bool expected = condition.toBool();
    return (has_bsm == expected);
  } else if (conditionType == "isAngleSteering") {
    auto cp_bytes = params.get("CarParamsPersistent");
    if (cp_bytes.empty()) {
      return false;
    }
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
    bool is_angle = (CP.getSteerControlType() == cereal::CarParams::SteerControlType::ANGLE);
    bool expected = condition.toBool();
    return (is_angle == expected);
  } else if (conditionType == "paramExists") {
    QString paramName = condition.toString();
    auto value = params.get(paramName.toStdString());
    return !value.empty();
  } else if (conditionType == "paramNotExists") {
    QString paramName = condition.toString();
    auto value = params.get(paramName.toStdString());
    return value.empty();
  } else if (conditionType == "brandEquals") {
    QString expectedBrand = condition.toString().toLower();

    // First check if there's a manually selected platform
    QString platform_bundle = QString::fromStdString(params.get("CarPlatformBundle"));
    if (!platform_bundle.isEmpty()) {
      QJsonDocument json = QJsonDocument::fromJson(platform_bundle.toUtf8());
      if (!json.isNull() && json.isObject()) {
        QString brand = json.object().value("brand").toString().toLower();
        return brand == expectedBrand;
      }
    }

    // Otherwise, check the detected brand from CarParams
    auto cp_bytes = params.get("CarParamsPersistent");
    if (cp_bytes.empty()) {
      return false;
    }
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
    QString carName = QString::fromStdString(CP.getCarFingerprint()).toLower();

    // Extract brand from car name (e.g., "FORD BRONCO SPORT 2021" -> "ford")
    if (carName.contains("ford")) return expectedBrand == "ford";
    if (carName.contains("toyota")) return expectedBrand == "toyota";
    if (carName.contains("honda")) return expectedBrand == "honda";
    if (carName.contains("hyundai") || carName.contains("kia") || carName.contains("genesis")) return expectedBrand == "hyundai";
    if (carName.contains("gm") || carName.contains("chevrolet") || carName.contains("cadillac")) return expectedBrand == "gm";
    if (carName.contains("chrysler") || carName.contains("jeep") || carName.contains("ram") || carName.contains("dodge")) return expectedBrand == "chrysler";
    if (carName.contains("mazda")) return expectedBrand == "mazda";
    if (carName.contains("nissan")) return expectedBrand == "nissan";
    if (carName.contains("subaru")) return expectedBrand == "subaru";
    if (carName.contains("volkswagen") || carName.contains("vw")) return expectedBrand == "volkswagen";
    if (carName.contains("tesla")) return expectedBrand == "tesla";
    if (carName.contains("rivian")) return expectedBrand == "rivian";

    return false;
  }

  return true;
}

bool PanelConditions::validateConditionObject(const QJsonObject &conditionObj) {
  if (conditionObj.contains("allConditionsTrue") || conditionObj.contains("anyConditionsTrue")) {
    return validateCompositeConditions(conditionObj);
  }

  for (auto it = conditionObj.begin(); it != conditionObj.end(); ++it) {
    if (!PanelConditions::validateSingleCondition(it.key(), it.value())) {
      return false;
    }
  }

  return true;
}

bool PanelConditions::validateCompositeConditions(const QJsonObject &conditions) {
  // Handle non-composite conditions (bare condition objects without allConditionsTrue/anyConditionsTrue)
  // If the object doesn't contain composite keys, treat it as a simple condition object
  if (!conditions.contains("allConditionsTrue") && !conditions.contains("anyConditionsTrue")) {
    return validateConditionObject(conditions);
  }

  bool result = true;

  if (conditions.contains("anyConditionsTrue")) {
    QJsonArray anyConditions = conditions["anyConditionsTrue"].toArray();
    if (!anyConditions.empty()) {
      bool anyTrue = false;
      for (const auto &condition : anyConditions) {
        if (condition.isObject()) {
          QJsonObject condObj = condition.toObject();
          bool conditionResult;

          if (condObj.contains("allConditionsTrue") || condObj.contains("anyConditionsTrue")) {
            conditionResult = validateCompositeConditions(condObj);
          } else {
            conditionResult = validateConditionObject(condObj);
          }

          if (conditionResult) {
            anyTrue = true;
            break;
          }
        }
      }
      result &= anyTrue;
    }
  }

  if (conditions.contains("allConditionsTrue")) {
    QJsonArray allConditions = conditions["allConditionsTrue"].toArray();
    for (const auto &condition : allConditions) {
      if (condition.isObject()) {
        QJsonObject condObj = condition.toObject();
        bool conditionResult;

        if (condObj.contains("allConditionsTrue") || condObj.contains("anyConditionsTrue")) {
          conditionResult = validateCompositeConditions(condObj);
        } else {
          conditionResult = validateConditionObject(condObj);
        }

        if (!conditionResult) {
          result = false;
          break;
        }
      }
    }
  }

  return result;
}

void PanelConditions::updateConditionsForAllControls(std::function<void()> updateGroupVisibility) {
  static std::set<QWidget *> processedControls;
  processedControls.clear();

  // Create a copy since we might modify during iteration
  auto conditions = controlConditions;
  for (const auto &pair : conditions) {
    QWidget *ctrl = pair.first;

    // Skip if control was deleted
    if (!ctrl || ctrl->parent() == nullptr) {
      controlConditions.erase(ctrl);
      continue;
    }

    const ControlConditions &conds = pair.second;

    // Check if this control has any conditions (legacy, enabled, or visible)
    if ((conds.hasConditions || conds.hasEnableConditions || conds.hasVisibleConditions) && ctrl) {
      if (processedControls.find(ctrl) != processedControls.end()) {
        continue;
      }
      processedControls.insert(ctrl);

      // Use the unified updateConditionsForWidget function which handles all condition types
      updateConditionsForWidget(ctrl, conds);
    }
  }

  updateGroupVisibility();
}

bool PanelConditions::isGitRemoteValid(const std::vector<std::string> &searchStrs, const std::vector<std::string> &branchNames) {
  std::string gitRemote = params.get("GitRemote");
  std::string gitBranch = params.get("GitBranch");

  bool debugMode = gitRemote.empty();

  if (debugMode || searchStrs.empty() || std::find(searchStrs.begin(), searchStrs.end(), "any") != searchStrs.end()) {
    return true;
  }

  if (gitRemote.empty()) {
    return false;
  }

  bool searchStrFound = false;
  for (const auto &searchStr : searchStrs) {
    if (!searchStr.empty() && gitRemote.find(searchStr) != std::string::npos) {
      searchStrFound = true;
      break;
    }
  }

  if (!searchStrFound) {
    return false;
  }

  if (!branchNames.empty()) {
    bool branchFound = false;
    for (const auto &branchName : branchNames) {
      if (!branchName.empty() && gitBranch == branchName) {
        branchFound = true;
        break;
      }
    }
    if (!branchFound) {
      return false;
    }
  }

  return true;
}

void PanelConditions::logConditionCheck(const QString &controlName, const std::function<void()> &logFunc) {
  const int width = 80;
  std::string separator(width, '=');
  std::string controlNameStr = controlName.toStdString();
  int padding = (width - controlNameStr.length() - 2) / 2;
  std::string centeredName = std::string(padding, ' ') + controlNameStr + std::string(padding, ' ');

  BPLog::bpInfo() << "[bp.panel.conditions] logConditionCheck | \n" << separator << std::endl;
  BPLog::bpInfo() << "[bp.panel.conditions] logConditionCheck | " << centeredName << std::endl;
  BPLog::bpInfo() << "[bp.panel.conditions] logConditionCheck | " << separator << std::endl;

  logFunc();

  BPLog::bpInfo() << "[bp.panel.conditions] logConditionCheck | " << separator << "\n" << std::endl;
}

bool PanelConditions::updateConditionsForWidget(QWidget *widget, const ControlConditions &conditions) {
  if (!widget) {
    return true;
  }

  // Performance optimization: use cached state if valid
  if (conditions.cachedStateValid) {
    // Quick path: just verify widget state matches cache
    if (widget->isEnabled() == conditions.lastEnabledState &&
        widget->isVisible() == conditions.lastVisibleState) {
      return conditions.lastEnabledState;
    }
  }

  // Determine enabled state
  bool shouldBeEnabled = true;
  if (conditions.hasEnableConditions) {
    // Use new enableConditions
    shouldBeEnabled = validateCompositeConditions(conditions.enableConditions);
  } else if (conditions.hasConditions) {
    // Legacy: use conditions for enabled state
    shouldBeEnabled = validateCompositeConditions(conditions.conditions);
  }

  // Determine visible state
  bool shouldBeVisible = true;
  if (conditions.hasVisibleConditions) {
    // Use new visibleConditions
    shouldBeVisible = validateCompositeConditions(conditions.visibleConditions);
  }
  // Note: Legacy "conditions" does NOT control visibility, only enabled state
  // This maintains backward compatibility where controls were always visible but could be disabled

  // Update cache
  const_cast<ControlConditions&>(conditions).lastEnabledState = shouldBeEnabled;
  const_cast<ControlConditions&>(conditions).lastVisibleState = shouldBeVisible;
  const_cast<ControlConditions&>(conditions).cachedStateValid = true;

  // Handle auto-reset when conditions are not met
  if (!shouldBeEnabled && conditions.hasAutoReset && !conditions.paramName.isEmpty()) {
    std::string currentValue = params.get(conditions.paramName.toStdString());
    std::string resetValue = conditions.autoResetValue.toStdString();

    if (currentValue != resetValue) {
      params.put(conditions.paramName.toStdString(), resetValue);
      BPLog::bpInfo() << "[bp.panel.conditions] updateConditionsForWidget | Auto-reset param - "
                      << conditions.paramName.toStdString() << ": " << currentValue << " -> " << resetValue << std::endl;
    }
  }

  // Handle dynamic descriptions
  if (conditions.hasDynamicDescriptions) {
    QString newDesc = evaluateDescription(conditions);

    // Try casting to different control types
    if (auto toggle = qobject_cast<BPToggleControl*>(widget)) {
      toggle->setDescription(newDesc);
    } else if (auto segmented = qobject_cast<BPSegmentedControl*>(widget)) {
      segmented->setDescription(newDesc);
    } else if (auto selection = qobject_cast<BPSelectionControl*>(widget)) {
      selection->setDescription(newDesc);
    } else if (auto numeric = qobject_cast<BPNumericControl*>(widget)) {
      numeric->setDescription(newDesc);
    }
  }

  // Apply visibility state
  bool visibilityChanged = false;
  if (widget->isVisible() != shouldBeVisible) {
    widget->setVisible(shouldBeVisible);
    visibilityChanged = true;
  }

  // Apply enabled state (only matters if widget is visible)
  bool enabledChanged = false;
  if (shouldBeVisible && widget->isEnabled() != shouldBeEnabled) {
    widget->setEnabled(shouldBeEnabled);
    widget->setProperty("enabled", QVariant(shouldBeEnabled));
    enabledChanged = true;
  }

  // Batch style updates - only recalculate once if anything changed
  if (enabledChanged && shouldBeVisible) {
    widget->style()->unpolish(widget);
    widget->style()->polish(widget);
  }

  // Single update call only if state actually changed
  if (visibilityChanged || enabledChanged) {
    widget->update();
  }

  // Handle disabled reasons for controls that aren't enabled
  if (!shouldBeEnabled && conditions.hasEnableConditions) {
    // Collect reasons from failed conditions
    QStringList reasons = getFailedConditionReasons(conditions.enableConditions, QJsonValue());
    reasons.removeDuplicates();

    // Try casting to different control types and set disabled reasons
    if (auto toggle = qobject_cast<BPToggleControl*>(widget)) {
      toggle->setDisabledReasons(reasons);
    } else if (auto paramToggle = qobject_cast<BPParamToggleButton*>(widget)) {
      paramToggle->setDisabledReasons(reasons);
    } else if (auto segmented = qobject_cast<BPSegmentedControl*>(widget)) {
      segmented->setDisabledReasons(reasons);
    } else if (auto selection = qobject_cast<BPSelectionControl*>(widget)) {
      selection->setDisabledReasons(reasons);
    } else if (auto numeric = qobject_cast<BPNumericControl*>(widget)) {
      numeric->setDisabledReasons(reasons);
    }
  } else {
    // Control is enabled or has no conditions - clear disabled reasons
    if (auto toggle = qobject_cast<BPToggleControl*>(widget)) {
      toggle->setDisabledReasons(QStringList());
    } else if (auto paramToggle = qobject_cast<BPParamToggleButton*>(widget)) {
      paramToggle->setDisabledReasons(QStringList());
    } else if (auto segmented = qobject_cast<BPSegmentedControl*>(widget)) {
      segmented->setDisabledReasons(QStringList());
    } else if (auto selection = qobject_cast<BPSelectionControl*>(widget)) {
      selection->setDisabledReasons(QStringList());
    } else if (auto numeric = qobject_cast<BPNumericControl*>(widget)) {
      numeric->setDisabledReasons(QStringList());
    }
  }

  return shouldBeEnabled;
}

QString PanelConditions::evaluateDescription(const ControlConditions &conditions) {
  QString description;

  if (!conditions.hasDynamicDescriptions || conditions.descriptions.isEmpty()) {
    description = conditions.defaultDescription;
  } else {
    // Iterate through description conditions, return first match
    for (auto it = conditions.descConditions.begin(); it != conditions.descConditions.end(); ++it) {
      QString key = it.key();
      QJsonObject conditionObj = it.value();

      if (validateCompositeConditions(conditionObj)) {
        // Found a matching condition, return its description
        if (conditions.descriptions.contains(key)) {
          description = conditions.descriptions[key];
          break;
        }
      }
    }

    // No conditions matched, use default
    if (description.isEmpty()) {
      description = conditions.defaultDescription;
    }
  }

  // Perform parameter substitution if configured
  if (conditions.hasParamSubstitutions) {
    description = performParameterSubstitution(description, conditions.paramSubstitutions);
  }

  return description;
}

QString PanelConditions::readCarParamSPValue(cereal::CarParamsSP::Reader &CP_SP, const QString &path) {
  // Parse nested path like "neuralNetworkLateralControl.model.name"
  QStringList parts = path.split(".");

  if (parts[0] == "neuralNetworkLateralControl") {
    auto nnlc = CP_SP.getNeuralNetworkLateralControl();
    if (parts.size() >= 2 && parts[1] == "model") {
      auto model = nnlc.getModel();
      if (parts.size() >= 3 && parts[2] == "name") {
        return QString::fromStdString(model.getName());
      }
    } else if (parts.size() >= 2 && parts[1] == "fuzzyFingerprint") {
      return nnlc.getFuzzyFingerprint() ? "true" : "false";
    }
  }

  return QString();
}

QString PanelConditions::readCarParamValue(cereal::CarParams::Reader &CP, const QString &path) {
  // Implement common CarParams fields as needed
  // For now, return empty - extend as needed for future use cases
  return QString();
}

QString PanelConditions::performParameterSubstitution(const QString &text, const QJsonObject &substitutions) {
  QString result = text;

  // Iterate through each substitution defined in the JSON
  for (auto it = substitutions.begin(); it != substitutions.end(); ++it) {
    QString placeholder = it.key();  // e.g., "model_name"
    QJsonObject substConfig = it.value().toObject();

    QString substValue;
    QString substType = substConfig["type"].toString();

    if (substType == "param") {
      // Simple param read
      QString paramName = substConfig["param"].toString();
      substValue = QString::fromStdString(params.get(paramName.toStdString()));
    } else if (substType == "carParam" || substType == "carParamSP") {
      // Read from CarParams or CarParamsSP
      std::string carParamsKey = (substType == "carParam") ? "CarParamsPersistent" : "CarParamsSPPersistent";
      auto cp_bytes = params.get(carParamsKey);

      if (!cp_bytes.empty()) {
        AlignedBuffer aligned_buf;
        capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));

        QString path = substConfig["path"].toString();

        if (substType == "carParam") {
          cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
          substValue = readCarParamValue(CP, path);
        } else {
          cereal::CarParamsSP::Reader CP_SP = cmsg.getRoot<cereal::CarParamsSP>();
          substValue = readCarParamSPValue(CP_SP, path);
        }
      }
    }

    // Apply value mapping if configured
    if (substConfig.contains("valueMap")) {
      QJsonObject valueMap = substConfig["valueMap"].toObject();
      if (valueMap.contains(substValue)) {
        substValue = valueMap[substValue].toString();
      }
    }

    // Replace placeholder in text (support both {placeholder} and {{placeholder}} formats)
    result.replace(QString("{%1}").arg(placeholder), substValue);
    result.replace(QString("{{%1}}").arg(placeholder), substValue);
  }

  return result;
}

QStringList PanelConditions::getFailedConditionReasons(const QJsonObject &conditions, const QJsonValue &disabledReasonConfig) {
  QStringList reasons;

  // Check each condition in the conditions object
  for (auto it = conditions.begin(); it != conditions.end(); ++it) {
    QString conditionType = it.key();
    QJsonValue conditionValue = it.value();

    // Skip git-related conditions (no user-facing reason needed)
    if (conditionType == "git_remote" || conditionType == "git_branch") {
      continue;
    }

    // Handle composite conditions by recursing into their children
    if (conditionType == "allConditionsTrue" || conditionType == "anyConditionsTrue") {
      QJsonArray condArray = conditionValue.toArray();
      for (const auto &subCond : condArray) {
        if (subCond.isObject()) {
          QJsonObject subCondObj = subCond.toObject();
          // Extract "reason" field if present
          QString reason;
          if (subCondObj.contains("reason")) {
            reason = subCondObj["reason"].toString();
          }

          // Create a copy without the "reason" field for validation
          QJsonObject conditionOnly = subCondObj;
          conditionOnly.remove("reason");

          // Recursively check sub-conditions
          reasons.append(getFailedConditionReasons(conditionOnly, disabledReasonConfig));

          // If this specific condition failed and has a reason, add it
          if (!reason.isEmpty()) {
            bool conditionMet = validateConditionObject(conditionOnly);
            if (!conditionMet) {
              reasons.append(reason);
            }
          }
        }
      }
      continue;
    }

    // For leaf conditions, the value might be an object with a "reason" field
    // e.g., { "isOffroad": true, "reason": "Must be offroad" }
    // But this doesn't match our JSON structure. Our structure is:
    // { "isOffroad": true, "reason": "..." } inside an array element

    // We don't need to handle single conditions here since they're handled above in composite conditions
  }

  return reasons;
}

QString PanelConditions::getDisabledReason(QWidget *widget) {
  if (!widget) {
    return QString();
  }

  auto it = controlConditions.find(widget);
  if (it == controlConditions.end()) {
    return QString();
  }

  const ControlConditions &conditions = it->second;

  // Only show disabled reason if control is actually disabled and has conditions
  if (!conditions.hasConditions || widget->isEnabled()) {
    return QString();
  }

  // Check if conditions are met
  bool conditionsMet = validateCompositeConditions(conditions.conditions);
  if (conditionsMet) {
    return QString();  // Conditions met, no disabled reason needed
  }

  // Extract reasons from the conditions (reasons are now embedded in the condition objects)
  QStringList reasons = getFailedConditionReasons(conditions.conditions, QJsonValue());
  reasons.removeDuplicates();

  if (reasons.isEmpty()) {
    return QString();
  } else if (reasons.size() == 1) {
    return reasons.first();
  } else {
    // Multiple reasons - format as bullet list
    return "• " + reasons.join("\n• ");
  }
}

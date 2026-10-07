"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp


class FiskerSettings(BrandSettings):
  def __init__(self):
    super().__init__()

    # ICC_0x52A spoof flips byte 4's ICCACCAutoSpdSts bit according to this toggle.
    # Default On matches the baseline on-vehicle behaviour; turning it off clears
    # just that bit (ACC itself still engages because ICC_ACCSwt stays 1).
    self.acc_auto_speed_toggle = toggle_item_sp(
      tr("ACC Auto-Speed"),
      tr("Allow ACC to automatically adopt the posted speed limit as its target speed "
         "(fed from TSR). When off, ACC holds the driver's last set speed until the "
         "driver changes it with the wheel buttons."),
      param="FiskerACCAutoSpeed",
    )

    self.items = [self.acc_auto_speed_toggle]

  def update_settings(self):
    pass

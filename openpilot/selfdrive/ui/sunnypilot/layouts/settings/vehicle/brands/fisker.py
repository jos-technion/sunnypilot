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

    # All three toggles plumb through to CP_SP flags via opendbc's _initialize_fisker;
    # the fisker carcontroller then feeds the current flag state into the ICC_0x52A
    # spoof packer / lateral-control builder every tick. Takes effect at start-of-drive
    # (CP_SP is built once per boot — flip toggles while offroad).

    # ICC_0x52A byte 4 bit 35 (ICCACCAutoSpdSts). Default On matches the baseline
    # on-vehicle behaviour; off clears just that bit (ACC itself still engages).
    self.acc_auto_speed_toggle = toggle_item_sp(
      tr("ACC Auto-Speed"),
      tr("Allow ACC to automatically adopt the posted speed limit as its target speed "
         "(fed from TSR). When off, ACC holds the driver's last set speed until the "
         "driver changes it with the wheel buttons."),
      param="FiskerACCAutoSpeed",
    )

    # ICC_0x52A byte 6 bit 0 (ICCACCTerrainSetting). Default Off matches what ADAS
    # sees without openpilot; turning on tells ADAS to run ACC in Terrain mode (longer
    # following distance, gentler acceleration profiles tuned for low-traction surfaces).
    self.acc_terrain_toggle = toggle_item_sp(
      tr("ACC Terrain Mode"),
      tr("Signal to the ADAS module that ACC should run in Terrain mode — intended for "
         "off-road / low-traction driving (longer follow gap, softer accel). Default off."),
      param="FiskerACCTerrain",
    )

    # ADAS_0x1C0 ADAS_LatCtrl_Typ. Default Off = LKA (Typ=1, the only mode the Ocean's
    # EPS honours in practice). On = LCA/TJA (Typ=3) — ON-VEHICLE TESTING SHOWED THE
    # OCEAN'S EPS RAISES AN ADAS DTC when it sees Typ=3. Toggle is left in the UI for
    # future trims that may support it, but keep it OFF on the Ocean.
    self.lateral_type_toggle = toggle_item_sp(
      tr("Use LCA / TJA instead of LKA (unsupported — raises DTC)"),
      tr("Switch ADAS_LatCtrl_Typ from LKA (1) to LCA_or_TJA (3). The Ocean's EPS "
         "RAISES AN ADAS DTC when it sees Typ=3 (confirmed on-vehicle), so leave this "
         "OFF. Kept in the UI for future trims that may support TJA. Default off (LKA)."),
      param="FiskerLateralType",
    )

    self.items = [self.acc_auto_speed_toggle, self.acc_terrain_toggle, self.lateral_type_toggle]

  def update_settings(self):
    pass

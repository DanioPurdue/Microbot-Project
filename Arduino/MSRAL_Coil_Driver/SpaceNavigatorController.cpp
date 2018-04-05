#include "SpaceNavigatorController.h"

SpaceNavigatorController::SpaceNavigatorController(Coil *msral_coil) :
coil(msral_coil)
{
  Tx=Ty=Tz=Rx=Ry=Rz=0;
}

void SpaceNavigatorController::Parse(HID *hid, bool is_rpt_id, uint32_t len, uint8_t *buf) {
    OnSpaceNavigatorChanged((const SpaceNavigatorEventData*)buf);
}

void SpaceNavigatorController::OnSpaceNavigatorChanged(const SpaceNavigatorEventData *evt) {
    // Translation vector
    if (evt->rptid == 1) {
      Tx = evt->xl+(evt->xh<<8);
      Ty = evt->yl+(evt->yh<<8);
      Tz = evt->zl+(evt->zh<<8);
    }

    // Rotation vector
    else if (evt->rptid == 2) {
      Rx = evt->xl+(evt->xh<<8);
      Ry = evt->yl+(evt->yh<<8);
      Rz = evt->zl+(evt->zh<<8);
    }

    // Buttons
    else if (evt->rptid == 3) {
    }

    coil->move_controller(Tx, Ty, -Rz);
}

void SpaceNavigatorController::OnButton(uint8_t but_id) {
}


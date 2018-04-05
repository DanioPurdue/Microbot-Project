#if !defined(__HIDSPACENAVIGATORRPTPARSER_H__)
#define __HIDSPACENAVIGATORRPTPARSER_H__

#include <hid.h>
#include "Coil.h"

struct SpaceNavigatorEventData {
    uint8_t rptid, xl, xh, yl, yh, zl, zh;
};


class SpaceNavigatorController : public HIDReportParser {
    Coil *coil;
    
public:
    SpaceNavigatorController(Coil *msral_coil);
    virtual void Parse(HID *hid, bool is_rpt_id, uint32_t len, uint8_t *buf);
    virtual void OnSpaceNavigatorChanged(const SpaceNavigatorEventData *evt);
    virtual void OnButton(uint8_t but_id);
    
    int16_t Tx, Ty, Tz, Rx, Ry, Rz;
};

#endif // __HIDSPACENAVIGATORRPTPARSER_H__

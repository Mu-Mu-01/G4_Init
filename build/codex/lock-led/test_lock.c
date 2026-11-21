#include "dpll_app.c"
#include <assert.h>
int main(void) {
    DPLL_PI_Snapshot s = {.signal_valid = true, .phase_error_rad = 0.07f};
    assert(!DPLL_App_IsLocked());
    for (int i=0;i<3;i++) { DPLL_App_UpdateLock(&s,true); assert(!DPLL_App_IsLocked()); }
    DPLL_App_UpdateLock(&s,true); assert(DPLL_App_IsLocked());
    s.phase_error_rad = -0.20f; DPLL_App_UpdateLock(&s,true); assert(DPLL_App_IsLocked());
    s.phase_error_rad = -0.30f; DPLL_App_UpdateLock(&s,true); assert(!DPLL_App_IsLocked());
    for (int j=0;j<4;j++) {
        s = (DPLL_PI_Snapshot){.signal_valid=true};
        for (int i=0;i<4;i++) DPLL_App_UpdateLock(&s,true);
        assert(DPLL_App_IsLocked());
        if(j==0) s.signal_valid=false;
        if(j==1) s.saturated=true;
        if(j==2) s.phase_error_rad=NAN;
        DPLL_App_UpdateLock(&s,j!=3); assert(!DPLL_App_IsLocked());
    }
}

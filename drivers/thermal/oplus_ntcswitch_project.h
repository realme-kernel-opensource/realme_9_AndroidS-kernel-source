#ifndef _OPLUS_TEMPNTC_H_
#define _OPLUS_TEMPNTC_H_

#include <soc/oplus/oplus_project.h>

static inline int is_ntcswitch_projects(void) {
    if (get_project() == 21261 ||
        get_project() == 21262 ||
        get_project() == 21265 ||
        get_project() == 21266 ||
        get_project() == 21267 ||
        get_project() == 21268 ||
        get_project() == 21091 ||
        get_project() == 21029 ||
        get_project() == 21030 ||
        get_project() == 21311 ||
        get_project() == 21312 ||
        get_project() == 21313 ||
        get_project() == 0x2172F ||
        get_project() == 21730 ||
        get_project() == 21731) {
        return 1;
    }
    return 0;
}

/* BB NTC */
int oplus_thermal_tmp_get_bb(void);
/* board NTC */
int oplus_thermal_tmp_get_board(void);
/*chg NTC */
int oplus_thermal_tmp_get_chg(void);
/* flash NTC */
int oplus_thermal_tmp_get_flash(void);

int oplus_thermal_tmp_get_typc_1(void);
int oplus_thermal_tmp_get_typc_2(void);
int oplus_thermal_tmp_get_fled(void);
int oplus_thermal_temp_get_board(void);

#endif

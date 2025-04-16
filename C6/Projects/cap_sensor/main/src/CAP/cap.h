#ifndef CAP_H
#define CAP_H

#include "adc.h"  //
#include <math.h>
double average_capacitance_from_data(uint16_t *adc_data, int length, double vref, double Vfinal, double R);
double extract_capacitance_charging(double V0, double Vt, double Vfinal, double t, double R);

#endif // CAP_H
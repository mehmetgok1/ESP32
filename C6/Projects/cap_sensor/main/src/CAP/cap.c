
#include "cap.h"
double extract_capacitance_charging(double V0, double Vt, double Vfinal, double t, double R) {
    double ln_argument = (Vfinal - Vt) / (Vfinal - V0);
    if (ln_argument <= 0) return -1.0;
    return -t / (R * log(ln_argument));
}

double average_capacitance_from_data(uint16_t *adc_data, int length, double vref, double Vfinal, double R) {
    int mid = length / 2;
    double sum = 0.0;
    int valid_count = 0;
    //ESP_LOGI(TAG,"location1=%d data1=%d, location2=%d,data2=%d",10000,adc_data[10000],mid+10000,adc_data[mid+10000]);
    for (int i = 0; i < mid; i=i+50) {
        double V0 = ((double)adc_data[i] * vref) / 4096.0;
        double Vt = ((double)adc_data[mid + i] * vref) / 4096.0;
        double t = (double)mid/SAMPLE_RATE;

        double C = extract_capacitance_charging(V0, Vt, Vfinal, t, R);
        if (C > 0) {
            sum += C;
            valid_count++;
        }
    }

    if (valid_count == 0) return -1.0;  // No valid data
    return (sum / valid_count)*1e9;
}
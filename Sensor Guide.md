# SFM3003 Sensor Guide
The Sensirion SFM3003 datasheet should be consulted for technical information.

The sensor reports temperature and flow in "standard litres per minute" (slm) and these are reported. The flow is also converted to m/s using the calibration equations reported in International Journal of Speleology, 53 (1), 63-73. Note that the logged data has flow in both slm and m/s in case an alternative calibration/scaling approach is to be used. Refer to that paper for information concerning sensor tube alignment and positioning. Positive flow is in the direction of the arrow on the sensor body.

The sensor-specific firmware allows it to be used with some flexibility according to pattern selection at the "Main" level and described in the Specialised Guide. The central difference is whether or not the ULP Core (an ultra-low-power co-processor) is used. If the ULP Core is used then it takes single samples at a short time period and when a reading is requested the firmware returns a mean value and standard deviation. The standard deviation is computed over the flow in slm units.

Settings guidance:

- SFM\_OFFSET\_TEMP = value to add to the raw temperature reading before logging. The settings page reports the raw reading but elsewhere the values are reported after applying the offset.
- SFM\_OFFSET\_SLM = similar for the flow in slm units.
- SFM\_LP\_PRD\_S = the inter-sample period in seconds if the ULP Core is used. This is strictly the delay between samples, each of which takes about 75ms. These are __not__ synchronised with the main logging loop.
- SFM\_LP\_SET\_SIZE = the number of samples to be used when computing the mean. If there are not enough samples available then a record is still logged but with no values. This can not exceed the SFM_LP_BUFF_LEN "Compiled Config" value.

Offsets are normally established when the logger is initially installed and then left. Any sensor "drift" can then be compensated for at analysis time. If the offsets are changed, take care so that there is clarity when the data is analysed:

- Perform a download of the "new" type so that the saved data is segregated between different offset values.
- Preferably perform a new "NVS Dump" and save the active settings along with the data.

The LP Core period and set size, and the overall logging interval (APP\_LOOP\_S) should be selected with care. The maximum (and recommended) value for SFM\_LP_SET\_SIZE is 1 less than the simple division of APP\_LOOP\_S by SFM\_LP\_PRD\_S. A larger value will cause warnings in the App Log and records being logged with missing quantities. For example, a 5 minute (300s) logging period with 30s sampling period might be expected to give 10 samples but the additional time to take the measurements means that only 9 samples can be guaranteed because the first sample might have happened only a fraction of a second before the 30s mark.
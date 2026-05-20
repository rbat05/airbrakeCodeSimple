import pandas as pd
df = pd.read_csv('time_filtered.csv')
num_rows = len(df)
with open('include/hitl_data.h', 'w') as f:
    f.write('#pragma once\n')
    f.write('#include <stdint.h>\n\n')
    f.write('struct SensorDataRow {\n')
    f.write('    uint32_t timestamp_ms;\n')
    f.write('    float accelX;\n')
    f.write('    float accelY;\n')
    f.write('    float accelZ;\n')
    f.write('    float gyroX;\n')
    f.write('    float gyroY;\n')
    f.write('    float gyroZ;\n')
    f.write('    float pressureHPa;\n')
    f.write('    float altitudeM;\n')
    f.write('};\n\n')
    f.write(f'const int HITL_DATA_SIZE = {num_rows};\n\n')
    f.write('const SensorDataRow hitl_data[] = {\n')
    for i, row in df.iterrows():
        t = int(row['timestamp_ms'])
        ax = row['accelX']
        ay = row['accelY']
        az = row['accelZ']
        gx = row['gyroX']
        gy = row['gyroY']
        gz = row['gyroZ']
        p = row['pressureHPa']
        alt = row['altitudeM']
        f.write(f'    {{{t}, {ax}f, {ay}f, {az}f, {gx}f, {gy}f, {gz}f, {p}f, {alt}f}},\n')
    f.write('};\n')

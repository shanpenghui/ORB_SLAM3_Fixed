# coding=gbk
import numpy as np
import csv
import os

with open('f_dataset-room4_512_mono.txt', 'r') as f:
    csv_data = np.loadtxt(f, str, delimiter = " ")
    # print(data)

[rows, cols] = csv_data.shape
# print(rows)
print("cols = ", cols)

csv_data_float = csv_data.astype(np.float)

for i in range(rows-1):
    for j in range(cols-1):
        if(j==0):
            csv_data_float[i][j] = csv_data_float[i][j]/1e9
        # print(csv_data_float[i][j])

# Have to calculate manually for fixing recycle bug
csv_data_float[rows-1][0] = csv_data_float[rows-1][0]/1e9

csvfile_write = open('f_dataset-room4_512_mono_calib.csv', 'w')

print("Writing data into file, wait...")

for i in csv_data_float:
    # print(i)
    writer = csv.writer(csvfile_write, delimiter=' ')
    writer.writerow(i)

csvfile_write.close()


    # TUM
    # writer.writerow([row['#timestamp [ns]']*10e9 + ' ' +
    #                  row[' p_RS_R_x [m]']+ ' '+
    #                  row[' p_RS_R_y [m]']+ ' '+
    #                  row[' p_RS_R_z [m]']+ ' '+
    #                  row[' q_RS_x []']+ ' '+
    #                  row[' q_RS_y []']+ ' '+
    #                  row[' q_RS_z []']+ ' '+
    #                  row[' q_RS_w []']
    #                  ],)

    # EuRoc
    # writer.writerow([timestamp_calib + ' ' +
    #                  row[' p_RS_R_x [m]']+ ' '+
    #                  row[' p_RS_R_y [m]']+ ' '+
    #                  row[' p_RS_R_z [m]']+ ' '+
    #                  row[' q_RS_w []']+ ' '+
    #                  row[' q_RS_x []']+ ' '+
    #                  row[' q_RS_y []']+ ' '+
    #                  row[' q_RS_z []']+ ' '+
    #                  row[' v_RS_R_x [m s^-1]']+ ' '+
    #                  row[' v_RS_R_y [m s^-1]']+ ' '+
    #                  row[' v_RS_R_z [m s^-1]']+ ' '+
    #                  row[' b_w_RS_S_x [rad s^-1]']+ ' '+
    #                  row[' b_w_RS_S_y [rad s^-1]']+ ' '+
    #                  row[' b_w_RS_S_z [rad s^-1]']+ ' '+
    #                  row[' b_a_RS_S_x [m s^-2]']+ ' '+
    #                  row[' b_a_RS_S_y [m s^-2]']+ ' '+
    #                  row[' b_a_RS_S_z [m s^-2]']
    #                  ],)

#     print(row)
# csvfile.close()



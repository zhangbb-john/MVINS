import rosbag
import sys
import csv
import time
import string
import os  # for file management make directory
import shutil  # for file management, copy file
import numpy as np
import pandas as pd
from numpy import *
import matplotlib.pyplot as plt
cols = ['file', 'distance [m]','RMSE(position) [m]', 'Final error [m]', 'Largest error [m]']

if __name__ == '__main__':
    root_dir = '/home/ubuntu/test_data'
    # root_dir = '/media/ubuntu/Bingbing-Data2/test_data'
    print('root dir is ', root_dir)
    print('\n')
    excel = root_dir+"/result.xlsx"
    writer = pd.ExcelWriter(excel, engine='xlsxwriter')
    
    for root,dirs,files in os.walk(root_dir):
        data = np.zeros((len(files) + 1, len(cols)), dtype='U256')  
  
        files.sort()
        file_index = 0
        for file in files:  
            print('There are ', len(files), ' in total; This is No. ', file_index + 1, 'file')  
            if not file.endswith('ba', -3, -1):
                continue
            
            data[file_index, 0] = file
            t0 = time.clock()
            try:
                bag = rosbag.Bag(os.path.join(root,file))
                # bagContents = bag.read_messages()
                bag_name = bag.filename
                print(bag_name)
                # for topic, msg, t in bagContents:
                #     if topic not in listOfTopics:
                #         listOfTopics.append(topic)
                topic_odom = "/pose_graph/pose_graph_odom"
                # print(bag.read_messages("/Locater/Odom"))
                first_iter = True  # allows header row
                t_odom = []
                x_odom = []
                y_odom = []
                z_odom = []

                # for each instant in time that has data for topicName
                for subtopic, msg, t in bag.read_messages(topic_odom):
                    # parse data from this instant, which is of the form of multiple lines of "Name: value\n"
                    #	- put it in the form of a list of 2-element lists
                    msgString = str(msg)
                    # print("msg")
                    # print(msg)
                    msgList = string.split(msgString, '\n')
                    sec_pair = msgList[3]
                    sec_split = string.split(sec_pair, ':')
                    #print(["sec: ", float(sec_split[1]), sec_split[1]])

                    nsec_pair = msgList[4]
                    nsec_split = string.split(nsec_pair, ':')
                    #print(["nsec: ", float(nsec_split[1]), nsec_split[1]])
                    t = float(sec_split[1]) + float(nsec_split[1]) * 1e-9
                    #print(["t: ", t])
                    t_odom.append(t)

                    x_pair = msgList[10]
                    x_split = string.split(x_pair, ':')
                    #print(["x: ", float(x_split[1])])
                    x_odom.append(float(x_split[1]))

                    y_pair = msgList[11]
                    y_split = string.split(y_pair, ':')
                    #print(["y:", float(y_split[1])])
                    y_odom.append(float(y_split[1]))

                    z_pair = msgList[12]
                    z_split = string.split(z_pair, ':')
                    #print(["z:", float(z_split[1])])
                    z_odom.append(float(z_split[1]))

                print(["odom reading finished, time cost is ", time.clock() - t0])
                t0 = time.clock()
                topic_truth = "/gps_publisher/gps"
                first_iter = True  # allows header row
                t_truth = []
                x_truth = []
                y_truth = []
                z_truth = []

                # for each instant in time that has data for topicName
                for subtopic, msg, t in bag.read_messages(topic_truth):
                    # parse data from this instant, which is of the form of multiple lines of "Name: value\n"
                    #	- put it in the form of a list of 2-element lists
                    msgString = str(msg)
                    # print("msg")
                    # print(msg)
                    msgList = string.split(msgString, '\n')
                    sec_pair = msgList[3]
                    sec_split = string.split(sec_pair, ':')
                    #print(["sec: ", float(sec_split[1]), sec_split[1]])

                    nsec_pair = msgList[4]
                    nsec_split = string.split(nsec_pair, ':')
                    #print(["nsec: ", float(nsec_split[1]), nsec_split[1]])
                    t = float(sec_split[1]) + float(nsec_split[1]) * 1e-9
                    #print(["t: ", t])
                    t_truth.append(t)

                    x_pair = msgList[10]
                    x_split = string.split(x_pair, ':')
                    #print(["x: ", float(x_split[1])])
                    x_truth.append(float(x_split[1]))

                    y_pair = msgList[11]
                    y_split = string.split(y_pair, ':')
                    #print(["y:", float(y_split[1])])
                    y_truth.append(float(y_split[1]))

                    z_pair = msgList[12]
                    z_split = string.split(z_pair, ':')
                    # print(["z:", float(z_split[1])])
                    z_truth.append(float(z_split[1]))

                t_start = t_odom[0]
                t_odom_vec = np.array(t_odom) - t_start
                
                t_truth_vec = np.array(t_truth) - t_start

                error2 = []
                dist = 0
                last_ind = -1
                for i in range(len(t_odom)):
                    t = t_odom[i] - t_start
                    if (t > 1600):
                        break
                    diff = np.abs(t_truth_vec - t)
                    ind = np.argmin(np.abs(t_truth_vec - t))
                    # print(diff[ind])
                    if(diff[ind] < 1.0e-1):
                        err_x = (x_odom[i] - x_truth[ind]) ** 2
                        err_y = (y_odom[i] - y_truth[ind]) ** 2
                        err_z = (z_odom[i] - z_truth[ind]) ** 2
                        error2.append(err_x + err_y + err_z)
                        if (last_ind > -1):
                            step2 = (x_truth[ind] - x_truth[last_ind]) ** 2 + (y_truth[ind] -
                                                                            y_truth[last_ind]) ** 2 + (z_truth[ind] - z_truth[last_ind]) ** 2
                            # print(["step size is ", step2 ** 0.5])
                            dist = dist + step2 ** 0.5
                        last_ind = ind
                
                print(["calculation time is ", time.clock() - t0])
                print(['Total distance is ', dist])
                rmse = (np.sum(np.array(error2)) / len(error2)) ** 0.5
                print(["rmse = ", rmse])
                print(["final error = ", error2[-1] ** 0.5])
                print(["maximum error = ", np.array(error2).max() ** 0.5])
                print(["error percentage is ", (np.sum(np.array(error2)) / len(error2)) ** 0.5 / dist])
                data[file_index, 1] = dist
                data[file_index, 2] = rmse
                data[file_index, 3] = error2[-1] ** 0.5
                data[file_index, 4] = np.array(error2).max() ** 0.5
                
                file_index = file_index + 1
                fig1 = plt.figure(1)
                fig1_ax = fig1.add_subplot(111)
                scatter1 = fig1_ax.scatter(np.array(x_odom), np.array(y_odom), marker='.', color='r')
                scatter2 = fig1_ax.scatter(np.array(x_truth), np.array(y_truth), marker='.', color='b')
                fig1_ax.legend([scatter1, scatter2], ["navigation solution", "gps"], loc = 'best')
                fig2 = plt.figure(2)
                fig2_ax1 = fig2.add_subplot(211)
                scatter1 = fig2_ax1.scatter(t_odom_vec, np.array(x_odom), marker='.', color='r')
                scatter2 = fig2_ax1.scatter(t_truth_vec, np.array(x_truth), marker='.', color='b')
                fig2_ax2 = fig2.add_subplot(212)
                scatter1 = fig2_ax2.scatter(t_odom_vec, np.array(y_odom), marker='.', color='r')
                scatter2 = fig2_ax2.scatter(t_truth_vec, np.array(y_truth), marker='.', color='b')

                plt.draw()
                plt.pause(500)
                # print('before sleep 5')
                # time.sleep(5)
                plt.close('all')
            except:
                print('error!')
        break
    df = pd.DataFrame(data, columns = cols)
    df.to_excel(writer, 'Sheet1')
    writer.save()
    
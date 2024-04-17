def main():
#范围为前20后10的数据
    numbers =[83,122,1061,1277,1042,374,380	,243,352,347,131,109]
    sum_vehicle_vel_error = [32.77586958,11.66,607.4,126.95,177.62,22.21,22.11,53.16,116.15,79.82,9.84,10.75]
    sum_vehicle_vel_heading_error = [2284.09,572.28,	30564.98,14886.56,10934.43,	474.94,	684.45,	3992.27,5673.5,	4633.3,	763.81,	881.13]
    sum_vehicle_box_width_error = [	6.88,	17.38,	152.31,	118.43,	156.26,	46.32,	47.74,	25.13,	58.11,	29.97,	20.64,	15.56]
    sum_vehicle_box_length_error = [12.55,	12.3,	114.35,	139.08,	65.5,	38.28,	40.98,	22.06,	29.19,	30.82,	11.27,	7.97]
   #总体误差
    # numbers =[133,	409,	1061,	1277,	1042,	374	,380,	322	,364,	347	,131,	109]
    # sum_vehicle_vel_error = [37.70586958,	99.15,	607.4,	126.95,	177.62,	22.21,	22.11,	58.21,	121.46,	79.82,	9.84,	10.75]
    # sum_vehicle_vel_heading_error = [2421.69,	4653.2,	30564.98,	14886.56,	10934.43,	474.94,	684.45,	4725.11,	6099.9,	4633.3,	763.81,	881.13]
    # sum_vehicle_box_width_error = [21.52,	97.07,	152.31,	118.43,	156.26,	46.32,	47.74,	49.15,	64.42,	29.97,	20.64,	15.56]
    # sum_vehicle_box_length_error = [23.64,	96.09,	114.35,	139.08,	65.5,	38.28,	40.98,	44.35,	31.71,	30.82,	11.27,	7.97]

#范围为前10后5的数据
    # numbers = [50,35,550,395,173,166,105,89,74,54,42,446]
    # sum_vehicle_vel_error = [2.86,	1.67, 65.68, 114.81, 14.03,	7.74,	20.75,	22.05,	6.37,	17.56143678,	3.35,	207.27]
    # sum_vehicle_vel_heading_error = [51.59,	135.92, 8462.84, 2659.76, 469.03,	257.74,	577.97,	1432.2,	1014.58,	391.21,	43.59,	9863.72]
    # sum_vehicle_box_width_error = [1.97,	5.68, 47.5, 43.74, 22.91,	23.73,	12.42,	13.99,	9.26,	12.84,	4.26,	54.53]
    # sum_vehicle_box_length_error = [1.59,	1.03, 29.41, 12.78, 12.05,	13.41,	5.82,	6.28,	5.69,	5.75,	1.97,	33.17]

    temp_vel_err = []
    temp_vel_heading_err = []
    temp_width_error =  []
    temp_length_error = []
    whole_snumbers = 0
    whole_vel_error = 0
    whole_angle_error = 0
    whole_width_error = 0
    whole_length_error = 0
    for index,data in enumerate( numbers):
        whole_snumbers += data
        whole_vel_error += sum_vehicle_vel_error[index]
        whole_angle_error += sum_vehicle_vel_heading_error[index]
        whole_width_error += sum_vehicle_box_width_error[index] 
        whole_length_error += sum_vehicle_box_length_error[index]

        temp_vel_err.append(round(sum_vehicle_vel_error[index] / data,2))
        temp_vel_heading_err.append(round(sum_vehicle_vel_heading_error[index] / data ,2))
        temp_width_error.append(round(sum_vehicle_box_width_error[index] / data ,2 ) )
        temp_length_error.append(round(sum_vehicle_box_length_error[index] / data ,2))


    sum_vel_err = round(whole_vel_error / whole_snumbers,2)
    sum_angle_err = round(whole_angle_error / whole_snumbers,2)
    sum_width_err = round(whole_width_error / whole_snumbers,2)
    sum_length_err = round(whole_length_error / whole_snumbers,2)

    print("VALUE :",temp_vel_err)
    print("VALUE :",temp_vel_heading_err)
    print("VALUE :",temp_width_error)
    print("VALUE :",temp_length_error)
    print("------sum_error--------")
    print("sum_vel_err",sum_vel_err)
    print("sum_angle_err",sum_angle_err)
    print("sum_width_err",sum_width_err)
    print("sum_length_err",sum_length_err)
    
    
if __name__ == '__main__':
    main()
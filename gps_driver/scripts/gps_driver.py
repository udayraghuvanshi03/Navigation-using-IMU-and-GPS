#!/usr/bin/python3

import rospy
import utm
import serial
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from numpy import float64
from gps_driver.msg import gpsdata
global latitude

if __name__ == '__main__':
    rospy.init_node('gps',anonymous=True)
    serial_port=rospy.get_param('~port','/dev/ttyUSB0') #defining port name
    serial_baudrate=rospy.get_param('~baudrate',4800) #defining baudrate
    port= serial.Serial(serial_port,serial_baudrate,timeout=None)
    #rospy.logdebug("Using gps sensor on port "+serial_port+" at "+str(serial_baudrate))
    gps_pub=rospy.Publisher('gpstopic',gpsdata,queue_size=20)
    
    #rospy.logdebug("Initialization complete")
    msg= gpsdata()
    
    try:
        while not rospy.is_shutdown():
            line=port.readline().decode('utf-8')

            if line == '':
                rospy.logwarn("No signal from sensor") # when gps gives no signal
            else:
                if line.startswith('$GPGGA'):
                    line=line.split(",")
                    time=line[1]
                    latitude=line[2]
                    longitude=line[4]
                    longitude_str=line[4]
                    altitude=line[9]
                    lat_indicator=line[3]
                    long_indicator=line[5]
                
                    #print(time)
                    #print(latitude)
                    if latitude== '':
                        rospy.logwarn('No data') #when gps gives empty signals...e.g when we are inside a closed room
                        continue
                    else:
                        latitude=float64(latitude)
                        longitude=float64(longitude)
                        altitude=float64(altitude)

                        lat_conv=((latitude/100)%1)/60 #converting latitude to degrees
                        lat_deg=int(latitude/100)
                        lat_min=lat_conv*100
                        lat_final=lat_deg+lat_min

                        if longitude_str[0]=='0': #converting longitude to degrees
                            long_conv=((longitude/100)%1)/60
                            long_deg=int(longitude/100)
                            long_min=long_conv*100
                            long_final=long_deg+long_min
                    

                        if long_indicator=='W':
                            long_final= -long_final
                    
                        if lat_indicator=='S':
                            lat_indicator=-lat_indicator

                    
                            #print(long_final)
                            # print(lat_final)
                            # print(altitude)

                        val=utm.from_latlon(lat_final,long_final) #converting to utm
                        utm_e=val[0]
                        utm_n=val[1]
                        zone_num=val[2]
                        zone_letter=val[3]
                        # print(utm_e)
                        # print(utm_n)
                        # print(zone_num)
                        # print(zone_letter)
                        # t=rospy.Time.now()
                        # msg.header.stamp = t

                        #assigning values to msg file
                        msg.latitude.data=lat_final
                        msg.longitude.data=long_final
                        msg.altitude.data=altitude
                        msg.utm_easting.data=utm_e
                        msg.utm_northing.data=utm_n
                        msg.zone_num.data=zone_num
                        msg.zone_letter.data=zone_letter
                        

                    rospy.loginfo(msg)
                    gps_pub.publish(msg)        
                    
    except rospy.ROSInterruptException:
        pass 
import global_params as gp
import time

def send_mesg(mesg):
    ack_received = False
    while not ack_received:
        print ("Python value sent: ")
        print (mesg)
        #uncomment below line to send to arduino
        
        gp.arduino.flush()
        gp.arduino.write(mesg.encode())
        time.sleep(3)
        #time.sleep(1)

        #check for ack from robot4\
        #uncomment below line to receieve from arduino
        ack_msg = gp.arduino.read(gp.arduino.inWaiting()) # read all characters in buffer
        #ack_msg = "done".encode() # remove this line and uncomment the previous line

        if (len(ack_msg)>0):
            print ("Message from arduino: ")
            print (ack_msg.decode())
            ack_received=True # Robot has moved
    return
    
if __name__ == '__main__':
    print(__file__ + " start!!")

    mesg='mr1mx00o'
    send_mesg(mesg)
    #mesg='mr1mx00c'
    #send_mesg(mesg)
##    mesg='mr1ml20x'
##    send_mesg(mesg)
##    mesg='mr1mf00x'
##    send_mesg(mesg)
##    mesg='mr1ml20x'
##    send_mesg(mesg)
    
    gp.arduino.close()    
    print(__file__ + " Done!!")

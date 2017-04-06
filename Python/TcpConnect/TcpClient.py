import socket
import threading
import time
from TcpConnect import TcpConnect

class TcpClient(object):
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connection = False
        while connection is False:
            try:
                self.sock.connect((self.host, self.port))
                connection = True
            except:
                print "Server closed"
                time.sleep(10)
        self.connect = TcpConnect("client")
        self.connect.setSocket(self.sock)
        self.connect.setHandler(self)
        threading.Thread(target = self.connect.listenToSocket).start()

    def sendToServer(self, msg):
        """Send data to Server"""
        self.connect.writeToSocket(msg)

    def sendMsgToServer(self, msg=""):
        """Send Message to other raspberry"""
        data = "Message:" + msg
        sendToServer(data)

    def sendErrorToServer(self, course="left"):
        """Send Start to other raspberry with left or right"""
        data = "Error:" + course
        sendToServer(data)

    def recvMsg(self, data):
        print "Client recived a Message: " + data
        
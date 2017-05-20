import socket
import threading
from TcpConnect import TcpConnect

class TcpServer(object):
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.connect = TcpConnect("server")

    def sendToClient(self, data):
        """Send data to Client"""
        self.connect.writeToSocket(data)

    def sendMsgToClient(self, msg=""):
        """Send Message to other raspberry"""
        data = "Message:" + msg
        sendToClient(data)

    def sendStartToClient(self, course="left"):
        """Send Start to other raspberry with left or right"""
        data = "Start:" + course
        sendToClient(data)

    def sendNumberToClient(self, number=1):
        """Send Number to other raspberry"""
        data = "Number:" + number
        sendToClient(data)

    def recvMsg(self, data):
        print "Server recived a Message: " + data

    def listen(self):
        self.sock.listen(5)
        while True:
            client, address = self.sock.accept()
            print "Server: Get connection"
            #client.settimeout(60)
            self.connect.setAddress(address)
            self.connect.setSocket(client) 
            self.connect.setHandler(self)
            threading.Thread(target = self.connect.listenToSocket).start()
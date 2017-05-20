import socket
import threading
import time
import TcpConnect
import logging

log = logging.getLogger(__name__)

class TcpClient(object):
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.queue = []
        connection = False
        while connection is False:
            try:
                self.sock.connect((self.host, self.port))
                connection = True
            except:
                print "Server closed"
                time.sleep(10)
        self.connect = TcpConnect.TcpConnect("client")
        self.connect.setSocket(self.sock)
        self.connect.setHandler(self)
        threading.Thread(target = self.connect.listenToSocket).start()

    def sendToServer(self, msg):
        """Send data to Server"""
        self.connect.writeToSocket(msg)

    def sendMsgToServer(self, msg=""):
        """Send Message to other raspberry"""
        data = "Message:" + msg
        self.sendToServer(data)

    def sendErrorToServer(self, course="left"):
        """Send Start to other raspberry with left or right"""
        data = "Error:" + course
        self.sendToServer(data)

    def data_available(self):
        return len(self.queue)>0

    def recvMsg(self, data):
        self.queue.append(data)
        log.info("Client received a Message: " + data)
        
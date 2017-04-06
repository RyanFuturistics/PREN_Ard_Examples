class TcpConnect(object):
    def __init__(self, name):
        self.socket = None
        self.address = None
        self.name = name
        self.handler = None

    def setSocket(self, socket):
        self.socket = socket
    
    def setAddress(self, address):
        self.address = address

    def setHandler(self, handler):
        self.handler = handler

    def listenToSocket(self):
        size = 1024
        while True:
            try:
                data = self.socket.recv(size)
                if data:
                    if data != "OK":
                        response = "OK" #data
                        if self.handler != None:
                            self.handler.recvMsg(data)
                        else:
                            print self.name + ": " + data
                        self.socket.send(response)
                else:
                    raise error(self.name + 'Client disconnected')
            except:
                self.socket.close()
                return False

    def writeToSocket(self, msg):
        try:
            self.socket.send(msg)
        except:
            if socket != None:
                self.socket.close()
            return False
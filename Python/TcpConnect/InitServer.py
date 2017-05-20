import threading
from TcpServer import TcpServer
import sys, traceback

class InitServer(object):
    def run(self):
        try:
            server = TcpServer("localhost", 4444)
            threading.Thread(target = server.listen).start()
            while 1:
                raw_input("Press Enter to continue...\n")
                server.sendToClient("Server: send to Client")
        except KeyboardInterrupt:
            print "Shutdown requested...exiting\n"
        except Exception:
            traceback.print_exc(file=sys.stdout)
        sys.exit(0)

if __name__ == "__main__":
    InitServer().run()
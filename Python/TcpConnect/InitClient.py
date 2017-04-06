import threading
from TcpClient import TcpClient
import sys, traceback

class InitClient(object):
    def run(self):
        try:
            client = TcpClient("localhost", 4444)
            while 1:
                raw_input("Press Enter to continue...\n")
                client.sendToServer("Client - send to Server")
        except KeyboardInterrupt:
            print "Shutdown requested...exiting\n"
        except Exception:
            traceback.print_exc(file=sys.stdout)
            raw_input("Press Enter to continue...\n")
        sys.exit(0)

if __name__ == "__main__":
    InitClient().run()
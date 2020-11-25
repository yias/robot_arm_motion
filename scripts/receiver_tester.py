
import argparse
import numpy as np
from socketStream_py import socketStream


def main(args):
    sockClient = socketStream.socketStream(
        svrIP=args.host, svrPort=args.port, socketStreamMode=0)

    # sockClient.setBufferSize(args.buffersize)

    sockClient.set_clientName("py_listener_example")

    # sockClient.initialize_msgStruct(["name","data", "dataInt"])

    # sockClient.updateMSG("name","Mary")

    # sockClient.updateMSG("data", np.array([[2.4,5.6,783.01],[34,55.6,1.2]]))

    # sockClient.updateMSG("dataInt", np.array([720, 1280, 3]))

    everything_ok = False
    if sockClient.initialize_socketStream() == 0:
        if sockClient.make_connection() == 0:
            everything_ok = True

    if everything_ok:

        while(True):
            try:
                if sockClient.socketStream_ok():
                    msg = sockClient.get_latest()
                    print(msg)
                    if msg is not None:
                        msg_data = msg["data"]
                        rt = np.array(msg_data, dtype=np.float32)
                        print(rt.shape)
                        # print(rt)
                    # sockClient.sendMsg()
                    # time.sleep(0.5)
            except KeyboardInterrupt:
                break

    sockClient.closeCommunication()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='TCP server for receiving inputs from a client with socketStream')
    parser.add_argument('--host', type=str,
                        help='the IP of the server', default='128.178.145.15')
    parser.add_argument(
        '--port', type=int, help='the port on which the server is listening', default=10352)
    parser.add_argument('--buffersize', type=int,
                        help='the size of the buffer for pakets receiving', default=64)
    args = parser.parse_args()
    main(args)

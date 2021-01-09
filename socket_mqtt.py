import socketserver
import signal
import time
import sys
import paho.mqtt.client as mqtt

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))


class MyTCPHandler(socketserver.BaseRequestHandler):
    """
    The RequestHandler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        self.data = ""
        # self.request is the TCP socket connected to the client
        while True:
            self.data = self.request.recv(2048).strip().decode("utf-8")
            if "}{" in self.data:
                self.data = self.data.replace("}{", "}|{")
            data = self.data.split("|")
            for j in data:
                if "{" not in j or "}" not in j:
                    print(j)
                    continue
                client.publish(topic="socket_mqtt", payload=j)


if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 60000

    def signal_handler(sig, frame):
        print('Exiting...')
        server.server_close()
        client.loop_stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("localhost", 1883, 60)
    client.loop_start()

    # Create the server, binding to localhost on port 9999
    server = socketserver.TCPServer((HOST, PORT), MyTCPHandler)

    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    server.serve_forever()
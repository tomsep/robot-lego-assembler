from __future__ import division
import threading
import json
from legoassembler.communication import Server, Client, URServer


class TestServerClientJSON:
    """ Test communication by sending JSON

    """

    def _client_send_and_recv_json_list(self, obj, ip, port):
        """ Send 'obj' as JSON and expect it back

        """

        client = Client()
        client.connect(ip, port)

        client.send(json.dumps(obj).encode())

        recv_obj = client.recv()
        recv_obj = json.loads(recv_obj.decode('utf-8'))

        assert obj == recv_obj

        client.close()

    def _echo_json_server(self, ip, port):
        """ Start echo server for sending back received data

        """

        serv = Server(ip, port)
        serv.accept()
        data = serv.recv()
        serv.send(data)
        serv.close()

    def test_sending_and_receiving_json(self):

        ip = 'localhost'
        port = 8000

        test_list = [10, "ok"]

        t = threading.Thread(target=self._echo_json_server, args=(ip, port))
        t.setDaemon(True)
        t.start()

        self._client_send_and_recv_json_list(test_list, ip, port)


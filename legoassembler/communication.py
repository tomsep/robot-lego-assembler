from __future__ import division
import socket
import io
import struct


class Server:

    socket = None
    connection = None
    client_addr = None

    def __init__(self, ip, port):
        self.socket = socket.socket()

        # Allow addres reuse
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.socket.bind((ip, port))
        self.socket.listen(0)

    def __del__(self):
        """ Destructor
        """
        self.close()

    def accept(self, print_info=True):
        """ Accept connection

        Parameters
        ----------
        print_info : bool
            If info about connections accepted should be printed.

        """

        if self.connection:
            self.connection.close()
        self.connection, self.client_addr = self.socket.accept()
        if print_info:
            print('Accepted connection with ' + self.client_addr[0] + ':'
                  + str(self.client_addr[1]))

    def close(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def send(self, data):
        _send(data, self.connection)

    def recv(self):
        return _recv(self.connection)


class Client:

    socket = None

    def __del__(self):
        """ Destructor
        """
        self.close()

    def connect(self, ip, port):
        self.socket = socket.socket()
        self.socket.settimeout(9)
        self.socket.connect((ip, port))

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def send(self, data):
        _send(data, self.socket)

    def recv(self):
        return _recv(self.socket)


class URServer(Server):
    """ Server for messaging with UR controller

    Uses '\n' as EOL char for received messages.
    Sent messages are strings encoded as is.
    """

    def send(self, data):
        if type(data) == str:
            self.connection.sendall(data.encode())
        elif type(data) == bytes:
            self.connection.sendall(data)
        else:
            raise TypeError('Data must be str or bytes, not {}'.format(type(data)))

    def recv(self, header=None):
        """ Receive data

        Parameters
        ----------
        header : str
            Optionally require header 'header' to be received.

        Returns
        -------
        data as string (coding utf-8)

        Raises
        ------
        ValueError
            If received data does not match 'header' when not None.

        """

        data = b''
        char = self.connection.recv(1)
        while char != b'\n':
            data += char
            char = self.connection.recv(1)

        data = data.decode('utf-8')
        if header:
            if data != header:
                raise ValueError('Expected header "{}" got "{}"'.format(header, data))
        return data


class URClient(Client):
    """ Implements client for sending scrip commands to UR controller

    """
    def send(self, data):
        if type(data) == str:
            data = data.encode()
        self.socket.sendall(data)

    def recv(self):
        raise NotImplementedError('URServer does not implement "recv"')


def _send(data, conn):
    """ Send bytes data over connection

    Sends the size of the data before the data itself.
    Data length is 4 bytes unsigned long integer.

    Parameters
    ----------
    data : bytes
    conn : socket.socket

    """

    conn = conn.makefile('wb')
    stream = io.BytesIO()

    #if type(data) is str:
    #    data = data.encode()
    stream.write(data)

    # Send size
    conn.write(struct.pack('<L', stream.tell()))
    conn.flush()

    # Send data
    stream.seek(0)
    conn.write(stream.read())
    stream.seek(0)
    stream.truncate()


def _recv(conn):
    """ Reads bytes data from socket connection

    Expects to receive the size of the data before the data itself.
    Data length is 4 bytes unsigned long integer.

    Parameters
    ----------
    conn : socket.socket

    Returns
    -------
    bytes
        Data received read from the socket.
    """

    conn = conn.makefile('rb')

    # Read incoming data size
    struct_size = struct.calcsize('<L')
    data_len = struct.unpack('<L', conn.read(struct_size))[0]

    # Write data from connection to stream
    stream = io.BytesIO()
    stream.write(conn.read(data_len))

    stream.seek(0)
    return stream.read()


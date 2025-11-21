import socket, time, json

PI_IP = "192.168.0.2"  # The server's hostname or IP address
PORT = 2009  # The port used by the server

def connect_with_retry(ip, port, retries=10, delay=1.0):
    for i in range(retries):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((ip, port))
            s.settimeout(None)
            return s
        except OSError as e:
            print(f"[PC] Connexion échouée ({e}), tentative {i+1}/{retries}")
            time.sleep(delay)
    raise RuntimeError("Impossible de se connecter à la Pi")

def send_json(sock, obj):
    line = (json.dumps(obj) + "\n").encode("utf-8")
    sock.sendall(line)

def recv_json(sock):
    # lecture ligne par ligne
    with sock.makefile("rb") as f:
        raw = f.readline()
        if not raw:
            return None
        return json.loads(raw.decode("utf-8").strip())

if __name__ == "__main__":
    sock = connect_with_retry(PI_IP, PORT)
    print("[PC] Connecté à la Pi.")
    # 1) ping
    send_json(sock, {"cmd": "ping"})
    print("[PC] ping ->", recv_json(sock))

    # 2) calcul
    send_json(sock, {"cmd": "sum", "a": 12, "b": 30.5})
    print("[PC] sum ->", recv_json(sock))

    sock.close()

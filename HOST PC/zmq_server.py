import zmq

# ZMQ 服务器：ROUTER 接收客户端消息，PUB 转发给订阅者，并回复 ACK

def main():
    context = zmq.Context()

    # ROUTER socket，接收来自 TransmissionClient 的消息
    router = context.socket(zmq.ROUTER)
    router.bind("tcp://*:5555")  # 监听端口5555
    print("[Server] ROUTER bound to tcp://*:5555")

    # PUB socket，转发接收到的消息给所有订阅者
    pub = context.socket(zmq.PUB)
    pub.bind("tcp://*:5556")  # 发布端口5556
    print("[Server] PUB bound to tcp://*:5556")

    try:
        while True:
            # 接收三段式消息：客户端标识, 空帧, 原始消息
            ident, empty, payload = router.recv_multipart()
            print(f"[Server] Received from {ident!r}: {payload!r}")

            # 转发给所有订阅者
            pub.send(payload)
            print(f"[Server] Forwarded payload: {payload!r}")

            # 回复 ACK 给客户端
            router.send_multipart([ident, b"", b"ack"])
    except KeyboardInterrupt:
        print("[Server] Interrupted, shutting down...")
    finally:
        router.close()
        pub.close()
        context.term()

if __name__ == '__main__':
    main()

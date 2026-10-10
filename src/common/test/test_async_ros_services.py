"""Controlled transport proof, independent of a station and ROS master.

Run with ASYNC_ROS_SERVICES_DRIVER=<compiled driver> python3 this-file.py.
The fixture withholds every native response until all twenty requests arrive.
"""
import asyncio
import os
import struct
import unittest
import xmlrpc.client


def frame(value):
    return struct.pack('<I', len(value)) + value


async def read_frame(reader):
    size = struct.unpack('<I', await reader.readexactly(4))[0]
    if size > 1024 * 1024:
        raise ValueError('oversize frame')
    return await reader.readexactly(size)


class TransportTest(unittest.IsolatedAsyncioTestCase):
    async def run_case(self, *, count=20, no_response=False, bad_header=False, stalled_master=False):
        received = []
        released = asyncio.Event()
        peers = []
        tasks = set()

        async def native(reader, writer):
            peers.append(writer)
            header = await read_frame(reader)
            self.assertIn(b'service=/robot', header)
            self.assertIn(b'md5sum=e09abbb4e5bae6b558e501096e7eb71e', header)
            md5 = b'bad' if bad_header else b'e09abbb4e5bae6b558e501096e7eb71e'
            writer.write(frame(frame(b'md5sum=' + md5) + frame(b'type=mavros_msgs/CommandBool')))
            await writer.drain()
            if bad_header:
                self.assertEqual(await reader.read(), b'')
                return
            self.assertEqual(await read_frame(reader), b'\1')
            received.append(writer)
            if len(received) == count:
                released.set()
            await released.wait()
            # The first vehicle deliberately never answers. Others complete.
            if no_response and writer is received[0]:
                self.assertEqual(await reader.read(), b'')
                return
            writer.write(b'\1' + frame(b'\1\0'))
            await writer.drain()
            await reader.read()

        async def master(reader, writer):
            peers.append(writer)
            header = await reader.readuntil(b'\r\n\r\n')
            length = int(next(line.split(b':')[1] for line in header.split(b'\r\n')
                              if line.lower().startswith(b'content-length:')))
            body = await reader.readexactly(length)
            params, method = xmlrpc.client.loads(body)
            self.assertEqual(method, 'lookupService')
            self.assertEqual(params[0], '/async_transport_test')
            if stalled_master:
                self.assertEqual(await reader.read(), b'')
                return
            response = xmlrpc.client.dumps(([1, '', f'rosrpc://localhost:{native_port}'],),
                                          methodresponse=True).encode()
            writer.write(b'HTTP/1.1 200 OK\r\nContent-Length: ' + str(len(response)).encode()
                         + b'\r\nConnection: close\r\n\r\n' + response)
            await writer.drain()
            writer.close()

        def track(callback):
            def accept(reader, writer):
                task = asyncio.create_task(callback(reader, writer))
                tasks.add(task)
                return task
            return accept

        async with await asyncio.start_server(track(native), '127.0.0.1', 0) as ns:
            native_port = ns.sockets[0].getsockname()[1]
            async with await asyncio.start_server(track(master), '127.0.0.1', 0) as ms:
                master_port = ms.sockets[0].getsockname()[1]
                process = await asyncio.create_subprocess_exec(os.environ['ASYNC_ROS_SERVICES_DRIVER'],
                    f'http://localhost:{master_port}', str(count), '600',
                    stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                stdout, stderr = await asyncio.wait_for(process.communicate(), 3)
                self.assertEqual(process.returncode, 0, stderr.decode())
                for writer in peers:
                    writer.close()
                if tasks:
                    await asyncio.wait_for(asyncio.gather(*tasks), 1)
                return [tuple(map(int, line.split())) for line in stdout.splitlines()], received

    async def test_all_twenty_arrive_before_any_response(self):
        results, requests = await self.run_case()
        self.assertEqual(len(requests), 20)
        self.assertEqual(len(results), 20)
        self.assertTrue(all(outcome == 0 and size == 2 for _, outcome, size in results))

    async def test_one_unknown_does_not_hold_other_nineteen(self):
        results, requests = await self.run_case(no_response=True)
        self.assertEqual(len(requests), 20)
        self.assertEqual(sum(outcome == 0 for _, outcome, _ in results), 19)
        self.assertEqual(results[-1][1:], (4, 0))

    async def test_type_md5_mismatch_never_sends_request(self):
        results, requests = await self.run_case(count=1, bad_header=True)
        self.assertEqual(requests, [])
        self.assertEqual(results[0][1:], (1, 0))

    async def test_discovery_deadline_closes_real_io(self):
        results, requests = await self.run_case(count=1, stalled_master=True)
        self.assertEqual(requests, [])
        self.assertEqual(results[0][1:], (2, 0))

    async def test_dns_deadline_and_cancel_release_resolver_io(self):
        class Sink(asyncio.DatagramProtocol):
            def __init__(self): self.received = 0
            def datagram_received(self, data, peer): self.received += 1
        loop = asyncio.get_running_loop()
        transport, sink = await loop.create_datagram_endpoint(Sink, local_addr=('127.0.0.1', 0))
        port = transport.get_extra_info('sockname')[1]
        try:
            for cancel_ms, outcome in ((None, 2), ('50', 3)):
                args = [os.environ['ASYNC_ROS_SERVICES_DRIVER'], 'http://never-answer.robot-test.invalid:11311', '1', '180', f'127.0.0.1:{port}']
                if cancel_ms is not None: args.append(cancel_ms)
                process = await asyncio.create_subprocess_exec(*args, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                stdout, stderr = await asyncio.wait_for(process.communicate(), 1)
                self.assertEqual(process.returncode, 0, stderr.decode())
                self.assertEqual(tuple(map(int, stdout.split())), (0, outcome, 0))
                count = sink.received
                await asyncio.sleep(0.2)
                self.assertEqual(sink.received, count, 'DNS operation remained after completion')
            self.assertGreaterEqual(sink.received, 2)
        finally:
            transport.close()

    async def test_master_connection_reuse_close_and_transport_failure(self):
        connections, requests, tasks = [], [], []

        async def master(reader, writer):
            connections.append(writer)
            try:
                while True:
                    header = await reader.readuntil(b'\r\n\r\n')
                    length = int(next(line.split(b':')[1] for line in header.split(b'\r\n')
                                      if line.lower().startswith(b'content-length:')))
                    requests.append(await reader.readexactly(length))
                    self.assertIn(b'getParam', requests[-1])
                    if len(requests) == 3:
                        break  # Lost response must fail this call, not replay it.
                    response = xmlrpc.client.dumps(([0, 'missing', 0],), methodresponse=True).encode()
                    closing = len(requests) == 4
                    writer.write(b'HTTP/1.1 200 OK\r\nContent-Length: ' + str(len(response)).encode()
                                 + b'\r\nConnection: ' + (b'close' if closing else b'keep-alive') + b'\r\n\r\n' + response)
                    await writer.drain()
                    if closing:
                        break
            except asyncio.IncompleteReadError:
                pass
            finally:
                writer.close()

        def accept(reader, writer):
            tasks.append(asyncio.create_task(master(reader, writer)))

        async with await asyncio.start_server(accept, '127.0.0.1', 0) as server:
            port = server.sockets[0].getsockname()[1]
            process = await asyncio.create_subprocess_exec(os.environ['ASYNC_ROS_SERVICES_DRIVER'],
                f'http://127.0.0.1:{port}', '5', '600', '', '0', 'master',
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
            stdout, stderr = await asyncio.wait_for(process.communicate(), 3)
            self.assertEqual(process.returncode, 0, stderr.decode())
            await asyncio.wait_for(asyncio.gather(*tasks), 1)
        self.assertEqual([int(line.split()[1]) for line in stdout.splitlines()], [0, 0, 1, 0, 0])
        self.assertEqual(len(requests), 5)
        self.assertEqual(len(connections), 3)


if __name__ == '__main__':
    unittest.main()

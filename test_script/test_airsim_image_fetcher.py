import importlib.util
from pathlib import Path
import threading
import time
import unittest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "CarlaAir_Release"
    / "source"
    / "python_api"
    / "examples"
    / "airsim_image_fetcher.py"
)
SPEC = importlib.util.spec_from_file_location("airsim_image_fetcher", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

AirSimImageFetcher = MODULE.AirSimImageFetcher
AirSimImageTimeoutError = MODULE.AirSimImageTimeoutError


class FakeSettings:
    def __init__(self, synchronous_mode=True):
        self.synchronous_mode = synchronous_mode


class FakeWorld:
    def __init__(self, synchronous_mode=True, release_event=None, release_tick=1):
        self.settings = FakeSettings(synchronous_mode)
        self.release_event = release_event
        self.release_tick = release_tick
        self.ticks = 0

    def get_settings(self):
        return self.settings

    def tick(self, timeout):
        self.ticks += 1
        if self.release_event is not None and self.ticks >= self.release_tick:
            self.release_event.set()
        time.sleep(min(timeout, 0.001))
        return self.ticks


class WaitingClient:
    def __init__(self, event, result=None, wait_seconds=1.0):
        self.event = event
        self.result = result if result is not None else ["image"]
        self.wait_seconds = wait_seconds

    def simGetImages(self, requests):
        self.event.wait(self.wait_seconds)
        return self.result


class RaisingClient:
    def simGetImages(self, requests):
        raise ValueError("rpc failed")


class TimingOutClient:
    def simGetImages(self, requests):
        raise TimeoutError("rpc timed out")


class AirSimImageFetcherTests(unittest.TestCase):
    def test_request_completes_while_caller_ticks(self):
        event = threading.Event()
        world = FakeWorld(release_event=event, release_tick=2)
        fetcher = AirSimImageFetcher(lambda: WaitingClient(event))

        try:
            future = fetcher.request(["scene"])
            result = fetcher.wait_while_ticking(world, future, timeout=1.0)
        finally:
            fetcher.close()

        self.assertEqual(result, ["image"])
        self.assertGreaterEqual(world.ticks, 2)

    def test_rpc_exception_is_propagated(self):
        world = FakeWorld()
        fetcher = AirSimImageFetcher(RaisingClient)

        try:
            with self.assertRaisesRegex(ValueError, "rpc failed"):
                fetcher.get_images_while_ticking(world, ["scene"], timeout=1.0)
        finally:
            fetcher.close()

    def test_total_timeout_marks_fetcher_unusable(self):
        event = threading.Event()
        world = FakeWorld()
        fetcher = AirSimImageFetcher(
            lambda: WaitingClient(event, wait_seconds=0.05)
        )

        with self.assertRaises(AirSimImageTimeoutError):
            fetcher.get_images_while_ticking(world, ["scene"], timeout=0.01)
        with self.assertRaisesRegex(RuntimeError, "cannot be reused"):
            fetcher.request(["scene"])
        fetcher.close()

    def test_rpc_timeout_uses_public_timeout_error(self):
        world = FakeWorld()
        fetcher = AirSimImageFetcher(TimingOutClient)

        try:
            with self.assertRaises(AirSimImageTimeoutError):
                fetcher.get_images_while_ticking(world, ["scene"], timeout=1.0)
        finally:
            fetcher.close()

    def test_second_in_flight_request_is_rejected(self):
        event = threading.Event()
        fetcher = AirSimImageFetcher(lambda: WaitingClient(event))

        future = fetcher.request(["first"])
        try:
            with self.assertRaisesRegex(RuntimeError, "already in progress"):
                fetcher.request(["second"])
        finally:
            event.set()
            future.result(timeout=1.0)
            fetcher.close()

    def test_async_world_is_rejected_before_rpc_starts(self):
        factory_called = threading.Event()

        def factory():
            factory_called.set()
            return WaitingClient(threading.Event())

        world = FakeWorld(synchronous_mode=False)
        fetcher = AirSimImageFetcher(factory)
        try:
            with self.assertRaisesRegex(ValueError, "synchronous mode"):
                fetcher.get_images_while_ticking(world, ["scene"])
        finally:
            fetcher.close()

        self.assertFalse(factory_called.is_set())

    def test_close_is_idempotent_and_blocks_new_requests(self):
        fetcher = AirSimImageFetcher(lambda: WaitingClient(threading.Event()))
        fetcher.close()
        fetcher.close()

        with self.assertRaisesRegex(RuntimeError, "closed"):
            fetcher.request(["scene"])

    def test_close_does_not_wait_for_running_rpc(self):
        event = threading.Event()
        fetcher = AirSimImageFetcher(lambda: WaitingClient(event))
        future = fetcher.request(["scene"])

        started = time.monotonic()
        fetcher.close()
        elapsed = time.monotonic() - started
        event.set()
        future.result(timeout=1.0)

        self.assertLess(elapsed, 0.1)


if __name__ == "__main__":
    unittest.main()

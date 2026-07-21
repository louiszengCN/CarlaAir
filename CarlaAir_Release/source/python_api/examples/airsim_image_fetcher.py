"""AirSim image capture helper for a CARLA synchronous world.

AirSim's ``simGetImages`` RPC waits for an Unreal render frame. In CARLA
synchronous mode that frame is only produced when the tick-owning thread calls
``world.tick()``. This helper runs the image RPC on a dedicated client thread so
the caller can continue advancing the world while it waits.
"""

from concurrent.futures import Future, ThreadPoolExecutor
import threading
import time


class AirSimImageTimeoutError(TimeoutError):
    """Raised when a synchronous AirSim image request exceeds its deadline."""


class AirSimImageFetcher:
    """Fetch AirSim images while the caller remains the CARLA tick owner."""

    def __init__(self, client_factory):
        if not callable(client_factory):
            raise TypeError("client_factory must be callable")

        self._client_factory = client_factory
        self._client = None
        self._executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="airsim-image-rpc",
        )
        self._lock = threading.Lock()
        self._in_flight = None
        self._closed = False
        self._unusable = False

    @classmethod
    def for_multirotor(cls, host="127.0.0.1", port=41451, rpc_timeout=10.0):
        """Create a fetcher backed by its own bounded-time AirSim client."""
        if rpc_timeout <= 0:
            raise ValueError("rpc_timeout must be greater than zero")

        def create_client():
            import airsim

            return airsim.MultirotorClient(
                ip=host,
                port=port,
                timeout_value=rpc_timeout,
            )

        return cls(create_client)

    def _check_usable(self):
        if self._closed:
            raise RuntimeError("AirSimImageFetcher is closed")
        if self._unusable:
            raise RuntimeError(
                "AirSimImageFetcher cannot be reused after a timeout; "
                "create a new instance"
            )

    def _capture(self, requests):
        if self._client is None:
            self._client = self._client_factory()
        return self._client.simGetImages(requests)

    def _clear_completed(self, future):
        with self._lock:
            if self._in_flight is future:
                self._in_flight = None

    def request(self, requests):
        """Start one image RPC and return its ``Future``."""
        request_list = list(requests)
        if not request_list:
            raise ValueError("requests must not be empty")

        with self._lock:
            self._check_usable()
            if self._in_flight is not None and not self._in_flight.done():
                raise RuntimeError("an AirSim image request is already in progress")

            future = self._executor.submit(self._capture, request_list)
            self._in_flight = future

        future.add_done_callback(self._clear_completed)
        return future

    def _mark_unusable(self, future):
        with self._lock:
            self._unusable = True
        future.cancel()

    def wait_while_ticking(self, world, future, timeout=10.0):
        """Advance a synchronous CARLA world until an image RPC completes."""
        if not isinstance(future, Future):
            raise TypeError("future must be returned by request()")
        if timeout <= 0:
            raise ValueError("timeout must be greater than zero")
        if not world.get_settings().synchronous_mode:
            self._mark_unusable(future)
            raise ValueError("CARLA world must be in synchronous mode")

        deadline = time.monotonic() + timeout
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._mark_unusable(future)
                raise AirSimImageTimeoutError(
                    "AirSim simGetImages timed out while advancing the CARLA world"
                )

            try:
                world.tick(remaining)
            except RuntimeError as exc:
                if future.done():
                    break
                self._mark_unusable(future)
                raise AirSimImageTimeoutError(
                    "CARLA world.tick timed out while waiting for AirSim images"
                ) from exc

        try:
            return future.result()
        except Exception as exc:
            if "timeout" in type(exc).__name__.lower():
                self._mark_unusable(future)
                raise AirSimImageTimeoutError(
                    "AirSim simGetImages RPC timed out"
                ) from exc
            raise

    def get_images_while_ticking(self, world, requests, timeout=10.0):
        """Start an image RPC and keep ticking until its response is ready."""
        if not world.get_settings().synchronous_mode:
            raise ValueError("CARLA world must be in synchronous mode")
        future = self.request(requests)
        return self.wait_while_ticking(world, future, timeout=timeout)

    def close(self):
        """Stop accepting work without waiting indefinitely for a stuck RPC."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            future = self._in_flight

        if future is not None:
            future.cancel()
        self._executor.shutdown(wait=False, cancel_futures=True)

    def __enter__(self):
        self._check_usable()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False

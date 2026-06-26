import os
import shutil
import subprocess
import tempfile
import time
from typing import Optional


# SDP descriptor for the Gazebo H264 RTP stream.
# ffmpeg needs this to decode the incoming UDP stream
# since raw UDP carries no stream metadata.
_SDP_TEMPLATE = """\
v=0
o=- 0 0 IN IP4 127.0.0.1
s=Gazebo Camera
c=IN IP4 127.0.0.1
t=0 0
m=video {port} RTP/AVP 96
a=rtpmap:96 H264/90000
a=fmtp:96 packetization-mode=1
"""


class Camera:
    """
    Standalone Gazebo camera capture.

    Responsibilities:
      - Enable the Gazebo camera stream via gz topic (once at startup)
      - Grab a single frame from the UDP stream using ffmpeg
      - Save the frame as a JPEG

    Gimbal/mount control is NOT handled here — it goes through ArduPilot's
    own mount control (see Drone.point_gimbal() in drone.py), since the
    gimbal joint is wired to a servo channel that ArduPilot continuously
    drives. Any direct Gazebo-topic command would get overwritten.

    No MAVLink dependency — completely separate from drone.py.
    """

    def __init__(
        self,
        udp_port: int = 5600,
        output_dir: str = "captures",
        world_name: str = "large_mission",
        drone_name: str = "drone1",
    ):
        os.makedirs(output_dir, exist_ok=True)
        self._udp_port   = udp_port
        self._output_dir = output_dir
        self._world_name = world_name
        self._drone_name = drone_name

    # -----------------------------------------------------------------------
    # Setup
    # -----------------------------------------------------------------------

    def enable_stream(self) -> None:
        """
        Tell Gazebo to start sending the camera stream over UDP.
        Call once after Gazebo is running, before any captures.
        """
        topic = (
            "/world/{world}/model/{drone}"
            "/model/gimbal/link/pitch_link/sensor/camera"
            "/image/enable_streaming"
        ).format(world=self._world_name, drone=self._drone_name)

        result = subprocess.run(
            ["gz", "topic", "-t", topic,
             "-m", "gz.msgs.Boolean", "-p", "data: 1"],
            capture_output=True, text=True,
        )
        if result.returncode == 0:
            print("[CAMERA] Stream enabled on UDP:{0}".format(self._udp_port))
        else:
            print("[CAMERA] Warning enabling stream: {0}".format(
                result.stderr.strip()))

    # -----------------------------------------------------------------------
    # Capture
    # -----------------------------------------------------------------------

    def capture(self, label, subdir: Optional[str] = None, max_retries: int = 6) -> Optional[str]:
        """
        Grab one frame from the Gazebo UDP stream using ffmpeg.

        `label` can be an int (legacy waypoint id, formatted as wp{id:02d})
        or a string (used directly, e.g. "nadir", "side_south").

        `subdir`, if given, groups the file under {output_dir}/{subdir}/
        (e.g. captures/wp02/nadir_HHMMSS.jpg) -- used to keep all photos
        for one waypoint together.

        Each capture spawns a fresh ffmpeg process that binds to this
        drone's dedicated UDP port. Under heavy load (multiple drones +
        Gazebo + SITL + YOLO all competing for CPU), the OS occasionally
        hasn't fully released the previous ffmpeg process's socket
        before the next one tries to bind, causing an intermittent
        "Address already in use" error. This is automatically retried
        with EXPONENTIAL BACKOFF (0.3s, 0.6s, 1.2s, 2.4s, ...) since a
        fixed short delay (previously 3 attempts at 0.3s) wasn't always
        enough under real multi-drone load -- it's a transient timing
        race, not a real failure; everything else about the capture is
        otherwise fine.

        Returns the saved file path, or None on failure (after retries
        are exhausted). Logging is left to the caller to avoid duplicate
        prints, except for the retry attempts themselves.
        """
        label_str = "wp{:02d}".format(label) if isinstance(label, int) else str(label)

        out_dir = self._output_dir
        if subdir:
            out_dir = os.path.join(self._output_dir, subdir)
            os.makedirs(out_dir, exist_ok=True)

        delay = 0.6 #old was 0.3
        for attempt in range(1, max_retries + 1):
            result_path = self._try_capture_once(label_str, out_dir)
            if result_path is not None:
                return result_path

            if attempt < max_retries:
                print("[CAMERA] Capture attempt {0}/{1} failed, retrying in {2:.1f}s...".format(
                    attempt, max_retries, delay))
                time.sleep(delay)
                delay *= 2.0

        return None

    def _try_capture_once(self, label_str: str, out_dir: str) -> Optional[str]:
        """Single capture attempt. Returns the saved path, or None on
        failure (caller decides whether to retry)."""
        # Write SDP to a temp file so ffmpeg can open the stream
        sdp_file = tempfile.NamedTemporaryFile(
            mode="w", suffix=".sdp", delete=False)
        sdp_file.write(_SDP_TEMPLATE.format(port=self._udp_port))
        sdp_file.close()

        # Temp output path — let ffmpeg create it cleanly
        out_file = tempfile.NamedTemporaryFile(suffix=".jpg", delete=False)
        out_path = out_file.name
        out_file.close()
        os.unlink(out_path)

        try:
            result = subprocess.run(
                [
                    "ffmpeg",
                    "-hide_banner", "-loglevel", "error",
                    "-protocol_whitelist", "file,udp,rtp",
                    "-fflags", "nobuffer+discardcorrupt",
                    "-flags", "low_delay",
                    "-i", sdp_file.name,
                    "-vframes", "1",
                    "-y", out_path,
                ],
                stdin=subprocess.DEVNULL,
                capture_output=True,
                timeout=20,
            )

            if result.returncode != 0:
                print("[CAMERA] ffmpeg error: {0}".format(
                    result.stderr.decode().strip()))
                return None

            if not os.path.exists(out_path):
                print("[CAMERA] ffmpeg produced no output")
                return None

            # Re-ensure the destination directory exists right before
            # writing -- defensive, in case anything unexpected removed
            # it between Camera.__init__() and this point.
            os.makedirs(out_dir, exist_ok=True)

            # Move to permanent location with meaningful name. Using
            # shutil.move() instead of os.rename() -- rename() requires
            # both paths on the same filesystem, and /tmp is sometimes
            # a separate tmpfs mount from the project directory, which
            # would make plain rename() fail unpredictably.
            save_path = os.path.join(
                out_dir,
                "{0}_{1}.jpg".format(label_str, time.strftime("%H%M%S")),
            )
            shutil.move(out_path, save_path)
            return save_path

        except subprocess.TimeoutExpired:
            print("[CAMERA] ffmpeg timed out")
            return None
        except Exception as e:
            print("[CAMERA] Unexpected error: {0}".format(e))
            return None
        finally:
            # Always clean up the SDP temp file
            if os.path.exists(sdp_file.name):
                os.unlink(sdp_file.name)
            # Clean up output temp file if ffmpeg failed
            if os.path.exists(out_path):
                os.unlink(out_path)
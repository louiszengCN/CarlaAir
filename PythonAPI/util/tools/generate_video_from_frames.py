"""Generate a video from a folder of frames."""

from __future__ import annotations

import argparse
import glob

from moviepy.video.io import ImageSequenceClip
from natsort import natsorted

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Output
_DEFAULT_VIDEO_NAME: str = "video"
_DEFAULT_FPS: int = 20
_VIDEO_EXTENSION: str = ".mp4"

# Frame filtering
_FRAME_EXCLUDE_COUNT: int = 1  # exclude last frame


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    argparser = argparse.ArgumentParser(
        description="Generate a video from a folder of frames",
    )
    argparser.add_argument(
        "--frames",
        required=True,
        help="Folder where frames are stored",
    )
    argparser.add_argument(
        "--video",
        default=_DEFAULT_VIDEO_NAME,
        type=str,
        help="Name of the output video",
    )
    argparser.add_argument(
        "--fps",
        default=_DEFAULT_FPS,
        type=int,
        help=f"Frames per second (default: {_DEFAULT_FPS})",
    )
    return argparser.parse_args()


def main() -> None:
    """Generate video from image frames."""
    args = _parse_args()

    frames_folder = args.frames
    video_name = args.video + _VIDEO_EXTENSION
    fps = args.fps

    # Get all frames in folder
    image_files: list[str] = natsorted(
        glob.glob(f"{frames_folder}/*"),
    )
    # Exclude last frame, which sometimes has not been rendered correctly
    if _FRAME_EXCLUDE_COUNT > 0:
        image_files = image_files[:-_FRAME_EXCLUDE_COUNT]

    # Create video and save
    clip = ImageSequenceClip.ImageSequenceClip(image_files, fps=fps)
    clip.write_videofile(video_name)


if __name__ == "__main__":
    main()

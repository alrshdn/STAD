from video_stream import VideoStream
from aruco_single_tracker import ArucoSingleTracker


def main():
    source = VideoStream()
    tracker = ArucoSingleTracker(
            id_to_find=0,
            marker_size=50,
            source=source,
            )

    tracker.track_live(verbose=True, show_video=True)



if __name__ == "__main__":
    main()

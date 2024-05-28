import numpy as np

import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class VideoStream(object):
    def __init__(self, port=5600):
        Gst.init(None)

        self._frame = None

        # setting up UPD connection
        self._port = port
        self._source = \
                f"udpsrc port={self._port}"

        # setting up pipeline description (codec, decoding, and sink configuration)
        self._codec = \
                "! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264"
        self._decode = \
                "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert"
        self._sink_conf = \
                "! appsink emit-signals=true sync=false max-buffers=2 drop=true"
        

        # initializing pipeline
        pipeline_description = [self._source, self._codec, self._decode, self._sink_conf]
        self._pipe = Gst.parse_launch(
                " ".join(pipeline_description)
                )

        # initialize `sink` as `None` until video is playing
        self._sink = None
        
        # running video stream at initialization
        self.run()


    @staticmethod
    def _gst_to_opencv(sample) -> np.ndarray:
        buf = sample.get_buffer()
        caps = sample.get_caps()

        return np.ndarray(
                    (
                        caps.get_structure(0).get_value("height"),
                        caps.get_structure(0).get_value("width"),
                        3
                    ),
                    buffer=buf.extract_dup(0, buf.get_size()),
                    dtype=np.uint8,
                )


    def _callback(self, sink):
        sample = sink.emit("pull-sample")
        new_frame = self._gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK


    def run(self, config=None):
        self._pipe.set_state(Gst.State.PLAYING)

        self._sink = self._pipe.get_by_name("appsink0")

        self._sink.connect("new-sample", self._callback)

    
    def _is_frame_available(self):
        return self._frame is not None


    def _get_frame(self):
        return self._frame

    
    # mimicks `OoenCV`'s `read`
    def read(self):
        if self._is_frame_available():
            return True, self._get_frame().copy()
        else:
            return False, None


    # mimicks `OpenCV`'s `release`
    def release(self):
        self._pipe.set_state(Gst.State.NULL)


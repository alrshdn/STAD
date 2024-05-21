import numpy as np


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
        pipeline_description = [self._codec, self._decode, self._sink_conf]
        self._pipeline = Gst.parse_launch(
                " ".join(pipeline_description)
                )

        # initialize `sink` as `None` until video is playing
        self._sink = None

    def _callback(self, sink):
        sample = sink.emit("pull-sample")
        self._frame = self._gst_to_opencv(sample)

        return Gst.FlowReturn.OK
    
    def run(self, config=None):
        self._pipe.set_state(Gst.State.PLAYING)

        self._sink = self._pipe.get_by_name("appsink0")

        self._sink.connect("new-sample", self._callback)

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
    
    def get_frame(self):
        return self._frame

    def is_frame_available(self):
        return self._frame is not None




class ArucoSingleTracker(object):
    def __init__(self, id_to_find, marker_size, source, camera_size=[640, 640], show_video=False):
        self._id_to_find = id_to_find
        self._marker_size = marker_size
        self._show_video = show_video
        self._camera_size = camera_size
        
        self._is_detected = False
        
        self._video = source



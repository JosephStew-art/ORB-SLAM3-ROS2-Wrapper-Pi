import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gst, GLib, GstVideo
import sys

Gst.init(None)

class VideoReceiver:
    def __init__(self):
        self.pipeline = Gst.parse_launch(
            "udpsrc port=5000 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! autovideosink"
        )

        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self.on_message)

        # Add a probe to the videosink
        videosink = self.pipeline.get_by_name("autovideosink0")
        pad = videosink.get_static_pad("sink")
        pad.add_probe(Gst.PadProbeType.BUFFER, self.probe_callback)

    def probe_callback(self, pad, info):
        buffer = info.get_buffer()
        pts = buffer.pts
        if pts != Gst.CLOCK_TIME_NONE:
            timestamp = pts / Gst.SECOND
            print(f"Timestamp: {timestamp:.6f} seconds")
        return Gst.PadProbeReturn.OK

    def run(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        GLib.MainLoop().run()

    def on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            print("End of stream")
            self.pipeline.set_state(Gst.State.NULL)
            sys.exit(0)
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}, {debug}")
            self.pipeline.set_state(Gst.State.NULL)
            sys.exit(1)

if __name__ == "__main__":
    receiver = VideoReceiver()
    receiver.run()
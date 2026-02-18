import sys
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GObject', '2.0')
gi.require_version('GLib', '2.0')

from gi.repository import Gst, GObject, GLib
#check if gst is initialized
Gst.init(None)


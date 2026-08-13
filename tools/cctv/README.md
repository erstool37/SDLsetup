# cctv node (vacant)

Whole-setup overview camera (CCTV). **Not implemented yet.** Shown on its own
display tab to watch the entire workcell.

Planned: `snapshot`, `stream`; status publishes online + last-frame age.

To implement: point at a USB webcam (OpenCV `VideoCapture`) or an IP camera
(RTSP/MJPEG). For an IP cam, store the RTSP/MJPEG URL here; for USB, capture via
the Windows side like the microscope cameras. Serve frames the display can embed.

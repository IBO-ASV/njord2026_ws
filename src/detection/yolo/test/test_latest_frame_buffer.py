import threading

import numpy as np

from yolo.perception_utils import LatestFrameBuffer, crop_image_roi, shift_and_clip_bbox


def test_latest_frame_replaces_stale_frame():
    frames = LatestFrameBuffer()
    first = object()
    latest = object()

    assert frames.put(first)
    assert frames.put(latest)
    assert frames.take() is latest


def test_stop_wakes_waiting_consumer_and_rejects_new_frames():
    frames = LatestFrameBuffer()
    result = []
    consumer = threading.Thread(target=lambda: result.append(frames.take()))
    consumer.start()

    frames.stop()
    consumer.join(timeout=1.0)

    assert not consumer.is_alive()
    assert result == [None]
    assert not frames.put(object())


def test_crop_image_roi_normalizes_reversed_bounds_and_keeps_one_pixel():
    image = np.zeros((10, 20, 3), dtype=np.uint8)

    cropped, offset_x, offset_y, rect = crop_image_roi(
        image, 0.75, 0.25, 1.5, -1.0)

    assert cropped.shape == (10, 10, 3)
    assert (offset_x, offset_y, rect) == (5, 0, (5, 0, 14, 9))


def test_shift_and_clip_bbox_returns_full_image_coordinates():
    bbox = shift_and_clip_bbox(
        np.array([-2.0, 1.0, 20.0, 12.0]), 5, 3, 20, 10)

    assert np.array_equal(bbox, np.array([3.0, 4.0, 20.0, 10.0]))

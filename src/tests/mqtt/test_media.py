from lite3_mqtt.contract import DetectionType, Topics
from lite3_mqtt.media import DetectionMediaPublisher


class FakeMediaSource:
    def capture_image(self, detection_type, event_id):
        return b"\xff\xd8annotated-jpeg\xff\xd9"

    def capture_video(self, detection_type, event_id, duration_ms):
        return b"\x00\x00\x00\x18ftypmp42"


def test_detection_publishes_image_before_video_with_same_event_id():
    published = []
    times = iter([1783652400123, 1783652405341])
    publisher = DetectionMediaPublisher(
        media_source=FakeMediaSource(),
        publish_json=lambda topic, payload: published.append((topic, payload)),
        duration_ms=5000,
        clock_ms=lambda: next(times),
    )

    publisher.publish_detection(DetectionType.BROKEN_CUP, event_id="event-1")

    assert [topic for topic, _ in published] == [
        Topics.BROKEN_CUP_IMAGE,
        Topics.BROKEN_CUP_VIDEO,
    ]
    assert published[0][1]["event_id"] == "event-1"
    assert published[1][1]["event_id"] == "event-1"
    assert published[0][1]["timestamp"] != published[1][1]["timestamp"]


class BrokenMediaSource:
    def capture_image(self, detection_type, event_id):
        raise RuntimeError("image failed")

    def capture_video(self, detection_type, event_id, duration_ms):
        raise RuntimeError("video failed")


def test_media_failures_publish_contract_fail_payloads():
    published = []
    publisher = DetectionMediaPublisher(
        media_source=BrokenMediaSource(),
        publish_json=lambda topic, payload: published.append((topic, payload)),
    )

    publisher.publish_detection(DetectionType.COYOTE, event_id="event-2")

    assert [payload["result"] for _, payload in published] == ["FAIL", "FAIL"]
    assert published[0][1]["image"]["data_base64"] == ""
    assert published[1][1]["video"]["data_base64"] == ""

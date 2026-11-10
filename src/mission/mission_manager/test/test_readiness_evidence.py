from mission_manager.readiness_evidence import filtered_cloud_ready


def test_filtered_cloud_requires_recent_stamped_frame():
    assert filtered_cloud_ready(10.0, 10.5, 1.0, 10.0, "base_link")
    assert not filtered_cloud_ready(10.0, 11.1, 1.0, 10.0, "base_link")
    assert not filtered_cloud_ready(10.0, 10.5, 1.0, None, "base_link")
    assert not filtered_cloud_ready(10.0, 10.5, 1.0, 10.0, "")

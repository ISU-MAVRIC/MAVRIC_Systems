import unittest

try:
    import app
except ModuleNotFoundError as exc:
    app = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


class TestAppCameraStream(unittest.TestCase):
    @unittest.skipIf(app is None, f"prototype app dependencies unavailable: {IMPORT_ERROR}")
    def test_camera_stream_returns_mjpeg_response(self):
        client = app.app.test_client()
        response = client.get("/camera/stream", buffered=False)

        self.assertEqual(response.status_code, 200)
        self.assertIn("multipart/x-mixed-replace", response.content_type)

        first_chunk = next(response.response)
        self.assertIn(b"Content-Type: image/jpeg", first_chunk)


if __name__ == "__main__":
    unittest.main()

#include <raspicam/raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <crow.h>

int main()
{
    raspicam::RaspiCam_Cv Camera;
    cv::Mat image;

    // Set camera parameters
    Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);

    // Open camera
    if (!Camera.open())
    {
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }

    // Initialize Crow server
    crow::SimpleApp app;

    CROW_ROUTE(app, "/stream")
    ([&]()
     {
        std::ostringstream stream;
        stream << "HTTP/1.1 200 OK\r\n";
        stream << "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";

        // Capture loop
        for (;;) {
            Camera.grab();
            Camera.retrieve(image);

            // Convert the image to a format suitable for web streaming
            std::vector<uchar> buffer;
            cv::imencode(".jpg", image, buffer);
            std::string encoded = std::string(buffer.begin(), buffer.end());

            stream << "--frame\r\n";
            stream << "Content-Type: image/jpeg\r\n\r\n";
            stream << encoded;
            stream << "\r\n";
        }

        Camera.release();

        return stream.str(); });

    app.port(8080).multithreaded().run();
}
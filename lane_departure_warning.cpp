#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Function declarations
void processFrame(Mat &frame, bool turnSignal);
vector<Vec4i> detectLaneLines(const Mat &edges);
void calculateLanePosition(const vector<Vec4i> &lines, Mat &frame, bool turnSignal);

int main()
{
    // Open video file
    VideoCapture cap("test2.mp4");
    if (!cap.isOpened())
    {
        cerr << "Error: Unable to open video file." << endl;
        return -1;
    }

    Mat frame;
    bool turnSignal = false;

    // Process each frame from the video
    while (cap.read(frame))
    {
        processFrame(frame, turnSignal);
        imshow("Lane Departure Warning", frame);

        // Wait for user input and toggle turn signal
        char c = (char)waitKey(25);
        if (c == 27) // ESC key to exit
            break;
        if (c == 'l' || c == 'r') // Toggle turn signal with 'l' or 'r'
            turnSignal = !turnSignal;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}

void processFrame(Mat &frame, bool turnSignal)
{
    Mat gray, edges;
    
    // Convert to grayscale
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    
    // Apply Gaussian blur and Canny edge detection
    GaussianBlur(gray, edges, Size(5, 5), 0);
    Canny(edges, edges, 50, 150);

    // Define and apply ROI mask
    Mat mask = Mat::zeros(edges.size(), edges.type());
    Point pts[4] = {
        Point(frame.cols * 0.1, frame.rows),        // Bottom-left corner
        Point(frame.cols * 0.05, frame.rows * 0.5), // Top-left corner
        Point(frame.cols, frame.rows * 0.6),        // Top-right corner
        Point(frame.cols * 0.9, frame.rows)         // Bottom-right corner
    };
    fillConvexPoly(mask, pts, 4, Scalar(255));
    Mat maskedEdges;
    bitwise_and(edges, mask, maskedEdges);

    // Detect lane lines and calculate lane position
    vector<Vec4i> lines = detectLaneLines(maskedEdges);
    calculateLanePosition(lines, frame, turnSignal);
}

vector<Vec4i> detectLaneLines(const Mat &edges)
{
    vector<Vec4i> lines;
    // Use Hough Line Transform to detect lines
    HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);
    return lines;
}

void calculateLanePosition(const vector<Vec4i> &lines, Mat &frame, bool turnSignal)
{
    int frameCenter = frame.cols / 2;
    int leftLanePos = 0, rightLanePos = frame.cols;
    bool leftLaneDetected = false, rightLaneDetected = false;

    for (const auto &l : lines)
    {
        double slope = (l[3] - l[1]) / (double)(l[2] - l[0]);
        if (abs(slope) < 0.5)
            continue;

        if (slope < 0) // Left lane
        {
            if (l[0] < frameCenter && l[2] < frameCenter)
            {
                leftLanePos = max(leftLanePos, max(l[0], l[2]));
                leftLaneDetected = true;
            }
        }
        else // Right lane
        {
            if (l[0] > frameCenter && l[2] > frameCenter)
            {
                rightLanePos = min(rightLanePos, min(l[0], l[2]));
                rightLaneDetected = true;
            }
        }

        // Draw detected lane lines on the frame
        line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 3, LINE_AA);
    }

    // Display lane position and warnings
    if (leftLaneDetected && rightLaneDetected)
    {
        int laneCenter = (leftLanePos + rightLanePos) / 2;
        if (laneCenter < frameCenter - 50 && !turnSignal)
        {
            putText(frame, "Warning: Drifting Left", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        else if (laneCenter > frameCenter + 50 && !turnSignal)
        {
            putText(frame, "Warning: Drifting Right", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        else
        {
            putText(frame, "Centered", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
        }
    }
}

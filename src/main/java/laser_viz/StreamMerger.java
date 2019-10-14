package laser_viz;

import com.atul.JavaOpenCV.Imshow;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.rhea_core.Stream;

import java.util.concurrent.TimeUnit;

import cv_bridge.CvImage;
import ros_eval.RosEvaluationStrategy;
import ros_eval.RosTopic;
import rx_eval.RxjavaEvaluationStrategy;
import sensor_msgs.Image;
import sensor_msgs.LaserScan;

import static nu.pattern.OpenCV.loadShared;

public class StreamMerger {
    private static final RosTopic<LaserScan> LASER = new RosTopic<>("/base_scan", LaserScan._TYPE);
    private static final RosTopic<Image> CAMERA = new RosTopic<>("/wide_stereo/left/image_rect_throttle", Image._TYPE);
    private static final Imshow window;
    static {
        loadShared();
        window = new Imshow("Live Feed");
    }

    public static void main(String[] args) {

        Stream.evaluationStrategy =
                new RosEvaluationStrategy(new RxjavaEvaluationStrategy(), "localhost", "myclient");

        // ROS Topics
        Stream<LaserScan> laser = Stream.from(LASER).sample(100, TimeUnit.MILLISECONDS);
        Stream<Mat> camera = Stream.from(CAMERA).sample(100, TimeUnit.MILLISECONDS).flatMap(im -> {
            // Convert to Mat
            try {
                return Stream.just(CvImage.toCvCopy(im).image);
            } catch (Exception e) {
                return Stream.error(e);
            }
        });

        // Combine
        Stream.combineLatest(camera, laser, StreamMerger::embedLaser)
               // .printAll();
              .subscribe(window::showImage);
    }

    private static Mat embedLaser(Mat im, LaserScan l) {
        Point center = new Point(im.width() / 2, im.height());
        float curAngle = l.getAngleMin();
        for (float r : l.getRanges()) {
            double x = center.x + ((im.width() / 2) * r * Math.cos(curAngle + (Math.PI / 2)));
            double y = center.y - ((im.width() / l.getRangeMax()) * r * Math.sin(curAngle + (Math.PI / 2)));
            if (Math.abs(curAngle) < 0.3)
                Core.line(im, center, new Point(x, y), new Scalar(0, 0, 255));
            curAngle += l.getAngleIncrement();
        }
        Core.circle(im, center, 2, new Scalar(0, 0, 0), -1);
        return im;
    }
}

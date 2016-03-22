package laser_viz;

import com.atul.JavaOpenCV.Imshow;
import cv_bridge.CvImage;
import org.opencv.core.*;
import org.rhea_core.Stream;
import ros_eval.RosTopic;
import sensor_msgs.Image;
import sensor_msgs.LaserScan;

public class StreamMerger {
    private static final RosTopic<LaserScan> LASER = new RosTopic<>("/scan");
    private static final RosTopic<Image> CAMERA = new RosTopic<>("/camera/rgb/image_color");
    private static final Imshow window = new Imshow("Live Feed");
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    public static void main(String[] args) {
        // Stream.setEvaluationStrategy(new RosEvaluationStrategy(reactiveTopic));

        // ROS Topics
        Stream<LaserScan> laser = Stream.from(LASER);
        Stream<Mat> camera = Stream.<Image>from(CAMERA).flatMap(im -> {
            // Convert to Mat
            try {
                return Stream.just(CvImage.toCvCopy(im).image);
            } catch (Exception e) {
                return Stream.error(e);
            }
        });

        // Combine
        Stream.combineLatest(camera, laser, StreamMerger::embedLaser)
              .subscribe(window::showImage);
    }

    private static Mat embedLaser(Mat im, LaserScan l) {
        Point center = new Point(im.width() / 2, im.height());
        float curAngle = l.getAngleMin();
        for (float r : l.getRanges()) {
            double x = (center.x + (im.width() / 2 * r * Math.cos(curAngle + Math.PI / 2)));
            double y = (center.y - (im.width() / l.getRangeMax() * r * Math.sin(curAngle + Math.PI / 2)));
            if (Math.abs(curAngle) < 0.3)
                Core.line(im, center, new Point(x, y), new Scalar(0, 0, 255));
            curAngle += l.getAngleIncrement();
        }
        Core.circle(im, center, 2, new Scalar(0, 0, 0), -1);    
        return im;
    }
}

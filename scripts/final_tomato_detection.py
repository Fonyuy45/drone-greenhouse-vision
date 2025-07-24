#!/usr/bin/env python3
"""
Enhanced Custom YOLO11 Tomato Detection for Drone Camera
WITH ROS2 TOPIC PUBLISHING FOR RQT VISUALIZATION
Optimized for your proven model: 30 tomatoes detected successfully!
Classes: fully_ripened, green, half_ripened with segmentation masks
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Int32, Float32MultiArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
import numpy as np
import os
from datetime import datetime
import time
import json

# Custom message for detections (you can also create a proper .msg file)
from std_msgs.msg import Header

class TomatoDetectionMsg:
    """Custom detection message structure"""
    def __init__(self):
        self.header = Header()
        self.detections = []

class FinalTomatoDetectionNode(Node):
    """
    Production-Ready YOLO11 Tomato Detection for Autonomous Drone
    WITH ROS2 TOPIC PUBLISHING FOR RQT VISUALIZATION
    """
    def __init__(self):
        super().__init__('final_tomato_detection_node')
        
        # Your proven model path
        model_path = os.path.expanduser("~/drone_farm_vision_ws/models/tomato_detection/best.pt")
        
        # Load your tested YOLO11 segmentation model
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'✅ YOLO11 Tomato Model Loaded: {model_path}')
            self.get_logger().info(f'📊 Model Classes: {self.model.model.names}')
            
            # Your exact class mapping from successful test
            self.class_names = {
                0: 'fully_ripened',
                1: 'green', 
                2: 'half_ripened'
            }
                
        except Exception as e:
            self.get_logger().error(f'❌ Model Loading Failed: {e}')
            return
            
        # Optimized detection parameters (based on your test results)
        self.conf_threshold = 0.4  # Your test used 0.4 successfully
        self.iou_threshold = 0.5
        
        # Advanced tomato counting system
        self.session_totals = {name: 0 for name in self.class_names.values()}
        self.frame_detections = []
        self.frame_count = 0
        self.detection_start_time = time.time()
        
        # Performance monitoring
        self.processing_times = []
        self.max_detections_per_frame = 0
        
        # ROS2 camera subscription
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.detection_callback,
            10)
        self.br = CvBridge()
        
        # =================== NEW: ROS2 PUBLISHERS FOR RQT VISUALIZATION ===================
        
        # 1. Compressed annotated image publisher (for rqt_image_view - FASTER & MORE STABLE)
        self.annotated_image_pub = self.create_publisher(
            CompressedImage, 
            '/tomato_detection/annotated_image/compressed', 
            10
        )
        
        # 2. Detection counts publisher (for rqt_plot)
        self.detection_counts_pub = self.create_publisher(
            Float32MultiArray, 
            '/tomato_detection/counts', 
            10
        )
        
        # 3. Total detections publisher (for rqt_plot)
        self.total_detections_pub = self.create_publisher(
            Int32, 
            '/tomato_detection/total_count', 
            10
        )
        
        # 4. Detection statistics JSON publisher (for rqt_console or custom viewers)
        self.detection_stats_pub = self.create_publisher(
            String, 
            '/tomato_detection/statistics', 
            10
        )
        
        # 5. Individual detection centers (for visualization)
        self.detection_centers_pub = self.create_publisher(
            Float32MultiArray, 
            '/tomato_detection/centers', 
            10
        )
        
        # 6. Confidence scores publisher (for rqt_plot)
        self.confidence_scores_pub = self.create_publisher(
            Float32MultiArray, 
            '/tomato_detection/confidences', 
            10
        )
        
        # 7. Performance metrics publisher (for rqt_plot)
        self.performance_pub = self.create_publisher(
            Float32MultiArray, 
            '/tomato_detection/performance', 
            10
        )
        
        # =================== END NEW PUBLISHERS ===================
        
        # Logging setup
        self.last_log_time = time.time()
        self.log_interval = 3.0  # Log every 3 seconds
        
        print("\n" + "="*70)
        print("🍅 PRODUCTION TOMATO DETECTION SYSTEM READY")
        print("📡 WITH ROS2 TOPIC PUBLISHING FOR RQT VISUALIZATION")
        print("="*70)
        print(f"📹 Processing: Drone Camera Feed")
        print(f"🤖 Model: YOLO11 Segmentation (Proven: 30 detections)")
        print(f"🎯 Classes: Fully Ripened, Green, Half Ripened")
        print(f"🔬 Confidence: ≥{self.conf_threshold}")
        print(f"⚡ Real-time: Segmentation + Counting + Publishing")
        print("\n📊 PUBLISHED TOPICS FOR RQT:")
        print("   /tomato_detection/annotated_image/compressed - Compressed annotated video (FASTER)")
        print("   /tomato_detection/counts                     - Detection counts by type")
        print("   /tomato_detection/total_count                - Total detections")
        print("   /tomato_detection/statistics                 - JSON statistics")
        print("   /tomato_detection/centers                    - Detection center points")
        print("   /tomato_detection/confidences                - Confidence scores")
        print("   /tomato_detection/performance                - Processing performance")
        print("="*70 + "\n")
        
    def detection_callback(self, data):
        """Main detection processing pipeline with ROS2 publishing"""
        start_time = time.time()
        
        try:
            self.frame_count += 1
            
            # Convert ROS image to OpenCV
            frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
            
            # Run YOLO11 detection (your proven configuration)
            results = self.model(
                frame, 
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False
            )
            
            # Process detections
            detections = self.process_detections(results[0])
            
            # Update session statistics
            self.update_session_stats(detections)
            
            # Create annotated frame using YOLO's built-in plotting (shows bounding boxes + masks)
            annotated_frame = results[0].plot()
            
            # Add your custom overlay on top of YOLO's annotations
            annotated_frame = self.add_custom_overlay(annotated_frame, detections)
            
            # Performance tracking
            processing_time = (time.time() - start_time) * 1000
            self.processing_times.append(processing_time)
            
            # =================== NEW: PUBLISH TO ROS2 TOPICS ===================
            self.publish_detection_data(annotated_frame, detections, processing_time, data.header)
            # =================== END NEW PUBLISHING ===================
            
            # Periodic detailed logging
            current_time = time.time()
            if current_time - self.last_log_time >= self.log_interval:
                self.log_detailed_stats(detections)
                self.last_log_time = current_time
            
            # Display results (optional - comment out if running headless)
            cv2.imshow("🍅 YOLO11 Production Tomato Detection", annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')
    
    def publish_detection_data(self, annotated_frame, detections, processing_time, original_header):
        """Publish all detection data to ROS2 topics for RQT visualization"""
        
        # 1. Publish compressed annotated image (FASTER & MORE STABLE)
        try:
            # Convert OpenCV image to compressed format
            success, encoded_image = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            if success:
                # Create compressed image message
                compressed_msg = CompressedImage()
                compressed_msg.header = original_header
                compressed_msg.header.frame_id = "camera_frame"
                compressed_msg.format = "jpeg"
                compressed_msg.data = encoded_image.tobytes()
                
                # Publish compressed image
                self.annotated_image_pub.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing annotated image: {e}')
        
        # 2. Publish detection counts by type
        counts_array = Float32MultiArray()
        counts_data = [0.0, 0.0, 0.0]  # [fully_ripened, green, half_ripened]
        
        for detection in detections:
            if detection['class_name'] == 'fully_ripened':
                counts_data[0] += 1
            elif detection['class_name'] == 'green':
                counts_data[1] += 1
            elif detection['class_name'] == 'half_ripened':
                counts_data[2] += 1
        
        counts_array.data = counts_data
        self.detection_counts_pub.publish(counts_array)
        
        # 3. Publish total detection count
        total_msg = Int32()
        total_msg.data = len(detections)
        self.total_detections_pub.publish(total_msg)
        
        # 4. Publish detection statistics as JSON
        stats = {
            'timestamp': time.time(),
            'frame_count': self.frame_count,
            'current_detections': len(detections),
            'session_totals': dict(self.session_totals),
            'processing_time_ms': processing_time,
            'session_duration': time.time() - self.detection_start_time,
            'detection_rate_per_minute': sum(self.session_totals.values()) / ((time.time() - self.detection_start_time) / 60) if time.time() - self.detection_start_time > 0 else 0,
            'max_detections_per_frame': self.max_detections_per_frame
        }
        
        stats_msg = String()
        stats_msg.data = json.dumps(stats, indent=2)
        self.detection_stats_pub.publish(stats_msg)
        
        # 5. Publish detection centers (for spatial visualization)
        centers_array = Float32MultiArray()
        centers_data = []
        
        for detection in detections:
            center_x, center_y = detection['center']
            centers_data.extend([center_x, center_y, float(detection['class_id'])])
        
        centers_array.data = centers_data
        self.detection_centers_pub.publish(centers_array)
        
        # 6. Publish confidence scores
        confidences_array = Float32MultiArray()
        confidences_data = [detection['confidence'] for detection in detections]
        confidences_array.data = confidences_data
        self.confidence_scores_pub.publish(confidences_array)
        
        # 7. Publish performance metrics
        performance_array = Float32MultiArray()
        avg_processing = np.mean(self.processing_times[-30:]) if self.processing_times else 0
        fps = 1000 / avg_processing if avg_processing > 0 else 0
        
        performance_data = [
            processing_time,           # Current processing time
            avg_processing,            # Average processing time (last 30 frames)
            fps,                      # Effective FPS
            float(len(detections)),   # Current detection count
            float(self.frame_count)   # Total frames processed
        ]
        
        performance_array.data = performance_data
        self.performance_pub.publish(performance_array)
    
    def process_detections(self, result):
        """Extract and process detection results"""
        detections = []
        
        if result.boxes is not None and len(result.boxes) > 0:
            # Extract detection data
            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()
            
            # Extract segmentation masks
            masks = None
            if hasattr(result, 'masks') and result.masks is not None:
                masks = result.masks.data.cpu().numpy()
            
            for i, (box, conf, cls) in enumerate(zip(boxes, confs, classes)):
                cls_id = int(cls)
                if cls_id in self.class_names:
                    detection = {
                        'box': box,
                        'confidence': conf,
                        'class_id': cls_id,
                        'class_name': self.class_names[cls_id],
                        'mask': masks[i] if masks is not None else None,
                        'center': self.calculate_center(box),
                        'area': self.calculate_area(box),
                        'timestamp': time.time()
                    }
                    detections.append(detection)
            
            # Track maximum detections per frame
            if len(detections) > self.max_detections_per_frame:
                self.max_detections_per_frame = len(detections)
        
        return detections
    
    def calculate_center(self, box):
        """Calculate center point of bounding box"""
        x1, y1, x2, y2 = box
        return ((x1 + x2) / 2, (y1 + y2) / 2)
    
    def calculate_area(self, box):
        """Calculate bounding box area"""
        x1, y1, x2, y2 = box
        return (x2 - x1) * (y2 - y1)
    
    def update_session_stats(self, detections):
        """Update comprehensive session statistics"""
        for detection in detections:
            self.session_totals[detection['class_name']] += 1
    
    def add_custom_overlay(self, annotated_frame, detections):
        """Add custom statistics overlay on top of YOLO's annotations"""
        # Count current frame detections
        current_counts = {'fully_ripened': 0, 'green': 0, 'half_ripened': 0}
        for det in detections:
            current_counts[det['class_name']] += 1
        
        # Create enhanced overlay
        overlay_height = 220
        overlay_width = 400
        overlay = np.zeros((overlay_height, overlay_width, 3), dtype=np.uint8)
        overlay.fill(30)  # Dark background
        
        # Header
        cv2.putText(overlay, "🍅 PRODUCTION TOMATO DETECTION", (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # ROS2 Publishing indicator
        cv2.putText(overlay, "📡 Publishing to ROS2 Topics", (10, 45), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 1)
        
        # Current frame section
        y_offset = 65
        cv2.putText(overlay, f"Current Frame: {len(detections)} tomatoes", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1)
        
        # Individual current counts
        colors = {'fully_ripened': (0, 0, 255), 'green': (0, 255, 0), 'half_ripened': (0, 165, 255)}
        y_offset = 85
        
        for tomato_type in ['fully_ripened', 'green', 'half_ripened']:
            count = current_counts[tomato_type]
            color = colors[tomato_type] if count > 0 else (100, 100, 100)
            display_name = tomato_type.replace('_', ' ').title()
            cv2.putText(overlay, f"  {display_name}: {count}", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            y_offset += 18
        
        # Session statistics
        y_offset += 10
        total_session = sum(self.session_totals.values())
        session_time = time.time() - self.detection_start_time
        
        cv2.putText(overlay, f"Session Total: {total_session} tomatoes", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        y_offset += 20
        
        cv2.putText(overlay, f"Runtime: {session_time:.1f}s", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y_offset += 15
        
        cv2.putText(overlay, f"Frames: {self.frame_count}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y_offset += 15
        
        cv2.putText(overlay, f"Max/Frame: {self.max_detections_per_frame}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # Performance stats
        if self.processing_times:
            avg_time = np.mean(self.processing_times[-30:])  # Last 30 frames
            fps = 1000 / avg_time if avg_time > 0 else 0
            y_offset += 15
            cv2.putText(overlay, f"Avg: {avg_time:.1f}ms ({fps:.1f} FPS)", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 255, 150), 1)
        
        # Blend overlay with frame
        frame_h, frame_w = annotated_frame.shape[:2]
        if frame_h > overlay_height and frame_w > overlay_width:
            overlay_y = 10
            overlay_x = frame_w - overlay_width - 10
            
            roi = annotated_frame[overlay_y:overlay_y + overlay_height, 
                       overlay_x:overlay_x + overlay_width]
            blended = cv2.addWeighted(roi, 0.6, overlay, 0.4, 0)
            annotated_frame[overlay_y:overlay_y + overlay_height, 
                  overlay_x:overlay_x + overlay_width] = blended
        
        return annotated_frame
    
    def log_detailed_stats(self, current_detections):
        """Log comprehensive detection statistics"""
        total_session = sum(self.session_totals.values())
        session_time = time.time() - self.detection_start_time
        
        if total_session > 0 or len(current_detections) > 0:
            self.get_logger().info(
                f"🍅 STATS | Frame: {len(current_detections)} | "
                f"Session: {total_session} | "
                f"Time: {session_time:.1f}s | "
                f"Rate: {total_session/session_time*60:.1f}/min | "
                f"Fully: {self.session_totals['fully_ripened']} | "
                f"Green: {self.session_totals['green']} | "
                f"Half: {self.session_totals['half_ripened']}"
            )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        detector = FinalTomatoDetectionNode()
        
        if hasattr(detector, 'model'):
            print("🚀 STARTING PRODUCTION TOMATO DETECTION...")
            print("📡 Connecting to drone camera feed...")
            print("⚡ Real-time processing active")
            print("📊 Publishing to ROS2 topics for RQT")
            print("🛑 Press Ctrl+C for final statistics\n")
            
            print("🎯 RQT VISUALIZATION COMMANDS:")
            print("   rqt_image_view /tomato_detection/annotated_image/compressed")
            print("   rqt_plot /tomato_detection/counts/data[0]:data[1]:data[2]")
            print("   rqt_plot /tomato_detection/total_count/data")
            print("   rqt_plot /tomato_detection/performance/data[1]:data[2]")
            print("   rqt_console (for statistics)")
            print("")
            
            rclpy.spin(detector)
            
    except KeyboardInterrupt:
        print("\n" + "="*60)
        print("🛑 MISSION COMPLETE - FINAL DETECTION REPORT")
        print("="*60)
        
        if hasattr(detector, 'session_totals'):
            total_time = time.time() - detector.detection_start_time
            total_detections = sum(detector.session_totals.values())
            
            print(f" DETECTION SUMMARY:")
            print(f"   Mission Duration: {total_time:.1f} seconds")
            print(f"   Frames Processed: {detector.frame_count}")
            print(f"   Total Tomatoes: {total_detections}")
            print(f"   Detection Rate: {total_detections/total_time*60:.1f} tomatoes/minute")
            print(f"   Max per Frame: {detector.max_detections_per_frame}")
            
            print(f"\n🍅 BY RIPENESS STAGE:")
            for tomato_type, count in detector.session_totals.items():
                percentage = (count / total_detections * 100) if total_detections > 0 else 0
                display_name = tomato_type.replace('_', ' ').title()
                print(f"   {display_name:15}: {count:4d} ({percentage:5.1f}%)")
            
            if detector.processing_times:
                avg_processing = np.mean(detector.processing_times)
                print(f"\n⚡ PERFORMANCE:")
                print(f"   Average Processing: {avg_processing:.1f}ms per frame")
                print(f"   Effective FPS: {1000/avg_processing:.1f}")
            
            print("\n🎯 MISSION STATUS: ✅ SUCCESSFUL")
    
    except Exception as e:
        print(f"❌ System Error: {e}")
    
    finally:
        if 'detector' in locals():
            detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

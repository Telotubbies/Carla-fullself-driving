"""
Lane Detection System
ตรวจจับเลนถนนด้วย Deep Learning (U-Net) + Computer Vision
"""

import cv2
import numpy as np
import torch
import torch.nn as nn
from typing import Tuple, List, Optional


class UNetLaneDetector(nn.Module):
    """U-Net สำหรับ Lane Segmentation"""
    
    def __init__(self, in_channels=3, out_channels=1):
        super().__init__()
        
        # Encoder
        self.enc1 = self.conv_block(in_channels, 64)
        self.enc2 = self.conv_block(64, 128)
        self.enc3 = self.conv_block(128, 256)
        self.enc4 = self.conv_block(256, 512)
        
        # Bottleneck
        self.bottleneck = self.conv_block(512, 1024)
        
        # Decoder
        self.upconv4 = nn.ConvTranspose2d(1024, 512, 2, stride=2)
        self.dec4 = self.conv_block(1024, 512)
        
        self.upconv3 = nn.ConvTranspose2d(512, 256, 2, stride=2)
        self.dec3 = self.conv_block(512, 256)
        
        self.upconv2 = nn.ConvTranspose2d(256, 128, 2, stride=2)
        self.dec2 = self.conv_block(256, 128)
        
        self.upconv1 = nn.ConvTranspose2d(128, 64, 2, stride=2)
        self.dec1 = self.conv_block(128, 64)
        
        # Output
        self.out = nn.Conv2d(64, out_channels, 1)
        
        self.pool = nn.MaxPool2d(2)
        
    def conv_block(self, in_ch, out_ch):
        return nn.Sequential(
            nn.Conv2d(in_ch, out_ch, 3, padding=1),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, 3, padding=1),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True)
        )
    
    def forward(self, x):
        # Encoder
        e1 = self.enc1(x)
        e2 = self.enc2(self.pool(e1))
        e3 = self.enc3(self.pool(e2))
        e4 = self.enc4(self.pool(e3))
        
        # Bottleneck
        b = self.bottleneck(self.pool(e4))
        
        # Decoder
        d4 = self.upconv4(b)
        d4 = torch.cat([d4, e4], dim=1)
        d4 = self.dec4(d4)
        
        d3 = self.upconv3(d4)
        d3 = torch.cat([d3, e3], dim=1)
        d3 = self.dec3(d3)
        
        d2 = self.upconv2(d3)
        d2 = torch.cat([d2, e2], dim=1)
        d2 = self.dec2(d2)
        
        d1 = self.upconv1(d2)
        d1 = torch.cat([d1, e1], dim=1)
        d1 = self.dec1(d1)
        
        return torch.sigmoid(self.out(d1))


class LaneDetector:
    """
    Lane Detection System แบบ Tesla
    - Deep Learning (U-Net) สำหรับ segmentation
    - Computer Vision สำหรับ lane fitting
    - BEV transformation
    """
    
    def __init__(
        self,
        model_path: Optional[str] = None,
        use_gpu: bool = True,
        image_size: Tuple[int, int] = (640, 480)
    ):
        self.image_size = image_size
        self.device = torch.device('cuda' if use_gpu and torch.cuda.is_available() else 'cpu')
        
        # Load model
        self.model = UNetLaneDetector(in_channels=3, out_channels=1).to(self.device)
        
        if model_path and os.path.exists(model_path):
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
            print(f"✅ Loaded lane detection model from {model_path}")
        else:
            print("⚠️ No pre-trained model, using random weights")
        
        self.model.eval()
        
        # BEV transformation matrix
        self.setup_bev_transform()
        
    def setup_bev_transform(self):
        """ตั้งค่า perspective transform สำหรับ BEV"""
        h, w = self.image_size[1], self.image_size[0]
        
        # Source points (trapezoid in perspective view)
        self.src_points = np.float32([
            [w * 0.25, h * 0.6],   # Top left
            [w * 0.75, h * 0.6],   # Top right
            [w * 0.95, h],         # Bottom right
            [w * 0.05, h]          # Bottom left
        ])
        
        # Destination points (rectangle in BEV)
        self.dst_points = np.float32([
            [w * 0.2, 0],
            [w * 0.8, 0],
            [w * 0.8, h],
            [w * 0.2, h]
        ])
        
        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        self.M_inv = cv2.getPerspectiveTransform(self.dst_points, self.src_points)
    
    def detect(self, image: np.ndarray) -> dict:
        """
        ตรวจจับเลนถนน
        
        Args:
            image: RGB image (H, W, 3)
            
        Returns:
            dict with:
                - lane_mask: binary mask
                - lane_lines: list of lane lines [(x1,y1,x2,y2), ...]
                - lane_polynomials: polynomial coefficients
                - bev_mask: BEV lane mask
                - lane_center_offset: offset from lane center (meters)
                - lane_heading: heading error (radians)
        """
        # Resize image
        img = cv2.resize(image, self.image_size)
        
        # Deep learning detection
        lane_mask = self._detect_with_dl(img)
        
        # Transform to BEV
        bev_mask = cv2.warpPerspective(lane_mask, self.M, self.image_size)
        
        # Fit lane polynomials in BEV
        left_poly, right_poly = self._fit_lane_polynomials(bev_mask)
        
        # Calculate lane center offset and heading
        lane_center_offset, lane_heading = self._calculate_lane_metrics(
            left_poly, right_poly, bev_mask.shape
        )
        
        # Extract lane lines for visualization
        lane_lines = self._extract_lane_lines(lane_mask)
        
        return {
            'lane_mask': lane_mask,
            'lane_lines': lane_lines,
            'left_polynomial': left_poly,
            'right_polynomial': right_poly,
            'bev_mask': bev_mask,
            'lane_center_offset': lane_center_offset,  # meters
            'lane_heading': lane_heading,  # radians
            'lane_detected': left_poly is not None or right_poly is not None
        }
    
    def _detect_with_dl(self, image: np.ndarray) -> np.ndarray:
        """ตรวจจับด้วย Deep Learning"""
        # Preprocess
        img_tensor = torch.from_numpy(image).permute(2, 0, 1).float() / 255.0
        img_tensor = img_tensor.unsqueeze(0).to(self.device)
        
        # Inference
        with torch.no_grad():
            output = self.model(img_tensor)
        
        # Postprocess
        mask = output.squeeze().cpu().numpy()
        mask = (mask > 0.5).astype(np.uint8) * 255
        
        return mask
    
    def _fit_lane_polynomials(
        self,
        bev_mask: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Fit polynomial curves to lane lines in BEV"""
        h, w = bev_mask.shape
        
        # Find lane pixels using sliding window
        histogram = np.sum(bev_mask[h//2:, :], axis=0)
        midpoint = w // 2
        
        # Find peaks for left and right lanes
        left_peak = np.argmax(histogram[:midpoint]) if np.max(histogram[:midpoint]) > 0 else None
        right_peak = midpoint + np.argmax(histogram[midpoint:]) if np.max(histogram[midpoint:]) > 0 else None
        
        # Sliding window parameters
        n_windows = 20
        window_height = h // n_windows
        margin = 50
        minpix = 50
        
        # Find lane pixels
        left_lane_inds = []
        right_lane_inds = []
        
        nonzero = bev_mask.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        # Sliding window for left lane
        if left_peak is not None:
            current_x = left_peak
            for window in range(n_windows):
                win_y_low = h - (window + 1) * window_height
                win_y_high = h - window * window_height
                win_x_low = current_x - margin
                win_x_high = current_x + margin
                
                good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                            (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                
                left_lane_inds.append(good_inds)
                
                if len(good_inds) > minpix:
                    current_x = int(np.mean(nonzerox[good_inds]))
        
        # Sliding window for right lane
        if right_peak is not None:
            current_x = right_peak
            for window in range(n_windows):
                win_y_low = h - (window + 1) * window_height
                win_y_high = h - window * window_height
                win_x_low = current_x - margin
                win_x_high = current_x + margin
                
                good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                            (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                
                right_lane_inds.append(good_inds)
                
                if len(good_inds) > minpix:
                    current_x = int(np.mean(nonzerox[good_inds]))
        
        # Concatenate indices
        left_lane_inds = np.concatenate(left_lane_inds) if left_lane_inds else np.array([])
        right_lane_inds = np.concatenate(right_lane_inds) if right_lane_inds else np.array([])
        
        # Fit polynomials
        left_poly = None
        right_poly = None
        
        if len(left_lane_inds) > 0:
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            left_poly = np.polyfit(lefty, leftx, 2)
        
        if len(right_lane_inds) > 0:
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]
            right_poly = np.polyfit(righty, rightx, 2)
        
        return left_poly, right_poly
    
    def _calculate_lane_metrics(
        self,
        left_poly: Optional[np.ndarray],
        right_poly: Optional[np.ndarray],
        shape: Tuple[int, int]
    ) -> Tuple[float, float]:
        """คำนวณ lane center offset และ heading error"""
        h, w = shape
        
        # Meters per pixel (approximate)
        ym_per_pix = 30.0 / h  # 30 meters ahead
        xm_per_pix = 3.7 / w   # 3.7 meters lane width
        
        # Calculate at bottom of image (vehicle position)
        y_eval = h - 1
        
        lane_center_offset = 0.0
        lane_heading = 0.0
        
        if left_poly is not None and right_poly is not None:
            # Calculate lane center
            left_x = left_poly[0] * y_eval**2 + left_poly[1] * y_eval + left_poly[2]
            right_x = right_poly[0] * y_eval**2 + right_poly[1] * y_eval + right_poly[2]
            lane_center = (left_x + right_x) / 2
            
            # Vehicle center (middle of image)
            vehicle_center = w / 2
            
            # Offset in meters
            lane_center_offset = (vehicle_center - lane_center) * xm_per_pix
            
            # Calculate heading (derivative of polynomial at y_eval)
            left_dx = 2 * left_poly[0] * y_eval + left_poly[1]
            right_dx = 2 * right_poly[0] * y_eval + right_poly[1]
            avg_dx = (left_dx + right_dx) / 2
            
            # Heading error in radians
            lane_heading = np.arctan(avg_dx * ym_per_pix / xm_per_pix)
        
        elif left_poly is not None:
            # Only left lane
            left_x = left_poly[0] * y_eval**2 + left_poly[1] * y_eval + left_poly[2]
            lane_center = left_x + (3.7 / 2) / xm_per_pix  # Assume 3.7m lane width
            vehicle_center = w / 2
            lane_center_offset = (vehicle_center - lane_center) * xm_per_pix
            
        elif right_poly is not None:
            # Only right lane
            right_x = right_poly[0] * y_eval**2 + right_poly[1] * y_eval + right_poly[2]
            lane_center = right_x - (3.7 / 2) / xm_per_pix
            vehicle_center = w / 2
            lane_center_offset = (vehicle_center - lane_center) * xm_per_pix
        
        return lane_center_offset, lane_heading
    
    def _extract_lane_lines(self, mask: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """Extract lane lines using Hough transform"""
        # Edge detection
        edges = cv2.Canny(mask, 50, 150)
        
        # Hough lines
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=50,
            maxLineGap=50
        )
        
        if lines is None:
            return []
        
        return [tuple(line[0]) for line in lines]
    
    def visualize(self, image: np.ndarray, detection_result: dict) -> np.ndarray:
        """วาดผลลัพธ์บนภาพ"""
        vis = image.copy()
        h, w = vis.shape[:2]
        
        # Draw lane mask overlay
        if detection_result['lane_mask'] is not None:
            mask_colored = np.zeros_like(vis)
            mask_colored[:, :, 1] = detection_result['lane_mask']  # Green channel
            vis = cv2.addWeighted(vis, 0.7, mask_colored, 0.3, 0)
        
        # Draw lane lines
        for line in detection_result['lane_lines']:
            x1, y1, x2, y2 = line
            cv2.line(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw lane center offset
        offset = detection_result['lane_center_offset']
        heading = detection_result['lane_heading']
        
        cv2.putText(vis, f"Offset: {offset:.2f}m", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis, f"Heading: {np.degrees(heading):.1f}deg", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw vehicle position indicator
        cv2.circle(vis, (w//2, h-20), 5, (0, 0, 255), -1)
        
        return vis


import os

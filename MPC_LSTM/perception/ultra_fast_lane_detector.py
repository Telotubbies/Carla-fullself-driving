"""
Ultra-Fast-Lane-Detection-v2 Integration.

Wrapper for pre-trained Ultra-Fast-Lane-Detection-v2 model.
Converts lane detection output to binary lane mask.
"""

import torch
import torch.nn as nn
import numpy as np
import cv2
import logging
from typing import Tuple, Optional, List, Dict
from pathlib import Path
import sys
import os

sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.device_utils import get_device

logger = logging.getLogger(__name__)


class UltraFastLaneDetector:
    """
    Wrapper for Ultra-Fast-Lane-Detection-v2 pre-trained model.
    
    This class loads a pre-trained model from Ultra-Fast-Lane-Detection-v2
    and converts its output to a binary lane mask compatible with our system.
    """
    
    def __init__(self, model_path: Optional[str] = None, dataset: str = "tusimple", 
                 backbone: str = "18", input_size: Tuple[int, int] = (320, 800)):
        """
        Initialize Ultra-Fast-Lane-Detection-v2 detector.
        
        Args:
            model_path: Path to pre-trained .pth model file
            dataset: Dataset name ("tusimple", "culane", "curvelanes")
            backbone: Backbone architecture ("18", "34")
            input_size: Model input size (height, width). Default: (320, 800) for TuSimple
        """
        self.model_path = model_path
        self.dataset = dataset.lower()
        self.backbone = backbone
        self.input_size = input_size
        self.model = None
        # Force CPU mode if environment variable is set
        if os.environ.get('FORCE_CPU', '').lower() in ('1', 'true', 'yes'):
            self.device = torch.device('cpu')
            logger.info("🔧 Using CPU (forced by FORCE_CPU env)")
        else:
            self.device = get_device()
        
        # TuSimple config (from configs/tusimple_res18.py)
        if self.dataset == "tusimple":
            self.num_row = 56
            self.num_col = 41
            self.num_cell_row = 100
            self.num_cell_col = 100
            self.num_lanes = 4
            self.use_aux = False
            self.train_height = 320
            self.train_width = 800
            self.fc_norm = False
            # Row anchors for TuSimple (from utils/common.py)
            self.row_anchor = np.linspace(160, 710, self.num_row) / 720
            self.col_anchor = np.linspace(0, 1, self.num_col)
        else:
            # Default to CULane config
            self.num_row = 72
            self.num_col = 81
            self.num_cell_row = 200
            self.num_cell_col = 200
            self.num_lanes = 4
            self.use_aux = False
            self.train_height = 320
            self.train_width = 1600
            self.fc_norm = False
            self.row_anchor = np.linspace(0.42, 1, self.num_row)
            self.col_anchor = np.linspace(0, 1, self.num_col)
        
        self._post_processor = None
        self._post_proc_size = None
        
        if model_path and Path(model_path).exists():
            try:
                self._load_model(model_path)
                logger.info(f"✅ Loaded Ultra-Fast-Lane-Detection-v2 model from {model_path}")
            except Exception as e:
                logger.error(f"Failed to load Ultra-Fast-Lane-Detection-v2 model: {e}")
                raise
        else:
            logger.warning("No model path provided. Use download_model() to download pre-trained model.")
    
    def _load_model(self, model_path: str):
        """
        Load pre-trained model from checkpoint.
        """
        try:
            # Import model architecture from external repository
            external_path = Path(__file__).parent.parent / "external" / "Ultra-Fast-Lane-Detection-v2"
            if not external_path.exists():
                raise FileNotFoundError(f"Ultra-Fast-Lane-Detection-v2 repository not found at {external_path}")
            
            # Add to path
            import sys
            import types
            external_str = str(external_path)
            if external_str not in sys.path:
                sys.path.insert(0, external_str)
            
            # Mock utils.common to avoid problematic dependencies (nvidia.dali, etc.)
            # We only need initialize_weights which is a simple function
            if 'utils.common' not in sys.modules:
                mock_common = types.ModuleType('utils.common')
                def initialize_weights(*args, **kwargs):
                    # Do nothing - weights are already initialized in pretrained model
                    # This is only called during model creation, not needed for inference
                    pass
                mock_common.initialize_weights = initialize_weights
                sys.modules['utils.common'] = mock_common
            
            # Import model - change to external directory for relative imports
            import os
            old_cwd = os.getcwd()
            
            try:
                # Change to external directory (required for relative imports)
                os.chdir(external_str)
                
                # Import model based on dataset
                if self.dataset == "tusimple":
                    # Import directly from model_culane (tusimple uses same architecture)
                    from model.model_culane import parsingNet
                    self.model = parsingNet(
                        pretrained=False,
                        backbone=self.backbone,
                        num_grid_row=self.num_cell_row,
                        num_cls_row=self.num_row,
                        num_grid_col=self.num_cell_col,
                        num_cls_col=self.num_col,
                        num_lane_on_row=self.num_lanes,
                        num_lane_on_col=self.num_lanes,
                        use_aux=self.use_aux,
                        input_height=self.train_height,
                        input_width=self.train_width,
                        fc_norm=self.fc_norm
                    )
                else:
                    # Use CULane model
                    from model.model_culane import parsingNet
                    self.model = parsingNet(
                        pretrained=False,
                        backbone=self.backbone,
                        num_grid_row=self.num_cell_row,
                        num_cls_row=self.num_row,
                        num_grid_col=self.num_cell_col,
                        num_cls_col=self.num_col,
                        num_lane_on_row=self.num_lanes,
                        num_lane_on_col=self.num_lanes,
                        use_aux=self.use_aux,
                        input_height=self.train_height,
                        input_width=self.train_width,
                        fc_norm=self.fc_norm
                    )
            finally:
                os.chdir(old_cwd)
            
            # Load checkpoint
            checkpoint = torch.load(model_path, map_location=self.device)
            
            # Handle different checkpoint formats
            if isinstance(checkpoint, dict):
                if 'model' in checkpoint:
                    state_dict = checkpoint['model']
                elif 'state_dict' in checkpoint:
                    state_dict = checkpoint['state_dict']
                else:
                    state_dict = checkpoint
            else:
                state_dict = checkpoint
            
            # Remove 'module.' prefix if present (from DataParallel)
            compatible_state_dict = {}
            for k, v in state_dict.items():
                if 'module.' in k:
                    compatible_state_dict[k[7:]] = v
                else:
                    compatible_state_dict[k] = v
            
            # Load state dict
            self.model.load_state_dict(compatible_state_dict, strict=True)
            self.model.to(self.device)
            self.model.eval()
            
            logger.info(f"✅ Model loaded successfully (dataset={self.dataset}, backbone={self.backbone})")
            
        except Exception as e:
            logger.error(f"Error loading model: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            raise
    
    def detect_lanes(self, image: np.ndarray, use_post_processing: bool = False) -> Tuple[np.ndarray, np.ndarray, List[List[Tuple[int, int]]]]:
        """
        Detect lanes in image using Ultra-Fast-Lane-Detection-v2.
        
        Args:
            image: Input image (H, W, 3) RGB, uint8
            use_post_processing: Enable post-processing (noise reduction, lane separation)
            
        Returns:
            lane_mask: Binary lane mask (H, W)
            lane_features: Lane feature vector (128,)
            lane_coords: List of lane coordinate lists (for post-processing)
        """
        if self.model is None:
            logger.warning("Model not loaded. Using fallback detection.")
            mask, features = self._fallback_detection(image)
            return mask, features, []
        
        try:
            original_h, original_w = image.shape[:2]
            
            # Preprocess image (matching demo.py)
            # 1. Resize: (int(train_height / crop_ratio), train_width)
            # For TuSimple: crop_ratio=0.8, so resize to (400, 800)
            # For CULane: crop_ratio=0.6, so resize to (533, 1600)
            crop_ratio = 0.8 if self.dataset == 'tusimple' else 0.6
            resize_h = int(self.train_height / crop_ratio)
            resize_w = self.train_width
            
            img_resized = cv2.resize(image, (resize_w, resize_h), interpolation=cv2.INTER_LINEAR)
            
            # 2. Crop from bottom (crop_size = train_height)
            # Crop top part, keep bottom train_height pixels
            crop_start = resize_h - self.train_height
            img_cropped = img_resized[crop_start:crop_start + self.train_height, :, :]
            
            # 3. Convert to tensor and normalize
            img_tensor = torch.FloatTensor(img_cropped).permute(2, 0, 1).unsqueeze(0) / 255.0
            img_tensor = img_tensor.to(self.device)
            
            # Normalize (ImageNet stats) - same as demo.py
            mean = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1).to(self.device)
            std = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1).to(self.device)
            img_tensor = (img_tensor - mean) / std
            
            # Inference
            with torch.no_grad():
                output = self.model(img_tensor)
            
            # Convert output to lane mask and coordinates
            # Scale coordinates back to original image size (CARLA image size)
            lane_mask, lane_coords = self._output_to_mask_and_coords(output, original_h, original_w)
            
            # Scale lane coordinates from model output size to original image size
            # Model outputs coordinates in training image size, need to scale to CARLA image size
            if len(lane_coords) > 0:
                scale_x = original_w / self.train_width
                scale_y = original_h / self.train_height
                
                scaled_lane_coords = []
                for lane in lane_coords:
                    scaled_lane = [(int(x * scale_x), int(y * scale_y)) for x, y in lane]
                    # Filter out-of-bounds points
                    scaled_lane = [(x, y) for x, y in scaled_lane if 0 <= x < original_w and 0 <= y < original_h]
                    if len(scaled_lane) >= 3:  # Keep only lanes with at least 3 valid points
                        scaled_lane_coords.append(scaled_lane)
                
                lane_coords = scaled_lane_coords
            
            # Post-processing (noise reduction, lane separation) - NO NUMBERING
            # Removed numbering to avoid overlapping issues
            if use_post_processing:
                try:
                    if self._post_processor is None or self._post_proc_size != (original_h, original_w):
                        from perception.lane_post_processor import LanePostProcessor
                        self._post_processor = LanePostProcessor(
                            image_height=original_h,
                            image_width=original_w,
                            polynomial_order=2,
                            temporal_smoothing=True,
                            enable_numbering=False
                        )
                        self._post_proc_size = (original_h, original_w)
                    cleaned_mask, lane_polynomials, processed_lanes = self._post_processor.process(
                        lane_mask, lane_coords
                    )
                    
                    # Use cleaned mask
                    lane_mask = cleaned_mask
                    
                    # Update lane_coords from processed lanes (convert polynomials back to points)
                    # processed_lanes is now a list, not numbered dict
                    lane_coords = self._polynomials_to_coords_list(lane_polynomials, original_h, original_w)
                    
                    logger.debug(f"Post-processing: {len(lane_polynomials)} lanes detected (no numbering)")
                except Exception as e:
                    logger.warning(f"Post-processing failed: {e}, using raw detection")
            
            # Extract features
            lane_features = self._extract_lane_features(lane_mask)
            
            return lane_mask, lane_features, lane_coords
            
        except Exception as e:
            logger.error(f"Lane detection failed: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            mask, features = self._fallback_detection(image)
            return mask, features, []
    
    def _polynomials_to_coords(
        self,
        numbered_lanes: Dict[int, np.ndarray],
        target_h: int,
        target_w: int
    ) -> List[List[Tuple[int, int]]]:
        """
        Convert polynomial coefficients back to coordinate lists.
        
        Args:
            numbered_lanes: Dict mapping lane_id to polynomial coefficients
            target_h: Target height
            target_w: Target width
            
        Returns:
            List of lane coordinate lists
        """
        lane_coords = []
        
        for lane_id in sorted(numbered_lanes.keys()):
            poly_coeffs = numbered_lanes[lane_id]
            
            # Generate points from polynomial
            y_coords = np.arange(0, target_h, 5)  # Sample every 5 rows
            x_coords = np.polyval(poly_coeffs, y_coords).astype(int)
            
            # Filter valid points
            valid_points = []
            for x, y in zip(x_coords, y_coords):
                if 0 <= x < target_w and 0 <= y < target_h:
                    valid_points.append((int(x), int(y)))
            
            if len(valid_points) >= 2:
                lane_coords.append(valid_points)
        
        return lane_coords
    
    def _polynomials_to_coords_list(
        self,
        lane_polynomials: List[np.ndarray],
        target_h: int,
        target_w: int
    ) -> List[List[Tuple[int, int]]]:
        """
        Convert polynomial coefficients list to coordinate lists (no numbering).
        
        Args:
            lane_polynomials: List of polynomial coefficients (no IDs)
            target_h: Target height
            target_w: Target width
            
        Returns:
            List of lane coordinate lists
        """
        lane_coords = []
        
        for poly_coeffs in lane_polynomials:
            if poly_coeffs is None:
                continue
            
            # Generate points from polynomial
            y_coords = np.arange(0, target_h, 5)  # Sample every 5 rows
            x_coords = np.polyval(poly_coeffs, y_coords).astype(int)
            
            # Filter valid points
            valid_points = []
            for x, y in zip(x_coords, y_coords):
                if 0 <= x < target_w and 0 <= y < target_h:
                    valid_points.append((int(x), int(y)))
            
            if len(valid_points) >= 2:
                lane_coords.append(valid_points)
        
        return lane_coords
    
    def _pred2coords(
        self,
        pred: dict,
        original_image_width: int,
        original_image_height: int,
        local_width: int = 1
    ) -> List[List[Tuple[int, int]]]:
        """
        Convert model prediction to lane coordinates.
        ใช้ logic จาก repository โดยตรง:
        - TuSimple: ใช้ generate_tusimple_lines จาก eval_wrapper.py
        - CULane: ใช้ generate_lines_local จาก eval_wrapper.py
        - Demo: ใช้ pred2coords จาก demo.py
        
        Args:
            pred: Model output dict
            original_image_width: Original image width
            original_image_height: Original image height
            local_width: Local width for softmax (default: 1)
            
        Returns:
            List of lane coordinate lists (เป็นจุดๆ แม่นๆ)
        """
        batch_size, num_grid_row, num_cls_row, num_lane_row = pred['loc_row'].shape
        batch_size, num_grid_col, num_cls_col, num_lane_col = pred['loc_col'].shape

        # ใช้วิธีเดียวกับ generate_lines_local ใน eval_wrapper.py
        max_indices_row = pred['loc_row'].argmax(1).cpu()
        valid_row = pred['exist_row'].argmax(1).cpu()

        max_indices_col = pred['loc_col'].argmax(1).cpu()
        valid_col = pred['exist_col'].argmax(1).cpu()

        pred['loc_row'] = pred['loc_row'].cpu()
        pred['loc_col'] = pred['loc_col'].cpu()

        coords = []

        # ใช้ logic จาก repository โดยตรง (ไม่ต้อง import - ใช้ logic ที่เขียนเองแต่ตรงกับ repository)
        # Note: eval_wrapper.py ต้องการ ujson ซึ่งอาจไม่มีในระบบ
        # ดังนั้นใช้ logic ที่เขียนเองแต่ตรงกับ repository แทน (จาก eval_wrapper.py และ demo.py)
        
        if self.dataset == 'tusimple':
            # TuSimple: ใช้ logic จาก generate_tusimple_lines (eval_wrapper.py line 677)
            # Mode: '4row' = detect all 4 row lanes
            row_num_grid, row_num_cls, row_num_lane = pred['loc_row'][0].shape
            col_num_grid, col_num_cls, col_num_lane = pred['loc_col'][0].shape
            
            row_max_indices = pred['loc_row'][0].argmax(0).cpu()
            row_valid = pred['exist_row'][0].argmax(0).cpu()
            row_out = pred['loc_row'][0].cpu()
            
            col_max_indices = pred['loc_col'][0].argmax(0).cpu()
            col_valid = pred['exist_col'][0].argmax(0).cpu()
            col_out = pred['loc_col'][0].cpu()
            
            # Parameters จาก eval_wrapper.py line 707-710
            local_width_row = 14
            local_width_col = 14
            min_lanepts_row = 3
            min_lanepts_col = 3
            
            # TuSimple h_sample (from eval_wrapper.py line 678)
            tusimple_h_sample = np.linspace(160, 710, 56)
            
            coords = []
            
            # Process row lanes (from eval_wrapper.py line 715-748)
            row_lane_list = list(range(row_num_lane))  # Mode '4row' = all lanes
            for row_lane_idx in row_lane_list:
                if row_valid[:, row_lane_idx].sum() > min_lanepts_row:
                    lane_coords = []
                    for row_cls_idx in range(row_num_cls):
                        if row_valid[row_cls_idx, row_lane_idx]:
                            all_ind = torch.tensor(list(range(
                                max(0, row_max_indices[row_cls_idx, row_lane_idx] - local_width_row),
                                min(row_num_grid-1, row_max_indices[row_cls_idx, row_lane_idx] + local_width_row) + 1
                            )))
                            coord = (row_out[all_ind, row_cls_idx, row_lane_idx].softmax(0) * all_ind.float()).sum() + 0.5
                            coord_x = coord / (row_num_grid - 1) * 1280  # TuSimple width
                            coord_y = self.row_anchor[row_cls_idx] * 720  # TuSimple height
                            
                            # Scale to original image size
                            x_scaled = int(coord_x * original_image_width / 1280)
                            y_scaled = int(coord_y * original_image_height / 720)
                            if 0 <= x_scaled < original_image_width and 0 <= y_scaled < original_image_height:
                                lane_coords.append((x_scaled, y_scaled))
                    
                    if len(lane_coords) >= 3:
                        coords.append(lane_coords)
            
            return coords
                
        elif self.dataset == 'culane':
            # CULane: ใช้ logic จาก generate_lines_local (eval_wrapper.py line 67)
            row_lane_idx = [1, 2]  # From eval_wrapper.py line 78
            col_lane_idx = [0, 3]  # From eval_wrapper.py line 126
            local_width = 1        # From eval_wrapper.py line 85
            
            coords = []
            
            # Process row lanes (from generate_lines_local line 95-111)
            for i in row_lane_idx:
                if valid_row[0,:,i].sum() > num_cls_row / 2:  # From eval_wrapper.py line 96
                    tmp = []
                    for k in range(valid_row.shape[1]):
                        if valid_row[0,k,i]:
                            all_ind = torch.tensor(list(range(
                                max(0, max_indices_row[0,k,i] - local_width),
                                min(num_grid_row-1, max_indices_row[0,k,i] + local_width) + 1
                            )))
                            out_tmp = (pred['loc_row'][0,all_ind,k,i].softmax(0) * all_ind.float()).sum() + 0.5
                            out_tmp = out_tmp / (num_grid_row-1) * 1640  # CULane width
                            x = int(out_tmp * original_image_width / 1640)
                            y = int(self.row_anchor[k] * 590 * original_image_height / 590)
                            if 0 <= x < original_image_width and 0 <= y < original_image_height:
                                tmp.append((x, y))
                    if len(tmp) >= 3:
                        coords.append(tmp)
            
            # Process col lanes (from generate_lines_col_local line 142-156)
            for i in col_lane_idx:
                if valid_col[0,:,i].sum() > num_cls_col / 4:  # From eval_wrapper.py line 143
                    tmp = []
                    for k in range(valid_col.shape[1]):
                        if valid_col[0,k,i]:
                            all_ind = torch.tensor(list(range(
                                max(0, max_indices_col[0,k,i] - local_width),
                                min(num_grid_col-1, max_indices_col[0,k,i] + local_width) + 1
                            )))
                            out_tmp = (pred['loc_col'][0,all_ind,k,i].softmax(0) * all_ind.float()).sum() + 0.5
                            out_tmp = out_tmp / (num_grid_col-1) * 590  # CULane height
                            x = int(self.col_anchor[k] * 1640 * original_image_width / 1640)
                            y = int(out_tmp * original_image_height / 590)
                            if 0 <= x < original_image_width and 0 <= y < original_image_height:
                                tmp.append((x, y))
                    if len(tmp) >= 3:
                        coords.append(tmp)
            
            return coords
        else:
            # Unknown dataset - return empty
            logger.warning(f"Unknown dataset: {self.dataset}, returning empty coords")
            return []
    def _output_to_mask_and_coords(
        self,
        output: dict,
        target_h: int,
        target_w: int
    ) -> Tuple[np.ndarray, List[List[Tuple[int, int]]]]:
        """
        Convert Ultra-Fast-Lane-Detection-v2 output to binary lane mask and coordinates.
        Uses pred2coords from demo.py for accurate coordinate extraction.
        
        Args:
            output: Model output dict with 'loc_row', 'exist_row', etc.
            target_h: Target mask height
            target_w: Target mask width
            
        Returns:
            Tuple of:
            - Binary lane mask (target_h, target_w)
            - List of lane coordinate lists
        """
        mask = np.zeros((target_h, target_w), dtype=np.uint8)
        
        try:
            # Use pred2coords (from demo.py) for accurate coordinate extraction
            lane_coords = self._pred2coords(output, target_w, target_h)
            
            # Draw lanes on mask
            for lane in lane_coords:
                if len(lane) > 1:
                    # Sort by y coordinate
                    lane_sorted = sorted(lane, key=lambda p: p[1])
                    
                    # Draw lines connecting points
                    for i in range(len(lane_sorted) - 1):
                        pt1 = lane_sorted[i]
                        pt2 = lane_sorted[i + 1]
                        cv2.line(mask, pt1, pt2, 255, 3)
                    
                    # Draw circles at points
                    for pt in lane_sorted:
                        cv2.circle(mask, pt, 3, 255, -1)
                elif len(lane) == 1:
                    # Single point
                    cv2.circle(mask, lane[0], 3, 255, -1)
            
            # Apply morphological operations to clean up
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
        except Exception as e:
            logger.warning(f"Error converting output to mask: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            lane_coords = []
        
        return mask, lane_coords
    
    def _output_to_mask(self, output: dict, target_h: int, target_w: int) -> np.ndarray:
        """
        Legacy method: Convert output to mask only (for backward compatibility).
        """
        mask, _ = self._output_to_mask_and_coords(output, target_h, target_w)
        return mask
    
    def _extract_lane_features(self, lane_mask: np.ndarray) -> np.ndarray:
        """
        Extract lane features from mask (same as LaneDetector).
        
        Args:
            lane_mask: Binary lane mask (H, W)
            
        Returns:
            Feature vector (128,)
        """
        h, w = lane_mask.shape
        
        features = []
        
        # Lane position
        lane_pixels = np.where(lane_mask > 0)
        if len(lane_pixels[0]) > 0:
            center_x = np.mean(lane_pixels[1])
            center_y = np.mean(lane_pixels[0])
            features.extend([center_x / w, center_y / h])
        else:
            features.extend([0.5, 0.5])
        
        # Lane width
        if len(lane_pixels[0]) > 0:
            width = np.std(lane_pixels[1]) * 2
            features.append(width / w)
        else:
            features.append(0.0)
        
        # Lane curvature
        if len(lane_pixels[0]) > 10:
            y_coords = lane_pixels[0]
            x_coords = lane_pixels[1]
            if len(np.unique(y_coords)) > 3:
                coeffs = np.polyfit(y_coords, x_coords, 2)
                features.extend([coeffs[0], coeffs[1], coeffs[2]])
            else:
                features.extend([0.0, 0.0, 0.0])
        else:
            features.extend([0.0, 0.0, 0.0])
        
        # Lane density
        regions = [
            (0, h//3, 0, w),
            (h//3, 2*h//3, 0, w),
            (2*h//3, h, 0, w),
        ]
        for y1, y2, x1, x2 in regions:
            region_mask = lane_mask[y1:y2, x1:x2]
            density = np.sum(region_mask > 0) / (region_mask.size + 1e-6)
            features.append(density)
        
        # Pad to 128 dimensions
        while len(features) < 128:
            features.append(0.0)
        
        return np.array(features[:128], dtype=np.float32)
    
    def _fallback_detection(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Fallback lane detection using simple edge detection.
        
        Args:
            image: Input image (H, W, 3)
            
        Returns:
            lane_mask: Binary lane mask
            lane_features: Feature vector
        """
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        h, w = image.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[h//2:, :] = edges[h//2:, :]
        features = self._extract_lane_features(mask)
        return mask, features


def download_pretrained_model(dataset: str = "culane", backbone: str = "res18", 
                             output_dir: str = "weights") -> str:
    """
    Download pre-trained Ultra-Fast-Lane-Detection-v2 model.
    
    Args:
        dataset: Dataset name ("culane", "tusimple", "curvelanes")
        backbone: Backbone architecture ("res18", "res34")
        output_dir: Directory to save model
        
    Returns:
        Path to downloaded model file
    """
    import urllib.request
    import os
    
    # Model URLs from Ultra-Fast-Lane-Detection-v2 repository
    model_urls = {
        ("culane", "res18"): "https://drive.google.com/uc?export=download&id=1oEjJraFr-3lxhX_OXduAGFWalWa6Xh3W",
        ("culane", "res34"): "https://drive.google.com/uc?export=download&id=1AjnvAD3qmqt_dGPveZJsLZ1bOyWv62Yj",
        ("tusimple", "res18"): "https://drive.google.com/uc?export=download&id=1Clnj9-dLz81S3wXiYtlkc4HVusCb978t",
        ("tusimple", "res34"): "https://drive.google.com/uc?export=download&id=1pkz8homK433z39uStGK3ZWkDXrnBAMmX",
        ("curvelanes", "res18"): "https://drive.google.com/uc?export=download&id=1VfbUvorKKMG4tUePNbLYPp63axgd-8BX",
        ("curvelanes", "res34"): "https://drive.google.com/uc?export=download&id=1O1kPSr85Icl2JbwV3RBlxWZYhLEHo8EN",
    }
    
    key = (dataset, backbone)
    if key not in model_urls:
        raise ValueError(f"Unknown dataset/backbone combination: {dataset}/{backbone}")
    
    url = model_urls[key]
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    model_filename = f"ufldv2_{dataset}_{backbone}.pth"
    model_path = output_dir / model_filename
    
    if model_path.exists():
        logger.info(f"Model already exists: {model_path}")
        return str(model_path)
    
    logger.info(f"Downloading {model_filename} from Ultra-Fast-Lane-Detection-v2...")
    logger.warning("Google Drive links require manual download. "
                 "Please download from: https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2")
    
    # Note: Google Drive direct download is complex
    # User should download manually and place in weights/ directory
    logger.info(f"Please download the model manually and place it at: {model_path}")
    
    return str(model_path)

import numpy as np
import cv2
from utils.sp_tensorrt import SuperPointNet_TensorRT
from time import time
from numba import jit, int32, float32, prange

@jit(nopython=True, cache=True)
def nms_fast_numba(in_corners, H, W, dist_thresh):
    """
    Fast Non-Maximum Suppression (NMS) using numba for acceleration.
    Inputs:
      in_corners - 3xN numpy array with corners [x_i, y_i, confidence_i]^T.
      H - Image height.
      W - Image width.
      dist_thresh - Distance to suppress, measured as an infinity norm distance.
    Returns:
      nmsed_corners - 3xN numpy matrix with surviving corners.
      nmsed_inds - N length numpy vector with surviving corner indices.
    """
    # Initialize grid and indices
    grid = np.zeros((H, W), dtype=int32)  # Track NMS data
    inds = np.zeros((H, W), dtype=int32)  # Store indices of points
    
    # Sort by confidence and round to nearest int
    inds1 = np.argsort(-in_corners[2, :])
    corners = in_corners[:, inds1].astype(np.float32)
    rcorners = np.rint(corners[:2, :]).astype(int32)  # Rounded corners using np.rint
    
    # Check for edge case of 0 or 1 corners
    if rcorners.shape[1] == 0:
        return np.zeros((3, 0), dtype=np.float32), np.zeros(0, dtype=int32)
    if rcorners.shape[1] == 1:
        # Manually construct the output instead of np.vstack
        out = np.zeros((3, 1), dtype=np.float32)
        out[:2, 0] = rcorners[:, 0].astype(np.float32)  # Ensure type consistency
        out[2, 0] = float32(in_corners[2, inds1[0]])  # Explicit type conversion
        return out, np.zeros((1), dtype=int32)
    
    # Initialize the grid
    for i in range(rcorners.shape[1]):
        x, y = rcorners[0, i], rcorners[1, i]
        grid[y, x] = 1
        inds[y, x] = i

    # Pad the grid to suppress points near borders
    pad = dist_thresh
    padded_grid = np.zeros((H + 2 * pad, W + 2 * pad), dtype=int32)
    padded_grid[pad:pad + H, pad:pad + W] = grid
    grid = padded_grid

    # Iterate through points, suppress neighborhood
    count = 0
    for i in range(rcorners.shape[1]):
        x, y = rcorners[0, i], rcorners[1, i]
        pt_x, pt_y = x + pad, y + pad
        if grid[pt_y, pt_x] == 1:  # If not yet suppressed
            grid[pt_y - pad:pt_y + pad + 1, pt_x - pad:pt_x + pad + 1] = 0
            grid[pt_y, pt_x] = -1
            count += 1
    
    # Get all surviving -1's
    keepy, keepx = np.where(grid == -1)
    keepy, keepx = keepy - pad, keepx - pad

    # Replace multi-array indexing with loop-based indexing
    inds_keep = np.zeros(keepy.shape[0], dtype=int32)
    for i in range(keepy.shape[0]):
        inds_keep[i] = inds[keepy[i], keepx[i]]

    # Prepare the output
    out = corners[:, inds_keep]
    values = out[-1, :]
    inds2 = np.argsort(-values)
    out = out[:, inds2]
    out_inds = inds1[inds_keep[inds2]].astype(int32)
    return out.astype(np.float32), out_inds

@jit(nopython=True, cache=True, parallel=True, fastmath=True)
def numba_grid_sample(input_array, grid, align_corners=True):
    """
    Numba implementation of torch.nn.functional.grid_sample.

    Parameters:
        input_array (ndarray): Input array of shape (N, C, H, W).
        grid (ndarray): Grid array of shape (N, H_out, W_out, 2).
        mode (str): Interpolation mode, 'bilinear' or 'nearest'.
        padding_mode (str): Padding mode, 'zeros', 'border', or 'reflection'.
        align_corners (bool): Whether to align corners.

    Returns:
        ndarray: Output sampled array of shape (N, C, H_out, W_out).
    """
    N, C, H, W = input_array.shape
    _, H_out, W_out, _ = grid.shape

    # Precompute scaling factors
    x_scale = (W - 1) / 2 if align_corners else W / 2
    y_scale = (H - 1) / 2 if align_corners else H / 2

    output = np.zeros((N, C, H_out, W_out), dtype=input_array.dtype)

    for n in prange(N):  # Parallelize batch dimension
        for h in range(H_out):
            for w in range(W_out):
                # Normalize grid to input array coordinates
                gx, gy = grid[n, h, w, 0], grid[n, h, w, 1]
                x = gx * x_scale + x_scale
                y = gy * y_scale + y_scale

                # Calculate indices for bilinear interpolation
                x0 = int(np.floor(x))
                x1 = x0 + 1
                y0 = int(np.floor(y))
                y1 = y0 + 1

                # Compute weights
                wx0, wx1 = x1 - x, x - x0
                wy0, wy1 = y1 - y, y - y0

                # Clip indices for border padding
                x0 = max(0, min(W - 1, x0))
                x1 = max(0, min(W - 1, x1))
                y0 = max(0, min(H - 1, y0))
                y1 = max(0, min(H - 1, y1))

                # Perform bilinear interpolation
                for c in range(C):
                    v00 = input_array[n, c, y0, x0]
                    v01 = input_array[n, c, y0, x1]
                    v10 = input_array[n, c, y1, x0]
                    v11 = input_array[n, c, y1, x1]

                    output[n, c, h, w] = (
                        wx0 * wy0 * v00 +
                        wx0 * wy1 * v10 +
                        wx1 * wy0 * v01 +
                        wx1 * wy1 * v11
                    )

    return output
#批量为1，grid的H为1的优化版本
@jit(nopython=True, cache=True, fastmath=True)
def numba_grid_sample_optimized(input_array, grid, align_corners):
    """
    Optimized Numba implementation of grid_sample.

    Parameters:
        input_array (ndarray): Input array of shape (1, C, H, W).
        grid (ndarray): Grid array of shape (1, 1, W_out, 2).
        align_corners (bool): Whether to align corners.

    Returns:
        ndarray: Output sampled array of shape (N, C, H_out, W_out).
    """

    N, C, H, W = input_array.shape
    _, H_out, W_out, _ = grid.shape

    # Precompute scaling factors
    x_scale = (W - 1) / 2 if align_corners else W / 2
    y_scale = (H - 1) / 2 if align_corners else H / 2

    output = np.zeros((N, C, H_out, W_out), dtype=input_array.dtype)

    for w in range(W_out):
        # Normalize grid to input array coordinates
        gx, gy = grid[0, 0, w, 0], grid[0, 0, w, 1]
        x = gx * x_scale + x_scale
        y = gy * y_scale + y_scale

        # Calculate indices for bilinear interpolation
        x0 = int(np.floor(x))
        x1 = x0 + 1
        y0 = int(np.floor(y))
        y1 = y0 + 1

        # Compute weights
        wx0, wx1 = x1 - x, x - x0
        wy0, wy1 = y1 - y, y - y0

        # Clip indices for border padding
        x0 = max(0, min(W - 1, x0))
        x1 = max(0, min(W - 1, x1))
        y0 = max(0, min(H - 1, y0))
        y1 = max(0, min(H - 1, y1))

        # Perform bilinear interpolation
        for c in range(C):
            v00 = input_array[0, c, y0, x0]
            v01 = input_array[0, c, y0, x1]
            v10 = input_array[0, c, y1, x0]
            v11 = input_array[0, c, y1, x1]

            output[0, c, 0, w] = (
                wx0 * wy0 * v00 +
                wx0 * wy1 * v10 +
                wx1 * wy0 * v01 +
                wx1 * wy1 * v11
            )

    return output

#tensorrt模型
class SuperPointFrontend_TensorRT(object):
  def __init__(self, engine_path, nms_dist, conf_thresh):
    self.nms_dist = nms_dist
    self.conf_thresh = conf_thresh
    self.cell = 8 # Size of each output cell. Keep this fixed.
    self.border_remove = 4 # Remove points this close to the border.
    self.trt_model = SuperPointNet_TensorRT(engine_path)

  # 异步线程中调用初始化
  def async_init(self):
    #load the trt engine
    self.trt_model.engine_init()
    print('==> Load tensorRT engine Successfully.')
    #precompile for numba-func
    # dummy_pts = np.zeros((3,1), dtype=np.float32)
    # H = 240
    # W = 320
    # nms_fast_numba(dummy_pts, H, W, dist_thresh=self.nms_dist)
    # dummy_desc = np.zeros((1, 256, 30, 40), dtype=np.float32)
    # dummy_samp_pts = np.zeros((1, 1, 10, 2), dtype=np.float32)
    # numba_grid_sample_optimized(dummy_desc, dummy_samp_pts, align_corners=True)

    # print("==> Precompile for numba function Successfully.")
     
  def nms_fast(self, in_corners, H, W, dist_thresh):
    """
    NOTE: The NMS first rounds points to integers, so NMS distance might not
    be exactly dist_thresh. It also assumes points are within image boundaries.
  
    Inputs
      in_corners - 3xN numpy array with corners [x_i, y_i, confidence_i]^T.
      H - Image height.
      W - Image width.
      dist_thresh - Distance to suppress, measured as an infinty norm distance.
    Returns
      nmsed_corners - 3xN numpy matrix with surviving corners.
      nmsed_inds - N length numpy vector with surviving corner indices.
    """
    grid = np.zeros((H, W)).astype(int) # Track NMS data.
    inds = np.zeros((H, W)).astype(int) # Store indices of points.
    # Sort by confidence and round to nearest int.
    inds1 = np.argsort(-in_corners[2,:])
    corners = in_corners[:,inds1]
    rcorners = corners[:2,:].round().astype(int) # Rounded corners.
    # Check for edge case of 0 or 1 corners.
    if rcorners.shape[1] == 0:
      return np.zeros((3,0)).astype(int), np.zeros(0).astype(int)
    if rcorners.shape[1] == 1:
      out = np.vstack((rcorners, in_corners[2])).reshape(3,1)
      return out, np.zeros((1)).astype(int)
    # Initialize the grid.
    for i, rc in enumerate(rcorners.T):
      grid[rcorners[1,i], rcorners[0,i]] = 1
      inds[rcorners[1,i], rcorners[0,i]] = i
    # Pad the border of the grid, so that we can NMS points near the border.
    pad = dist_thresh
    grid = np.pad(grid, ((pad,pad), (pad,pad)), mode='constant')
    # Iterate through points, highest to lowest conf, suppress neighborhood.
    count = 0
    for i, rc in enumerate(rcorners.T):
      # Account for top and left padding.
      pt = (rc[0]+pad, rc[1]+pad)
      if grid[pt[1], pt[0]] == 1: # If not yet suppressed.
        grid[pt[1]-pad:pt[1]+pad+1, pt[0]-pad:pt[0]+pad+1] = 0
        grid[pt[1], pt[0]] = -1
        count += 1
    # Get all surviving -1's and return sorted array of remaining corners.
    keepy, keepx = np.where(grid==-1)
    keepy, keepx = keepy - pad, keepx - pad
    inds_keep = inds[keepy, keepx]
    out = corners[:, inds_keep]
    values = out[-1, :]
    inds2 = np.argsort(-values)
    out = out[:, inds2]
    out_inds = inds1[inds_keep[inds2]]
    return out, out_inds
  
  def numpy_grid_sample(self, input_array, grid, mode='bilinear', padding_mode='zeros', align_corners=True):
    """
    NumPy implementation of torch.nn.functional.grid_sample.

    Parameters:
        input_array (ndarray): Input array of shape (N, C, H, W).
        grid (ndarray): Grid array of shape (N, H_out, W_out, 2).
        mode (str): Interpolation mode, 'bilinear' or 'nearest'.
        padding_mode (str): Padding mode, 'zeros', 'border', or 'reflection'.
        align_corners (bool): Whether to align corners.

    Returns:
        ndarray: Output sampled array of shape (N, C, H_out, W_out).
    """
    N, C, H, W = input_array.shape
    _, H_out, W_out, _ = grid.shape

    # Normalize grid to [-1, 1]
    if align_corners:
        x = grid[..., 0] * (W - 1) / 2 + (W - 1) / 2
        y = grid[..., 1] * (H - 1) / 2 + (H - 1) / 2
    else:
        x = ((grid[..., 0] + 1) * W - 1) / 2
        y = ((grid[..., 1] + 1) * H - 1) / 2

    # Clip coordinates for border mode
    if padding_mode == 'border':
        x = np.clip(x, 0, W - 1)
        y = np.clip(y, 0, H - 1)

    # Reflection padding
    if padding_mode == 'reflection':
        x = np.abs((x + W) % (2 * W) - W)
        y = np.abs((y + H) % (2 * H) - H)

    # Padding with zeros (default)
    x0 = np.floor(x).astype(np.int32)
    x1 = x0 + 1
    y0 = np.floor(y).astype(np.int32)
    y1 = y0 + 1

    # Clipping for zero padding
    x0 = np.clip(x0, 0, W - 1)
    x1 = np.clip(x1, 0, W - 1)
    y0 = np.clip(y0, 0, H - 1)
    y1 = np.clip(y1, 0, H - 1)

    # Interpolation weights
    if mode == 'bilinear':
        wx0 = x1 - x
        wx1 = x - x0
        wy0 = y1 - y
        wy1 = y - y0
    else:  # 'nearest'
        x0 = np.round(x).astype(np.int32)
        y0 = np.round(y).astype(np.int32)
        wx0, wx1, wy0, wy1 = 1, 0, 1, 0

    # Gather pixel values
    output = np.zeros((N, C, H_out, W_out), dtype=input_array.dtype)
    for n in range(N):
        for c in range(C):
            v00 = input_array[n, c, y0[n], x0[n]]
            v01 = input_array[n, c, y0[n], x1[n]]
            v10 = input_array[n, c, y1[n], x0[n]]
            v11 = input_array[n, c, y1[n], x1[n]]

            output[n, c] = (
                wx0 * wy0 * v00 +
                wx0 * wy1 * v10 +
                wx1 * wy0 * v01 +
                wx1 * wy1 * v11
            )

    return output
  
  def run(self, img, conf_thresh=0.015):
    """ Process a numpy image to extract points and descriptors.
    Input
      img - HxW numpy float32 input image in range [0,1].
    Output
      corners - 3xN numpy array with corners [x_i, y_i, confidence_i]^T.
      desc - 256xN numpy array of corresponding unit normalized descriptors.
      heatmap - HxW numpy heatmap in range [0,1] of point confidences.
      """
    assert img.ndim == 2, 'Image must be grayscale.'
    assert img.dtype == np.float32, 'Image must be float32.'
    H = img.shape[0]
    W = img.shape[1]
    # Forward pass of network.
    start_time = time()
    semi, coarse_desc = self.trt_model.infer(img)
    assert coarse_desc.shape[1] == 256, 'coarse desc dim must be 256.'
    print("trt infer time is {}ms:".format((time() - start_time)*1000.))

    # --- Process points.
    semi = np.squeeze(semi)
    dense = np.exp(semi) # Softmax.
    dense = dense / (np.sum(dense, axis=0)+.00001) # Should sum to 1.
    # Remove dustbin.
    nodust = dense[:-1, :, :]
    # Reshape to get full resolution heatmap.
    Hc = int(H / self.cell)
    Wc = int(W / self.cell)
    nodust = nodust.transpose(1, 2, 0)
    heatmap = np.reshape(nodust, [Hc, Wc, self.cell, self.cell])
    heatmap = np.transpose(heatmap, [0, 2, 1, 3])
    heatmap = np.reshape(heatmap, [Hc*self.cell, Wc*self.cell])
   
    xs, ys = np.where(heatmap >= conf_thresh) # Confidence threshold.
    if len(xs) == 0:
      return np.zeros((3, 0)), None, None
    pts = np.zeros((3, len(xs)),dtype=np.float32) # Populate point data sized 3xN.
    pts[0, :] = ys
    pts[1, :] = xs
    pts[2, :] = heatmap[xs, ys]
    start_time = time()
    pts, _ = nms_fast_numba(pts, H, W, dist_thresh=self.nms_dist) # Apply NMS.
    print("nms_fast time is {}ms:".format((time() - start_time)*1000.))

    inds = np.argsort(pts[2,:])
    pts = pts[:,inds[::-1]] # Sort by confidence.
    # Remove points along border.
    bord = self.border_remove
    toremoveW = np.logical_or(pts[0, :] < bord, pts[0, :] >= (W-bord))
    toremoveH = np.logical_or(pts[1, :] < bord, pts[1, :] >= (H-bord))
    toremove = np.logical_or(toremoveW, toremoveH)
    pts = pts[:, ~toremove]

    # ??
    # if conf_thresh == 0.015 and pts.shape[1] < 100:
    #   return pts, _, _

    # --- Process descriptor.
    D = coarse_desc.shape[1]
    if pts.shape[1] == 0:
      desc = np.zeros((D, 0))
    else:
      # Interpolate into descriptor map using 2D point locations.
      samp_pts = pts[:2, :].copy()
      samp_pts[0, :] = (samp_pts[0, :] / (float(W) / 2.)) - 1.
      samp_pts[1, :] = (samp_pts[1, :] / (float(H) / 2.)) - 1.
      samp_pts = samp_pts.T
      samp_pts = np.expand_dims(samp_pts, axis=0)
      samp_pts = np.expand_dims(samp_pts, axis=0)
      samp_pts = samp_pts.astype(np.float32)
      start_time = time()
      #print("coarse_desc's shape is {}, type is {}, samp_pts's shape is {}, type is {}".format(coarse_desc.shape, coarse_desc.dtype, samp_pts.shape, samp_pts.dtype))
      #desc = self.numpy_grid_sample(coarse_desc, samp_pts) #(1, 256, 1, -1)
      #desc = numba_grid_sample(coarse_desc, samp_pts)
      desc = numba_grid_sample_optimized(coarse_desc, samp_pts, align_corners=True)
      print("grid_sample time is {}ms:".format((time() - start_time)*1000.))
      desc = desc.reshape(D, -1)
      desc /= np.linalg.norm(desc, axis=0)[np.newaxis, :]

      # print('#'*10+" keypoint number "+'#'*10)
      # print(pts.shape)
      # print(desc.shape)
    return pts, desc, heatmap
  
class PointTracker(object):
  """ Class to manage a fixed memory of points and descriptors that enables
  sparse optical flow point tracking.

  Internally, the tracker stores a 'tracks' matrix sized M x (2+L), of M
  tracks with maximum length L, where each row corresponds to:
  row_m = [track_id_m, avg_desc_score_m, point_id_0_m, ..., point_id_L-1_m].
  """

  def __init__(self, nn_thresh):

    self.nn_thresh = nn_thresh
    self.last_desc = None

  def nn_match_two_way(self, desc1, desc2, nn_thresh=0.7):
    """
    Performs two-way nearest neighbor matching of two sets of descriptors, such
    that the NN match from descriptor A->B must equal the NN match from B->A.

    Inputs:
      desc1 - NxM numpy matrix of N corresponding M-dimensional descriptors.
      desc2 - NxM numpy matrix of N corresponding M-dimensional descriptors.
      nn_thresh - Optional descriptor distance below which is a good match.

    Returns:
      matches - 3xL numpy array, of L matches, where L <= N and each column i is
                a match of two descriptors, d_i in image 1 and d_j' in image 2:
                [d_i index, d_j' index, match_score]^T
    """
    assert desc1.shape[0] == desc2.shape[0]
    if desc1.shape[1] == 0 or desc2.shape[1] == 0:
      return np.zeros((3, 0))
    if nn_thresh < 0.0:
      raise ValueError('\'nn_thresh\' should be non-negative')
    # Compute L2 distance. Easy since vectors are unit normalized.
    dmat = np.dot(desc1.T, desc2)
    dmat = np.sqrt(2-2*np.clip(dmat, -1, 1))
    # Get NN indices and scores.
    idx = np.argmin(dmat, axis=1)
    scores = dmat[np.arange(dmat.shape[0]), idx]
    # Threshold the NN matches.
    keep = scores < nn_thresh
    # Check if nearest neighbor goes both directions and keep those.
    idx2 = np.argmin(dmat, axis=0)
    keep_bi = np.arange(len(idx)) == idx2[idx]
    keep = np.logical_and(keep, keep_bi)
    idx = idx[keep]
    scores = scores[keep]
    # Get the surviving point indices.
    m_idx1 = np.arange(desc1.shape[1])[keep]
    m_idx2 = idx
    # Populate the final 3xN match data structure.
    matches = np.zeros((3, int(keep.sum())))
    matches[0, :] = m_idx1
    matches[1, :] = m_idx2
    matches[2, :] = scores

    # print('*'*10 + " matches number " + '*'*10)
    # print(matches.shape)

    # m_kp1 = np.array([cv2.KeyPoint(kp1[0, idx], kp1[1, idx], 1).pt for idx in m_idx1], dtype=np.float32)
    # m_kp2 = np.array([cv2.KeyPoint(kp2[0, idx], kp2[1, idx], 1).pt for idx in m_idx2], dtype=np.float32)


    # # Estimate the homography between the matches using RANSAC
    # _, inliers = cv2.findHomography(m_kp1, m_kp2, cv2.RANSAC)
    # inliers = inliers.flatten()

    # good_matches = np.zeros((3, 0))

    # for k in range(len(inliers)):
    #   if inliers[k] == 1 :
    #     good_matches = np.append(good_matches, matches[:,k:k+1], axis=1)
    
    # print('*'*10 + " good matches number " + '*'*10)
    # print(good_matches.shape)

    return matches

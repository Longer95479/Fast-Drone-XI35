trtexec \
--onnx=plnet_s1.onnx \
--minShapes=juncs_pred:1x2,lines_pred:1x4,idx_lines_for_junctions:1x2,inverse:1x1,iskeep_index:1x1,loi_features:1x16x16x16,loi_features_thin:1x4x16x16,loi_features_aux:1x4x16x16 \
--optShapes=juncs_pred:250x2,lines_pred:20000x4,idx_lines_for_junctions:20000x2,inverse:20000x1,iskeep_index:20000x1,loi_features:1x128x128x128,loi_features_thin:1x4x128x128,loi_features_aux:1x4x128x128 \
--maxShapes=juncs_pred:500x2,lines_pred:50000x4,idx_lines_for_junctions:50000x2,inverse:50000x1,iskeep_index:50000x1,loi_features:1x512x512x512,loi_features_thin:1x4x512x512,loi_features_aux:1x4x512x512 \
--workspace=4096 \
--fp16 \
--saveEngine=plnet_s1.engine
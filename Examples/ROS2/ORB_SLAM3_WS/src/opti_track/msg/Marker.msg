int32 id # Unique identifier:
geometry_msgs/Point32 position  # marker position
int32 state # host defined parameters.  Bit values:
                                                # 0 : Occluded
                                                # 1 : PointCloudSolved
                                                # 2 : ModelFilled
                                                # 3 : HasModel
                                                # 4 : Unlabeled
                                                # 5 : Active
                                                # 6 : Established
                                                # 7 : Measurement
float32 residual #marker error residual, in mm/ray
int32 id                             # RigidBody identifier: 
                                       # For rigid body assets, this is the Streaming ID value. 
                                       # For skeleton assets, this combines both skeleton ID (High-bit) and Bone ID (Low-bit).
geometry_msgs/Pose pose             # marker position
float32 mean_error                  # Mean measure-to-solve deviation
int32 tracking_flag                 # Host defined tracking flags
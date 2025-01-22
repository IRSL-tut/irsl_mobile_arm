import os
exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())

class BaseArm(ru.ImportedRobotModel):
    def __init__(self, robot=None, item=True, world=False, **kwargs):
        super().__init__(robot=robot, item=item, world=world, **kwargs)
        off_ = coordinates([0, 0.13, 0], [0.5, 0.5, -0.5, -0.5])
        self.registerEndEffector('arm', ## end-effector
                                 'WRIST_Y', ## tip-link
                                 tip_link_to_eef = off_,
                                 joint_tuples = (('BASE_Y', 'shoulder-y'),
                                                 ('BASE_R', 'shoulder-r'),
                                                 ('BASE_P', 'shoulder-p'),
                                                 ('ELBOW_P', 'elbow-p'),
                                                 ('ELBOW_Y', 'elbow-y'),
                                                 ('WRIST_P', 'wrist-p'),
                                                 ('WRIST_Y', 'wrist-y'),
                                                 )
                                 )
        self.registerEndEffector('gripper', ## end-effector
                                 'GRIPPER1',
                                 joint_tuples = (('GRIPPER0', 'gripper-r'),
                                                 ('GRIPPER1', 'gripper-l'),
                                                 )
                                 )
        self.registerNamedPose('default',
                               fv(0, 0, -0.8, 1.6, 0, PI/2 - 0.8, 0,
                                  -0.3, 0.3))
        self.registerNamedPose('arm',
                               fv(0, 0, 0, PI/2, 0, 0.8, 0,
                                  -0.0, 0.0))
    def setArmPose(self):
        self.setNamedPose('arm')

    @property
    def gripper(self):
        return self.getLimb('gripper')

    def setGripper(self, angle):
        """
        Args:
            angle (float) : Set gripper angle. 0.0 means closed
        """
        self.gripper.angleVector(fv(-angle, angle))

### settings of model_file
BaseArm.model_file = f'{os.path.dirname(__file__)}/base_arm.body'

### robot_class: 
robot_class = BaseArm

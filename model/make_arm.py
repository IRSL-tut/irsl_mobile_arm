# jupyter console --kernel=choreonoid
exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())

#### making gripper shape
#scale=1.0
scale=0.14
width=0.5*scale
theta = 35/180*PI
etha  = 10/180*PI
tTheta=math.tan(theta)
thin = 0.1*scale
L = 1.3*scale
L1 = 0.45*scale
L2 = 0.2*scale
b = thin*0.5
H = (L1-b*0.5 + thin/math.cos(theta))*tTheta
A = np.array([[tTheta, 1], [math.tan(etha), 1]])
invA = np.linalg.inv(A)
q = np.array([2*L1*tTheta, math.tan(etha) * (L - L2)])
xx = invA @ q
yy = xx + thin*np.array([math.sin(theta), math.cos(theta)])
obj =mkshapes.makeExtrusion([[0, 0], [L1, L1*tTheta], xx.tolist(), [L - L2, 0], [L, 0], [L, thin], [L - L2, thin], yy.tolist(), [L1+b, H], [L1-b, H], [0, thin/math.cos(theta)], [0, 0]], [[0, -width/2, 0], [0, width/2, 0]], rawShape=True, color=[0, 1, 1])
mkshapes.exportScene('gripper_plate.scen', obj)

#### making model (graph)
lst = [
    {'node': 'arm_base', 'type': 'link', 'joint': {'type': 'root'},
     # 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.0], 'inertia': [[0.00064, 0.0, 0.0], [0.0, 0.00064, 0.0], [0.0, 0.0, 0.00064]]},
     'relative': True, 'translation': [0.0, 0.0, 0.0], 'rotation': [1.0, 0.0, 0.0, 0.0]},
    ## geom
    {'node': 'G_base', 'type': 'geom', 'geometry': {'primitive': 'box', 'args': {'x': 0.10, 'y': 0.20, 'z': 0.02, 'color': [0, 1, 1]}},
     'relative': True, 'translation': [0.0, 0.0, 0.01], 'rotation': [1.0, 0.0, 0.0, 0.0]},
    ##
    {'node': 'BASE_Y', 'type': 'link', 'joint': {'type': 'revolute_yaw', 'axis': [0.0, 0.0, 1.0]},
     # 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.0], 'inertia': [[0.00064, 0.0, 0.0], [0.0, 0.00064, 0.0], [0.0, 0.0, 0.00064]]},
     'relative': True, 'translation': [0, 0, 0.02], 'rotation': [1.0, 0.0, 0.0, 0.0]},
    {'node': 'BASE_R', 'type': 'link', 'joint': {'type': 'revolute', 'axis': [1.0, 0.0, 0.0]},
     # 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.0], 'inertia': [[0.00064, 0.0, 0.0], [0.0, 0.00064, 0.0], [0.0, 0.0, 0.00064]]},
     'relative': True, 'translation': [0.0, 0.0, 0.025], 'rotation': [1.0, 0.0, 0.0, 0.0]},
    {'node': 'BASE_P', 'type': 'link', 'joint': {'type': 'revolute', 'axis': [0.0, 1.0, 0.0]},
     # 'mass-param': {'mass': 3.0, 'COM': [0.0, 0.0, 0.0], 'inertia': [[0.00064, 0.0, 0.0], [0.0, 0.00064, 0.0], [0.0, 0.0, 0.00064]]},
     'relative': True, 'translation': [0.0, 0.0, 0.0], 'rotation': [1.0, 0.0, 0.0, 0.0]},
     ##
    {'node': 'ELBOW_P', 'type': 'link', 'joint': {'type': 'revolute', 'axis': [0.0, 1.0, 0.0]},
     # 'mass-param': {'mass': 3.0, 'COM': [0.0, 0.0, 0.3], 'inertia': [[0.03192500000000001, 0.0, 0.0], [0.0, 0.03452500000000001, 0.0], [0.0, 0.0, 0.00865]]},
     'relative': True, 'translation': [0.0, 0.0, 0.25], 'rotation': [1.0, 0.0, 0.0, 0.0]},
     {'node': 'ELBOW_Y', 'type': 'link', 'joint': {'type': 'revolute_yaw', 'axis': [0.0, 0.0, 1.0]},
      # 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.03], 'inertia': [[0.00064, 0.0, 0.0], [0.0, 0.00064, 0.0], [0.0, 0.0, 0.00064]]},
      'relative': True, 'translation': [0.0, 0.0, 0.025], 'rotation': [1.0, 0.0, 0.0, 0.0]},
    ##
     {'node': 'WRIST_P', 'type': 'link', 'joint': {'type': 'revolute', 'axis': [0.0, 1.0, 0.0]},
      # 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.27], 'inertia': [[0.00064, 0.0, 0.0], [0.0, 0.00064, 0.0], [0.0, 0.0, 0.00064]]},
      'relative': True, 'translation': [0.0, 0.0, 0.225], 'rotation': [1.0, 0.0, 0.0, 0.0]},
     {'node': 'WRIST_Y', 'type': 'link', 'joint': {'type': 'revolute_yaw', 'axis': [0.0, 0.0, 1.0]},
      # 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.03], 'inertia': [[0.00064, 0.0, 0.0], [0.0, 0.00064, 0.0], [0.0, 0.0, 0.00064]]},
      'relative': True, 'translation': [0.0, 0.0, 0.025], 'rotation': [1.0, 0.0, 0.0, 0.0]},
    ##
    [
        {'node': 'GRIPPER0', 'type': 'link', 'joint': {'type': 'revolute', 'axis': [1.0, 0.0, 0.0]},
         # 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.03], 'inertia': [[0.00064, 0.0, 0.0], [0.0, 0.00064, 0.0], [0.0, 0.0, 0.00064]]},
         'relative': True, 'translation': [0.0, 0.0, 0.025], 'rotation': [1.0, 0.0, 0.0, 0.0]},
       {'node': 'G_gripper0', 'type': 'geom', 'geometry': {'primitive': 'scen', 'args': {'fname': 'gripper_plate.scen'}},
         'relative': True, 'translation': [0.0, 0.0, 0.0], 'rotation': [ -0.57735027, -0.57735027, -0.57735027,  2.0943951 ]},
    ],
    [
        {'node': 'GRIPPER1', 'type': 'link', 'joint': {'type': 'revolute', 'axis': [1.0, 0.0, 0.0]},
         # 'mass-param': {'mass': 0.04, 'COM': [0.0, 0.0, 0.07], 'inertia': [[0.00011, 0.0, 0.0], [0.0, 0.00010, 0.0], [0.0, 0.0, 0.00003]]},
         'relative': True, 'translation': [0.0, 0.0, 0.025], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        {'node': 'G_gripper1', 'type': 'geom', 'geometry': {'primitive': 'scen', 'args': {'fname': 'gripper_plate.scen'}},
         'relative': True, 'translation': [0.0, 0.0, 0.0], 'rotation': [ 0.57735027, -0.57735027,  0.57735027,  2.0943951 ]},
    ]
]

import make_robot_graph as rg
gg = rg.RobotTree.generate_from_list(lst, add_root=False);gg.update_coords();gg.add_geometries_for_joints(scale=0.16);gg.add_geometries_for_links(scale=0.22);
rtb = rg.RobotTreeBuilder(); rtb.buildRobotFromTree(gg)
#
# joint only_valid, only_invalid, both
# link  only_valid, only_invalid, both
#
def rename(builder_inst, prefix='', suffix=''):
    for lk in rtb.body.links:
        if lk.jointId >= 0:
            name = lk.name
            lk.setJointName(name)
            lk.setName('{}{}{}'.format(prefix, name, suffix))

rename(rtb, suffix='_lk')
rtb.exportBody('arm.body')
rtb.exportURDF('arm.urdf', MeshURLPrefix='package://simple_move/model/', useXacro=True)

###
def _fixMassparam(lk):
    if lk.mass == 1.0 and lk.c[0] == 0.0 and lk.c[1] == 0.0 and lk.c[2] == 0.0 and lk.I[0][0] == 1.0 and lk.I[0][1] == 0.0 and lk.I[0][2] == 0.0 and lk.I[1][1] == 1.0 and lk.I[1][2] == 0.0 and lk.I[2][2] == 1.0:
        lk.mass == 0.0
        lk.I[0][0] == 0.0
        lk.I[1][1] == 0.0
        lk.I[2][2] == 0.0

def fixMassparam(bd):
    for lk in bd.links:
        _fixMassparam(lk)

bd=iu.loadRobot('simple_move.urdf')
fixMassparam(bd)
iu.exportBody('simple_move.body', bd)

class JointChain(object):

    def __init__(self, joints):
        self.joints = joints

    def __len__(self):
        return len(self.joints)

    def __repr__(self):
        return f"{self}"

    def __str__(self):
        joints_as_stirng = [f"{joint}" for joint in self.joints]
        return f"Joints: {joints_as_stirng}"

    def last(self):
        return self.joints[len(self.joints) - 1]

    def get(self, index):
        return self.joints[index]

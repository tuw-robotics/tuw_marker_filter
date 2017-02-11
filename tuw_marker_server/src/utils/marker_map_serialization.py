from marker_msgs.msg import MarkerWithCovarianceArray
from marker_msgs.msg import MarkerWithCovariance
from marker_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion


class MarkerWithCovarianceArraySerializer():

    @classmethod
    def to_json(cls, msg):
        return {'markers': map(MarkerWithCovarianceSerializer.to_json, msg.markers)}

    @classmethod
    def from_json(cls, obj):
        msg = MarkerWithCovarianceArray()
        msg.markers = map(MarkerWithCovarianceSerializer.from_json, obj['markers'])
        return msg


class MarkerWithCovarianceSerializer():

    @classmethod
    def to_json(cls, msg):
        return {
          'marker': MarkerSerializer.to_json(msg.marker),
          'covariance': msg.covariance
        }

    @classmethod
    def from_json(cls, obj):
        msg = MarkerWithCovariance()
        msg.marker = MarkerSerializer.from_json(obj['marker'])
        msg.covariance = obj['covariance']
        return msg


class MarkerSerializer():

    @classmethod
    def to_json(cls, msg):
        return {
            'ids': msg.ids,
            'ids_confidence': msg.ids_confidence,
            'pose': PoseSerializer.to_json(msg.pose)
        }

    @classmethod
    def from_json(cls, obj):
        msg = Marker()
        msg.ids = obj['ids']
        msg.ids_confidence = obj['ids_confidence']
        msg.pose = PoseSerializer.from_json(obj['pose'])
        return msg


class PoseSerializer():

    @classmethod
    def to_json(cls, msg):
        return {
            'position': {'x': msg.position.x, 'y': msg.position.y, 'z': msg.position.z},
            'orientation': {'x': msg.orientation.x, 'y': msg.orientation.y, 'z': msg.orientation.z, 'w': msg.orientation.w}
        }

    @classmethod
    def from_json(cls, obj):
        msg = Pose()
        msg.position = Point(obj['position']['x'], obj['position']['y'], obj['position']['z'])
        msg.orientation = Quaternion(obj['orientation']['x'], obj['orientation']['y'], obj['orientation']['z'], obj['orientation']['w'])
        return msg

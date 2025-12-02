#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Virtual Geomagic Touch (simulation-only) using RViz Interactive Marker.
# Publishes:
#   /geomagic/pose    (geometry_msgs/PoseStamped)
#   /geomagic/buttons (std_msgs/UInt8)  bit0=clutch, bit1=freeze toggle

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler


class VirtualGeomagic(Node):
    def __init__(self):
        super().__init__('virtual_geomagic')

        # Publishers that mimic a Geomagic Touch
        self.pose_pub = self.create_publisher(PoseStamped, '/geomagic/pose', 10)
        self.button_pub = self.create_publisher(UInt8, '/geomagic/buttons', 10)

        # Interactive marker server + context menu
        self.server = InteractiveMarkerServer(self, 'virtual_geomagic_im_server')
        self.menu_handler = MenuHandler()

        self.clutch_enabled = False
        self.menu_enable = self.menu_handler.insert("Enable clutch", callback=self.on_menu)
        self.menu_disable = self.menu_handler.insert("Disable clutch", callback=self.on_menu)
        self.menu_toggle_freeze = self.menu_handler.insert("Toggle freeze", callback=self.on_menu)

        # Create the 6-DoF marker
        im = InteractiveMarker()
        im.header.frame_id = 'world'  # you can change to 'wx250/base_link' if you prefer
        im.name = 'geomagic_stylus'
        im.description = 'Virtual Geomagic Stylus'
        im.scale = 0.2

        # Visual: small sphere at the stylus tip
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.03
        sphere.scale.y = 0.03
        sphere.scale.z = 0.03
        sphere.color.r = 0.2
        sphere.color.g = 0.7
        sphere.color.b = 1.0
        sphere.color.a = 1.0

        control_visual = InteractiveMarkerControl()
        control_visual.always_visible = True
        control_visual.markers.append(sphere)
        im.controls.append(control_visual)

        # Helper to add rotation/translation axes with valid float quaternions
        def make_axis_control(name, axis_tuple, mode):
            # axis_tuple like ('x',1.0,'y',0.0,'z',0.0,'w',1.0)
            c = InteractiveMarkerControl()
            c.name = name
            c.interaction_mode = mode
            c.orientation.x = float(axis_tuple[1])
            c.orientation.y = float(axis_tuple[3])
            c.orientation.z = float(axis_tuple[5])
            c.orientation.w = float(axis_tuple[7])
            return c

        # 6-DoF controls (X/Y/Z rotate + move)
        im.controls.append(make_axis_control("rotate_x", ('x', 1.0, 'y', 0.0, 'z', 0.0, 'w', 1.0),
                                             InteractiveMarkerControl.ROTATE_AXIS))
        im.controls.append(make_axis_control("move_x",   ('x', 1.0, 'y', 0.0, 'z', 0.0, 'w', 1.0),
                                             InteractiveMarkerControl.MOVE_AXIS))
        im.controls.append(make_axis_control("rotate_z", ('x', 0.0, 'y', 1.0, 'z', 0.0, 'w', 1.0),
                                             InteractiveMarkerControl.ROTATE_AXIS))
        im.controls.append(make_axis_control("move_z",   ('x', 0.0, 'y', 1.0, 'z', 0.0, 'w', 1.0),
                                             InteractiveMarkerControl.MOVE_AXIS))
        im.controls.append(make_axis_control("rotate_y", ('x', 0.0, 'y', 0.0, 'z', 1.0, 'w', 1.0),
                                             InteractiveMarkerControl.ROTATE_AXIS))
        im.controls.append(make_axis_control("move_y",   ('x', 0.0, 'y', 0.0, 'z', 1.0, 'w', 1.0),
                                             InteractiveMarkerControl.MOVE_AXIS))

        # Register marker + menu and apply
        self.server.insert(im)
        self.server.setCallback(im.name, self.cb_feedback)
        self.menu_handler.apply(self.server, im.name)
        self.server.applyChanges()

        self.get_logger().info("Virtual Geomagic ready. Move 'Virtual Geomagic Stylus' in RViz; right-click for clutch/freeze.")

    # Menu callback
    def on_menu(self, feedback):
        title = self.menu_handler.getTitle(feedback.menu_entry_id)
        if title == "Enable clutch":
            self.clutch_enabled = True
            self._publish_buttons(clutch=True, freeze_toggle=False)
            self.get_logger().info("Clutch ENABLED")
        elif title == "Disable clutch":
            self.clutch_enabled = False
            self._publish_buttons(clutch=False, freeze_toggle=False)
            self.get_logger().info("Clutch DISABLED")
        elif title == "Toggle freeze":
            self._publish_buttons(clutch=self.clutch_enabled, freeze_toggle=True)
            self.get_logger().info("Freeze toggled")
        else:
            self.get_logger().warn(f"Unknown menu item: {title}")

    # Publish button bitmask (bit0=clutch, bit1=freeze toggle)
    def _publish_buttons(self, clutch: bool, freeze_toggle: bool):
        b = 0
        if clutch:
            b |= 0x01
        if freeze_toggle:
            b |= 0x02
        msg = UInt8()
        msg.data = b
        self.button_pub.publish(msg)

    # IM feedback → PoseStamped
    def cb_feedback(self, feedback):
        ps = PoseStamped()
        ps.header.frame_id = feedback.header.frame_id
        ps.header.stamp = Clock().now().to_msg()
        ps.pose = feedback.pose
        self.pose_pub.publish(ps)


def main():
    rclpy.init()
    node = VirtualGeomagic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

